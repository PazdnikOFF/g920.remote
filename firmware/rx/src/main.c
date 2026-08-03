/*
 * RX — донгл у хоста.
 *
 * M0: каркас. M3: замерочный режим — отражает кадры обратно, чтобы TX мог
 * посчитать RTT, и считает свою долю потерь и дубликатов. Роль USB Device
 * появится в M2.
 */

/*
 * ⚠ Развод прошивок — флагом сборки, а не `build_src_filter`: для
 * `framework = espidf` PlatformIO собирает `src/` через CMake, и фильтр
 * scons там не работает вовсе (проверено — файлы всё равно попадают в
 * сборку, и два `app_main` не линкуются). Поэтому лишний файл компилируется
 * пустым.
 */
#ifndef G920_MODE_GIP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "g920/board.h"
#include "g920/link.h"
#include "g920/log.h"
#include "g920/proto.h"
#include "g920/queue.h"
#include "g920/store.h"
#include "g920/timestamp.h"
#include "g920/version.h"

static const char *TAG = "boot";
static const char *M3 = "m3";

#define REPORT_EVERY_MS 5000
#define BLINK_EVERY_MS 200

/*
 * Базовый такт цикла. Раньше он был 200 мс, и пятнадцать попыток надёжного
 * кадра растягивались на три секунды вместо девяноста миллисекунд: ретраи
 * двигает g920_link_tick, и чаще, чем его зовут, они не идут.
 */
#define TICK_MS 1

/*
 * RX доли потерь не знает и знать не может: сколько кадров ушло с TX, ему
 * никто не сообщает. Раньше здесь печаталась общая строка замера, у которой
 * колонка loss структурно всегда нулевая — она выглядела результатом и им
 * не была. Осталось то, что RX действительно видит: сколько эхо не ушло.
 */
static uint32_t echo_failed;

static portMUX_TYPE stats_lock = portMUX_INITIALIZER_UNLOCKED;

static void on_frame(void *ctx, const g920_frame_t *frame,
                     const uint8_t *peer_mac, g920_rx_verdict_t verdict)
{
    (void)ctx;
    (void)peer_mac;
    (void)verdict;

    /*
     * Дубликаты и опоздавшие сюда не доходят — их отсеял линк, и их считают
     * его счётчики. Сюда попадает только то, что реально пошло бы в дело.
     */
    if (frame->type != G920_FRAME_INPUT) {
        return;
    }

    /*
     * Эхо возвращает только метку времени TX — восемь байт вместо
     * шестидесяти четырёх. Часы плат синхронизировать не нужно: метка
     * едет туда и обратно нетронутой.
     *
     * Зеркалить весь кадр было бы нечестно: в бою обратный поток это FFB,
     * а не копия отчётов, и полное зеркало мерило бы вдвое худшие
     * условия, чем будут на самом деле.
     *
     * Уходит прямо отсюда, а не через свежую очередь: очередь разгребается
     * тактом главного цикла, и на пути измерения это добавило бы к RTT свой
     * такт. Мерить надо радио, а не свой планировщик. Восемь байт нагрузки
     * идут коротким путём внутри линка — стек задачи Wi-Fi буфера в кадр
     * целиком не переживает.
     */
    if (frame->length >= sizeof(uint64_t)) {
        bool ok = g920_link_send(G920_FRAME_INPUT, frame->sequence, 0,
                                 frame->payload, sizeof(uint64_t))
                  == G920_LINK_OK;

        if (!ok) {
            portENTER_CRITICAL_SAFE(&stats_lock);
            echo_failed++;
            portEXIT_CRITICAL_SAFE(&stats_lock);
        }
    }
}

void app_main(void)
{
    char version[G920_VERSION_STR_MAX];
    g920_store_status_t status;
    bool led;
    bool on = false;

    if (g920_version_format(version, sizeof(version), g920_firmware_version())
        < 0) {
        version[0] = '?';
        version[1] = '\0';
    }
    G920_LOGI(TAG, "fw %s, link proto %u", version,
              (unsigned)G920_LINK_PROTO_VERSION);

    status = g920_store_init();
    if (status != G920_STORE_OK) {
        G920_LOGE(TAG, "store init: %s", g920_store_status_name(status));
    }

    led = g920_board_led_init();
    G920_LOGI(TAG, "led %s %s", g920_board_led_kind(), led ? "ok" : "absent");

    if (g920_link_init(on_frame, NULL) != G920_LINK_OK) {
        G920_LOGE(TAG, "link init failed");
        g920_board_led_set(G920_IND_FAULT);
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    uint32_t since_report_ms = 0;
    uint32_t since_blink_ms = 0;

    for (;;) {
        /* Миллисекундный такт, а не сон на пять секунд: спящий приёмник не
         * возобновит поиск, если пир замолчал, а ретраи надёжной дисциплины
         * идут не чаще, чем зовут этот такт. */
        g920_link_tick(g920_timestamp_us());

        since_blink_ms += TICK_MS;
        if (since_blink_ms >= BLINK_EVERY_MS) {
            since_blink_ms = 0;
            if (g920_link_has_peer()) {
                g920_board_led_set(G920_IND_OK);
            } else {
                g920_board_led_set(on ? G920_IND_DETECT : G920_IND_OFF);
                on = !on;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
        since_report_ms += TICK_MS;
        if (since_report_ms >= REPORT_EVERY_MS) {
            g920_link_rx_counters_t rx;
            uint32_t failed;

            since_report_ms = 0;
            /* Снимок со сбросом: строка описывает прошедшие пять секунд, а
             * не всю жизнь платы. */
            g920_link_rx_counters(&rx, true);

            portENTER_CRITICAL_SAFE(&stats_lock);
            failed = echo_failed;
            echo_failed = 0;
            portEXIT_CRITICAL_SAFE(&stats_lock);

            /* Эпоха в строке — чтобы перезагрузка донгла посреди развёртки
             * читалась как перезагрузка, а не как странные числа. */
            G920_LOGI(M3,
                      "in %u, dup %u, stale %u, foreign %u, gaps %u, "
                      "peer restarts %u, echo failed %u, epoch %u",
                      (unsigned)rx.delivered, (unsigned)rx.duplicates,
                      (unsigned)rx.stale, (unsigned)rx.foreign,
                      (unsigned)rx.gaps, (unsigned)rx.sessions,
                      (unsigned)failed, (unsigned)g920_link_epoch());
        }
    }
}

#endif /* !G920_MODE_GIP */
