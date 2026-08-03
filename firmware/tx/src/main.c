/*
 * TX — передатчик у руля.
 *
 * M0: каркас. M3: замерочный режим — гонит кадры с меткой времени и
 * считает RTT по эху от RX. Настоящее чтение руля появится в M1.
 */

/*
 * Замерочная прошивка вехи M3. Профиль `tx-host` собирает вместо неё
 * `main_host.c` — развод флагом, а не `build_src_filter`: при
 * `framework = espidf` фильтр scons не действует.
 */
#ifndef G920_MODE_HOST

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "g920/board.h"
#include "g920/link.h"
#include "g920/link_stats.h"
#include "g920/log.h"
#include "g920/proto.h"
#include "g920/queue.h"
#include "g920/store.h"
#include "g920/timestamp.h"
#include "g920/version.h"

static const char *TAG = "boot";
static const char *M3 = "m3";

/*
 * Развёртка по частотам. Вопрос M3 — не «работает ли», а «на какой частоте
 * ещё держится»: по этому числу в M5 выбирается частота отчётов.
 *
 * Периоды в миллисекундах, потому что тик планировщика 1 кГц и дробных
 * периодов он не отработает: 50, 100, 200, 250 и 500 Гц.
 */
static const uint16_t PERIODS_MS[] = { 20, 10, 5, 4, 2 };
#define STEP_MS 5000

/*
 * Базовый такт цикла. Раньше цикл спал на период развёртки, и на нижней
 * ступени такт линка выходил 20 мс: пятнадцать попыток растягивались на
 * триста миллисекунд вместо девяноста, то есть далеко за чужой ACK timeout
 * 100 мс, ради которого число попыток и выбиралось. Теперь такт линка
 * миллисекунда — как в модели, — а период развёртки отсчитывается
 * накопителем.
 */
#define TICK_MS 1

/*
 * Эхо возвращает только метку времени, а не весь кадр. В бою обратный
 * поток — это FFB, а не зеркало отчётов, и гнать назад 64 байта значило бы
 * мерить вдвое худшие условия, чем будут на самом деле.
 */
#define ECHO_LEN 8

/* Метка отправки едет в кадре и возвращается обратно — так RTT считается
 * без синхронизации часов между платами. */
typedef struct {
    uint64_t sent_at_us;
    uint8_t filler[56]; /* до 64 байт: размер отчёта руля по GIP */
} probe_t;

static g920_link_stats_t stats;
static g920_fresh_queue_t fresh;

/*
 * Статистику пишут два контекста: задача Wi-Fi (эхо приходит в приёмном
 * колбэке) и главная задача (отправка, печать, обнуление между ступенями).
 * Без замка обнуление на границе ступени может разъехаться с обновлением из
 * колбэка — и испортить ровно те числа, ради которых всё и затевалось.
 */
static portMUX_TYPE stats_lock = portMUX_INITIALIZER_UNLOCKED;

static void on_frame(void *ctx, const g920_frame_t *frame,
                     const uint8_t *peer_mac, g920_rx_verdict_t verdict)
{
    (void)ctx;
    (void)peer_mac;
    (void)verdict;

    if (frame->type != G920_FRAME_INPUT
        || frame->length < sizeof(uint64_t)) {
        return;
    }

    uint64_t sent_at;
    uint64_t now = g920_timestamp_us();

    /* Дубликаты и опоздавшие эха сюда не доходят: их отсеял линк, и его
     * счётчики снимаются на границе ступени. */
    memcpy(&sent_at, frame->payload, sizeof(sent_at));
    portENTER_CRITICAL_SAFE(&stats_lock);
    if (now >= sent_at) {
        g920_link_stats_on_rtt(&stats, (uint32_t)(now - sent_at));
    }
    portEXIT_CRITICAL_SAFE(&stats_lock);
}

void app_main(void)
{
    char version[G920_VERSION_STR_MAX];
    char line[G920_LINK_STATS_STR_MAX];
    g920_store_status_t status;
    probe_t probe;
    /* Кадр, забранный из свежей очереди, и буфер под его нагрузку. */
    g920_frame_t out;
    uint8_t out_buf[sizeof(probe_t)];
    uint16_t sequence = 0;
    uint32_t since_report_ms = 0;
    size_t step = 0;
    uint32_t reliable_sent = 0;
    /*
     * Что из кумулятивных счётчиков уже показано прошлой ступенью. Разность
     * с текущим значением и есть число этой ступени.
     */
    uint32_t reliable_sent_seen = 0;
    uint32_t retries_seen = 0;
    uint32_t gave_up_seen = 0;
    uint32_t refused_seen = 0;
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

    g920_link_stats_init(&stats);
    g920_fresh_init(&fresh);
    memset(&probe, 0, sizeof(probe));

    if (g920_link_init(on_frame, NULL) != G920_LINK_OK) {
        G920_LOGE(TAG, "link init failed");
        g920_board_led_set(G920_IND_FAULT);
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    uint32_t since_probe_ms = 0;
    uint32_t since_blink_ms = 0;

    for (;;) {
        /* Такт линка зовём всегда: он сам решает, надо ли в эфир, и сам
         * ограничивает частоту. Раз в миллисекунду — от этого зависит срок
         * жизни надёжного кадра до отказа. */
        g920_link_tick(g920_timestamp_us());

        /* Индикатор и отправка — по «пир жив», а не «адрес известен»:
         * MAC из NVS не доказывает, что модуль на том конце существует. */
        if (!g920_link_has_peer()) {
            since_blink_ms += TICK_MS;
            if (since_blink_ms >= 200) {
                since_blink_ms = 0;
                g920_board_led_set(on ? G920_IND_DETECT : G920_IND_OFF);
                on = !on;
            }
            vTaskDelay(pdMS_TO_TICKS(TICK_MS));
            continue;
        }

        uint16_t period = PERIODS_MS[step];

        /*
         * Отчёты идут через свежую очередь, а не прямо в эфир. Так меряется
         * тот путь, который будет в продукте: источник кладёт последнее
         * состояние, такт линка забирает.
         *
         * На этом стенде `evicted` останется нулём: источник даёт кадр раз
         * в 2 мс и реже, а забирают каждую миллисекунду. Это не проверка
         * коалесценции (она в юнит-тестах), а канарейка: единица здесь
         * означает, что источник обогнал радио.
         */
        if (since_probe_ms >= period) {
            since_probe_ms = 0;
            probe.sent_at_us = g920_timestamp_us();
            (void)g920_fresh_push(&fresh, G920_FRAME_INPUT, sequence++,
                                  (const uint8_t *)&probe, sizeof(probe));
        }

        if (g920_fresh_pop(&fresh, &out, out_buf, sizeof(out_buf)) == 1) {
            bool ok = g920_link_send((g920_frame_type_t)out.type, out.sequence,
                                     out.flags, out.payload, out.length)
                      == G920_LINK_OK;

            portENTER_CRITICAL_SAFE(&stats_lock);
            g920_link_stats_on_sent(&stats, ok);
            portEXIT_CRITICAL_SAFE(&stats_lock);
        }

        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
        since_probe_ms += TICK_MS;
        since_report_ms += TICK_MS;

        if (since_report_ms >= STEP_MS) {
            g920_link_stats_t snapshot;
            g920_link_rx_counters_t rx;

            since_report_ms = 0;
            g920_board_led_set(G920_IND_OK);

            /* Счётчики приёмника — снимком со сбросом: ступень описывает
             * себя, а не всю жизнь платы. */
            g920_link_rx_counters(&rx, true);

            /* Снимок под замком, печать — без него: строка форматируется
             * долго, и держать на это время спинлок значило бы тормозить
             * приёмный колбэк. */
            portENTER_CRITICAL_SAFE(&stats_lock);
            g920_link_stats_add_rx(&stats, rx.delivered, rx.duplicates,
                                   rx.stale);
            snapshot = stats;
            portEXIT_CRITICAL_SAFE(&stats_lock);

            if (g920_link_stats_format(line, sizeof(line), &snapshot) > 0) {
                G920_LOGI(M3, "%u Hz: %s", (unsigned)(1000u / period), line);
            }
            /* Кумулятивно, в отличие от остальных: вытеснение — это не
             * свойство ступени, а признак того, что источник вообще обогнал
             * радио хоть раз. На этом стенде обязан остаться нулём. */
            G920_LOGI(M3, "fresh: evicted %u total",
                      (unsigned)g920_fresh_evicted(&fresh));

            /* dup и stale — про поток эха, а не про надёжную дисциплину:
             * подписывать их «reliable» значило бы называть число не тем,
             * что оно есть. */
            G920_LOGI(M3, "echo: dup %u, stale %u, foreign %u, peer restarts %u",
                      (unsigned)rx.duplicates, (unsigned)rx.stale,
                      (unsigned)rx.foreign, (unsigned)rx.sessions);

            /*
             * Проверка надёжной дисциплины на живом линке. Считаем ДО
             * постановки следующего: если подтверждения не ходят, кадр
             * бросается через 15 попыток по 6 мс, и «брошено» начнёт расти
             * на единицу каждые пять секунд.
             */
            /*
             * «Повторов» здесь не для красоты: «брошено» показывает только
             * исходы, а ноль брошенных ценой трёх повторов и ценой трёхсот
             * — это разные каналы, и по второму замер надо переделывать.
             * Эпоха рядом затем, чтобы в логе было видно, к какой загрузке
             * относятся числа: без неё перезагрузка платы посреди развёртки
             * выглядит просто сменой цифр.
             *
             * **Числа поступенчатые, а не от загрузки.** Сами счётчики
             * линка кумулятивны и обнулять их снаружи нечем, поэтому
             * разность считается здесь. Кумулятивное число рядом с
             * поступенчатым в одной строке — это ровно та конструкция, за
             * которую веха получила пятый BLOCK: она выглядит замером
             * ступени, не будучи им. Итоговые значения печатаются отдельно
             * и подписаны «total».
             */
            {
                uint32_t retries = g920_link_reliable_retries();
                uint32_t gave_up = g920_link_reliable_gave_up();
                uint32_t refused = g920_link_reliable_send_failed();

                G920_LOGI(M3,
                          "reliable: sent %u, pending %u, retries %u, "
                          "gave up %u, gaps %u, driver refused %u, epoch %u",
                          (unsigned)(reliable_sent - reliable_sent_seen),
                          (unsigned)g920_link_reliable_pending(),
                          (unsigned)(retries - retries_seen),
                          (unsigned)(gave_up - gave_up_seen),
                          (unsigned)rx.gaps,
                          (unsigned)(refused - refused_seen),
                          (unsigned)g920_link_epoch());
                if (retries != 0 || gave_up != 0 || refused != 0) {
                    G920_LOGI(M3,
                              "reliable total: retries %u, gave up %u, "
                              "driver refused %u",
                              (unsigned)retries, (unsigned)gave_up,
                              (unsigned)refused);
                }
                reliable_sent_seen = reliable_sent;
                retries_seen = retries;
                gave_up_seen = gave_up;
                refused_seen = refused;
            }
            if (g920_link_send_reliable(G920_FRAME_CONTROL, "ping", 4)
                == G920_LINK_OK) {
                reliable_sent++;
            }

            /* Счётчики обнуляем между ступенями: иначе перегрузка на
             * верхней частоте испортит числа нижних. */
            portENTER_CRITICAL_SAFE(&stats_lock);
            g920_link_stats_init(&stats);
            portEXIT_CRITICAL_SAFE(&stats_lock);
            step = (step + 1) % (sizeof(PERIODS_MS) / sizeof(PERIODS_MS[0]));
        }
    }
}

#endif /* !G920_MODE_HOST */
