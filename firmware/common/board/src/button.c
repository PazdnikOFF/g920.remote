#include "g920/button.h"

#ifdef G920_BUTTON_GPIO

#include "driver/gpio.h"

#include "g920/log.h"

#define TAG "button"

/*
 * Пороги. Короткое — всё, что отпущено раньше `ARM_MS`; дальше идёт
 * предупреждение, а на `LONG_MS` срабатывает долгое.
 *
 * Полторы секунды до предупреждения — чтобы случайное залипание пальца на
 * выборе яркости не показывало страшную надпись. Три до срабатывания —
 * чтобы между «увидел предупреждение» и «поздно» оставалось столько же
 * времени, сколько человек уже продержал: успеть отпустить можно всегда.
 */
#define DEBOUNCE_MS 30
#define ARM_MS 1500
#define LONG_MS 3000

/*
 * Кнопка замыкает вывод на землю, поэтому подтяжка вверх и нажатие — ноль.
 * У GPIO0 подтяжка есть и снаружи (её требует загрузчик), но своя не
 * мешает и снимает зависимость от конкретной разводки.
 */
#define PRESSED_LEVEL 0

static bool s_ready;
static bool s_down;          /* устоявшееся состояние */
static bool s_raw_last;      /* последнее сырое чтение */
static uint32_t s_raw_at_ms; /* когда сырое изменилось */
static uint32_t s_down_at_ms;
static bool s_armed_sent;
static bool s_long_sent;

bool g920_button_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << (G920_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    if (gpio_config(&cfg) != ESP_OK) {
        G920_LOGE(TAG, "gpio config failed");
        return false;
    }
    s_ready = true;
    s_raw_last = (gpio_get_level((gpio_num_t)(G920_BUTTON_GPIO))
                  == PRESSED_LEVEL);
    s_down = s_raw_last;
    return true;
}

bool g920_button_down(void)
{
    return s_down;
}

g920_button_event_t g920_button_poll(uint32_t now_ms)
{
    bool raw;

    if (!s_ready) {
        return G920_BTN_NONE;
    }
    raw = (gpio_get_level((gpio_num_t)(G920_BUTTON_GPIO)) == PRESSED_LEVEL);

    /*
     * Дребезг гасится выдержкой, а не счётчиком опросов: цикл донгла
     * тикает раз в 2 мс, но при печати журнала или перерисовке экрана
     * промежуток вырастает, и «сколько раз подряд прочли одно и то же»
     * означало бы разное время в разных местах работы.
     */
    if (raw != s_raw_last) {
        s_raw_last = raw;
        s_raw_at_ms = now_ms;
        return G920_BTN_NONE;
    }
    if (raw == s_down || (uint32_t)(now_ms - s_raw_at_ms) < DEBOUNCE_MS) {
        /* Состояние не менялось — но удержание всё равно надо отмерить. */
        if (s_down) {
            uint32_t held = (uint32_t)(now_ms - s_down_at_ms);

            if (!s_long_sent && held >= LONG_MS) {
                s_long_sent = true;
                return G920_BTN_LONG;
            }
            if (!s_armed_sent && held >= ARM_MS) {
                s_armed_sent = true;
                return G920_BTN_ARMED;
            }
        }
        return G920_BTN_NONE;
    }

    /* Состояние устоялось и оно новое. */
    s_down = raw;
    if (s_down) {
        s_down_at_ms = now_ms;
        s_armed_sent = false;
        s_long_sent = false;
        return G920_BTN_NONE;
    }
    /*
     * Отпустили. Короткое событие отдаём только если долгое **не**
     * срабатывало: иначе человек, додержавший до режима прошивки, на
     * отпускании получил бы ещё и смену яркости — то есть увидел бы, что
     * кнопка сделала не то, что обещала.
     */
    if (!s_long_sent) {
        return G920_BTN_SHORT;
    }
    return G920_BTN_NONE;
}

#else /* кнопки в сборке нет */

bool g920_button_init(void)
{
    return false;
}

g920_button_event_t g920_button_poll(uint32_t now_ms)
{
    (void)now_ms;
    return G920_BTN_NONE;
}

bool g920_button_down(void)
{
    return false;
}

#endif
