#include "g920/epoch.h"

g920_epoch_result_t g920_epoch_next(const g920_epoch_input_t *input)
{
    g920_epoch_result_t out = { 0, 0, false };
    uint32_t base;

    if (input == NULL) {
        /* Без входных данных единственное честное поведение — считать, что
         * не знаем ничего, и сказать об этом. */
        out.counter = 1;
        out.epoch = 1;
        out.degraded = true;
        return out;
    }

    if (input->nvs_readable) {
        base = input->nvs_empty ? 0 : input->stored;
    } else {
        base = input->rtc;
        out.degraded = true;
    }

    /* Больший из двух: счётчику нельзя идти назад, а RTC может обогнать
     * NVS, если запись не удалась. */
    if (input->rtc > base) {
        base = input->rtc;
    }

    /*
     * Мусор в RTC на холодном старте может оказаться под самым потолком, и
     * тогда счётчик загрузок перестанет расти. Приёмник это переживёт —
     * сравнение эпох идёт по модулю 256, — но «счётчик загрузок» навсегда
     * перестал бы им быть. Начинаем отсчёт заново.
     */
    if (base == UINT32_MAX) {
        base = 0;
    }
    out.counter = base + 1;
    /*
     * В кадр едет младший байт. Обёртка 255 → 0 не особый случай: приёмник
     * сравнивает эпохи со знаком по модулю 256, и ноль после 255 для него
     * шаг вперёд, а не откат на 255 назад.
     */
    out.epoch = (uint8_t)out.counter;
    return out;
}
