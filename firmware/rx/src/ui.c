#include "ui.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "g920/board.h"
#include "g920/button.h"
#include "g920/display.h"
#include "g920/log.h"
#include "g920/store.h"
#include "g920/version.h"

#define TAG "ui"

/*
 * Перерисовка — пять раз в секунду, и не больше трёх строк за проход.
 *
 * Обе величины про одно: экран не имеет права занимать главный цикл. Через
 * этот же цикл идут такты линка (повторы security по кадру за вызов) и
 * разбор очереди ответов руля, а строка на 160 точек — это около 0.8 мс
 * занятой шины SPI. Пять кадров в секунду человек читает как «живой», а
 * три строки за проход держат худший случай в пределах пары миллисекунд.
 *
 * Поток ввода этим не задевается вовсе: он идёт из колбэка радио прямо в
 * эндпоинт, мимо главного цикла.
 */
#define REFRESH_MS 200
#define LINES_PER_TICK 3

/* Ключ настроек экрана в NVS. Не длиннее G920_STORE_KEY_MAX. */
#define UI_KEY "ui"
#define UI_SCHEMA_VERSION 1

/*
 * Границы шкалы качества, dBm.
 *
 * ESP-NOW на 2.4 ГГц уверенно работает примерно до -85 dBm, после чего
 * начинаются повторы и потери; -40 и лучше — это «платы на одном столе».
 * Шкала растянута между ними, чтобы полоска показывала **запас**, а не
 * факт наличия связи: к моменту, когда кадры уже теряются, смотреть на
 * индикатор поздно.
 */
#define RSSI_WORST (-90)
#define RSSI_BEST (-40)

typedef struct {
    char text[G920_DISPLAY_COLS + 1];
    uint16_t fg;
    uint16_t bg;
    int cells;
    bool dirty;
} line_t;

static bool s_ready;
static line_t s_lines[G920_DISPLAY_ROWS];
static uint8_t s_next_line;
static uint32_t s_last_refresh_ms;
static bool s_warned; /* показано предупреждение о режиме прошивки */
static int s_shown_quality = -1;

/* Предыдущий снимок счётчиков — для темпа. */
static uint32_t s_prev_input;
static uint32_t s_prev_ffb;
static uint32_t s_prev_ms;
static uint32_t s_input_rate;
static uint32_t s_ffb_rate;

/* --- ступень подсветки в NVS -------------------------------------------- */

static void backlight_load(void)
{
    uint8_t step = G920_DISPLAY_BL_STEPS - 1;
    size_t got = 0;

    if (g920_store_read(UI_KEY, G920_STORE_KIND_UI, UI_SCHEMA_VERSION, &step,
                        sizeof(step), &got, NULL)
            == G920_STORE_OK
        && got == sizeof(step) && step < G920_DISPLAY_BL_STEPS) {
        g920_display_backlight_set(step);
        return;
    }
    /* Ничего не лежит — полная яркость: первый показ должен быть виден. */
    g920_display_backlight_set(G920_DISPLAY_BL_STEPS - 1);
}

static void backlight_save(uint8_t step)
{
    /*
     * Запись на каждое нажатие, а не по таймеру. Нажатий за жизнь донгла
     * единицы тысяч, ресурс NVS — сотни тысяч циклов на сектор, а
     * отложенная запись потерялась бы ровно в том случае, ради которого
     * настройка и заводится: выключили питание после того, как погасили
     * экран.
     */
    (void)g920_store_write(UI_KEY, G920_STORE_KIND_UI, UI_SCHEMA_VERSION,
                           &step, sizeof(step));
}

/* --- строки -------------------------------------------------------------- */

static void set_line(int row, uint16_t fg, uint16_t bg, int cells,
                     const char *fmt, ...) __attribute__((format(printf, 5, 6)));

static void set_line(int row, uint16_t fg, uint16_t bg, int cells,
                     const char *fmt, ...)
{
    char buf[G920_DISPLAY_COLS + 1];
    va_list args;
    line_t *line;

    if (row < 0 || row >= G920_DISPLAY_ROWS) {
        return;
    }
    line = &s_lines[row];

    va_start(args, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    /*
     * Строка помечается на перерисовку только если **изменилась**.
     *
     * Это не экономия ради экономии: без сравнения экран переписывался бы
     * целиком пять раз в секунду, то есть занимал бы шину и цикл там, где
     * меняются одна-две строки из десяти. Заодно исчезает мерцание — точка,
     * которую переписали тем же цветом, всё равно моргает.
     */
    if (line->fg == fg && line->bg == bg && line->cells == cells
        && strcmp(line->text, buf) == 0) {
        return;
    }
    (void)snprintf(line->text, sizeof(line->text), "%s", buf);
    line->fg = fg;
    line->bg = bg;
    line->cells = cells;
    line->dirty = true;
}

static void flush_lines(void)
{
    int drawn = 0;

    for (int i = 0; i < G920_DISPLAY_ROWS && drawn < LINES_PER_TICK; i++) {
        int row = (s_next_line + i) % G920_DISPLAY_ROWS;
        line_t *line = &s_lines[row];

        if (!line->dirty) {
            continue;
        }
        g920_display_text_n(0, row, line->text, line->cells, line->fg,
                            line->bg);
        line->dirty = false;
        drawn++;
        /*
         * Следующий проход начинается со строки за последней нарисованной:
         * иначе при постоянно грязных верхних строках нижние не
         * обновлялись бы никогда.
         */
        s_next_line = (uint8_t)((row + 1) % G920_DISPLAY_ROWS);
    }
}

static void mark_all_dirty(void)
{
    for (int i = 0; i < G920_DISPLAY_ROWS; i++) {
        s_lines[i].dirty = true;
    }
    s_shown_quality = -1;
}

/* --- содержимое ---------------------------------------------------------- */

/* Качество связи в процентах шкалы, 0..100. */
static int quality_of(int8_t rssi, bool peer)
{
    int q;

    if (!peer || rssi == 0) {
        return 0;
    }
    q = ((int)rssi - RSSI_WORST) * 100 / (RSSI_BEST - RSSI_WORST);
    if (q < 0) {
        q = 0;
    }
    if (q > 100) {
        q = 100;
    }
    return q;
}

/*
 * Одно слово о состоянии всей связки — то, ради чего на экран смотрят
 * первым делом.
 *
 * Порядок проверок — от своего к чужому: сперва то, что чинится здесь и
 * сейчас (нет передатчика), потом то, что зависит от консоли. Иначе
 * «нет хоста» перекрывало бы «нет связи с рулём», а чинить надо разное.
 */
static const char *verdict(const g920_ui_state_t *st, uint16_t *color)
{
    if (!st->link_peer) {
        *color = G920_COLOR_RED;
        return "NO TX";
    }
    if (!st->usb_configured) {
        *color = G920_COLOR_YELLOW;
        return "NO HOST";
    }
    if (st->usb_suspended) {
        *color = G920_COLOR_GRAY;
        return "SLEEP";
    }
    if (!st->auth_done) {
        *color = G920_COLOR_YELLOW;
        return "AUTH";
    }
    *color = G920_COLOR_GREEN;
    return "READY";
}

static void compose(const g920_ui_state_t *st, uint32_t now_ms)
{
    uint16_t state_color = G920_COLOR_GRAY;
    const char *state_word = verdict(st, &state_color);
    uint32_t up_s = now_ms / 1000u;
    int quality = quality_of(st->rssi, st->link_peer);

    /* Шапка: состояние словом и цветом фона — читается через комнату. */
    set_line(0, G920_COLOR_BLACK, state_color, G920_DISPLAY_COLS, " G920  %s",
             state_word);

    /*
     * Уровень сигнала: цифра и полоска. Полоска рисуется отдельно (см.
     * ниже), поэтому текст здесь ограничен по ширине.
     */
    if (st->link_peer && st->rssi != 0) {
        set_line(1, G920_COLOR_WHITE, G920_COLOR_BLACK, 14, "LINK %ddBm",
                 (int)st->rssi);
    } else {
        set_line(1, G920_COLOR_GRAY, G920_COLOR_BLACK, 14, "LINK --");
    }

    /* Руль: чем представляемся консоли и жив ли он по словам передатчика. */
    if (st->have_identity) {
        set_line(2, st->wheel_ready ? G920_COLOR_WHITE : G920_COLOR_GRAY,
                 G920_COLOR_BLACK, G920_DISPLAY_COLS, "WHEEL %04x %s",
                 (unsigned)st->product_id, st->wheel_ready ? "UP" : "WAIT");
    } else {
        set_line(2, G920_COLOR_YELLOW, G920_COLOR_BLACK, G920_DISPLAY_COLS,
                 "WHEEL no identity");
    }

    /* Консоль: три различимых состояния, а не «есть/нет». */
    set_line(3, G920_COLOR_WHITE, G920_COLOR_BLACK, G920_DISPLAY_COLS,
             "HOST  %s",
             st->usb_suspended ? "asleep"
                               : (st->usb_configured ? "configured" : "absent"));

    set_line(4, st->auth_done ? G920_COLOR_GREEN : G920_COLOR_YELLOW,
             G920_COLOR_BLACK, G920_DISPLAY_COLS, "AUTH  %s  START %s",
             st->auth_done ? "ok" : "...", st->start_seen ? "y" : "n");

    /* Поток ввода: темп и потери — то, что видно руками. */
    set_line(5, st->input_lost != 0 ? G920_COLOR_YELLOW : G920_COLOR_WHITE,
             G920_COLOR_BLACK, G920_DISPLAY_COLS, "IN  %u/s  lost %u",
             (unsigned)s_input_rate, (unsigned)st->input_lost);

    set_line(6, G920_COLOR_WHITE, G920_COLOR_BLACK, G920_DISPLAY_COLS,
             "FFB %u/s  retry %u", (unsigned)s_ffb_rate,
             (unsigned)st->retries);

    /* Что отсеял приёмник: пока тут нули, эфир ни при чём. */
    set_line(7,
             (st->rx_gaps != 0 || st->gave_up != 0) ? G920_COLOR_YELLOW
                                                    : G920_COLOR_GRAY,
             G920_COLOR_BLACK, G920_DISPLAY_COLS, "RF  gap%u dup%u lost%u",
             (unsigned)st->rx_gaps, (unsigned)st->rx_dup,
             (unsigned)st->gave_up);

    set_line(8, G920_COLOR_GRAY, G920_COLOR_BLACK, G920_DISPLAY_COLS,
             "UP  %u:%02u:%02u", (unsigned)(up_s / 3600u),
             (unsigned)((up_s / 60u) % 60u), (unsigned)(up_s % 60u));

    /* Подсказка про кнопку: без неё про режим прошивки никто не догадается. */
    set_line(9, G920_COLOR_GRAY, G920_COLOR_BLACK, G920_DISPLAY_COLS,
             "BL %u/%u  hold=flash",
             (unsigned)(g920_display_backlight_step() + 1u),
             (unsigned)G920_DISPLAY_BL_STEPS);

    /*
     * Полоска сигнала — в пикселях, справа от строки 1.
     *
     * Перерисовывается только при заметном изменении: RSSI шевелится на
     * единицы dBm постоянно, и перерисовка на каждый шаг — это мерцание
     * там, где ничего не произошло.
     */
    if (s_shown_quality < 0 || quality / 10 != s_shown_quality / 10) {
        s_shown_quality = quality;
        g920_display_bar(14 * G920_DISPLAY_CHAR_W + 2,
                         1 * G920_DISPLAY_CHAR_H + 1,
                         G920_DISPLAY_COLS * G920_DISPLAY_CHAR_W
                             - 14 * G920_DISPLAY_CHAR_W - 4,
                         G920_DISPLAY_CHAR_H - 2, quality, 100,
                         quality >= 50 ? G920_COLOR_GREEN
                                       : (quality >= 25 ? G920_COLOR_YELLOW
                                                        : G920_COLOR_RED),
                         G920_COLOR_BLACK);
    }
}

/* --- наружу -------------------------------------------------------------- */

bool g920_ui_init(void)
{
    char version[G920_VERSION_STR_MAX];

    if (!g920_display_init()) {
        /* Экрана нет — это девкит, и это нормально. Кнопку всё равно
         * поднимаем: на плате без экрана она тоже уводит в загрузчик. */
        (void)g920_button_init();
        return false;
    }
    s_ready = true;
    backlight_load();
    (void)g920_button_init();

    g920_display_clear(G920_COLOR_BLACK);
    if (g920_version_format(version, sizeof(version), g920_firmware_version())
        < 0) {
        version[0] = '\0';
    }
    /* Заставка живёт до первого такта: человеку важно увидеть, что плата
     * ожила и какая на ней прошивка, ещё до того, как появится связь. */
    g920_display_text(0, 0, " G920 DONGLE", G920_COLOR_BLACK,
                      G920_COLOR_CYAN);
    g920_display_text(0, 2, version, G920_COLOR_WHITE, G920_COLOR_BLACK);
    g920_display_text(0, 4, "starting...", G920_COLOR_GRAY, G920_COLOR_BLACK);
    mark_all_dirty();
    G920_LOGI(TAG, "display up, backlight step %u",
              (unsigned)g920_display_backlight_step());
    return true;
}

void g920_ui_show_flash_mode(void)
{
    if (!s_ready) {
        return;
    }
    g920_display_clear(G920_COLOR_BLACK);
    g920_display_text(0, 3, "  FLASH MODE", G920_COLOR_BLACK, G920_COLOR_RED);
    g920_display_text(0, 5, " connect to PC", G920_COLOR_WHITE,
                      G920_COLOR_BLACK);
    g920_display_text(0, 6, " and run esptool", G920_COLOR_GRAY,
                      G920_COLOR_BLACK);
    /* Подсветку на полную: человек смотрит на экран именно сейчас, а
     * ступень могла быть погашенной. */
    g920_display_backlight_set(G920_DISPLAY_BL_STEPS - 1);
}

static void show_warning(void)
{
    if (!s_ready) {
        return;
    }
    g920_display_backlight_set(G920_DISPLAY_BL_STEPS - 1);
    g920_display_clear(G920_COLOR_BLACK);
    g920_display_text(0, 3, " KEEP HOLDING", G920_COLOR_BLACK,
                      G920_COLOR_YELLOW);
    g920_display_text(0, 5, " -> flash mode", G920_COLOR_WHITE,
                      G920_COLOR_BLACK);
    g920_display_text(0, 6, " release=cancel", G920_COLOR_GRAY,
                      G920_COLOR_BLACK);
}

g920_ui_action_t g920_ui_tick(const g920_ui_state_t *state, uint32_t now_ms)
{
    g920_button_event_t event = g920_button_poll(now_ms);

    switch (event) {
    case G920_BTN_SHORT:
        /* Кнопка отпущена коротко — следующая ступень подсветки. Если
         * висело предупреждение, оно и так уходит: экран перерисуется
         * целиком. */
        if (s_warned) {
            s_warned = false;
            if (s_ready) {
                g920_display_clear(G920_COLOR_BLACK);
            }
            mark_all_dirty();
        } else {
            backlight_save(g920_display_backlight_next());
        }
        break;
    case G920_BTN_ARMED:
        s_warned = true;
        show_warning();
        break;
    case G920_BTN_LONG:
        return G920_UI_FLASH;
    case G920_BTN_NONE:
    default:
        break;
    }

    if (state == NULL || !s_ready) {
        return G920_UI_NOTHING;
    }
    /*
     * Пока висит предупреждение, экран не трогаем: перерисовка стёрла бы
     * ровно то, что человек в этот момент читает. Отпустил — вернёмся к
     * состоянию (см. ветку SHORT выше).
     */
    if (s_warned && !g920_button_down()) {
        s_warned = false;
        g920_display_clear(G920_COLOR_BLACK);
        mark_all_dirty();
    }
    if (s_warned) {
        return G920_UI_NOTHING;
    }

    if ((uint32_t)(now_ms - s_last_refresh_ms) >= REFRESH_MS) {
        uint32_t dt = now_ms - s_prev_ms;

        s_last_refresh_ms = now_ms;
        /*
         * Темп считается по разнице накопленных счётчиков. Первый проход
         * (`s_prev_ms == 0`) пропускается: делить на время с начала времён
         * значит показать человеку осмысленно выглядящую чушь.
         */
        if (s_prev_ms != 0 && dt > 0) {
            s_input_rate = (state->input_fwd - s_prev_input) * 1000u / dt;
            s_ffb_rate = (state->ffb_sent - s_prev_ffb) * 1000u / dt;
        }
        s_prev_input = state->input_fwd;
        s_prev_ffb = state->ffb_sent;
        s_prev_ms = now_ms;

        compose(state, now_ms);
    }
    flush_lines();
    return G920_UI_NOTHING;
}
