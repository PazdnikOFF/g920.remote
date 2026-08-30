#include "g920/display.h"

#include "g920/log.h"

#define TAG "lcd"

#if defined(G920_LCD_ST7735) && defined(G920_LCD_MOSI) && defined(G920_LCD_CLK)

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef G920_LCD_BL
#include "driver/ledc.h"
#endif

/* --- настройки панели, которые различаются от партии к партии ------------ */

#ifndef G920_LCD_MADCTL
/*
 * Поворот в альбом (MV|MX) плюс порядок BGR. Почему именно так — в
 * `display.h`, там же таблица симптомов.
 */
#define G920_LCD_MADCTL 0x68
#endif
#ifndef G920_LCD_COL_OFFSET
#define G920_LCD_COL_OFFSET 1
#endif
#ifndef G920_LCD_ROW_OFFSET
#define G920_LCD_ROW_OFFSET 26
#endif
#ifndef G920_LCD_INVERT
#define G920_LCD_INVERT 1
#endif
#ifndef G920_LCD_HZ
/*
 * 26 МГц. ST7735S держит и 40, но провод до экрана на этих платах идёт
 * через переходное отверстие и разъём шлейфа, а цена ошибки — не «чуть
 * медленнее», а мусор на экране, который выглядит как сдохший драйвер.
 */
#define G920_LCD_HZ 26000000
#endif

#define LCD_W 160
#define LCD_H 80

#ifndef G920_LCD_HOST
#define G920_LCD_HOST SPI2_HOST
#endif

/* --- шрифт 5×7 ----------------------------------------------------------- */

/*
 * Каждый глиф — пять столбцов слева направо, в байте столбца младший бит
 * это верхняя точка. Такой раскладкой глиф выводится одним проходом по
 * столбцам, без битовых перестановок на каждый пиксель.
 *
 * Таблица покрывает печатный ASCII 0x20..0x7E. Всё вне диапазона рисуется
 * пустым знакоместом: на экране 26×10 подстановка «квадратика» только
 * путает — непечатных символов в наших строках быть не должно, а если они
 * появились, виновата строка, а не шрифт.
 */
static const uint8_t FONT5X7[95][5] = {
    { 0x00, 0x00, 0x00, 0x00, 0x00 }, /*   */
    { 0x00, 0x00, 0x5F, 0x00, 0x00 }, /* ! */
    { 0x00, 0x07, 0x00, 0x07, 0x00 }, /* " */
    { 0x14, 0x7F, 0x14, 0x7F, 0x14 }, /* # */
    { 0x24, 0x2A, 0x7F, 0x2A, 0x12 }, /* $ */
    { 0x23, 0x13, 0x08, 0x64, 0x62 }, /* % */
    { 0x36, 0x49, 0x55, 0x22, 0x50 }, /* & */
    { 0x00, 0x05, 0x03, 0x00, 0x00 }, /* ' */
    { 0x00, 0x1C, 0x22, 0x41, 0x00 }, /* ( */
    { 0x00, 0x41, 0x22, 0x1C, 0x00 }, /* ) */
    { 0x14, 0x08, 0x3E, 0x08, 0x14 }, /* * */
    { 0x08, 0x08, 0x3E, 0x08, 0x08 }, /* + */
    { 0x00, 0x50, 0x30, 0x00, 0x00 }, /* , */
    { 0x08, 0x08, 0x08, 0x08, 0x08 }, /* - */
    { 0x00, 0x60, 0x60, 0x00, 0x00 }, /* . */
    { 0x20, 0x10, 0x08, 0x04, 0x02 }, /* / */
    { 0x3E, 0x51, 0x49, 0x45, 0x3E }, /* 0 */
    { 0x00, 0x42, 0x7F, 0x40, 0x00 }, /* 1 */
    { 0x42, 0x61, 0x51, 0x49, 0x46 }, /* 2 */
    { 0x21, 0x41, 0x45, 0x4B, 0x31 }, /* 3 */
    { 0x18, 0x14, 0x12, 0x7F, 0x10 }, /* 4 */
    { 0x27, 0x45, 0x45, 0x45, 0x39 }, /* 5 */
    { 0x3C, 0x4A, 0x49, 0x49, 0x30 }, /* 6 */
    { 0x01, 0x71, 0x09, 0x05, 0x03 }, /* 7 */
    { 0x36, 0x49, 0x49, 0x49, 0x36 }, /* 8 */
    { 0x06, 0x49, 0x49, 0x29, 0x1E }, /* 9 */
    { 0x00, 0x36, 0x36, 0x00, 0x00 }, /* : */
    { 0x00, 0x56, 0x36, 0x00, 0x00 }, /* ; */
    { 0x08, 0x14, 0x22, 0x41, 0x00 }, /* < */
    { 0x14, 0x14, 0x14, 0x14, 0x14 }, /* = */
    { 0x00, 0x41, 0x22, 0x14, 0x08 }, /* > */
    { 0x02, 0x01, 0x51, 0x09, 0x06 }, /* ? */
    { 0x32, 0x49, 0x79, 0x41, 0x3E }, /* @ */
    { 0x7E, 0x11, 0x11, 0x11, 0x7E }, /* A */
    { 0x7F, 0x49, 0x49, 0x49, 0x36 }, /* B */
    { 0x3E, 0x41, 0x41, 0x41, 0x22 }, /* C */
    { 0x7F, 0x41, 0x41, 0x22, 0x1C }, /* D */
    { 0x7F, 0x49, 0x49, 0x49, 0x41 }, /* E */
    { 0x7F, 0x09, 0x09, 0x09, 0x01 }, /* F */
    { 0x3E, 0x41, 0x49, 0x49, 0x7A }, /* G */
    { 0x7F, 0x08, 0x08, 0x08, 0x7F }, /* H */
    { 0x00, 0x41, 0x7F, 0x41, 0x00 }, /* I */
    { 0x20, 0x40, 0x41, 0x3F, 0x01 }, /* J */
    { 0x7F, 0x08, 0x14, 0x22, 0x41 }, /* K */
    { 0x7F, 0x40, 0x40, 0x40, 0x40 }, /* L */
    { 0x7F, 0x02, 0x0C, 0x02, 0x7F }, /* M */
    { 0x7F, 0x04, 0x08, 0x10, 0x7F }, /* N */
    { 0x3E, 0x41, 0x41, 0x41, 0x3E }, /* O */
    { 0x7F, 0x09, 0x09, 0x09, 0x06 }, /* P */
    { 0x3E, 0x41, 0x51, 0x21, 0x5E }, /* Q */
    { 0x7F, 0x09, 0x19, 0x29, 0x46 }, /* R */
    { 0x46, 0x49, 0x49, 0x49, 0x31 }, /* S */
    { 0x01, 0x01, 0x7F, 0x01, 0x01 }, /* T */
    { 0x3F, 0x40, 0x40, 0x40, 0x3F }, /* U */
    { 0x1F, 0x20, 0x40, 0x20, 0x1F }, /* V */
    { 0x7F, 0x20, 0x18, 0x20, 0x7F }, /* W */
    { 0x63, 0x14, 0x08, 0x14, 0x63 }, /* X */
    { 0x07, 0x08, 0x70, 0x08, 0x07 }, /* Y */
    { 0x61, 0x51, 0x49, 0x45, 0x43 }, /* Z */
    { 0x00, 0x7F, 0x41, 0x41, 0x00 }, /* [ */
    { 0x02, 0x04, 0x08, 0x10, 0x20 }, /* \ */
    { 0x00, 0x41, 0x41, 0x7F, 0x00 }, /* ] */
    { 0x04, 0x02, 0x01, 0x02, 0x04 }, /* ^ */
    { 0x40, 0x40, 0x40, 0x40, 0x40 }, /* _ */
    { 0x00, 0x01, 0x02, 0x04, 0x00 }, /* ` */
    { 0x20, 0x54, 0x54, 0x54, 0x78 }, /* a */
    { 0x7F, 0x48, 0x44, 0x44, 0x38 }, /* b */
    { 0x38, 0x44, 0x44, 0x44, 0x20 }, /* c */
    { 0x38, 0x44, 0x44, 0x48, 0x7F }, /* d */
    { 0x38, 0x54, 0x54, 0x54, 0x18 }, /* e */
    { 0x08, 0x7E, 0x09, 0x01, 0x02 }, /* f */
    { 0x0C, 0x52, 0x52, 0x52, 0x3E }, /* g */
    { 0x7F, 0x08, 0x04, 0x04, 0x78 }, /* h */
    { 0x00, 0x44, 0x7D, 0x40, 0x00 }, /* i */
    { 0x20, 0x40, 0x44, 0x3D, 0x00 }, /* j */
    { 0x7F, 0x10, 0x28, 0x44, 0x00 }, /* k */
    { 0x00, 0x41, 0x7F, 0x40, 0x00 }, /* l */
    { 0x7C, 0x04, 0x18, 0x04, 0x78 }, /* m */
    { 0x7C, 0x08, 0x04, 0x04, 0x78 }, /* n */
    { 0x38, 0x44, 0x44, 0x44, 0x38 }, /* o */
    { 0x7C, 0x14, 0x14, 0x14, 0x08 }, /* p */
    { 0x08, 0x14, 0x14, 0x18, 0x7C }, /* q */
    { 0x7C, 0x08, 0x04, 0x04, 0x08 }, /* r */
    { 0x48, 0x54, 0x54, 0x54, 0x20 }, /* s */
    { 0x04, 0x3F, 0x44, 0x40, 0x20 }, /* t */
    { 0x3C, 0x40, 0x40, 0x20, 0x7C }, /* u */
    { 0x1C, 0x20, 0x40, 0x20, 0x1C }, /* v */
    { 0x3C, 0x40, 0x30, 0x40, 0x3C }, /* w */
    { 0x44, 0x28, 0x10, 0x28, 0x44 }, /* x */
    { 0x0C, 0x50, 0x50, 0x50, 0x3C }, /* y */
    { 0x44, 0x64, 0x54, 0x4C, 0x44 }, /* z */
    { 0x00, 0x08, 0x36, 0x41, 0x00 }, /* { */
    { 0x00, 0x00, 0x7F, 0x00, 0x00 }, /* | */
    { 0x00, 0x41, 0x36, 0x08, 0x00 }, /* } */
    { 0x08, 0x04, 0x08, 0x10, 0x08 }, /* ~ */
};

/* --- шина ---------------------------------------------------------------- */

static spi_device_handle_t s_spi;
static bool s_ready;
static uint8_t s_bl_step = G920_DISPLAY_BL_STEPS - 1;

/*
 * Буфер одной текстовой строки: 160 точек × 8 рядов × 2 байта.
 *
 * Статический, а не на стеке: 2560 байт на стеке главного цикла — это
 * четверть его запаса, и переполнение проявилось бы не здесь, а в соседней
 * задаче. Экран рисует только главный цикл, второго владельца нет.
 */
static uint16_t s_line[LCD_W * G920_DISPLAY_CHAR_H];

static void dc_set(int level)
{
    (void)gpio_set_level((gpio_num_t)(G920_LCD_DC), level);
}

static void write_cmd(uint8_t cmd)
{
    spi_transaction_t t = { 0 };

    t.length = 8;
    t.tx_buffer = &cmd;
    dc_set(0);
    (void)spi_device_polling_transmit(s_spi, &t);
}

static void write_data(const void *data, size_t len)
{
    spi_transaction_t t = { 0 };

    if (len == 0) {
        return;
    }
    t.length = len * 8;
    t.tx_buffer = data;
    dc_set(1);
    (void)spi_device_polling_transmit(s_spi, &t);
}

static void write_data_byte(uint8_t byte)
{
    write_data(&byte, 1);
}

/*
 * Окно вывода. Дальше контроллер сам двигает курсор по мере прихода
 * пикселей, поэтому прямоугольник уходит одной посылкой, а не по точке.
 */
static void set_window(int x, int y, int w, int h)
{
    uint8_t buf[4];
    int x0 = x + (G920_LCD_COL_OFFSET);
    int y0 = y + (G920_LCD_ROW_OFFSET);
    int x1 = x0 + w - 1;
    int y1 = y0 + h - 1;

    write_cmd(0x2A); /* CASET */
    buf[0] = (uint8_t)(x0 >> 8);
    buf[1] = (uint8_t)(x0 & 0xFF);
    buf[2] = (uint8_t)(x1 >> 8);
    buf[3] = (uint8_t)(x1 & 0xFF);
    write_data(buf, sizeof(buf));

    write_cmd(0x2B); /* RASET */
    buf[0] = (uint8_t)(y0 >> 8);
    buf[1] = (uint8_t)(y0 & 0xFF);
    buf[2] = (uint8_t)(y1 >> 8);
    buf[3] = (uint8_t)(y1 & 0xFF);
    write_data(buf, sizeof(buf));

    write_cmd(0x2C); /* RAMWR */
}

/*
 * Отправка пикселей. Цвет уходит старшим байтом вперёд, как того хочет
 * контроллер, поэтому в буфере он лежит уже переставленным — переставлять
 * на каждый пиксель в момент отправки значило бы делать это заново на
 * каждую перерисовку строки.
 */
static void push_pixels(const uint16_t *pixels, size_t count)
{
    write_data(pixels, count * sizeof(uint16_t));
}

static uint16_t swap16(uint16_t v)
{
    return (uint16_t)((v >> 8) | (v << 8));
}

/* --- наружу -------------------------------------------------------------- */

bool g920_display_present(void)
{
    return s_ready;
}

void g920_display_fill(int x, int y, int w, int h, uint16_t color)
{
    uint16_t packed = swap16(color);
    size_t chunk;

    if (!s_ready) {
        return;
    }
    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if (x >= LCD_W || y >= LCD_H || w <= 0 || h <= 0) {
        return;
    }
    if (x + w > LCD_W) {
        w = LCD_W - x;
    }
    if (y + h > LCD_H) {
        h = LCD_H - y;
    }

    /* Заливка идёт полосами по одному буферу: он и так есть, а второго
     * такого же в этой памяти не нужно. */
    chunk = (size_t)w;
    if (chunk > sizeof(s_line) / sizeof(s_line[0])) {
        chunk = sizeof(s_line) / sizeof(s_line[0]);
    }
    for (size_t i = 0; i < chunk; i++) {
        s_line[i] = packed;
    }
    set_window(x, y, w, h);
    for (int row = 0; row < h; row++) {
        int left = w;

        while (left > 0) {
            int now = (left > (int)chunk) ? (int)chunk : left;

            push_pixels(s_line, (size_t)now);
            left -= now;
        }
    }
}

void g920_display_clear(uint16_t color)
{
    g920_display_fill(0, 0, LCD_W, LCD_H, color);
}

void g920_display_text(int col, int row, const char *text, uint16_t fg,
                       uint16_t bg)
{
    g920_display_text_n(col, row, text, G920_DISPLAY_COLS - col, fg, bg);
}

void g920_display_text_n(int col, int row, const char *text, int cells,
                         uint16_t fg, uint16_t bg)
{
    uint16_t packed_fg = swap16(fg);
    uint16_t packed_bg = swap16(bg);
    int width;

    if (!s_ready || text == NULL) {
        return;
    }
    if (col < 0 || row < 0 || col >= G920_DISPLAY_COLS
        || row >= G920_DISPLAY_ROWS) {
        return;
    }
    if (cells > G920_DISPLAY_COLS - col) {
        cells = G920_DISPLAY_COLS - col;
    }
    if (cells <= 0) {
        return;
    }
    width = cells * G920_DISPLAY_CHAR_W;

    /*
     * Строка собирается целиком в памяти и уходит **одной** посылкой.
     *
     * По знакоместу это было бы 26 отдельных транзакций с переключением
     * DC и CS на каждой; на экране статуса, который перерисовывается
     * несколько раз в секунду, разница видна не в скорости, а в том, что
     * длинная череда мелких транзакций держит шину и главный цикл дольше,
     * чем одна крупная.
     */
    for (int cell = 0; cell < cells; cell++) {
        char ch = text[0];
        const uint8_t *glyph = NULL;
        int x0 = cell * G920_DISPLAY_CHAR_W;

        if (ch != '\0') {
            if ((unsigned char)ch >= 0x20u && (unsigned char)ch <= 0x7Eu) {
                glyph = FONT5X7[(unsigned char)ch - 0x20u];
            }
            text++;
        }
        for (int gx = 0; gx < G920_DISPLAY_CHAR_W; gx++) {
            uint8_t bits = (glyph != NULL && gx < 5) ? glyph[gx] : 0u;

            for (int gy = 0; gy < G920_DISPLAY_CHAR_H; gy++) {
                bool on = (gy < 7) && ((bits >> gy) & 1u) != 0u;

                s_line[(size_t)gy * (size_t)width + (size_t)(x0 + gx)] =
                    on ? packed_fg : packed_bg;
            }
        }
    }

    set_window(col * G920_DISPLAY_CHAR_W, row * G920_DISPLAY_CHAR_H, width,
               G920_DISPLAY_CHAR_H);
    push_pixels(s_line, (size_t)width * G920_DISPLAY_CHAR_H);
}

void g920_display_bar(int x, int y, int w, int h, int value, int max,
                      uint16_t fg, uint16_t bg)
{
    int inner;
    int filled;

    if (!s_ready || w < 2 || h < 2) {
        return;
    }
    if (max <= 0) {
        max = 1;
    }
    if (value < 0) {
        value = 0;
    }
    if (value > max) {
        value = max;
    }
    /* Рамка — четыре полоски: сплошной прямоугольник поверх заливки стоил
     * бы лишней отправки всей площади. */
    g920_display_fill(x, y, w, 1, fg);
    g920_display_fill(x, y + h - 1, w, 1, fg);
    g920_display_fill(x, y, 1, h, fg);
    g920_display_fill(x + w - 1, y, 1, h, fg);

    inner = w - 2;
    filled = (inner * value) / max;
    if (filled > 0) {
        g920_display_fill(x + 1, y + 1, filled, h - 2, fg);
    }
    if (filled < inner) {
        g920_display_fill(x + 1 + filled, y + 1, inner - filled, h - 2, bg);
    }
}

/* --- подсветка ----------------------------------------------------------- */

#ifdef G920_LCD_BL

#ifndef G920_LCD_BL_ACTIVE_LOW
/*
 * У T-Dongle-S3 подсветка зажигается **низким** уровнем (`LCD_BK_LIGHT_ON
 * = 0` в примерах LilyGo, см. docs/HARDWARE.md). Плата другой ревизии —
 * перевернуть флагом, это одна строка.
 */
#define G920_LCD_BL_ACTIVE_LOW 1
#endif

#define BL_TIMER LEDC_TIMER_1
#define BL_CHANNEL LEDC_CHANNEL_1
#define BL_RES LEDC_TIMER_10_BIT
#define BL_MAX 1023

/*
 * Ступени яркости — не линейные.
 *
 * Глаз воспринимает яркость примерно логарифмически: линейные 25/50/75%
 * на глаз выглядят как «почти одинаково ярко» три раза подряд. Здесь
 * ступени разведены так, чтобы каждая читалась как заметно другая: ночная,
 * комнатная, полная.
 */
static const uint16_t BL_DUTY[G920_DISPLAY_BL_STEPS] = { 0, 60, 300, BL_MAX };

static void backlight_apply(uint8_t step)
{
    uint32_t duty;

    if (step >= G920_DISPLAY_BL_STEPS) {
        step = G920_DISPLAY_BL_STEPS - 1;
    }
    duty = BL_DUTY[step];
#if G920_LCD_BL_ACTIVE_LOW
    duty = BL_MAX - duty;
#endif
    (void)ledc_set_duty(LEDC_LOW_SPEED_MODE, BL_CHANNEL, duty);
    (void)ledc_update_duty(LEDC_LOW_SPEED_MODE, BL_CHANNEL);
}

static bool backlight_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = BL_RES,
        .timer_num = BL_TIMER,
        /* 2 кГц: выше слышимого писка дросселей и ниже частот, на которых
         * начинает мерцать при съёмке. */
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_channel_config_t channel = {
        .gpio_num = G920_LCD_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = BL_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BL_TIMER,
        .duty = 0,
        .hpoint = 0,
    };

    if (ledc_timer_config(&timer) != ESP_OK
        || ledc_channel_config(&channel) != ESP_OK) {
        G920_LOGE(TAG, "backlight ledc failed");
        return false;
    }
    backlight_apply(s_bl_step);
    return true;
}

void g920_display_backlight_set(uint8_t step)
{
    if (step >= G920_DISPLAY_BL_STEPS) {
        step = G920_DISPLAY_BL_STEPS - 1;
    }
    s_bl_step = step;
    backlight_apply(step);
}

#else /* подсветкой не управляем */

static bool backlight_init(void)
{
    return true;
}

void g920_display_backlight_set(uint8_t step)
{
    s_bl_step = (step < G920_DISPLAY_BL_STEPS) ? step
                                               : G920_DISPLAY_BL_STEPS - 1;
}

#endif /* G920_LCD_BL */

uint8_t g920_display_backlight_step(void)
{
    return s_bl_step;
}

uint8_t g920_display_backlight_next(void)
{
    g920_display_backlight_set(
        (uint8_t)((s_bl_step + 1u) % G920_DISPLAY_BL_STEPS));
    return s_bl_step;
}

/* --- подъём -------------------------------------------------------------- */

bool g920_display_init(void)
{
    spi_bus_config_t bus = {
        .mosi_io_num = G920_LCD_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = G920_LCD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        /* Самая крупная посылка — текстовая строка целиком. */
        .max_transfer_sz = (int)sizeof(s_line),
    };
    spi_device_interface_config_t dev = {
        .clock_speed_hz = G920_LCD_HZ,
        .mode = 0,
        .spics_io_num = G920_LCD_CS,
        .queue_size = 1,
    };
    gpio_config_t pins = {
        .pin_bit_mask = (1ULL << (G920_LCD_DC)) | (1ULL << (G920_LCD_RST)),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    if (gpio_config(&pins) != ESP_OK) {
        G920_LOGE(TAG, "gpio config failed");
        return false;
    }
    if (spi_bus_initialize(G920_LCD_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) {
        G920_LOGE(TAG, "spi bus failed");
        return false;
    }
    if (spi_bus_add_device(G920_LCD_HOST, &dev, &s_spi) != ESP_OK) {
        G920_LOGE(TAG, "spi device failed");
        return false;
    }

    /* Аппаратный сброс: 10 мс низкого и 120 мс на подъём контроллера —
     * времена из даташита ST7735S, экономить на них нечего. */
    (void)gpio_set_level((gpio_num_t)(G920_LCD_RST), 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    (void)gpio_set_level((gpio_num_t)(G920_LCD_RST), 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    (void)gpio_set_level((gpio_num_t)(G920_LCD_RST), 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    s_ready = true; /* дальше идут команды, а они уже требуют готовой шины */

    write_cmd(0x01); /* SWRESET */
    vTaskDelay(pdMS_TO_TICKS(150));
    write_cmd(0x11); /* SLPOUT */
    vTaskDelay(pdMS_TO_TICKS(120));

    write_cmd(0x3A); /* COLMOD */
    write_data_byte(0x05); /* 16 бит на точку */

    write_cmd(0x36); /* MADCTL */
    write_data_byte(G920_LCD_MADCTL);

#if G920_LCD_INVERT
    write_cmd(0x21); /* INVON */
#else
    write_cmd(0x20); /* INVOFF */
#endif

    write_cmd(0x13); /* NORON */
    vTaskDelay(pdMS_TO_TICKS(10));

    /*
     * Экран чистится **до** включения вывода: иначе первые кадры покажут
     * то, что осталось в памяти контроллера с прошлого включения, а это
     * выглядит как сбой прошивки.
     */
    g920_display_clear(G920_COLOR_BLACK);

    write_cmd(0x29); /* DISPON */
    vTaskDelay(pdMS_TO_TICKS(10));

    if (!backlight_init()) {
        /* Экран жив, но не светится: это не повод объявлять его
         * отсутствующим — на просвет содержимое всё же видно, а лог скажет
         * причину. */
        G920_LOGW(TAG, "display up, backlight is not");
    }
    G920_LOGI(TAG, "st7735 %dx%d up on spi, madctl %02x, offsets %d/%d",
              LCD_W, LCD_H, (unsigned)(G920_LCD_MADCTL),
              (int)(G920_LCD_COL_OFFSET), (int)(G920_LCD_ROW_OFFSET));
    return true;
}

#else /* экрана в сборке нет */

bool g920_display_init(void)
{
    return false;
}

bool g920_display_present(void)
{
    return false;
}

void g920_display_clear(uint16_t color)
{
    (void)color;
}

void g920_display_fill(int x, int y, int w, int h, uint16_t color)
{
    (void)x;
    (void)y;
    (void)w;
    (void)h;
    (void)color;
}

void g920_display_text(int col, int row, const char *text, uint16_t fg,
                       uint16_t bg)
{
    (void)col;
    (void)row;
    (void)text;
    (void)fg;
    (void)bg;
}

void g920_display_text_n(int col, int row, const char *text, int cells,
                         uint16_t fg, uint16_t bg)
{
    (void)col;
    (void)row;
    (void)text;
    (void)cells;
    (void)fg;
    (void)bg;
}

void g920_display_bar(int x, int y, int w, int h, int value, int max,
                      uint16_t fg, uint16_t bg)
{
    (void)x;
    (void)y;
    (void)w;
    (void)h;
    (void)value;
    (void)max;
    (void)fg;
    (void)bg;
}

void g920_display_backlight_set(uint8_t step)
{
    (void)step;
}

uint8_t g920_display_backlight_step(void)
{
    return 0;
}

uint8_t g920_display_backlight_next(void)
{
    return 0;
}

#endif
