#include "g920/board.h"

#include "g920/log.h"

#define TAG "led"

/* Яркость индикатора. Светодиод в полусантиметре от глаз человека, который
 * сидит над платой, а не на сцене — 12% хватает с запасом. */
#define BRIGHTNESS 32

#if defined(G920_LED_WS2812)

#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* 10 МГц: один тик — 0.1 мкс, в этих единицах и заданы длительности бит. */
#define RMT_RESOLUTION_HZ 10000000

static rmt_channel_handle_t s_chan;
static rmt_encoder_handle_t s_encoder;

bool g920_board_led_init(void)
{
    rmt_tx_channel_config_t chan_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = G920_LED_WS2812,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };
    /* Тайминги WS2812B: T0H 0.4 / T0L 0.85, T1H 0.8 / T1L 0.45 мкс. */
    rmt_bytes_encoder_config_t enc_cfg = {
        .bit0 = { .level0 = 1, .duration0 = 4, .level1 = 0, .duration1 = 8 },
        .bit1 = { .level0 = 1, .duration0 = 8, .level1 = 0, .duration1 = 5 },
        .flags = { .msb_first = 1 },
    };

    if (rmt_new_tx_channel(&chan_cfg, &s_chan) != ESP_OK) {
        G920_LOGE(TAG, "rmt channel failed");
        return false;
    }
    if (rmt_new_bytes_encoder(&enc_cfg, &s_encoder) != ESP_OK) {
        G920_LOGE(TAG, "rmt encoder failed");
        return false;
    }
    if (rmt_enable(s_chan) != ESP_OK) {
        G920_LOGE(TAG, "rmt enable failed");
        return false;
    }
    return true;
}

void g920_board_led_set(g920_indication_t state)
{
    rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
    uint8_t grb[3];

    if (s_chan == NULL || s_encoder == NULL) {
        return;
    }
    if (g920_rgb_pack_grb(grb, sizeof(grb),
                          g920_rgb_scale(g920_rgb_state_for_role(
                                             state, g920_build_role()),
                                         BRIGHTNESS))
        == 0) {
        return;
    }
    if (rmt_transmit(s_chan, s_encoder, grb, sizeof(grb), &tx_cfg) != ESP_OK) {
        return;
    }
    /* Ждём конца посылки: следующая пауза длиннее 50 мкс и служит сбросом
     * для светодиода. Посылка занимает ~30 мкс, ждать не жалко. */
    (void)rmt_tx_wait_all_done(s_chan, pdMS_TO_TICKS(100));
}

const char *g920_board_led_kind(void)
{
    return "ws2812";
}

#elif defined(G920_LED_APA102_DIN) && defined(G920_LED_APA102_CLK)

#include "driver/gpio.h"

/* Поле яркости самого APA102 — 5 бит. */
#define APA102_BRIGHTNESS 4

static void write_byte(uint8_t byte)
{
    for (int bit = 7; bit >= 0; bit--) {
        gpio_set_level(G920_LED_APA102_DIN, (byte >> bit) & 1);
        gpio_set_level(G920_LED_APA102_CLK, 1);
        gpio_set_level(G920_LED_APA102_CLK, 0);
    }
}

bool g920_board_led_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << G920_LED_APA102_DIN)
                        | (1ULL << G920_LED_APA102_CLK),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    if (gpio_config(&cfg) != ESP_OK) {
        G920_LOGE(TAG, "gpio config failed");
        return false;
    }
    gpio_set_level(G920_LED_APA102_CLK, 0);
    return true;
}

void g920_board_led_set(g920_indication_t state)
{
    uint8_t pixel[4];

    if (g920_rgb_pack_apa102(pixel, sizeof(pixel),
                             g920_rgb_state_for_role(state,
                                                     g920_build_role()),
                             APA102_BRIGHTNESS)
        == 0) {
        return;
    }
    for (int i = 0; i < 4; i++) { /* start frame */
        write_byte(0x00);
    }
    for (int i = 0; i < 4; i++) {
        write_byte(pixel[i]);
    }
    for (int i = 0; i < 4; i++) { /* end frame: один светодиод в цепочке */
        write_byte(0xFF);
    }
}

const char *g920_board_led_kind(void)
{
    return "apa102";
}

#elif defined(G920_LED_GPIO)

#include "driver/gpio.h"

bool g920_board_led_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << G920_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    if (gpio_config(&cfg) != ESP_OK) {
        G920_LOGE(TAG, "gpio config failed");
        return false;
    }
    return true;
}

void g920_board_led_set(g920_indication_t state)
{
    /* Одноцветный светодиод различает только «горит» и «нет». Полная
     * кодировка состояний морганием — M10. */
    g920_rgb_t c = g920_rgb_state_for_role(state, g920_build_role());

    gpio_set_level(G920_LED_GPIO, (c.r || c.g || c.b) ? 1 : 0);
}

const char *g920_board_led_kind(void)
{
    return "gpio";
}

#else

bool g920_board_led_init(void)
{
    G920_LOGW(TAG, "no indicator configured");
    return false;
}

void g920_board_led_set(g920_indication_t state)
{
    (void)state;
}

const char *g920_board_led_kind(void)
{
    return "none";
}

#endif
