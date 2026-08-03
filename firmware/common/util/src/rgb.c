#include "g920/rgb.h"

#define APA102_BRIGHTNESS_MAX 31

g920_rgb_t g920_rgb_state(g920_indication_t state)
{
    g920_rgb_t c = { 0, 0, 0 };

    switch (state) {
    case G920_IND_BOOT:
        c.b = 255;
        break;
    case G920_IND_DETECT:
        c.r = 255;
        c.g = 160;
        break;
    case G920_IND_OK:
        c.g = 255;
        break;
    case G920_IND_FAULT:
        c.r = 255;
        break;
    case G920_IND_OFF:
    default:
        break;
    }
    return c;
}

g920_rgb_t g920_rgb_state_for_role(g920_indication_t state, g920_role_t role)
{
    g920_rgb_t c = g920_rgb_state(state);

    /*
     * Роль красит только рабочее состояние. Голубой, а не синий: синим уже
     * занят G920_IND_BOOT, и различать «поднимаюсь» и «работаю» по оттенку
     * одного цвета никто на глаз не станет.
     */
    if (state == G920_IND_OK && role == G920_ROLE_RX) {
        c.r = 0;
        c.g = 160;
        c.b = 255;
    }
    return c;
}

static uint8_t scale_one(uint8_t value, uint8_t brightness)
{
    return (uint8_t)(((uint32_t)value * brightness + 127u) / 255u);
}

g920_rgb_t g920_rgb_scale(g920_rgb_t color, uint8_t brightness)
{
    g920_rgb_t out = { scale_one(color.r, brightness),
                       scale_one(color.g, brightness),
                       scale_one(color.b, brightness) };
    return out;
}

size_t g920_rgb_pack_grb(uint8_t *out, size_t size, g920_rgb_t color)
{
    if (out == NULL || size < 3) {
        return 0;
    }
    out[0] = color.g;
    out[1] = color.r;
    out[2] = color.b;
    return 3;
}

size_t g920_rgb_pack_apa102(uint8_t *out, size_t size, g920_rgb_t color,
                            uint8_t brightness5)
{
    if (out == NULL || size < 4) {
        return 0;
    }
    if (brightness5 > APA102_BRIGHTNESS_MAX) {
        brightness5 = APA102_BRIGHTNESS_MAX;
    }
    out[0] = (uint8_t)(0xE0u | brightness5);
    out[1] = color.b;
    out[2] = color.g;
    out[3] = color.r;
    return 4;
}
