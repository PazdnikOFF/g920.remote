#include <stdint.h>
#include <string.h>

#include <unity.h>

#include "g920/rgb.h"

void setUp(void) { }
void tearDown(void) { }

static void test_states_match_hardware_scheme(void)
{
    /* Схема из HARDWARE.md: зелёный работает, жёлтый определение,
     * красный авария. */
    g920_rgb_t off = g920_rgb_state(G920_IND_OFF);
    g920_rgb_t detect = g920_rgb_state(G920_IND_DETECT);
    g920_rgb_t ok = g920_rgb_state(G920_IND_OK);
    g920_rgb_t fault = g920_rgb_state(G920_IND_FAULT);

    TEST_ASSERT_EQUAL_UINT8(0, off.r);
    TEST_ASSERT_EQUAL_UINT8(0, off.g);
    TEST_ASSERT_EQUAL_UINT8(0, off.b);

    /* Жёлтый — красный плюс зелёный, синего нет. */
    TEST_ASSERT_TRUE(detect.r > 0 && detect.g > 0);
    TEST_ASSERT_EQUAL_UINT8(0, detect.b);

    TEST_ASSERT_EQUAL_UINT8(0, ok.r);
    TEST_ASSERT_TRUE(ok.g > 0);

    TEST_ASSERT_TRUE(fault.r > 0);
    TEST_ASSERT_EQUAL_UINT8(0, fault.g);
}

static void test_unknown_state_is_dark_not_garbage(void)
{
    g920_rgb_t c = g920_rgb_state((g920_indication_t)99);

    TEST_ASSERT_EQUAL_UINT8(0, c.r);
    TEST_ASSERT_EQUAL_UINT8(0, c.g);
    TEST_ASSERT_EQUAL_UINT8(0, c.b);
}

static void test_scale_bounds(void)
{
    g920_rgb_t c = { 255, 128, 1 };
    g920_rgb_t full = g920_rgb_scale(c, 255);
    g920_rgb_t dark = g920_rgb_scale(c, 0);

    TEST_ASSERT_EQUAL_UINT8(255, full.r);
    TEST_ASSERT_EQUAL_UINT8(128, full.g);
    TEST_ASSERT_EQUAL_UINT8(1, full.b);

    TEST_ASSERT_EQUAL_UINT8(0, dark.r);
    TEST_ASSERT_EQUAL_UINT8(0, dark.g);
    TEST_ASSERT_EQUAL_UINT8(0, dark.b);
}

static void test_scale_rounds_to_nearest(void)
{
    g920_rgb_t half = g920_rgb_scale((g920_rgb_t){ 255, 255, 255 }, 128);

    /* 255*128/255 = 128 ровно. */
    TEST_ASSERT_EQUAL_UINT8(128, half.r);

    /* 1*128 = 128, /255 с округлением к ближайшему даёт 1, а не 0:
     * отбрасывание гасило бы тусклые каналы целиком. */
    TEST_ASSERT_EQUAL_UINT8(1, g920_rgb_scale((g920_rgb_t){ 1, 0, 0 }, 128).r);
    TEST_ASSERT_EQUAL_UINT8(0, g920_rgb_scale((g920_rgb_t){ 1, 0, 0 }, 100).r);
}

static void test_scale_never_overflows(void)
{
    for (unsigned v = 0; v <= 255; v++) {
        for (unsigned b = 0; b <= 255; b += 17) {
            g920_rgb_t out =
                g920_rgb_scale((g920_rgb_t){ (uint8_t)v, 0, 0 }, (uint8_t)b);
            TEST_ASSERT_TRUE(out.r <= v);
        }
    }
}

static void test_pack_grb_order(void)
{
    uint8_t buf[3] = { 0, 0, 0 };

    /* WS2812 принимает зелёный первым — самая частая ошибка в таких
     * драйверах, поэтому проверяется отдельно. */
    TEST_ASSERT_EQUAL_size_t(
        3, g920_rgb_pack_grb(buf, sizeof(buf), (g920_rgb_t){ 1, 2, 3 }));
    TEST_ASSERT_EQUAL_UINT8(2, buf[0]);
    TEST_ASSERT_EQUAL_UINT8(1, buf[1]);
    TEST_ASSERT_EQUAL_UINT8(3, buf[2]);

    TEST_ASSERT_EQUAL_size_t(
        0, g920_rgb_pack_grb(buf, 2, (g920_rgb_t){ 1, 2, 3 }));
    TEST_ASSERT_EQUAL_size_t(0,
                             g920_rgb_pack_grb(NULL, 8, (g920_rgb_t){ 1, 2, 3 }));
}

static void test_pack_apa102_frame(void)
{
    uint8_t buf[4];

    /* Порядок BGR и поле яркости 0xE0|b — см. HARDWARE.md. */
    TEST_ASSERT_EQUAL_size_t(
        4, g920_rgb_pack_apa102(buf, sizeof(buf), (g920_rgb_t){ 1, 2, 3 }, 4));
    TEST_ASSERT_EQUAL_HEX8(0xE4, buf[0]);
    TEST_ASSERT_EQUAL_UINT8(3, buf[1]);
    TEST_ASSERT_EQUAL_UINT8(2, buf[2]);
    TEST_ASSERT_EQUAL_UINT8(1, buf[3]);
}

static void test_pack_apa102_clamps_brightness(void)
{
    uint8_t buf[4];

    /* Поле яркости пятибитное: 255 не должно затереть старшие биты 0xE0 и
     * превратить кадр в конец цепочки. */
    TEST_ASSERT_EQUAL_size_t(
        4, g920_rgb_pack_apa102(buf, sizeof(buf), (g920_rgb_t){ 0, 0, 0 },
                                255));
    TEST_ASSERT_EQUAL_HEX8(0xFF, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xE0, buf[0] & 0xE0);

    TEST_ASSERT_EQUAL_size_t(
        4, g920_rgb_pack_apa102(buf, sizeof(buf), (g920_rgb_t){ 0, 0, 0 }, 31));
    TEST_ASSERT_EQUAL_HEX8(0xFF, buf[0]);

    TEST_ASSERT_EQUAL_size_t(
        0, g920_rgb_pack_apa102(buf, 3, (g920_rgb_t){ 0, 0, 0 }, 4));
    TEST_ASSERT_EQUAL_size_t(
        0, g920_rgb_pack_apa102(NULL, 8, (g920_rgb_t){ 0, 0, 0 }, 4));
}


/*
 * Роль красит только рабочее состояние. Проверяется именно это: комплект
 * из двух одинаковых плат различим на столе лишь по цвету, но авария
 * обязана читаться одинаково на любой — иначе теряется единственное
 * состояние, которое нельзя перепутать.
 */
static void test_role_colours_only_the_working_state(void)
{
    g920_rgb_t tx_ok = g920_rgb_state_for_role(G920_IND_OK, G920_ROLE_TX);
    g920_rgb_t rx_ok = g920_rgb_state_for_role(G920_IND_OK, G920_ROLE_RX);
    g920_rgb_t tx_fault = g920_rgb_state_for_role(G920_IND_FAULT, G920_ROLE_TX);
    g920_rgb_t rx_fault = g920_rgb_state_for_role(G920_IND_FAULT, G920_ROLE_RX);
    g920_rgb_t tx_boot = g920_rgb_state_for_role(G920_IND_BOOT, G920_ROLE_TX);
    g920_rgb_t rx_boot = g920_rgb_state_for_role(G920_IND_BOOT, G920_ROLE_RX);

    /* TX работает зелёным, RX — голубым. */
    TEST_ASSERT_EQUAL_UINT8(0, tx_ok.r);
    TEST_ASSERT_EQUAL_UINT8(255, tx_ok.g);
    TEST_ASSERT_EQUAL_UINT8(0, tx_ok.b);
    TEST_ASSERT_EQUAL_UINT8(0, rx_ok.r);
    TEST_ASSERT_TRUE(rx_ok.b > rx_ok.g);
    TEST_ASSERT_TRUE(rx_ok.g > 0);

    /* И это разные цвета, а не оттенок одного. */
    TEST_ASSERT_TRUE(rx_ok.b != tx_ok.b);

    /* Голубой не должен совпасть с синим «поднимаюсь»: их показывает одна
     * и та же плата, и спутать их значит не понять, работает она или нет. */
    TEST_ASSERT_TRUE(rx_ok.g != rx_boot.g);

    /* Авария и загрузка роли не знают. */
    TEST_ASSERT_EQUAL_UINT8(tx_fault.r, rx_fault.r);
    TEST_ASSERT_EQUAL_UINT8(tx_fault.g, rx_fault.g);
    TEST_ASSERT_EQUAL_UINT8(tx_fault.b, rx_fault.b);
    TEST_ASSERT_EQUAL_UINT8(tx_boot.b, rx_boot.b);

    /* Неизвестная роль ведёт себя как безролевой цвет. */
    {
        g920_rgb_t unknown =
            g920_rgb_state_for_role(G920_IND_OK, G920_ROLE_UNKNOWN);
        g920_rgb_t plain = g920_rgb_state(G920_IND_OK);

        TEST_ASSERT_EQUAL_UINT8(plain.g, unknown.g);
        TEST_ASSERT_EQUAL_UINT8(plain.b, unknown.b);
    }
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_role_colours_only_the_working_state);
    RUN_TEST(test_states_match_hardware_scheme);
    RUN_TEST(test_unknown_state_is_dark_not_garbage);
    RUN_TEST(test_scale_bounds);
    RUN_TEST(test_scale_rounds_to_nearest);
    RUN_TEST(test_scale_never_overflows);
    RUN_TEST(test_pack_grb_order);
    RUN_TEST(test_pack_apa102_frame);
    RUN_TEST(test_pack_apa102_clamps_brightness);
    return UNITY_END();
}
