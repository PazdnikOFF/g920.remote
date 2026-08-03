#include <string.h>

#include <unity.h>

#include "g920/version.h"

static g920_version_t mk(uint8_t major, uint8_t minor, uint8_t patch)
{
    g920_version_t v = { major, minor, patch };
    return v;
}

void setUp(void) { }
void tearDown(void) { }

/* --- роли ------------------------------------------------------------- */

static void test_build_role_is_host_test(void)
{
    /* Хостовый тест собирается с -DG920_ROLE=G920_ROLE_HOST_TEST — заодно
     * проверяем, что общая библиотека действительно берётся из
     * firmware/common, а не из копии внутри проекта. */
    TEST_ASSERT_EQUAL_INT(G920_ROLE_HOST_TEST, g920_build_role());
}

static void test_role_names(void)
{
    TEST_ASSERT_EQUAL_STRING("TX", g920_role_name(G920_ROLE_TX));
    TEST_ASSERT_EQUAL_STRING("RX", g920_role_name(G920_ROLE_RX));
    TEST_ASSERT_EQUAL_STRING("TEST", g920_role_name(G920_ROLE_HOST_TEST));
    TEST_ASSERT_EQUAL_STRING("??", g920_role_name(G920_ROLE_UNKNOWN));
    TEST_ASSERT_EQUAL_STRING("??", g920_role_name((g920_role_t)200));
}

/* --- упаковка --------------------------------------------------------- */

static void test_pack_layout(void)
{
    TEST_ASSERT_EQUAL_UINT32(0x010203u, g920_version_pack(mk(1, 2, 3)));
    TEST_ASSERT_EQUAL_UINT32(0x000000u, g920_version_pack(mk(0, 0, 0)));
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFu, g920_version_pack(mk(255, 255, 255)));
}

static void test_pack_unpack_roundtrip(void)
{
    const g920_version_t cases[] = { { 0, 0, 0 },   { 1, 2, 3 },
                                     { 255, 0, 1 }, { 0, 255, 0 },
                                     { 7, 13, 200 }, { 255, 255, 255 } };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        g920_version_t back = g920_version_unpack(g920_version_pack(cases[i]));
        TEST_ASSERT_EQUAL_INT(0, g920_version_compare(cases[i], back));
    }
}

static void test_unpack_ignores_high_byte(void)
{
    /* Старший байт зарезервирован под флаги кадра линка — он не должен
     * протекать в major. */
    g920_version_t v = g920_version_unpack(0xAB010203u);
    TEST_ASSERT_EQUAL_UINT8(1, v.major);
    TEST_ASSERT_EQUAL_UINT8(2, v.minor);
    TEST_ASSERT_EQUAL_UINT8(3, v.patch);
}

/* --- сравнение -------------------------------------------------------- */

static void test_compare_orders_by_significance(void)
{
    TEST_ASSERT_EQUAL_INT(0, g920_version_compare(mk(1, 2, 3), mk(1, 2, 3)));
    TEST_ASSERT_EQUAL_INT(-1, g920_version_compare(mk(0, 9, 9), mk(1, 0, 0)));
    TEST_ASSERT_EQUAL_INT(1, g920_version_compare(mk(1, 0, 0), mk(0, 9, 9)));
    TEST_ASSERT_EQUAL_INT(-1, g920_version_compare(mk(1, 2, 9), mk(1, 3, 0)));
    TEST_ASSERT_EQUAL_INT(1, g920_version_compare(mk(1, 3, 0), mk(1, 2, 9)));
    TEST_ASSERT_EQUAL_INT(-1, g920_version_compare(mk(1, 2, 3), mk(1, 2, 4)));
    TEST_ASSERT_EQUAL_INT(1, g920_version_compare(mk(1, 2, 4), mk(1, 2, 3)));
}

/* --- совместимость протокола ------------------------------------------ */

static void test_proto_zero_is_incompatible_with_everything(void)
{
    /* 0 означает «протокола нет» и не совместим даже сам с собой: иначе
     * две прошивки без линка решат, что договорились. */
    TEST_ASSERT_FALSE(g920_link_proto_compatible(0u, 0u));
    TEST_ASSERT_FALSE(g920_link_proto_compatible(0u, 1u));
    TEST_ASSERT_FALSE(g920_link_proto_compatible(1u, 0u));
}

static void test_proto_requires_exact_match(void)
{
    TEST_ASSERT_TRUE(g920_link_proto_compatible(1u, 1u));
    TEST_ASSERT_TRUE(g920_link_proto_compatible(42u, 42u));
    TEST_ASSERT_FALSE(g920_link_proto_compatible(1u, 2u));
    TEST_ASSERT_FALSE(g920_link_proto_compatible(2u, 1u));
}

static void test_current_proto_version_is_set(void)
{
    /* Версия протокола линка. Ноль означал бы, что common/proto есть, а
     * сговориться по нему стороны не могут. Поднимается вместе с любым
     * изменением раскладки кадра или состава типов. */
    TEST_ASSERT_TRUE((uint32_t)G920_LINK_PROTO_VERSION >= 1u);
}

/* --- форматирование --------------------------------------------------- */

static void test_format_basic(void)
{
    char buf[G920_VERSION_STR_MAX];

    TEST_ASSERT_EQUAL_INT(5, g920_version_format(buf, sizeof(buf), mk(1, 2, 3)));
    TEST_ASSERT_EQUAL_STRING("1.2.3", buf);

    TEST_ASSERT_EQUAL_INT(5, g920_version_format(buf, sizeof(buf), mk(0, 0, 0)));
    TEST_ASSERT_EQUAL_STRING("0.0.0", buf);

    TEST_ASSERT_EQUAL_INT(
        11, g920_version_format(buf, sizeof(buf), mk(255, 255, 255)));
    TEST_ASSERT_EQUAL_STRING("255.255.255", buf);

    TEST_ASSERT_EQUAL_INT(
        8, g920_version_format(buf, sizeof(buf), mk(10, 200, 3)));
    TEST_ASSERT_EQUAL_STRING("10.200.3", buf);
}

static void test_format_max_len_fits_declared_buffer(void)
{
    /* G920_VERSION_STR_MAX обязан вмещать худший случай целиком. */
    char buf[G920_VERSION_STR_MAX];
    int len = g920_version_format(buf, sizeof(buf), mk(255, 255, 255));

    TEST_ASSERT_TRUE(len > 0);
    TEST_ASSERT_EQUAL_size_t((size_t)len, strlen(buf));
    TEST_ASSERT_TRUE((size_t)len + 1u <= (size_t)G920_VERSION_STR_MAX);
}

static void test_format_rejects_small_buffer_without_touching_it(void)
{
    char buf[16];

    memset(buf, 'x', sizeof(buf));
    /* "1.2.3" нужно 6 байт, даём 5 */
    TEST_ASSERT_EQUAL_INT(-1, g920_version_format(buf, 5, mk(1, 2, 3)));
    for (size_t i = 0; i < sizeof(buf); i++) {
        TEST_ASSERT_EQUAL_CHAR('x', buf[i]);
    }

    TEST_ASSERT_EQUAL_INT(-1, g920_version_format(buf, 0, mk(1, 2, 3)));
    TEST_ASSERT_EQUAL_CHAR('x', buf[0]);
}

static void test_format_rejects_null(void)
{
    TEST_ASSERT_EQUAL_INT(-1, g920_version_format(NULL, 32, mk(1, 2, 3)));
}

static void test_format_exact_fit(void)
{
    char buf[6];

    TEST_ASSERT_EQUAL_INT(5, g920_version_format(buf, sizeof(buf), mk(1, 2, 3)));
    TEST_ASSERT_EQUAL_STRING("1.2.3", buf);
}

/* --- версия прошивки -------------------------------------------------- */

static void test_firmware_version_matches_macros(void)
{
    g920_version_t v = g920_firmware_version();

    TEST_ASSERT_EQUAL_UINT8(G920_FW_VERSION_MAJOR, v.major);
    TEST_ASSERT_EQUAL_UINT8(G920_FW_VERSION_MINOR, v.minor);
    TEST_ASSERT_EQUAL_UINT8(G920_FW_VERSION_PATCH, v.patch);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_build_role_is_host_test);
    RUN_TEST(test_role_names);
    RUN_TEST(test_pack_layout);
    RUN_TEST(test_pack_unpack_roundtrip);
    RUN_TEST(test_unpack_ignores_high_byte);
    RUN_TEST(test_compare_orders_by_significance);
    RUN_TEST(test_proto_zero_is_incompatible_with_everything);
    RUN_TEST(test_proto_requires_exact_match);
    RUN_TEST(test_current_proto_version_is_set);
    RUN_TEST(test_format_basic);
    RUN_TEST(test_format_max_len_fits_declared_buffer);
    RUN_TEST(test_format_rejects_small_buffer_without_touching_it);
    RUN_TEST(test_format_rejects_null);
    RUN_TEST(test_format_exact_fit);
    RUN_TEST(test_firmware_version_matches_macros);
    return UNITY_END();
}
