#include <stdint.h>
#include <string.h>

#include <unity.h>

#include "g920/hexdump.h"
#include "g920/log.h"
#include "g920/timestamp.h"

/* --- перехват вывода лога ---------------------------------------------- */

#define CAP_MAX 8192

static char cap_buf[CAP_MAX];
static size_t cap_len;
static int cap_lines;
static int cap_overflow;

static void cap_sink(void *ctx, const char *text, size_t len)
{
    (void)ctx;
    if (cap_len + len + 1u < CAP_MAX) {
        memcpy(cap_buf + cap_len, text, len);
        cap_len += len;
        cap_buf[cap_len] = '\0';
    } else {
        cap_overflow = 1;
    }
    cap_lines++;
}

static void cap_reset(void)
{
    cap_len = 0;
    cap_lines = 0;
    cap_overflow = 0;
    cap_buf[0] = '\0';
}

void setUp(void)
{
    cap_reset();
    g920_log_set_sink(cap_sink, NULL);
    g920_log_set_level(G920_LOG_DEBUG);
}

void tearDown(void)
{
    g920_log_set_sink(NULL, NULL);
}

/* --- метки времени ------------------------------------------------------ */

static void test_timestamp_format_basic(void)
{
    char buf[G920_TIMESTAMP_STR_MAX];

    TEST_ASSERT_EQUAL_INT(8, g920_timestamp_format(buf, sizeof(buf), 0));
    TEST_ASSERT_EQUAL_STRING("0.000000", buf);

    TEST_ASSERT_EQUAL_INT(8, g920_timestamp_format(buf, sizeof(buf), 1));
    TEST_ASSERT_EQUAL_STRING("0.000001", buf);

    TEST_ASSERT_EQUAL_INT(8, g920_timestamp_format(buf, sizeof(buf), 999999));
    TEST_ASSERT_EQUAL_STRING("0.999999", buf);

    TEST_ASSERT_EQUAL_INT(8, g920_timestamp_format(buf, sizeof(buf), 1000000));
    TEST_ASSERT_EQUAL_STRING("1.000000", buf);

    /* Ведущие нули в дробной части — то, на чём обычно ломаются самописные
     * форматтеры: 1.000042, а не 1.42. */
    TEST_ASSERT_EQUAL_INT(8, g920_timestamp_format(buf, sizeof(buf), 1000042));
    TEST_ASSERT_EQUAL_STRING("1.000042", buf);

    TEST_ASSERT_EQUAL_INT(9, g920_timestamp_format(buf, sizeof(buf), 12345678));
    TEST_ASSERT_EQUAL_STRING("12.345678", buf);
}

static void test_timestamp_format_max_fits_declared_buffer(void)
{
    char buf[G920_TIMESTAMP_STR_MAX];
    int len = g920_timestamp_format(buf, sizeof(buf), UINT64_MAX);

    TEST_ASSERT_EQUAL_INT(21, len);
    TEST_ASSERT_EQUAL_STRING("18446744073709.551615", buf);
    TEST_ASSERT_TRUE((size_t)len + 1u <= (size_t)G920_TIMESTAMP_STR_MAX);
}

static void test_timestamp_format_rejects_small_buffer(void)
{
    char buf[16];

    memset(buf, 'x', sizeof(buf));
    /* "0.000000" нужно 9 байт, даём 8 */
    TEST_ASSERT_EQUAL_INT(-1, g920_timestamp_format(buf, 8, 0));
    for (size_t i = 0; i < sizeof(buf); i++) {
        TEST_ASSERT_EQUAL_CHAR('x', buf[i]);
    }
    TEST_ASSERT_EQUAL_INT(-1, g920_timestamp_format(buf, 0, 0));
    TEST_ASSERT_EQUAL_INT(-1, g920_timestamp_format(NULL, 32, 0));
}

static void test_timestamp_us_is_monotonic(void)
{
    uint64_t prev = g920_timestamp_us();

    for (int i = 0; i < 1000; i++) {
        uint64_t now = g920_timestamp_us();
        TEST_ASSERT_TRUE(now >= prev);
        prev = now;
    }
}

static void test_timestamp_us_advances(void)
{
    uint64_t start = g920_timestamp_us();
    uint64_t now = start;

    /* Разрешение обязано быть микросекундным, а не «тик планировщика»:
     * на миллисекундных часах замеры T1/T2 и джиттера бессмысленны. */
    for (int i = 0; i < 10000000 && now == start; i++) {
        now = g920_timestamp_us();
    }
    TEST_ASSERT_TRUE(now > start);
}

/* --- hexdump ------------------------------------------------------------ */

static void test_hexdump_line_count(void)
{
    TEST_ASSERT_EQUAL_size_t(0, g920_hexdump_line_count(0));
    TEST_ASSERT_EQUAL_size_t(1, g920_hexdump_line_count(1));
    TEST_ASSERT_EQUAL_size_t(1, g920_hexdump_line_count(15));
    TEST_ASSERT_EQUAL_size_t(1, g920_hexdump_line_count(16));
    TEST_ASSERT_EQUAL_size_t(2, g920_hexdump_line_count(17));
    TEST_ASSERT_EQUAL_size_t(2, g920_hexdump_line_count(32));
    TEST_ASSERT_EQUAL_size_t(3, g920_hexdump_line_count(33));
}

static void test_hexdump_full_line(void)
{
    /* Команда переключения руля в PC-режим плюс мусор — байты те же, что в
     * ROADMAP: 0F 00 01 01 42. */
    const uint8_t data[16] = { 0x0F, 0x00, 0x01, 0x01, 0x42, 0x20, 0x03, 0x00,
                               'G',  '9',  '2',  '0',  0x7F, 0x80, 0xFF, 0x41 };
    char buf[G920_HEXDUMP_LINE_MAX];
    int len = g920_hexdump_line(buf, sizeof(buf), 0x10, data, sizeof(data));

    TEST_ASSERT_EQUAL_INT(78, len);
    TEST_ASSERT_EQUAL_STRING("00000010  0f 00 01 01 42 20 03 00  "
                             "47 39 32 30 7f 80 ff 41  |....B ..G920...A|",
                             buf);
    TEST_ASSERT_TRUE((size_t)len + 1u <= (size_t)G920_HEXDUMP_LINE_MAX);
}

/* Позиция открывающей '|' в полной строке: она обязана быть одна и та же у
 * полных и неполных строк, иначе столбец ASCII поедет. */
#define ASCII_COLUMN 60

static void test_hexdump_partial_line_keeps_ascii_column(void)
{
    const uint8_t data[3] = { 0xDE, 0xAD, 0xBE };
    const uint8_t sixteen[16] = { 0 };
    char buf[G920_HEXDUMP_LINE_MAX];
    char full[G920_HEXDUMP_LINE_MAX];

    TEST_ASSERT_EQUAL_INT(65, g920_hexdump_line(buf, sizeof(buf), 0, data, 3));
    TEST_ASSERT_EQUAL_INT(0, strncmp(buf, "00000000  de ad be ", 19));
    TEST_ASSERT_EQUAL_STRING("|...|", buf + ASCII_COLUMN);

    TEST_ASSERT_EQUAL_INT(
        78, g920_hexdump_line(full, sizeof(full), 0, sixteen, 16));
    TEST_ASSERT_EQUAL_CHAR('|', full[ASCII_COLUMN]);

    /* Между байтами и ASCII — только пробелы, ничего не потерялось. */
    for (size_t i = 19; i < ASCII_COLUMN; i++) {
        TEST_ASSERT_EQUAL_CHAR(' ', buf[i]);
    }
}

static void test_hexdump_empty(void)
{
    char buf[G920_HEXDUMP_LINE_MAX];

    TEST_ASSERT_EQUAL_INT(62, g920_hexdump_line(buf, sizeof(buf), 0, NULL, 0));
    TEST_ASSERT_EQUAL_INT(0, strncmp(buf, "00000000  ", 10));
    TEST_ASSERT_EQUAL_STRING("||", buf + ASCII_COLUMN);
}

static void test_hexdump_rejects_bad_arguments(void)
{
    const uint8_t data[16] = { 0 };
    char buf[G920_HEXDUMP_LINE_MAX];

    TEST_ASSERT_EQUAL_INT(-1, g920_hexdump_line(NULL, 128, 0, data, 16));
    TEST_ASSERT_EQUAL_INT(-1, g920_hexdump_line(buf, sizeof(buf), 0, NULL, 1));
    TEST_ASSERT_EQUAL_INT(-1, g920_hexdump_line(buf, sizeof(buf), 0, data, 17));
    TEST_ASSERT_EQUAL_INT(-1, g920_hexdump_line(buf, 78, 0, data, 16));
    TEST_ASSERT_EQUAL_INT(78, g920_hexdump_line(buf, 79, 0, data, 16));
}

static void test_hexdump_offset_digits(void)
{
    const uint8_t data[1] = { 0 };
    char buf[G920_HEXDUMP_LINE_MAX];

    TEST_ASSERT_TRUE(
        g920_hexdump_line(buf, sizeof(buf), 0xABCDEF12u, data, 1) > 0);
    TEST_ASSERT_EQUAL_INT(0, strncmp(buf, "abcdef12  ", 10));

#if SIZE_MAX > 0xFFFFFFFFu
    /* Смещение шире восьми цифр сломало бы ширину строки — лучше отказ. */
    TEST_ASSERT_EQUAL_INT(
        -1, g920_hexdump_line(buf, sizeof(buf), (size_t)0x100000000ull, data,
                              1));
#endif
}

/* --- лог ---------------------------------------------------------------- */

static void test_log_level_char(void)
{
    TEST_ASSERT_EQUAL_CHAR('E', g920_log_level_char(G920_LOG_ERROR));
    TEST_ASSERT_EQUAL_CHAR('W', g920_log_level_char(G920_LOG_WARN));
    TEST_ASSERT_EQUAL_CHAR('I', g920_log_level_char(G920_LOG_INFO));
    TEST_ASSERT_EQUAL_CHAR('D', g920_log_level_char(G920_LOG_DEBUG));
    TEST_ASSERT_EQUAL_CHAR('?', g920_log_level_char((g920_log_level_t)99));
}

static void test_log_prefix_format(void)
{
    char buf[G920_LOG_PREFIX_MAX];
    int len = g920_log_prefix(buf, sizeof(buf), G920_LOG_INFO, 12345678,
                              "gip");

    /* Роль берётся из общего кода: сборка тестов помечена HOST_TEST. */
    TEST_ASSERT_EQUAL_STRING("I 12.345678 TEST gip: ", buf);
    TEST_ASSERT_EQUAL_INT(22, len);
}

static void test_log_prefix_truncates_long_tag(void)
{
    char buf[G920_LOG_PREFIX_MAX];

    TEST_ASSERT_TRUE(g920_log_prefix(buf, sizeof(buf), G920_LOG_ERROR, 0,
                                     "abcdefghijklmnopqrstuvwxyz")
                     > 0);
    TEST_ASSERT_EQUAL_STRING("E 0.000000 TEST abcdefghijklmnop: ", buf);
}

static void test_log_prefix_worst_case_fits_declared_buffer(void)
{
    char buf[G920_LOG_PREFIX_MAX];
    int len = g920_log_prefix(buf, sizeof(buf), G920_LOG_DEBUG, UINT64_MAX,
                              "abcdefghijklmnopqrstuvwxyz");

    TEST_ASSERT_TRUE(len > 0);
    TEST_ASSERT_EQUAL_INT(47, len);
    TEST_ASSERT_TRUE((size_t)len + 1u <= (size_t)G920_LOG_PREFIX_MAX);
}

static void test_log_prefix_rejects_bad_arguments(void)
{
    char buf[G920_LOG_PREFIX_MAX];

    TEST_ASSERT_EQUAL_INT(-1, g920_log_prefix(NULL, 64, G920_LOG_INFO, 0, "x"));
    TEST_ASSERT_EQUAL_INT(-1, g920_log_prefix(buf, 4, G920_LOG_INFO, 0, "x"));
    /* tag == NULL не должен ронять лог: он и так последняя линия обороны. */
    TEST_ASSERT_TRUE(g920_log_prefix(buf, sizeof(buf), G920_LOG_INFO, 0, NULL)
                     > 0);
    TEST_ASSERT_EQUAL_STRING("I 0.000000 TEST ?: ", buf);
}

static size_t count_newlines(void)
{
    size_t n = 0;

    for (size_t i = 0; i < cap_len; i++) {
        if (cap_buf[i] == '\n') {
            n++;
        }
    }
    return n;
}

static void test_log_emits_one_line_with_newline(void)
{
    G920_LOGI("link", "peer %d rssi %d", 2, -42);

    TEST_ASSERT_EQUAL_INT(1, cap_lines);
    TEST_ASSERT_FALSE(cap_overflow);
    TEST_ASSERT_EQUAL_CHAR('I', cap_buf[0]);
    TEST_ASSERT_NOT_NULL(strstr(cap_buf, " TEST link: peer 2 rssi -42\n"));
    /* Приёмник получает строку целиком и ровно один раз: склеивать куски
     * ему не придётся ни в UART, ни при записи на TF-карту. */
    TEST_ASSERT_EQUAL_CHAR('\n', cap_buf[cap_len - 1]);
    TEST_ASSERT_EQUAL_size_t(1, count_newlines());
}

static void test_log_level_filters(void)
{
    g920_log_set_level(G920_LOG_WARN);

    G920_LOGD("x", "debug");
    G920_LOGI("x", "info");
    TEST_ASSERT_EQUAL_INT(0, cap_lines);

    G920_LOGW("x", "warn");
    G920_LOGE("x", "error");
    TEST_ASSERT_EQUAL_INT(2, cap_lines);
}

static void test_log_marks_truncated_message(void)
{
    char long_msg[G920_LOG_MSG_MAX + 64];

    memset(long_msg, 'a', sizeof(long_msg) - 1);
    long_msg[sizeof(long_msg) - 1] = '\0';

    G920_LOGI("x", "%s", long_msg);

    TEST_ASSERT_EQUAL_INT(1, cap_lines);
    /* Хвост потерян — в строке обязана остаться видимая метка обрезки. */
    TEST_ASSERT_EQUAL_CHAR('\n', cap_buf[cap_len - 1]);
    TEST_ASSERT_EQUAL_CHAR('~', cap_buf[cap_len - 2]);
}

static void test_log_short_message_is_not_marked(void)
{
    G920_LOGI("x", "short");

    TEST_ASSERT_EQUAL_CHAR('\n', cap_buf[cap_len - 1]);
    TEST_ASSERT_EQUAL_CHAR('t', cap_buf[cap_len - 2]);
}

static void test_log_hexdump_header_plus_lines(void)
{
    uint8_t data[33];

    for (size_t i = 0; i < sizeof(data); i++) {
        data[i] = (uint8_t)i;
    }
    g920_log_hexdump(G920_LOG_INFO, "usb", data, sizeof(data));

    /* Заголовок с длиной плюс три строки дампа. */
    TEST_ASSERT_EQUAL_INT(4, cap_lines);
    TEST_ASSERT_NOT_NULL(strstr(cap_buf, "usb: hexdump: 33 bytes\n"));
    TEST_ASSERT_NOT_NULL(strstr(cap_buf, "usb: 00000000  00 01 02"));
    TEST_ASSERT_NOT_NULL(strstr(cap_buf, "usb: 00000010  10 11 12"));
    TEST_ASSERT_NOT_NULL(strstr(cap_buf, "usb: 00000020  20 "));
}

static void test_log_hexdump_empty_still_reports(void)
{
    g920_log_hexdump(G920_LOG_INFO, "usb", NULL, 0);

    TEST_ASSERT_EQUAL_INT(1, cap_lines);
    TEST_ASSERT_NOT_NULL(strstr(cap_buf, "usb: hexdump: 0 bytes\n"));
}

static void test_log_hexdump_null_with_length_is_reported_not_crashed(void)
{
    g920_log_hexdump(G920_LOG_INFO, "usb", NULL, 8);

    TEST_ASSERT_EQUAL_INT(1, cap_lines);
    TEST_ASSERT_NOT_NULL(strstr(cap_buf, "NULL, 8 bytes claimed"));
}

static void test_log_hexdump_respects_level(void)
{
    const uint8_t data[4] = { 1, 2, 3, 4 };

    g920_log_set_level(G920_LOG_WARN);
    g920_log_hexdump(G920_LOG_DEBUG, "usb", data, sizeof(data));
    TEST_ASSERT_EQUAL_INT(0, cap_lines);
}

/*
 * Уровень сборки обещает больше, чем «строка не напечатается»: он обещает,
 * что **вызова не будет вовсе**, вместе с вычислением аргументов. Ровно за
 * это его и заводили — в горячем пути дорога не печать, а подготовка к ней.
 *
 * Проверяется побочным действием: если аргумент вычислен, счётчик двинулся.
 * Уровнем выполнения такую проверку не сделать — там аргументы вычисляются
 * всегда, и в этом вся разница между двумя выключателями.
 */
static int side_effects;

static int bump_and_return_zero(void)
{
    side_effects++;
    return 0;
}

static void test_log_build_level_does_not_evaluate_arguments(void)
{
    /* Уровень выполнения самый разговорчивый: мешать он не должен. */
    g920_log_set_level(G920_LOG_DEBUG);

    side_effects = 0;
    /* DEBUG выше уровня сборки по умолчанию (INFO) — ветки нет. */
    G920_LOGD("t", "%d", bump_and_return_zero());
    TEST_ASSERT_EQUAL_INT(0, side_effects);
    TEST_ASSERT_EQUAL_INT(0, cap_lines);

    /* INFO в уровень сборки укладывается — аргумент вычислен, строка есть. */
    G920_LOGI("t", "%d", bump_and_return_zero());
    TEST_ASSERT_EQUAL_INT(1, side_effects);
    TEST_ASSERT_EQUAL_INT(1, cap_lines);
}

static void test_log_build_level_default_is_info(void)
{
    /* Умолчание — часть договора со сборкой: профили, ничего не задающие,
     * обязаны вести себя как раньше. */
    TEST_ASSERT_EQUAL_INT(G920_LOG_LEVEL_INFO, G920_LOG_BUILD_LEVEL);
    TEST_ASSERT_TRUE(G920_LOG_ENABLED(G920_LOG_LEVEL_ERROR));
    TEST_ASSERT_TRUE(G920_LOG_ENABLED(G920_LOG_LEVEL_WARN));
    TEST_ASSERT_TRUE(G920_LOG_ENABLED(G920_LOG_LEVEL_INFO));
    TEST_ASSERT_FALSE(G920_LOG_ENABLED(G920_LOG_LEVEL_DEBUG));
}

static void test_log_hexdump_macro_obeys_build_level(void)
{
    const uint8_t data[4] = { 1, 2, 3, 4 };

    g920_log_set_level(G920_LOG_DEBUG);

    /* Дамп дороже строки на порядок, и выключаться обязан тем же рычагом. */
    G920_LOGD_HEXDUMP("usb", data, sizeof(data));
    TEST_ASSERT_EQUAL_INT(0, cap_lines);

    G920_LOGI_HEXDUMP("usb", data, sizeof(data));
    TEST_ASSERT_EQUAL_INT(2, cap_lines); /* заголовок и одна строка дампа */
}

static void test_log_set_sink_null_restores_default(void)
{
    g920_log_set_sink(NULL, NULL);
    G920_LOGI("x", "goes to console, not to capture");
    TEST_ASSERT_EQUAL_INT(0, cap_lines);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_timestamp_format_basic);
    RUN_TEST(test_timestamp_format_max_fits_declared_buffer);
    RUN_TEST(test_timestamp_format_rejects_small_buffer);
    RUN_TEST(test_timestamp_us_is_monotonic);
    RUN_TEST(test_timestamp_us_advances);

    RUN_TEST(test_hexdump_line_count);
    RUN_TEST(test_hexdump_full_line);
    RUN_TEST(test_hexdump_partial_line_keeps_ascii_column);
    RUN_TEST(test_hexdump_empty);
    RUN_TEST(test_hexdump_rejects_bad_arguments);
    RUN_TEST(test_hexdump_offset_digits);

    RUN_TEST(test_log_level_char);
    RUN_TEST(test_log_prefix_format);
    RUN_TEST(test_log_prefix_truncates_long_tag);
    RUN_TEST(test_log_prefix_worst_case_fits_declared_buffer);
    RUN_TEST(test_log_prefix_rejects_bad_arguments);
    RUN_TEST(test_log_emits_one_line_with_newline);
    RUN_TEST(test_log_level_filters);
    RUN_TEST(test_log_marks_truncated_message);
    RUN_TEST(test_log_short_message_is_not_marked);
    RUN_TEST(test_log_hexdump_header_plus_lines);
    RUN_TEST(test_log_hexdump_empty_still_reports);
    RUN_TEST(test_log_hexdump_null_with_length_is_reported_not_crashed);
    RUN_TEST(test_log_hexdump_respects_level);
    RUN_TEST(test_log_build_level_does_not_evaluate_arguments);
    RUN_TEST(test_log_build_level_default_is_info);
    RUN_TEST(test_log_hexdump_macro_obeys_build_level);
    RUN_TEST(test_log_set_sink_null_restores_default);

    return UNITY_END();
}
