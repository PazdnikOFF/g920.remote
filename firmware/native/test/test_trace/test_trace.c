#include <stdint.h>
#include <string.h>

#include <unity.h>

#include "g920/log.h"
#include "g920/trace.h"

/*
 * Буфер намеренно маленький: переполнение и перенос должны наступать в
 * тесте на десятке записей, а не на миллионе. На плате он будет мегабайтом
 * PSRAM, но код тот же.
 */
#define BUF 256

static uint8_t storage[BUF];
static g920_trace_t trace;

void setUp(void)
{
    memset(storage, 0, sizeof(storage));
    g920_trace_init(&trace, storage, sizeof(storage), G920_TRACE_KEEP_NEWEST);
}

void tearDown(void) { }

/* --- запись и чтение ------------------------------------------------------ */

static void test_write_then_read_roundtrip(void)
{
    const uint8_t payload[] = { 0xDE, 0xAD, 0xBE, 0xEF };
    g920_trace_cursor_t cursor;
    g920_trace_record_t record;

    TEST_ASSERT_TRUE(g920_trace_write(&trace, 12345678, G920_TRACE_USB_IN,
                                      payload, sizeof(payload)));

    g920_trace_rewind(&trace, &cursor);
    TEST_ASSERT_TRUE(g920_trace_next(&trace, &cursor, &record));
    TEST_ASSERT_EQUAL_UINT64(12345678, record.at_us);
    TEST_ASSERT_EQUAL_UINT8(G920_TRACE_USB_IN, record.kind);
    TEST_ASSERT_EQUAL_UINT16(sizeof(payload), record.length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, record.payload, sizeof(payload));
    /* Нагрузка отдаётся указателем внутрь буфера: копии нет. */
    TEST_ASSERT_TRUE(record.payload > storage
                     && record.payload < storage + BUF);
    TEST_ASSERT_FALSE(g920_trace_next(&trace, &cursor, &record));
}

static void test_order_is_oldest_first(void)
{
    g920_trace_cursor_t cursor;
    g920_trace_record_t record;
    uint8_t data[4] = { 0 };

    for (uint8_t i = 0; i < 5; i++) {
        data[0] = i;
        TEST_ASSERT_TRUE(g920_trace_write(&trace, 1000u + i,
                                          G920_TRACE_GIP_IN, data,
                                          sizeof(data)));
    }
    g920_trace_rewind(&trace, &cursor);
    for (uint8_t i = 0; i < 5; i++) {
        TEST_ASSERT_TRUE(g920_trace_next(&trace, &cursor, &record));
        TEST_ASSERT_EQUAL_UINT64(1000u + i, record.at_us);
        TEST_ASSERT_EQUAL_UINT8(i, record.payload[0]);
    }
    TEST_ASSERT_FALSE(g920_trace_next(&trace, &cursor, &record));
}

static void test_zero_length_record(void)
{
    g920_trace_cursor_t cursor;
    g920_trace_record_t record;

    /* Метка без нагрузки — тоже факт трассы: «здесь я дёрнул руль». */
    TEST_ASSERT_TRUE(g920_trace_write(&trace, 7, G920_TRACE_MARK, NULL, 0));
    g920_trace_rewind(&trace, &cursor);
    TEST_ASSERT_TRUE(g920_trace_next(&trace, &cursor, &record));
    TEST_ASSERT_EQUAL_UINT16(0, record.length);
    TEST_ASSERT_NULL(record.payload);
}

/* --- переполнение: две политики ------------------------------------------- */

static void test_ring_evicts_oldest(void)
{
    g920_trace_cursor_t cursor;
    g920_trace_record_t record;
    uint8_t data[20] = { 0 };
    /* 32 байта на запись (12 + 20) — восемь записей заполняют 256 байт. */
    const uint32_t fits = BUF / (G920_TRACE_HEADER_SIZE + 20);

    for (uint32_t i = 0; i < fits + 3; i++) {
        data[0] = (uint8_t)i;
        TEST_ASSERT_TRUE(g920_trace_write(&trace, i, G920_TRACE_USB_IN, data,
                                          sizeof(data)));
    }
    /* Затёрлись ровно три самые старые. */
    TEST_ASSERT_EQUAL_UINT32(3, g920_trace_evicted(&trace));
    TEST_ASSERT_EQUAL_UINT32(fits, g920_trace_count(&trace));
    TEST_ASSERT_EQUAL_UINT32(fits + 3, g920_trace_written(&trace));
    TEST_ASSERT_EQUAL_UINT32(0, g920_trace_refused(&trace));

    /* Первая уцелевшая — третья по счёту, последняя — самая новая. */
    g920_trace_rewind(&trace, &cursor);
    TEST_ASSERT_TRUE(g920_trace_next(&trace, &cursor, &record));
    TEST_ASSERT_EQUAL_UINT64(3, record.at_us);
    while (g920_trace_next(&trace, &cursor, &record)) {
        /* дочитываем до конца */
    }
    TEST_ASSERT_EQUAL_UINT64(fits + 2, record.at_us);
}

static void test_keep_oldest_stops_instead_of_overwriting(void)
{
    g920_trace_cursor_t cursor;
    g920_trace_record_t record;
    uint8_t data[20] = { 0 };
    const uint32_t fits = BUF / (G920_TRACE_HEADER_SIZE + 20);

    g920_trace_init(&trace, storage, sizeof(storage), G920_TRACE_KEEP_OLDEST);

    for (uint32_t i = 0; i < fits; i++) {
        TEST_ASSERT_TRUE(g920_trace_write(&trace, i, G920_TRACE_USB_IN, data,
                                          sizeof(data)));
    }
    /*
     * Полный дамп из M1: ценно начало. Заполнились — молчим, но потерю
     * считаем, иначе «дамп полный» окажется неправдой без единого признака.
     */
    TEST_ASSERT_FALSE(g920_trace_write(&trace, 999, G920_TRACE_USB_IN, data,
                                       sizeof(data)));
    TEST_ASSERT_EQUAL_UINT32(1, g920_trace_refused(&trace));
    TEST_ASSERT_EQUAL_UINT32(0, g920_trace_evicted(&trace));
    TEST_ASSERT_EQUAL_UINT32(fits, g920_trace_count(&trace));

    /* Голова на месте. */
    g920_trace_rewind(&trace, &cursor);
    TEST_ASSERT_TRUE(g920_trace_next(&trace, &cursor, &record));
    TEST_ASSERT_EQUAL_UINT64(0, record.at_us);
}

static void test_record_larger_than_buffer_is_refused(void)
{
    static uint8_t big[BUF];

    /* Сколько ни выбрасывай, не влезет. Отказ, а не бесконечная чистка. */
    TEST_ASSERT_FALSE(g920_trace_write(&trace, 1, G920_TRACE_USB_IN, big,
                                       sizeof(big)));
    TEST_ASSERT_EQUAL_UINT32(1, g920_trace_refused(&trace));
    TEST_ASSERT_TRUE(g920_trace_empty(&trace));
}

/* --- перенос через границу буфера ---------------------------------------- */

static void test_record_never_straddles_the_wrap(void)
{
    g920_trace_cursor_t cursor;
    g920_trace_record_t record;
    uint8_t data[30];
    uint32_t seen = 0;

    /*
     * 42 байта на запись, 256 не делится нацело — значит рано или поздно
     * запись упрётся в конец буфера. Она обязана начаться с нуля, а не
     * разорваться: иначе читателю понадобился бы буфер для склейки.
     */
    for (uint32_t i = 0; i < 40; i++) {
        memset(data, (uint8_t)i, sizeof(data));
        TEST_ASSERT_TRUE(g920_trace_write(&trace, i, G920_TRACE_LINK_TX, data,
                                          sizeof(data)));
    }

    g920_trace_rewind(&trace, &cursor);
    while (g920_trace_next(&trace, &cursor, &record)) {
        /* Нагрузка лежит одним куском внутри буфера — целиком. */
        TEST_ASSERT_TRUE(record.payload >= storage);
        TEST_ASSERT_TRUE(record.payload + record.length <= storage + BUF);
        for (uint16_t b = 0; b < record.length; b++) {
            TEST_ASSERT_EQUAL_UINT8((uint8_t)record.at_us, record.payload[b]);
        }
        seen++;
    }
    TEST_ASSERT_EQUAL_UINT32(g920_trace_count(&trace), seen);
    TEST_ASSERT_TRUE(g920_trace_used(&trace) <= BUF);
}

static void test_mixed_sizes_survive_many_wraps(void)
{
    g920_trace_cursor_t cursor;
    g920_trace_record_t record;
    uint8_t data[64];
    uint64_t previous = 0;
    bool first = true;
    uint32_t seen = 0;

    /* Длины вразнобой: именно они ломают наивную арифметику переноса. */
    for (uint32_t i = 0; i < 500; i++) {
        uint16_t len = (uint16_t)(1 + (i * 7) % 60);

        memset(data, (uint8_t)i, sizeof(data));
        TEST_ASSERT_TRUE(
            g920_trace_write(&trace, i, G920_TRACE_USB_OUT, data, len));
    }

    g920_trace_rewind(&trace, &cursor);
    while (g920_trace_next(&trace, &cursor, &record)) {
        if (!first) {
            /* Порядок не поехал: метки строго растут. */
            TEST_ASSERT_TRUE(record.at_us > previous);
        }
        for (uint16_t b = 0; b < record.length; b++) {
            TEST_ASSERT_EQUAL_UINT8((uint8_t)record.at_us, record.payload[b]);
        }
        previous = record.at_us;
        first = false;
        seen++;
    }
    TEST_ASSERT_EQUAL_UINT32(g920_trace_count(&trace), seen);
    /* Самая новая запись обязана быть на месте — ради неё кольцо и заведено. */
    TEST_ASSERT_EQUAL_UINT64(499, previous);
}

static void test_used_returns_to_zero_after_reset(void)
{
    uint8_t data[30] = { 0 };

    for (uint32_t i = 0; i < 20; i++) {
        g920_trace_write(&trace, i, G920_TRACE_USB_IN, data, sizeof(data));
    }
    g920_trace_reset(&trace);

    TEST_ASSERT_TRUE(g920_trace_empty(&trace));
    TEST_ASSERT_EQUAL_UINT32(0, g920_trace_count(&trace));
    TEST_ASSERT_EQUAL_size_t(0, g920_trace_used(&trace));
    /* А счётчики за жизнь не обнуляются: сброс — не амнезия. */
    TEST_ASSERT_EQUAL_UINT32(20, g920_trace_written(&trace));

    /* И после сброса буфер снова принимает записи. */
    TEST_ASSERT_TRUE(g920_trace_write(&trace, 1, G920_TRACE_MARK, NULL, 0));
    TEST_ASSERT_EQUAL_UINT32(1, g920_trace_count(&trace));
}

/* --- вывод ---------------------------------------------------------------- */

static void test_format_line(void)
{
    char line[G920_TRACE_LINE_MAX];
    g920_trace_record_t record = {
        .at_us = 12345678,
        .kind = G920_TRACE_USB_IN,
        .flags = 0,
        .length = 64,
        .payload = NULL,
    };

    TEST_ASSERT_TRUE(g920_trace_format(line, sizeof(line), &record) > 0);
    TEST_ASSERT_EQUAL_STRING("    12.345678 USB_IN 64 B", line);
}

static void test_format_into_small_buffer(void)
{
    char line[8];
    g920_trace_record_t record = {
        .at_us = 1,
        .kind = G920_TRACE_LOG,
        .flags = 0,
        .length = 0,
        .payload = NULL,
    };

    /* Обрезанная строка трассы хуже отсутствующей: её потом прочитают как
     * настоящую. */
    TEST_ASSERT_EQUAL_INT(-1, g920_trace_format(line, sizeof(line), &record));
    TEST_ASSERT_EQUAL_INT(-1, g920_trace_format(NULL, 32, &record));
    TEST_ASSERT_EQUAL_INT(-1, g920_trace_format(line, sizeof(line), NULL));
}

static void test_kind_names(void)
{
    TEST_ASSERT_EQUAL_STRING("USB_IN", g920_trace_kind_name(G920_TRACE_USB_IN));
    TEST_ASSERT_EQUAL_STRING("LINK_RX",
                             g920_trace_kind_name(G920_TRACE_LINK_RX));
    TEST_ASSERT_EQUAL_STRING("?", g920_trace_kind_name(G920_TRACE_NONE));
    TEST_ASSERT_EQUAL_STRING("?", g920_trace_kind_name((g920_trace_kind_t)200));
}

/* --- приёмник лога -------------------------------------------------------- */

static void test_log_sink_stores_lines(void)
{
    g920_trace_cursor_t cursor;
    g920_trace_record_t record;
    const char *text = "I 1.000000 RX gip: hello\n";

    g920_trace_log_sink(&trace, text, strlen(text));

    g920_trace_rewind(&trace, &cursor);
    TEST_ASSERT_TRUE(g920_trace_next(&trace, &cursor, &record));
    TEST_ASSERT_EQUAL_UINT8(G920_TRACE_LOG, record.kind);
    TEST_ASSERT_EQUAL_UINT16(strlen(text), record.length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(text, record.payload, strlen(text));
}

static void test_dump_does_not_feed_itself(void)
{
    uint32_t before;

    /* Трасса подключена приёмником лога, и выгрузка идёт через лог: без
     * защиты дамп писал бы сам в себя и не кончился бы никогда. */
    g920_log_set_sink(g920_trace_log_sink, &trace);
    g920_trace_write(&trace, 1, G920_TRACE_MARK, NULL, 0);
    before = g920_trace_written(&trace);

    g920_trace_dump(&trace, "test", true);
    g920_log_set_sink(NULL, NULL);

    TEST_ASSERT_EQUAL_UINT32(before, g920_trace_written(&trace));
    TEST_ASSERT_EQUAL_UINT32(1, g920_trace_count(&trace));
}

/* --- аргументы ------------------------------------------------------------ */

static void test_bad_arguments(void)
{
    g920_trace_t empty;
    g920_trace_cursor_t cursor;
    g920_trace_record_t record;
    uint8_t data[4] = { 0 };

    g920_trace_init(&empty, NULL, 1024, G920_TRACE_KEEP_NEWEST);
    TEST_ASSERT_FALSE(
        g920_trace_write(&empty, 1, G920_TRACE_USB_IN, data, sizeof(data)));
    TEST_ASSERT_TRUE(g920_trace_empty(&empty));

    TEST_ASSERT_FALSE(g920_trace_write(NULL, 1, G920_TRACE_USB_IN, data, 4));
    /* Длина без данных — молчаливый мусор в трассе, отказ. */
    TEST_ASSERT_FALSE(g920_trace_write(&trace, 1, G920_TRACE_USB_IN, NULL, 4));

    g920_trace_rewind(NULL, &cursor);
    TEST_ASSERT_EQUAL_UINT32(0, cursor.remaining);
    TEST_ASSERT_FALSE(g920_trace_next(NULL, &cursor, &record));
    TEST_ASSERT_FALSE(g920_trace_next(&trace, NULL, &record));
    TEST_ASSERT_FALSE(g920_trace_next(&trace, &cursor, NULL));

    TEST_ASSERT_EQUAL_UINT32(0, g920_trace_count(NULL));
    TEST_ASSERT_EQUAL_UINT32(0, g920_trace_written(NULL));
    TEST_ASSERT_EQUAL_UINT32(0, g920_trace_evicted(NULL));
    TEST_ASSERT_EQUAL_UINT32(0, g920_trace_refused(NULL));
    TEST_ASSERT_EQUAL_size_t(0, g920_trace_used(NULL));
    TEST_ASSERT_TRUE(g920_trace_empty(NULL));
    g920_trace_init(NULL, storage, BUF, G920_TRACE_KEEP_NEWEST);
    g920_trace_reset(NULL);
    g920_trace_dump(NULL, "test", false);
    g920_trace_log_sink(NULL, "x", 1);
    g920_trace_log_sink(&trace, NULL, 1);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_write_then_read_roundtrip);
    RUN_TEST(test_order_is_oldest_first);
    RUN_TEST(test_zero_length_record);

    RUN_TEST(test_ring_evicts_oldest);
    RUN_TEST(test_keep_oldest_stops_instead_of_overwriting);
    RUN_TEST(test_record_larger_than_buffer_is_refused);

    RUN_TEST(test_record_never_straddles_the_wrap);
    RUN_TEST(test_mixed_sizes_survive_many_wraps);
    RUN_TEST(test_used_returns_to_zero_after_reset);

    RUN_TEST(test_format_line);
    RUN_TEST(test_format_into_small_buffer);
    RUN_TEST(test_kind_names);

    RUN_TEST(test_log_sink_stores_lines);
    RUN_TEST(test_dump_does_not_feed_itself);

    RUN_TEST(test_bad_arguments);

    return UNITY_END();
}
