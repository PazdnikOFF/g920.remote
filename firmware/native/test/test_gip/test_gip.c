/*
 * Векторы взяты из `docs/gip-official/txt/H001419 - Original GIP Spec.txt`,
 * раздел «Simple ACK Example» и «Reliable Large Message Transmission» —
 * это трассы реального обмена с известным разбором, а не выдуманные байты.
 */

#include <stdint.h>
#include <string.h>

#include <unity.h>

#include "g920/gip.h"

void setUp(void) { }
void tearDown(void) { }

/* Разбор + сборка обратно с побайтовой сверкой. Именно эта пара, а не
 * разбор в одиночку: туннель обязан быть вербатим (инвариант И1). */
static void roundtrip(const uint8_t *bytes, size_t len,
                      g920_gip_header_t *out)
{
    uint8_t rebuilt[G920_GIP_HEADER_MAX];
    int built;

    TEST_ASSERT_EQUAL_INT(G920_GIP_OK,
                          g920_gip_header_parse(out, bytes, len));
    TEST_ASSERT_EQUAL_UINT8(len, out->header_length);

    built = g920_gip_header_build(rebuilt, sizeof(rebuilt), out);
    TEST_ASSERT_EQUAL_INT((int)len, built);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(bytes, rebuilt, len);
}

/* --- расширяемые поля длины -------------------------------------------- */

static void test_varint_width(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, g920_gip_varint_width(0));
    TEST_ASSERT_EQUAL_UINT8(1, g920_gip_varint_width(127));
    TEST_ASSERT_EQUAL_UINT8(2, g920_gip_varint_width(128));
    TEST_ASSERT_EQUAL_UINT8(2, g920_gip_varint_width(16383));
    TEST_ASSERT_EQUAL_UINT8(3, g920_gip_varint_width(16384));
    TEST_ASSERT_EQUAL_UINT8(4, g920_gip_varint_width(G920_GIP_VARINT_MAX));
}

static void test_varint_decode_single_byte(void)
{
    const uint8_t buf[] = { 0x0E };
    uint32_t value = 0;
    uint8_t used = 0;

    TEST_ASSERT_EQUAL_INT(
        G920_GIP_OK, g920_gip_varint_decode(&value, &used, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_UINT32(14, value);
    TEST_ASSERT_EQUAL_UINT8(1, used);
}

static void test_varint_decode_two_bytes(void)
{
    /* Из спеки: Length = (LowByte & 0x7F) + HighByte * 0x80. */
    const uint8_t buf[] = { 0xBA, 0x00 };
    uint32_t value = 0;
    uint8_t used = 0;

    TEST_ASSERT_EQUAL_INT(
        G920_GIP_OK, g920_gip_varint_decode(&value, &used, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_UINT32(0x3A, value);
    TEST_ASSERT_EQUAL_UINT8(2, used);

    const uint8_t high[] = { 0x81, 0x02 };
    TEST_ASSERT_EQUAL_INT(
        G920_GIP_OK,
        g920_gip_varint_decode(&value, &used, high, sizeof(high)));
    TEST_ASSERT_EQUAL_UINT32(1 + 2 * 128, value);
}

static void test_varint_decode_truncated(void)
{
    const uint8_t buf[] = { 0xBA };

    uint32_t value = 0;
    TEST_ASSERT_EQUAL_INT(
        G920_GIP_TRUNCATED,
        g920_gip_varint_decode(&value, NULL, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_INT(G920_GIP_TRUNCATED,
                          g920_gip_varint_decode(&value, NULL, buf, 0));
}

static void test_varint_decode_rejects_overlong(void)
{
    /* Пять байт подряд с признаком продолжения: спека разрешает четыре. */
    const uint8_t buf[] = { 0x80, 0x80, 0x80, 0x80, 0x01 };
    uint32_t value = 0;

    TEST_ASSERT_EQUAL_INT(
        G920_GIP_MALFORMED,
        g920_gip_varint_decode(&value, NULL, buf, sizeof(buf)));
}

static void test_varint_encode_minimal(void)
{
    uint8_t buf[4];

    TEST_ASSERT_EQUAL_INT(1, g920_gip_varint_encode(buf, sizeof(buf), 14, 0));
    TEST_ASSERT_EQUAL_HEX8(0x0E, buf[0]);

    TEST_ASSERT_EQUAL_INT(2, g920_gip_varint_encode(buf, sizeof(buf), 128, 0));
    TEST_ASSERT_EQUAL_HEX8(0x80, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[1]);
}

static void test_varint_encode_padded_width(void)
{
    uint8_t buf[4];

    /* Тот самый приём из таблицы 4-6: 58 записано двумя байтами, чтобы
     * заголовок вышел чётной длины. */
    TEST_ASSERT_EQUAL_INT(2, g920_gip_varint_encode(buf, sizeof(buf), 0x3A, 2));
    TEST_ASSERT_EQUAL_HEX8(0xBA, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[1]);
}

static void test_varint_encode_rejects_narrow_and_overflow(void)
{
    uint8_t buf[4];

    /* Уже, чем нужно значению, — нельзя. */
    TEST_ASSERT_EQUAL_INT(-1, g920_gip_varint_encode(buf, sizeof(buf), 128, 1));
    TEST_ASSERT_EQUAL_INT(-1, g920_gip_varint_encode(buf, sizeof(buf), 1, 5));
    TEST_ASSERT_EQUAL_INT(
        -1, g920_gip_varint_encode(buf, sizeof(buf),
                                   G920_GIP_VARINT_MAX + 1, 0));
    TEST_ASSERT_EQUAL_INT(-1, g920_gip_varint_encode(buf, 1, 128, 0));
    TEST_ASSERT_EQUAL_INT(-1, g920_gip_varint_encode(NULL, 4, 1, 0));
}

static void test_varint_roundtrip_all_widths(void)
{
    const uint32_t values[] = { 0,     1,     127,   128,        255,
                                16383, 16384, 65535, 0x0FFFFFFFu };
    uint8_t buf[G920_GIP_VARINT_MAX_BYTES];

    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
        uint32_t back = 0;
        uint8_t used = 0;
        int written = g920_gip_varint_encode(buf, sizeof(buf), values[i], 0);

        TEST_ASSERT_TRUE(written > 0);
        TEST_ASSERT_EQUAL_INT(
            G920_GIP_OK,
            g920_gip_varint_decode(&back, &used, buf, (size_t)written));
        TEST_ASSERT_EQUAL_UINT32(values[i], back);
        TEST_ASSERT_EQUAL_UINT8((uint8_t)written, used);
    }
}

/* --- векторы из H001419 ------------------------------------------------- */

static void test_transaction_895_security_data(void)
{
    /* Security Data, хост → устройство. Не фрагмент, системное, ACME. */
    const uint8_t bytes[] = { 0x06, 0x30, 0x02, 0x0E };
    g920_gip_header_t h;

    roundtrip(bytes, sizeof(bytes), &h);

    TEST_ASSERT_EQUAL_INT(G920_GIP_CLASS_COMMAND, g920_gip_data_class(&h));
    TEST_ASSERT_EQUAL_UINT8(6, g920_gip_message_number(&h));
    TEST_ASSERT_EQUAL_UINT8(0x02, h.sequence);
    TEST_ASSERT_EQUAL_UINT32(14, h.payload_length);
    TEST_ASSERT_EQUAL_UINT8(1, h.length_bytes);

    TEST_ASSERT_FALSE(g920_gip_is_fragment(&h));
    TEST_ASSERT_FALSE(g920_gip_is_initial_fragment(&h));
    TEST_ASSERT_TRUE(g920_gip_is_system(&h));
    TEST_ASSERT_TRUE(g920_gip_wants_ack(&h));
    TEST_ASSERT_EQUAL_UINT8(0, g920_gip_expansion_index(&h));
    TEST_ASSERT_EQUAL_UINT8(0, h.tlo_bytes);
}

static void test_transaction_900_protocol_control_ack(void)
{
    /* Тот же заголовок у транзакций #900, #931 и #942 — Protocol Control
     * с ACK. Различаются только полезной нагрузкой. */
    const uint8_t bytes[] = { 0x01, 0x20, 0x02, 0x09 };
    g920_gip_header_t h;

    roundtrip(bytes, sizeof(bytes), &h);

    TEST_ASSERT_EQUAL_INT(G920_GIP_CLASS_COMMAND, g920_gip_data_class(&h));
    TEST_ASSERT_EQUAL_UINT8(1, g920_gip_message_number(&h));
    TEST_ASSERT_EQUAL_UINT32(9, h.payload_length);
    TEST_ASSERT_TRUE(g920_gip_is_system(&h));
    TEST_ASSERT_FALSE(g920_gip_wants_ack(&h));
    TEST_ASSERT_FALSE(g920_gip_is_fragment(&h));
}

static void test_transaction_925_first_fragment(void)
{
    /* Security Data Response 1: фрагмент, первый, ACME. Длина полезной
     * части записана двумя байтами ради чётного размера заголовка. */
    const uint8_t bytes[] = { 0x06, 0xF0, 0x02, 0xBA, 0x00, 0x5A };
    g920_gip_header_t h;

    roundtrip(bytes, sizeof(bytes), &h);

    TEST_ASSERT_TRUE(g920_gip_is_fragment(&h));
    TEST_ASSERT_TRUE(g920_gip_is_initial_fragment(&h));
    TEST_ASSERT_TRUE(g920_gip_is_system(&h));
    TEST_ASSERT_TRUE(g920_gip_wants_ack(&h));

    TEST_ASSERT_EQUAL_UINT32(58, h.payload_length);
    TEST_ASSERT_EQUAL_UINT8(2, h.length_bytes);
    /* InitFrag=1 → поле TLO это полная длина сообщения. */
    TEST_ASSERT_EQUAL_UINT32(90, h.tlo);
    TEST_ASSERT_EQUAL_UINT8(1, h.tlo_bytes);
    TEST_ASSERT_EQUAL_UINT8(6, h.header_length);
}

static void test_transaction_936_middle_fragment(void)
{
    /* Security Data Response 2: фрагмент, не первый. Здесь поле TLO —
     * смещение, а не полная длина. */
    const uint8_t bytes[] = { 0x06, 0xB0, 0x02, 0xA0, 0x00, 0x3A };
    g920_gip_header_t h;

    roundtrip(bytes, sizeof(bytes), &h);

    TEST_ASSERT_TRUE(g920_gip_is_fragment(&h));
    TEST_ASSERT_FALSE(g920_gip_is_initial_fragment(&h));
    TEST_ASSERT_TRUE(g920_gip_wants_ack(&h));

    TEST_ASSERT_EQUAL_UINT32(32, h.payload_length);
    TEST_ASSERT_EQUAL_UINT8(2, h.length_bytes);
    TEST_ASSERT_EQUAL_UINT32(58, h.tlo); /* смещение */
    TEST_ASSERT_EQUAL_UINT8(6, h.header_length);
}

/* --- поведение разбора --------------------------------------------------- */

static void test_init_frag_needs_fragment_bit(void)
{
    /* InitFrag осмыслен только вместе с Fragment (таблица 4-3). Голый
     * InitFrag не должен превращать сообщение в первый фрагмент. */
    const uint8_t bytes[] = { 0x06, 0x40, 0x02, 0x04 };
    g920_gip_header_t h;

    TEST_ASSERT_EQUAL_INT(G920_GIP_OK,
                          g920_gip_header_parse(&h, bytes, sizeof(bytes)));
    TEST_ASSERT_FALSE(g920_gip_is_fragment(&h));
    TEST_ASSERT_FALSE(g920_gip_is_initial_fragment(&h));
    TEST_ASSERT_EQUAL_UINT8(0, h.tlo_bytes);
}

static void test_data_classes_and_expansion_index(void)
{
    const uint8_t low[] = { 0x20, 0x00, 0x01, 0x0C };
    const uint8_t audio[] = { 0x60, 0x03, 0x01, 0x0C };
    g920_gip_header_t h;

    TEST_ASSERT_EQUAL_INT(G920_GIP_OK,
                          g920_gip_header_parse(&h, low, sizeof(low)));
    TEST_ASSERT_EQUAL_INT(G920_GIP_CLASS_LOW_LATENCY, g920_gip_data_class(&h));
    TEST_ASSERT_EQUAL_UINT8(0, g920_gip_message_number(&h));

    TEST_ASSERT_EQUAL_INT(G920_GIP_OK,
                          g920_gip_header_parse(&h, audio, sizeof(audio)));
    TEST_ASSERT_EQUAL_INT(G920_GIP_CLASS_AUDIO, g920_gip_data_class(&h));
    /* Индекс расширения — младшие три бита флагов, до 7 подустройств. */
    TEST_ASSERT_EQUAL_UINT8(3, g920_gip_expansion_index(&h));
}

static void test_reserved_bit_is_carried_not_rejected(void)
{
    /* Спека требует нуля в бите 3, но туннель обязан пропускать байты
     * как есть: своё мнение о чужом трафике — прямой путь к нарушению И1. */
    const uint8_t bytes[] = { 0x06, 0x38, 0x02, 0x04 };
    g920_gip_header_t h;
    uint8_t rebuilt[G920_GIP_HEADER_MAX];

    TEST_ASSERT_EQUAL_INT(G920_GIP_OK,
                          g920_gip_header_parse(&h, bytes, sizeof(bytes)));
    TEST_ASSERT_EQUAL_HEX8(G920_GIP_FLAG_RESERVED,
                           h.flags & G920_GIP_FLAG_RESERVED);
    TEST_ASSERT_EQUAL_INT(
        4, g920_gip_header_build(rebuilt, sizeof(rebuilt), &h));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(bytes, rebuilt, sizeof(bytes));
}

static void test_parse_truncated(void)
{
    const uint8_t full[] = { 0x06, 0xF0, 0x02, 0xBA, 0x00, 0x5A };
    g920_gip_header_t h;

    for (size_t len = 0; len < sizeof(full); len++) {
        TEST_ASSERT_EQUAL_INT(G920_GIP_TRUNCATED,
                              g920_gip_header_parse(&h, full, len));
    }
    TEST_ASSERT_EQUAL_INT(G920_GIP_OK,
                          g920_gip_header_parse(&h, full, sizeof(full)));
}

static void test_parse_bad_arguments(void)
{
    const uint8_t bytes[] = { 0x06, 0x30, 0x02, 0x0E };
    g920_gip_header_t h;

    TEST_ASSERT_EQUAL_INT(G920_GIP_BAD_ARG,
                          g920_gip_header_parse(NULL, bytes, sizeof(bytes)));
    TEST_ASSERT_EQUAL_INT(G920_GIP_BAD_ARG,
                          g920_gip_header_parse(&h, NULL, sizeof(bytes)));
}

static void test_build_into_small_buffer(void)
{
    const uint8_t bytes[] = { 0x06, 0xF0, 0x02, 0xBA, 0x00, 0x5A };
    g920_gip_header_t h;
    uint8_t buf[G920_GIP_HEADER_MAX];

    TEST_ASSERT_EQUAL_INT(G920_GIP_OK,
                          g920_gip_header_parse(&h, bytes, sizeof(bytes)));
    for (size_t size = 0; size < sizeof(bytes); size++) {
        TEST_ASSERT_EQUAL_INT(-1, g920_gip_header_build(buf, size, &h));
    }
    TEST_ASSERT_EQUAL_INT(6, g920_gip_header_build(buf, sizeof(bytes), &h));
}

static void test_build_widens_payload_length_on_demand(void)
{
    /* Сборка заголовка вниз к устройству: длина 58 записывается двумя
     * байтами, чтобы заголовок вышел чётной длины. */
    g920_gip_header_t h = {
        .message_type = 0x06,
        .flags = 0xF0,
        .sequence = 0x02,
        .payload_length = 58,
        .tlo = 90,
        .length_bytes = 2,
        .tlo_bytes = 1,
        .header_length = 0,
    };
    const uint8_t want[] = { 0x06, 0xF0, 0x02, 0xBA, 0x00, 0x5A };
    uint8_t buf[G920_GIP_HEADER_MAX];

    TEST_ASSERT_EQUAL_INT(6, g920_gip_header_build(buf, sizeof(buf), &h));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, buf, sizeof(want));

    /* Та же длина минимальной шириной даёт нечётный заголовок — тоже
     * законный, но для устройства не годится. */
    h.length_bytes = 0;
    TEST_ASSERT_EQUAL_INT(5, g920_gip_header_build(buf, sizeof(buf), &h));
    TEST_ASSERT_EQUAL_HEX8(0x3A, buf[3]);
}

static void test_status_names(void)
{
    TEST_ASSERT_EQUAL_STRING("OK", g920_gip_status_name(G920_GIP_OK));
    TEST_ASSERT_EQUAL_STRING("TRUNCATED",
                             g920_gip_status_name(G920_GIP_TRUNCATED));
    TEST_ASSERT_EQUAL_STRING("MALFORMED",
                             g920_gip_status_name(G920_GIP_MALFORMED));
    TEST_ASSERT_EQUAL_STRING("?", g920_gip_status_name((g920_gip_status_t)99));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_varint_width);
    RUN_TEST(test_varint_decode_single_byte);
    RUN_TEST(test_varint_decode_two_bytes);
    RUN_TEST(test_varint_decode_truncated);
    RUN_TEST(test_varint_decode_rejects_overlong);
    RUN_TEST(test_varint_encode_minimal);
    RUN_TEST(test_varint_encode_padded_width);
    RUN_TEST(test_varint_encode_rejects_narrow_and_overflow);
    RUN_TEST(test_varint_roundtrip_all_widths);

    RUN_TEST(test_transaction_895_security_data);
    RUN_TEST(test_transaction_900_protocol_control_ack);
    RUN_TEST(test_transaction_925_first_fragment);
    RUN_TEST(test_transaction_936_middle_fragment);

    RUN_TEST(test_init_frag_needs_fragment_bit);
    RUN_TEST(test_data_classes_and_expansion_index);
    RUN_TEST(test_reserved_bit_is_carried_not_rejected);
    RUN_TEST(test_parse_truncated);
    RUN_TEST(test_parse_bad_arguments);
    RUN_TEST(test_build_into_small_buffer);
    RUN_TEST(test_build_widens_payload_length_on_demand);
    RUN_TEST(test_status_names);

    return UNITY_END();
}
