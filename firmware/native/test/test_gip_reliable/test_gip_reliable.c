/*
 * Protocol Control и сборка фрагментов.
 *
 * Вектора — из `docs/gip-official/txt/H001419 - Original GIP Spec.txt`,
 * трасса «Reliable Large Message Transmission»: security-обмен на 90 байт,
 * разложенный на два фрагмента с тремя ACK.
 */

#include <stdint.h>
#include <string.h>

#include <unity.h>

#include "g920/gip.h"
#include "g920/gip_control.h"
#include "g920/gip_reasm.h"

static uint8_t buffer[256];
static g920_gip_reasm_t reasm;

void setUp(void)
{
    memset(buffer, 0, sizeof(buffer));
    g920_gip_reasm_init(&reasm, buffer, sizeof(buffer));
}

void tearDown(void) { }

static g920_gip_header_t parse(const uint8_t *bytes, size_t len)
{
    g920_gip_header_t h;

    TEST_ASSERT_EQUAL_INT(G920_GIP_OK, g920_gip_header_parse(&h, bytes, len));
    return h;
}

/* --- Protocol Control: вектора из трассы -------------------------------- */

static void test_control_parse_transaction_900(void)
{
    /* ACK на Security Data (#895): принято 14 байт, ждать больше нечего. */
    const uint8_t payload[] = { 0x00, 0x06, 0x20, 0x0E, 0x00,
                                0x00, 0x00, 0x00, 0x00 };
    g920_gip_control_t c;

    TEST_ASSERT_EQUAL_INT(
        G920_GIP_OK, g920_gip_control_parse(&c, payload, sizeof(payload)));
    TEST_ASSERT_EQUAL_HEX8(G920_GIP_CONTROL_ACK, c.control_code);
    TEST_ASSERT_EQUAL_HEX8(0x06, c.ref_message_type);
    TEST_ASSERT_EQUAL_HEX8(0x20, c.ref_message_flags);
    TEST_ASSERT_EQUAL_UINT32(14, c.fragment_offset);
    TEST_ASSERT_EQUAL_UINT16(0, c.remaining_buffer);
}

static void test_control_parse_transaction_931(void)
{
    /* ACK на первый фрагмент (#925): принято 58 из 90, осталось 32. */
    const uint8_t payload[] = { 0x00, 0x06, 0x20, 0x3A, 0x00,
                                0x00, 0x00, 0x20, 0x00 };
    g920_gip_control_t c;

    TEST_ASSERT_EQUAL_INT(
        G920_GIP_OK, g920_gip_control_parse(&c, payload, sizeof(payload)));
    TEST_ASSERT_EQUAL_UINT32(58, c.fragment_offset);
    TEST_ASSERT_EQUAL_UINT16(32, c.remaining_buffer);
}

static void test_control_roundtrip(void)
{
    const uint8_t payload[] = { 0x00, 0x06, 0x20, 0x3A, 0x00,
                                0x00, 0x00, 0x20, 0x00 };
    g920_gip_control_t c;
    uint8_t rebuilt[G920_GIP_CONTROL_PAYLOAD_SIZE];

    TEST_ASSERT_EQUAL_INT(
        G920_GIP_OK, g920_gip_control_parse(&c, payload, sizeof(payload)));
    TEST_ASSERT_EQUAL_INT(
        G920_GIP_CONTROL_PAYLOAD_SIZE,
        g920_gip_control_build(rebuilt, sizeof(rebuilt), &c));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, rebuilt, sizeof(payload));
}

static void test_control_parse_rejects_short_and_null(void)
{
    const uint8_t payload[G920_GIP_CONTROL_PAYLOAD_SIZE] = { 0 };
    g920_gip_control_t c;
    uint8_t buf[G920_GIP_CONTROL_PAYLOAD_SIZE];

    TEST_ASSERT_EQUAL_INT(
        G920_GIP_TRUNCATED,
        g920_gip_control_parse(&c, payload, sizeof(payload) - 1));
    TEST_ASSERT_EQUAL_INT(G920_GIP_BAD_ARG,
                          g920_gip_control_parse(NULL, payload, 9));
    TEST_ASSERT_EQUAL_INT(G920_GIP_BAD_ARG,
                          g920_gip_control_parse(&c, NULL, 9));
    TEST_ASSERT_EQUAL_INT(-1, g920_gip_control_build(buf, 8, &c));
}

/* --- сборка ACK на принятое сообщение ----------------------------------- */

static void test_build_ack_matches_transaction_900(void)
{
    /* #895 Security Data → #900 ACK. Пакет целиком, заголовок и нагрузка. */
    const uint8_t acked_bytes[] = { 0x06, 0x30, 0x02, 0x0E };
    const uint8_t want[] = { 0x01, 0x20, 0x02, 0x09, 0x00, 0x06, 0x20,
                             0x0E, 0x00, 0x00, 0x00, 0x00, 0x00 };
    g920_gip_header_t acked = parse(acked_bytes, sizeof(acked_bytes));
    uint8_t packet[32];

    TEST_ASSERT_EQUAL_INT(
        13, g920_gip_build_ack(packet, sizeof(packet), &acked, 14, 0));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, packet, sizeof(want));
}

static void test_build_ack_matches_transaction_931(void)
{
    /* #925 первый фрагмент (флаги 0xF0) → #931 ACK. Проверяет, что в
     * RefMessageFlags уезжает только System+Index: 0xF0 → 0x20. */
    const uint8_t acked_bytes[] = { 0x06, 0xF0, 0x02, 0xBA, 0x00, 0x5A };
    const uint8_t want[] = { 0x01, 0x20, 0x02, 0x09, 0x00, 0x06, 0x20,
                             0x3A, 0x00, 0x00, 0x00, 0x20, 0x00 };
    g920_gip_header_t acked = parse(acked_bytes, sizeof(acked_bytes));
    uint8_t packet[32];

    TEST_ASSERT_EQUAL_INT(
        13, g920_gip_build_ack(packet, sizeof(packet), &acked, 58, 32));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, packet, sizeof(want));
}

static void test_build_ack_matches_transaction_942(void)
{
    /* #936 средний фрагмент (флаги 0xB0) → #942 ACK, всё дособрано. */
    const uint8_t acked_bytes[] = { 0x06, 0xB0, 0x02, 0xA0, 0x00, 0x3A };
    const uint8_t want[] = { 0x01, 0x20, 0x02, 0x09, 0x00, 0x06, 0x20,
                             0x5A, 0x00, 0x00, 0x00, 0x00, 0x00 };
    g920_gip_header_t acked = parse(acked_bytes, sizeof(acked_bytes));
    uint8_t packet[32];

    TEST_ASSERT_EQUAL_INT(
        13, g920_gip_build_ack(packet, sizeof(packet), &acked, 90, 0));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, packet, sizeof(want));
}

static void test_build_ack_carries_expansion_index(void)
{
    /* Подтверждение обязано уйти тому же подустройству. */
    const uint8_t acked_bytes[] = { 0x06, 0x35, 0x07, 0x04 };
    g920_gip_header_t acked = parse(acked_bytes, sizeof(acked_bytes));
    uint8_t packet[32];

    TEST_ASSERT_EQUAL_INT(
        13, g920_gip_build_ack(packet, sizeof(packet), &acked, 4, 0));
    TEST_ASSERT_EQUAL_HEX8(0x25, packet[1]); /* System + index 5 */
    TEST_ASSERT_EQUAL_HEX8(0x07, packet[2]); /* Sequence ID подтверждаемого */
    TEST_ASSERT_EQUAL_HEX8(0x25, packet[6]); /* RefMessageFlags по маске */
}

static void test_build_ack_into_small_buffer(void)
{
    const uint8_t acked_bytes[] = { 0x06, 0x30, 0x02, 0x0E };
    g920_gip_header_t acked = parse(acked_bytes, sizeof(acked_bytes));
    uint8_t packet[32];

    for (size_t size = 0; size < 13; size++) {
        TEST_ASSERT_EQUAL_INT(-1,
                              g920_gip_build_ack(packet, size, &acked, 14, 0));
    }
    TEST_ASSERT_EQUAL_INT(13, g920_gip_build_ack(packet, 13, &acked, 14, 0));
    TEST_ASSERT_EQUAL_INT(-1, g920_gip_build_ack(NULL, 32, &acked, 14, 0));
    TEST_ASSERT_EQUAL_INT(-1,
                          g920_gip_build_ack(packet, 32, NULL, 14, 0));
}

/* --- политика ACME ------------------------------------------------------ */

static void test_acme_policy(void)
{
    /* Спека: первый и последний обязательны, из средних — каждый
     * четвёртый или пятый. */
    TEST_ASSERT_TRUE(g920_gip_should_request_ack(0, false, 4));
    TEST_ASSERT_TRUE(g920_gip_should_request_ack(7, true, 4));
    TEST_ASSERT_TRUE(g920_gip_should_request_ack(4, false, 4));
    TEST_ASSERT_TRUE(g920_gip_should_request_ack(8, false, 4));

    TEST_ASSERT_FALSE(g920_gip_should_request_ack(1, false, 4));
    TEST_ASSERT_FALSE(g920_gip_should_request_ack(2, false, 4));
    TEST_ASSERT_FALSE(g920_gip_should_request_ack(3, false, 4));

    /* every_n == 0 — подтверждаем только края. */
    TEST_ASSERT_FALSE(g920_gip_should_request_ack(5, false, 0));
    TEST_ASSERT_TRUE(g920_gip_should_request_ack(0, false, 0));
}

/* --- сборка сообщения: трасса из спеки ---------------------------------- */

static void test_reassembly_of_the_90_byte_security_message(void)
{
    /* #925: первый фрагмент, 58 байт из 90. */
    const uint8_t first_bytes[] = { 0x06, 0xF0, 0x02, 0xBA, 0x00, 0x5A };
    /* #936: второй, 32 байта со смещения 58 — итого ровно 90. */
    const uint8_t second_bytes[] = { 0x06, 0xB0, 0x02, 0xA0, 0x00, 0x3A };
    /* Нулевой completion после того, как всё принято. */
    const uint8_t completion_bytes[] = { 0x06, 0x80, 0x02, 0x00, 0x5A };

    uint8_t first[58];
    uint8_t second[32];
    g920_gip_header_t h;

    for (size_t i = 0; i < sizeof(first); i++) {
        first[i] = (uint8_t)i;
    }
    for (size_t i = 0; i < sizeof(second); i++) {
        second[i] = (uint8_t)(0x80 + i);
    }

    h = parse(first_bytes, sizeof(first_bytes));
    TEST_ASSERT_EQUAL_INT(
        G920_GIP_REASM_MORE,
        g920_gip_reasm_push(&reasm, &h, first, sizeof(first)));
    TEST_ASSERT_TRUE(g920_gip_reasm_in_progress(&reasm));
    TEST_ASSERT_EQUAL_size_t(58, g920_gip_reasm_length(&reasm));
    /* Ровно то, что уедет в поле Remaining Buffer ответного ACK. */
    TEST_ASSERT_EQUAL_UINT16(32, g920_gip_reasm_remaining(&reasm));

    h = parse(second_bytes, sizeof(second_bytes));
    TEST_ASSERT_EQUAL_INT(
        G920_GIP_REASM_DONE,
        g920_gip_reasm_push(&reasm, &h, second, sizeof(second)));
    TEST_ASSERT_FALSE(g920_gip_reasm_in_progress(&reasm));
    TEST_ASSERT_EQUAL_size_t(90, g920_gip_reasm_length(&reasm));
    TEST_ASSERT_EQUAL_UINT16(0, g920_gip_reasm_remaining(&reasm));

    TEST_ASSERT_EQUAL_UINT8_ARRAY(first, buffer, sizeof(first));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(second, buffer + 58, sizeof(second));

    /* Нулевой фрагмент после собранного — это закрытие передачи, а не
     * потерянный пакет. */
    h = parse(completion_bytes, sizeof(completion_bytes));
    TEST_ASSERT_EQUAL_INT(G920_GIP_REASM_COMPLETION,
                          g920_gip_reasm_push(&reasm, &h, NULL, 0));
}

static void test_single_packet_message_is_not_reassembly(void)
{
    const uint8_t bytes[] = { 0x06, 0x30, 0x02, 0x04 };
    const uint8_t payload[] = { 1, 2, 3, 4 };
    g920_gip_header_t h = parse(bytes, sizeof(bytes));

    TEST_ASSERT_EQUAL_INT(
        G920_GIP_REASM_SINGLE,
        g920_gip_reasm_push(&reasm, &h, payload, sizeof(payload)));
    TEST_ASSERT_FALSE(g920_gip_reasm_in_progress(&reasm));
    TEST_ASSERT_EQUAL_size_t(4, g920_gip_reasm_length(&reasm));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, buffer, sizeof(payload));
}

static void test_single_fragment_message_completes_immediately(void)
{
    /* Первый фрагмент, который сразу закрывает объявленную длину. */
    const uint8_t bytes[] = { 0x06, 0xF0, 0x02, 0x04, 0x04 };
    const uint8_t payload[] = { 9, 9, 9, 9 };
    g920_gip_header_t h = parse(bytes, sizeof(bytes));

    TEST_ASSERT_EQUAL_INT(
        G920_GIP_REASM_DONE,
        g920_gip_reasm_push(&reasm, &h, payload, sizeof(payload)));
    TEST_ASSERT_EQUAL_size_t(4, g920_gip_reasm_length(&reasm));
}

/* --- сборка: потери и мусор --------------------------------------------- */

static void test_fragment_without_start_is_orphan(void)
{
    /* Включились посреди чужой передачи или потеряли первый пакет. */
    const uint8_t bytes[] = { 0x06, 0xB0, 0x02, 0x20, 0x3A };
    uint8_t payload[32] = { 0 };
    g920_gip_header_t h = parse(bytes, sizeof(bytes));

    TEST_ASSERT_EQUAL_INT(
        G920_GIP_REASM_ORPHAN,
        g920_gip_reasm_push(&reasm, &h, payload, sizeof(payload)));
}

static void test_wrong_offset_is_detected(void)
{
    const uint8_t first_bytes[] = { 0x06, 0xF0, 0x02, 0x0A, 0x28 };
    /* Смещение 20 вместо ожидаемых 10: пакет потерялся. */
    const uint8_t gap_bytes[] = { 0x06, 0xB0, 0x02, 0x0A, 0x14 };
    uint8_t payload[10] = { 0 };
    g920_gip_header_t h;

    h = parse(first_bytes, sizeof(first_bytes));
    TEST_ASSERT_EQUAL_INT(
        G920_GIP_REASM_MORE,
        g920_gip_reasm_push(&reasm, &h, payload, sizeof(payload)));

    h = parse(gap_bytes, sizeof(gap_bytes));
    TEST_ASSERT_EQUAL_INT(
        G920_GIP_REASM_OUT_OF_ORDER,
        g920_gip_reasm_push(&reasm, &h, payload, sizeof(payload)));
    /* Сборка не испорчена: принятое остаётся принятым, ACK попросит
     * повтор с нужного смещения. */
    TEST_ASSERT_EQUAL_size_t(10, g920_gip_reasm_length(&reasm));
    TEST_ASSERT_TRUE(g920_gip_reasm_in_progress(&reasm));
}

static void test_sequence_change_mid_message_is_detected(void)
{
    const uint8_t first_bytes[] = { 0x06, 0xF0, 0x02, 0x0A, 0x28 };
    const uint8_t other_seq[] = { 0x06, 0xB0, 0x03, 0x0A, 0x0A };
    uint8_t payload[10] = { 0 };
    g920_gip_header_t h;

    h = parse(first_bytes, sizeof(first_bytes));
    TEST_ASSERT_EQUAL_INT(
        G920_GIP_REASM_MORE,
        g920_gip_reasm_push(&reasm, &h, payload, sizeof(payload)));

    h = parse(other_seq, sizeof(other_seq));
    TEST_ASSERT_EQUAL_INT(
        G920_GIP_REASM_SEQUENCE_MISMATCH,
        g920_gip_reasm_push(&reasm, &h, payload, sizeof(payload)));
}

static void test_new_first_fragment_restarts_assembly(void)
{
    const uint8_t first_bytes[] = { 0x06, 0xF0, 0x02, 0x0A, 0x28 };
    const uint8_t restart_bytes[] = { 0x06, 0xF0, 0x03, 0x0A, 0x14 };
    uint8_t payload[10] = { 0 };
    g920_gip_header_t h;

    h = parse(first_bytes, sizeof(first_bytes));
    TEST_ASSERT_EQUAL_INT(
        G920_GIP_REASM_MORE,
        g920_gip_reasm_push(&reasm, &h, payload, sizeof(payload)));

    /* Отправитель начал заново — старая сборка выбрасывается. */
    h = parse(restart_bytes, sizeof(restart_bytes));
    TEST_ASSERT_EQUAL_INT(
        G920_GIP_REASM_MORE,
        g920_gip_reasm_push(&reasm, &h, payload, sizeof(payload)));
    TEST_ASSERT_EQUAL_size_t(10, g920_gip_reasm_length(&reasm));
}

static void test_payload_longer_than_declared_total(void)
{
    const uint8_t first_bytes[] = { 0x06, 0xF0, 0x02, 0x0A, 0x0C };
    const uint8_t tail_bytes[] = { 0x06, 0xB0, 0x02, 0x0A, 0x0A };
    uint8_t payload[10] = { 0 };
    g920_gip_header_t h;

    h = parse(first_bytes, sizeof(first_bytes));
    TEST_ASSERT_EQUAL_INT(
        G920_GIP_REASM_MORE,
        g920_gip_reasm_push(&reasm, &h, payload, sizeof(payload)));

    /* Объявлено 12, принято 10, приехало ещё 10 — не лезет. */
    h = parse(tail_bytes, sizeof(tail_bytes));
    TEST_ASSERT_EQUAL_INT(
        G920_GIP_REASM_OVERFLOW,
        g920_gip_reasm_push(&reasm, &h, payload, sizeof(payload)));
}

static void test_message_larger_than_buffer_is_refused_upfront(void)
{
    /* Объявленная длина 300 при буфере 16: отказываем сразу, а не на
     * середине приёма. */
    const uint8_t bytes[] = { 0x06, 0xF0, 0x02, 0x04, 0xAC, 0x02 };
    uint8_t small[16];
    uint8_t payload[4] = { 0 };
    g920_gip_reasm_t tiny;
    g920_gip_header_t h = parse(bytes, sizeof(bytes));

    TEST_ASSERT_EQUAL_UINT32(300, h.tlo);
    g920_gip_reasm_init(&tiny, small, sizeof(small));
    TEST_ASSERT_EQUAL_INT(
        G920_GIP_REASM_OVERFLOW,
        g920_gip_reasm_push(&tiny, &h, payload, sizeof(payload)));
}

static void test_push_rejects_length_mismatch_and_null(void)
{
    const uint8_t bytes[] = { 0x06, 0x30, 0x02, 0x04 };
    uint8_t payload[4] = { 0 };
    g920_gip_header_t h = parse(bytes, sizeof(bytes));

    /* Заголовок обещает 4 байта, а пришло 3 — где-то обрезали пакет. */
    TEST_ASSERT_EQUAL_INT(G920_GIP_REASM_BAD_ARG,
                          g920_gip_reasm_push(&reasm, &h, payload, 3));
    TEST_ASSERT_EQUAL_INT(G920_GIP_REASM_BAD_ARG,
                          g920_gip_reasm_push(NULL, &h, payload, 4));
    TEST_ASSERT_EQUAL_INT(G920_GIP_REASM_BAD_ARG,
                          g920_gip_reasm_push(&reasm, NULL, payload, 4));
    TEST_ASSERT_EQUAL_INT(G920_GIP_REASM_BAD_ARG,
                          g920_gip_reasm_push(&reasm, &h, NULL, 4));
}

static void test_reset_drops_partial_assembly(void)
{
    const uint8_t first_bytes[] = { 0x06, 0xF0, 0x02, 0x0A, 0x28 };
    uint8_t payload[10] = { 0 };
    g920_gip_header_t h = parse(first_bytes, sizeof(first_bytes));

    TEST_ASSERT_EQUAL_INT(
        G920_GIP_REASM_MORE,
        g920_gip_reasm_push(&reasm, &h, payload, sizeof(payload)));
    g920_gip_reasm_reset(&reasm);
    TEST_ASSERT_FALSE(g920_gip_reasm_in_progress(&reasm));
    TEST_ASSERT_EQUAL_size_t(0, g920_gip_reasm_length(&reasm));
}

static void test_reasm_status_names(void)
{
    TEST_ASSERT_EQUAL_STRING("DONE",
                             g920_gip_reasm_status_name(G920_GIP_REASM_DONE));
    TEST_ASSERT_EQUAL_STRING(
        "OUT_OF_ORDER",
        g920_gip_reasm_status_name(G920_GIP_REASM_OUT_OF_ORDER));
    TEST_ASSERT_EQUAL_STRING(
        "?", g920_gip_reasm_status_name((g920_gip_reasm_status_t)99));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_control_parse_transaction_900);
    RUN_TEST(test_control_parse_transaction_931);
    RUN_TEST(test_control_roundtrip);
    RUN_TEST(test_control_parse_rejects_short_and_null);

    RUN_TEST(test_build_ack_matches_transaction_900);
    RUN_TEST(test_build_ack_matches_transaction_931);
    RUN_TEST(test_build_ack_matches_transaction_942);
    RUN_TEST(test_build_ack_carries_expansion_index);
    RUN_TEST(test_build_ack_into_small_buffer);

    RUN_TEST(test_acme_policy);

    RUN_TEST(test_reassembly_of_the_90_byte_security_message);
    RUN_TEST(test_single_packet_message_is_not_reassembly);
    RUN_TEST(test_single_fragment_message_completes_immediately);

    RUN_TEST(test_fragment_without_start_is_orphan);
    RUN_TEST(test_wrong_offset_is_detected);
    RUN_TEST(test_sequence_change_mid_message_is_detected);
    RUN_TEST(test_new_first_fragment_restarts_assembly);
    RUN_TEST(test_payload_longer_than_declared_total);
    RUN_TEST(test_message_larger_than_buffer_is_refused_upfront);
    RUN_TEST(test_push_rejects_length_mismatch_and_null);
    RUN_TEST(test_reset_drops_partial_assembly);
    RUN_TEST(test_reasm_status_names);

    return UNITY_END();
}
