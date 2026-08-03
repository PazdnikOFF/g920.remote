/*
 * Последовательность хоста при появлении устройства.
 *
 * Вектора — из `docs/gip-official/txt/H001419 - Original GIP Spec.txt`:
 * таблица 4-26 (Metadata Request), таблицы 4-31/4-32 (Set Device State),
 * § 2.1 (повторы запроса метаданных), и трасса метаданных на 186 байт из
 * § 3.1, где подтверждаются только первый и последний фрагменты.
 *
 * Проверяется то, что нельзя проверить на живом руле, пока он молчит: что
 * хост говорит **ровно то и ровно тогда**, что предписано спекой.
 */

#include <stdint.h>
#include <string.h>

#include <unity.h>

#include "g920/gip_host.h"

static uint8_t metadata[256];
static g920_gip_host_t host;
static g920_gip_host_packet_t out[G920_GIP_HOST_OUT_MAX];

void setUp(void)
{
    memset(metadata, 0, sizeof(metadata));
    memset(out, 0, sizeof(out));
    g920_gip_host_init(&host, metadata, sizeof(metadata));
}

void tearDown(void) { }

static int feed(const uint8_t *packet, size_t length, uint64_t now_ms)
{
    return g920_gip_host_on_packet(&host, packet, length, now_ms, out,
                                   G920_GIP_HOST_OUT_MAX);
}

/* --- Hello → запрос метаданных ------------------------------------------ */

static void test_hello_asks_for_metadata(void)
{
    /* Hello: одиночный системный пакет, подтверждения не просит. */
    const uint8_t hello[] = { 0x02, 0x20, 0x05, 0x00 };
    /* Таблица 4-26: тип 0x04, флаги 0x20, Sequence ID 1, длина 0. */
    const uint8_t expected[] = { 0x04, 0x20, 0x01, 0x00 };

    TEST_ASSERT_EQUAL_INT(1, feed(hello, sizeof(hello), 0));
    TEST_ASSERT_EQUAL_UINT8(sizeof(expected), out[0].length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out[0].data, sizeof(expected));
    TEST_ASSERT_EQUAL_INT(G920_GIP_HOST_METADATA, host.state);
}

static void test_hello_repeats_reset_the_exchange(void)
{
    const uint8_t hello[] = { 0x02, 0x20, 0x05, 0x00 };

    TEST_ASSERT_EQUAL_INT(1, feed(hello, sizeof(hello), 0));
    TEST_ASSERT_EQUAL_UINT32(1, host.metadata_requests);

    /* Устройство передумало и объявилось заново — счётчик попыток начинается
     * с нуля, иначе четыре потерянных Hello закрыли бы обмен навсегда. */
    TEST_ASSERT_EQUAL_INT(1, feed(hello, sizeof(hello), 700));
    TEST_ASSERT_EQUAL_UINT32(1, host.metadata_requests);
    TEST_ASSERT_EQUAL_UINT32(2, host.hellos);
}

/* --- метаданные одним пакетом ------------------------------------------- */

static void test_single_metadata_starts_the_device(void)
{
    const uint8_t hello[] = { 0x02, 0x20, 0x05, 0x00 };
    const uint8_t response[] = { 0x04, 0x20, 0x01, 0x03, 0xAA, 0xBB, 0xCC };
    /* Таблица 4-31: тип 0x05, флаги 0x20, длина 1, состояние 0x00 = Start. */
    const uint8_t expected[] = { 0x05, 0x20, 0x01, 0x01, 0x00 };
    const uint8_t *blob;
    size_t length = 0;

    (void)feed(hello, sizeof(hello), 0);

    TEST_ASSERT_EQUAL_INT(1, feed(response, sizeof(response), 10));
    TEST_ASSERT_EQUAL_UINT8(sizeof(expected), out[0].length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out[0].data, sizeof(expected));
    TEST_ASSERT_EQUAL_INT(G920_GIP_HOST_ACTIVE, host.state);

    blob = g920_gip_host_metadata(&host, &length);
    TEST_ASSERT_NOT_NULL(blob);
    TEST_ASSERT_EQUAL_UINT32(3, length);
    TEST_ASSERT_EQUAL_HEX8(0xAA, blob[0]);
    TEST_ASSERT_EQUAL_HEX8(0xCC, blob[2]);
}

static void test_acme_is_acknowledged_before_start(void)
{
    const uint8_t hello[] = { 0x02, 0x20, 0x05, 0x00 };
    /* Тот же ответ, но с флагом ACME (0x10) — «подтверди меня». */
    const uint8_t response[] = { 0x04, 0x30, 0x07, 0x02, 0x11, 0x22 };

    (void)feed(hello, sizeof(hello), 0);
    TEST_ASSERT_EQUAL_INT(2, feed(response, sizeof(response), 10));

    /* Первым идёт подтверждение: Protocol Control, 13 байт. */
    TEST_ASSERT_EQUAL_UINT8(13, out[0].length);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[0].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x20, out[0].data[1]);
    /* Sequence ID подтверждения обязан совпадать с подтверждаемым. */
    TEST_ASSERT_EQUAL_HEX8(0x07, out[0].data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x09, out[0].data[3]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[0].data[4]); /* ControlCode: ACK */
    TEST_ASSERT_EQUAL_HEX8(0x04, out[0].data[5]); /* RefMessageType */

    /* Вторым — Start. */
    TEST_ASSERT_EQUAL_HEX8(0x05, out[1].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[1].data[4]);
}

/* --- метаданные фрагментами --------------------------------------------- */

static void test_fragmented_metadata(void)
{
    const uint8_t hello[] = { 0x02, 0x20, 0x05, 0x00 };
    /* Первый фрагмент: Fragment|InitFrag|System|ACME, 6 байт из 10. */
    const uint8_t first[] = { 0x04, 0xF0, 0x02, 0x06, 0x0A,
                              0x01, 0x02, 0x03, 0x04, 0x05, 0x06 };
    /* Второй: Fragment|System, смещение 6, оставшиеся 4 байта. ACME нет —
     * по трассе середина сообщения подтверждения не просит. */
    const uint8_t second[] = { 0x04, 0xA0, 0x02, 0x04, 0x06,
                               0x07, 0x08, 0x09, 0x0A };
    const uint8_t *blob;
    size_t length = 0;

    (void)feed(hello, sizeof(hello), 0);

    /* Первый фрагмент просит подтверждения и получает его: принято 6 байт,
     * осталось 4. Start ещё рано — сообщение не собрано. */
    TEST_ASSERT_EQUAL_INT(1, feed(first, sizeof(first), 10));
    /* Раскладка ACK: 4 байта заголовка, затем ControlCode, RefMessageType,
     * RefMessageFlags, смещение (32 бита LE) и остаток буфера (16 бит LE). */
    TEST_ASSERT_EQUAL_HEX8(0x01, out[0].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x04, out[0].data[5]); /* RefMessageType */
    TEST_ASSERT_EQUAL_HEX8(0x20, out[0].data[6]); /* RefMessageFlags: System */
    TEST_ASSERT_EQUAL_HEX8(0x06, out[0].data[7]); /* принято 6 байт */
    TEST_ASSERT_EQUAL_HEX8(0x04, out[0].data[11]); /* осталось 4 */
    TEST_ASSERT_EQUAL_INT(G920_GIP_HOST_METADATA, host.state);

    /*
     * Второй досбирает сообщение — но Start ещё **рано**. Транзакция
     * закрывается нулевым completion-фрагментом, и ранний Start живой руль
     * отвергает: 02.08.2026 он в ответ вернулся в Arrival и прислал Hello
     * заново. Вектор из спеки этого не показывал — реакции устройства в
     * трассе нет, есть только байты.
     */
    TEST_ASSERT_EQUAL_INT(0, feed(second, sizeof(second), 20));
    TEST_ASSERT_EQUAL_INT(G920_GIP_HOST_METADATA, host.state);
    TEST_ASSERT_TRUE(host.metadata_ready);

    blob = g920_gip_host_metadata(&host, &length);
    TEST_ASSERT_EQUAL_UINT32(10, length);
    TEST_ASSERT_EQUAL_HEX8(0x01, blob[0]);
    TEST_ASSERT_EQUAL_HEX8(0x0A, blob[9]);

    /* Нулевой completion-фрагмент с ACME: подтверждаем — и вот теперь Start. */
    {
        const uint8_t completion[] = { 0x04, 0xB0, 0x02, 0x00, 0x0A };

        TEST_ASSERT_EQUAL_INT(2, feed(completion, sizeof(completion), 30));
        TEST_ASSERT_EQUAL_HEX8(0x01, out[0].data[0]); /* ACK */
        TEST_ASSERT_EQUAL_HEX8(0x0A, out[0].data[7]); /* принято 10 байт */
        TEST_ASSERT_EQUAL_HEX8(0x05, out[1].data[0]); /* Set Device State */
        TEST_ASSERT_EQUAL_HEX8(0x00, out[1].data[4]); /* Start */
        TEST_ASSERT_EQUAL_INT(G920_GIP_HOST_ACTIVE, host.state);
    }
}

static void test_start_goes_out_even_if_completion_is_lost(void)
{
    const uint8_t hello[] = { 0x02, 0x20, 0x05, 0x00 };
    const uint8_t first[] = { 0x04, 0xF0, 0x02, 0x06, 0x0A,
                              0x01, 0x02, 0x03, 0x04, 0x05, 0x06 };
    const uint8_t second[] = { 0x04, 0xA0, 0x02, 0x04, 0x06,
                               0x07, 0x08, 0x09, 0x0A };

    (void)feed(hello, sizeof(hello), 0);
    (void)feed(first, sizeof(first), 10);
    (void)feed(second, sizeof(second), 20);

    /* Ждём завершения транзакции, но не бесконечно: метаданные уже все у
     * нас, а устройство без Start останется в Idle навсегда. */
    TEST_ASSERT_EQUAL_INT(
        0, g920_gip_host_tick(&host, 20 + G920_GIP_COMPLETION_WAIT_MS - 1, out,
                              G920_GIP_HOST_OUT_MAX));
    TEST_ASSERT_EQUAL_INT(
        1, g920_gip_host_tick(&host, 20 + G920_GIP_COMPLETION_WAIT_MS, out,
                              G920_GIP_HOST_OUT_MAX));
    TEST_ASSERT_EQUAL_HEX8(0x05, out[0].data[0]);
    TEST_ASSERT_EQUAL_INT(G920_GIP_HOST_ACTIVE, host.state);
}

/* --- повторы запроса ----------------------------------------------------- */

static void test_metadata_request_is_repeated_up_to_four_times(void)
{
    const uint8_t hello[] = { 0x02, 0x20, 0x05, 0x00 };
    uint64_t now = 0;

    TEST_ASSERT_EQUAL_INT(1, feed(hello, sizeof(hello), now));
    TEST_ASSERT_EQUAL_UINT32(1, host.metadata_requests);

    /* Раньше 500 мс повторять нечего. */
    TEST_ASSERT_EQUAL_INT(0, g920_gip_host_tick(&host, now + 499, out,
                                                G920_GIP_HOST_OUT_MAX));

    for (uint32_t i = 2; i <= G920_GIP_METADATA_RETRY_MAX; i++) {
        now += G920_GIP_METADATA_RETRY_MS;
        TEST_ASSERT_EQUAL_INT(
            1, g920_gip_host_tick(&host, now, out, G920_GIP_HOST_OUT_MAX));
        TEST_ASSERT_EQUAL_UINT32(i, host.metadata_requests);
    }

    /* Спека: после четырёх попыток хост помечает устройство на удаление, а
     * не долбит его вечно. */
    now += G920_GIP_METADATA_RETRY_MS;
    TEST_ASSERT_EQUAL_INT(
        0, g920_gip_host_tick(&host, now, out, G920_GIP_HOST_OUT_MAX));
}

static void test_active_device_is_asked_about_itself_once(void)
{
    const uint8_t hello[] = { 0x02, 0x20, 0x05, 0x00 };
    const uint8_t response[] = { 0x04, 0x20, 0x01, 0x01, 0x42 };
    /*
     * H001861: запрос начальных отчётов. Флаги 0x00 — сообщение не
     * системное; длина нагрузки всегда 3, даже когда значащий байт один.
     * Ответом руль присылает статическую конфигурацию 0x21 — разрядности
     * осей, маску FFB и пределы угла, которых нет больше нигде.
     */
    const uint8_t expected[] = { 0x0A, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00 };

    (void)feed(hello, sizeof(hello), 0);
    (void)feed(response, sizeof(response), 10);
    TEST_ASSERT_EQUAL_INT(G920_GIP_HOST_ACTIVE, host.state);

    TEST_ASSERT_EQUAL_INT(
        1, g920_gip_host_tick(&host, 5000, out, G920_GIP_HOST_OUT_MAX));
    TEST_ASSERT_EQUAL_UINT8(sizeof(expected), out[0].length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out[0].data, sizeof(expected));

    /* И только один раз: спека не обещает повторного ответа, а повторять
     * запрос к работающему устройству значит мерить свою настойчивость. */
    TEST_ASSERT_EQUAL_INT(
        0, g920_gip_host_tick(&host, 6000, out, G920_GIP_HOST_OUT_MAX));
}

/* --- прочий трафик -------------------------------------------------------- */

static void test_input_reports_need_no_answer(void)
{
    const uint8_t hello[] = { 0x02, 0x20, 0x05, 0x00 };
    const uint8_t response[] = { 0x04, 0x20, 0x01, 0x01, 0x42 };
    /* Ввод: класс 1 (low latency), номер 0x20, без ACME. */
    const uint8_t input[] = { 0x20, 0x00, 0x11, 0x02, 0x00, 0x00 };

    (void)feed(hello, sizeof(hello), 0);
    (void)feed(response, sizeof(response), 10);

    TEST_ASSERT_EQUAL_INT(0, feed(input, sizeof(input), 20));
    TEST_ASSERT_EQUAL_UINT32(1, host.inputs);
}

static void test_garbage_is_not_answered(void)
{
    /* Поле длины тянется дольше четырёх байт — заголовок негоден. */
    const uint8_t garbage[] = { 0x04, 0x20, 0x01, 0xFF, 0xFF,
                                0xFF, 0xFF, 0xFF, 0xFF };

    TEST_ASSERT_EQUAL_INT(0, feed(garbage, sizeof(garbage), 0));
    TEST_ASSERT_EQUAL_INT(G920_GIP_HOST_ARRIVAL, host.state);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_hello_asks_for_metadata);
    RUN_TEST(test_hello_repeats_reset_the_exchange);
    RUN_TEST(test_single_metadata_starts_the_device);
    RUN_TEST(test_acme_is_acknowledged_before_start);
    RUN_TEST(test_fragmented_metadata);
    RUN_TEST(test_start_goes_out_even_if_completion_is_lost);
    RUN_TEST(test_metadata_request_is_repeated_up_to_four_times);
    RUN_TEST(test_active_device_is_asked_about_itself_once);
    RUN_TEST(test_input_reports_need_no_answer);
    RUN_TEST(test_garbage_is_not_answered);
    return UNITY_END();
}
