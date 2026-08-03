#include <stdint.h>
#include <string.h>

#include <unity.h>

#include "g920/hostlog.h"

#define CAPACITY 32

static g920_host_event_t storage[CAPACITY];
static g920_hostlog_t trace;

void setUp(void)
{
    memset(storage, 0, sizeof(storage));
    g920_hostlog_init(&trace, storage, CAPACITY);
}

void tearDown(void) { }

/* Правдоподобная трасса энумерации: время в микросекундах. */
static void enumerate(void)
{
    g920_hostlog_mark_origin(&trace, 1000000);
    g920_hostlog_add(&trace, 1000000, G920_HOST_EV_VBUS);
    g920_hostlog_add(&trace, 1120000, G920_HOST_EV_BUS_RESET);
    /* GET_DESCRIPTOR DEVICE, первые 8 байт */
    g920_hostlog_add_setup(&trace, 1121000, 0x80, 0x06, 0x0100, 0x0000, 8,
                           G920_HOST_ANSWER_DATA);
    g920_hostlog_add_setup(&trace, 1130000, 0x00, 0x05, 0x0002, 0x0000, 0,
                           G920_HOST_ANSWER_DATA); /* SET_ADDRESS */
    g920_hostlog_add_setup(&trace, 1140000, 0x80, 0x06, 0x0100, 0x0000, 18,
                           G920_HOST_ANSWER_DATA);
    /* Device Qualifier — виндовизм, отвечаем STALL */
    g920_hostlog_add_setup(&trace, 1145000, 0x80, 0x06, 0x0600, 0x0000, 10,
                           G920_HOST_ANSWER_STALL);
    g920_hostlog_add_setup(&trace, 1150000, 0x80, 0x06, 0x0200, 0x0000, 32,
                           G920_HOST_ANSWER_DATA);
    /* Строка 0xEE — MSFT100 */
    g920_hostlog_add_setup(&trace, 1155000, 0x80, 0x06, 0x03EE, 0x0000, 18,
                           G920_HOST_ANSWER_DATA);
    g920_hostlog_add_setup(&trace, 1160000, 0x00, 0x09, 0x0001, 0x0000, 0,
                           G920_HOST_ANSWER_DATA); /* SET_CONFIGURATION */
    g920_hostlog_add(&trace, 1164000, G920_HOST_EV_IN_POLLED);
}

/* --- точка отсчёта ------------------------------------------------------ */

static void test_origin_is_vbus_not_bus_reset(void)
{
    /* Инвариант И4: отсчёт от появления питания. Bus reset приходит позже
     * и обязан оказаться уже со смещением, а не в нуле. */
    enumerate();

    const g920_host_event_t *vbus = g920_hostlog_at(&trace, 0);
    const g920_host_event_t *reset = g920_hostlog_at(&trace, 1);

    TEST_ASSERT_EQUAL_UINT8(G920_HOST_EV_VBUS, vbus->type);
    TEST_ASSERT_EQUAL_UINT32(0, vbus->at_us);
    TEST_ASSERT_EQUAL_UINT8(G920_HOST_EV_BUS_RESET, reset->type);
    TEST_ASSERT_EQUAL_UINT32(120000, reset->at_us);
    TEST_ASSERT_FALSE(trace.origin_implicit);
}

static void test_event_before_origin_sets_it_and_flags_it(void)
{
    /* Если первое событие пришло раньше отметки VBUS, отсчёт всё равно
     * встанет — но с пометкой, что начало трассы могло быть пропущено. */
    TEST_ASSERT_TRUE(g920_hostlog_add(&trace, 500000, G920_HOST_EV_BUS_RESET));
    TEST_ASSERT_TRUE(trace.origin_implicit);
    TEST_ASSERT_EQUAL_UINT32(0, g920_hostlog_at(&trace, 0)->at_us);

    TEST_ASSERT_TRUE(g920_hostlog_add(&trace, 510000, G920_HOST_EV_IN_POLLED));
    TEST_ASSERT_EQUAL_UINT32(10000, g920_hostlog_at(&trace, 1)->at_us);
}

static void test_explicit_origin_clears_implicit_flag(void)
{
    g920_hostlog_add(&trace, 500000, G920_HOST_EV_BUS_RESET);
    TEST_ASSERT_TRUE(trace.origin_implicit);

    g920_hostlog_mark_origin(&trace, 600000);
    TEST_ASSERT_FALSE(trace.origin_implicit);
}

static void test_time_going_backwards_clamps_to_zero(void)
{
    g920_hostlog_mark_origin(&trace, 1000000);
    TEST_ASSERT_TRUE(g920_hostlog_add(&trace, 900000, G920_HOST_EV_BUS_RESET));
    TEST_ASSERT_EQUAL_UINT32(0, g920_hostlog_at(&trace, 0)->at_us);
}

/* --- переполнение ------------------------------------------------------- */

static void test_overflow_keeps_the_head_and_counts_the_tail(void)
{
    /* Кольцевой буфер здесь был бы вреден: в фингерпринте ценно начало. */
    g920_host_event_t small_storage[4];
    g920_hostlog_t small;

    g920_hostlog_init(&small, small_storage, 4);
    g920_hostlog_mark_origin(&small, 0);

    for (int i = 0; i < 10; i++) {
        bool ok = g920_hostlog_add_setup(&small, (uint64_t)(i * 1000), 0x80,
                                         0x06, (uint16_t)(0x0100 + i), 0, 8,
                                         G920_HOST_ANSWER_DATA);
        TEST_ASSERT_EQUAL_INT(i < 4, ok);
    }

    TEST_ASSERT_EQUAL_UINT16(4, g920_hostlog_count(&small));
    TEST_ASSERT_EQUAL_UINT16(6, g920_hostlog_dropped(&small));
    /* Первые четыре — те самые первые, а не последние. */
    TEST_ASSERT_EQUAL_HEX16(0x0100, g920_hostlog_at(&small, 0)->w_value);
    TEST_ASSERT_EQUAL_HEX16(0x0103, g920_hostlog_at(&small, 3)->w_value);
    TEST_ASSERT_NULL(g920_hostlog_at(&small, 4));
}

static void test_no_storage_is_survivable(void)
{
    g920_hostlog_t none;

    g920_hostlog_init(&none, NULL, 16);
    TEST_ASSERT_FALSE(g920_hostlog_add(&none, 0, G920_HOST_EV_VBUS));
    TEST_ASSERT_EQUAL_UINT16(0, g920_hostlog_count(&none));
}

/* --- замеры ------------------------------------------------------------- */

static void test_t1_is_first_host_activity_after_vbus(void)
{
    uint32_t t1 = 0;

    enumerate();
    TEST_ASSERT_TRUE(g920_hostlog_t1_us(&trace, &t1));
    /* Сам VBUS активностью не считается: первый признак жизни — bus reset
     * через 120 мс. */
    TEST_ASSERT_EQUAL_UINT32(120000, t1);
}

static void test_t1_absent_while_host_is_silent(void)
{
    uint32_t t1 = 0;

    g920_hostlog_mark_origin(&trace, 1000000);
    g920_hostlog_add(&trace, 1000000, G920_HOST_EV_VBUS);
    /* Тишина — это и есть признак ПК по решающему дереву M6. Функция
     * обязана честно сказать «активности не было». */
    TEST_ASSERT_FALSE(g920_hostlog_t1_us(&trace, &t1));
}

static void test_t2_is_measured_from_set_configuration(void)
{
    uint32_t t2 = 0;

    enumerate();
    TEST_ASSERT_TRUE(g920_hostlog_t2_us(&trace, &t2));
    TEST_ASSERT_EQUAL_UINT32(4000, t2); /* 1164000 − 1160000 */
}

static void test_t2_absent_without_polling(void)
{
    uint32_t t2 = 0;

    g920_hostlog_mark_origin(&trace, 0);
    g920_hostlog_add_setup(&trace, 1000, 0x00, 0x09, 1, 0, 0,
                           G920_HOST_ANSWER_DATA);
    /* Конфигурация есть, опроса нет — ровно тот случай, по которому в M6
     * происходит откат из HID обратно в GIP. */
    TEST_ASSERT_FALSE(g920_hostlog_t2_us(&trace, &t2));
}

static void test_t2_ignores_polling_before_configuration(void)
{
    uint32_t t2 = 0;

    g920_hostlog_mark_origin(&trace, 0);
    g920_hostlog_add(&trace, 1000, G920_HOST_EV_IN_POLLED);
    g920_hostlog_add_setup(&trace, 2000, 0x00, 0x09, 1, 0, 0,
                           G920_HOST_ANSWER_DATA);
    g920_hostlog_add(&trace, 5000, G920_HOST_EV_IN_POLLED);

    TEST_ASSERT_TRUE(g920_hostlog_t2_us(&trace, &t2));
    TEST_ASSERT_EQUAL_UINT32(3000, t2);
}

/* --- признаки ------------------------------------------------------------ */

static void test_request_counts_and_descriptor_probes(void)
{
    enumerate();

    /* Пять GET_DESCRIPTOR: DEVICE дважды (сначала 8 байт, потом 18),
     * QUALIFIER, CONFIG, строка 0xEE. Двойной запрос DEVICE — часть
     * фингерпринта: стеки различаются в том числе этим. */
    TEST_ASSERT_EQUAL_UINT16(5, g920_hostlog_count_request(
                                    &trace, G920_USB_REQ_GET_DESCRIPTOR));
    TEST_ASSERT_EQUAL_UINT16(
        1, g920_hostlog_count_request(&trace, G920_USB_REQ_SET_ADDRESS));
    TEST_ASSERT_EQUAL_UINT16(
        1, g920_hostlog_count_request(&trace, G920_USB_REQ_SET_CONFIGURATION));

    TEST_ASSERT_TRUE(g920_hostlog_saw_descriptor(&trace, 0x01)); /* DEVICE */
    TEST_ASSERT_TRUE(g920_hostlog_saw_descriptor(&trace, 0x02)); /* CONFIG */
    TEST_ASSERT_TRUE(g920_hostlog_saw_descriptor(&trace, 0x06)); /* QUALIFIER */
    TEST_ASSERT_TRUE(g920_hostlog_saw_descriptor(&trace, 0x03)); /* STRING */
    TEST_ASSERT_FALSE(g920_hostlog_saw_descriptor(&trace, 0x0F));
}

static void test_stalls_are_counted(void)
{
    enumerate();
    /* Один STALL — на Device Qualifier. */
    TEST_ASSERT_EQUAL_UINT16(1, g920_hostlog_count_stalls(&trace));
}

static void test_saw_event(void)
{
    enumerate();
    TEST_ASSERT_TRUE(g920_hostlog_saw_event(&trace, G920_HOST_EV_IN_POLLED));
    TEST_ASSERT_TRUE(g920_hostlog_saw_event(&trace, G920_HOST_EV_SET_CONFIG));
    TEST_ASSERT_FALSE(g920_hostlog_saw_event(&trace, G920_HOST_EV_GIP_IN));
    TEST_ASSERT_FALSE(g920_hostlog_saw_event(&trace, G920_HOST_EV_SUSPEND));
}

static void test_gip_events(void)
{
    g920_hostlog_mark_origin(&trace, 0);
    /* Security-сообщение от хоста — единственный сигнал с правом вето (И5). */
    TEST_ASSERT_TRUE(g920_hostlog_add_gip(&trace, 5000, true, 0x06, 14));
    TEST_ASSERT_TRUE(g920_hostlog_add_gip(&trace, 6000, false, 0x01, 9));

    TEST_ASSERT_TRUE(g920_hostlog_saw_event(&trace, G920_HOST_EV_GIP_IN));
    TEST_ASSERT_EQUAL_HEX8(0x06, g920_hostlog_at(&trace, 0)->b_request);
    TEST_ASSERT_EQUAL_UINT16(14, g920_hostlog_at(&trace, 0)->w_length);
    TEST_ASSERT_EQUAL_UINT8(G920_HOST_EV_GIP_OUT,
                            g920_hostlog_at(&trace, 1)->type);
}

/* --- сброс --------------------------------------------------------------- */

static void test_reset_clears_records_and_origin(void)
{
    enumerate();
    TEST_ASSERT_TRUE(g920_hostlog_count(&trace) > 0);

    g920_hostlog_reset(&trace);
    TEST_ASSERT_EQUAL_UINT16(0, g920_hostlog_count(&trace));
    TEST_ASSERT_EQUAL_UINT16(0, g920_hostlog_dropped(&trace));
    TEST_ASSERT_FALSE(trace.origin_set);
}

/* --- вывод --------------------------------------------------------------- */

static void test_format_setup_line(void)
{
    char buf[G920_HOSTLOG_LINE_MAX];
    g920_host_event_t ev = {
        .at_us = 1234,
        .type = G920_HOST_EV_SETUP,
        .answer = G920_HOST_ANSWER_DATA,
        .bm_request_type = 0x80,
        .b_request = 0x06,
        .w_value = 0x0100,
        .w_index = 0x0000,
        .w_length = 18,
    };

    TEST_ASSERT_TRUE(g920_hostlog_format(buf, sizeof(buf), &ev) > 0);
    TEST_ASSERT_EQUAL_STRING("0.001234 SETUP 80 06 0100 0000 len 18", buf);
}

static void test_format_marks_stall(void)
{
    char buf[G920_HOSTLOG_LINE_MAX];
    g920_host_event_t ev = {
        .at_us = 0,
        .type = G920_HOST_EV_SETUP,
        .answer = G920_HOST_ANSWER_STALL,
        .bm_request_type = 0x80,
        .b_request = 0x06,
        .w_value = 0x0600,
        .w_index = 0x0000,
        .w_length = 10,
    };

    TEST_ASSERT_TRUE(g920_hostlog_format(buf, sizeof(buf), &ev) > 0);
    TEST_ASSERT_EQUAL_STRING("0.000000 SETUP 80 06 0600 0000 len 10 STALL",
                             buf);
}

static void test_format_simple_and_gip(void)
{
    char buf[G920_HOSTLOG_LINE_MAX];
    g920_host_event_t vbus = { .at_us = 0, .type = G920_HOST_EV_VBUS };
    g920_host_event_t gip = { .at_us = 2000000,
                              .type = G920_HOST_EV_GIP_IN,
                              .b_request = 0x06,
                              .w_length = 14 };

    TEST_ASSERT_TRUE(g920_hostlog_format(buf, sizeof(buf), &vbus) > 0);
    TEST_ASSERT_EQUAL_STRING("0.000000 VBUS", buf);

    TEST_ASSERT_TRUE(g920_hostlog_format(buf, sizeof(buf), &gip) > 0);
    TEST_ASSERT_EQUAL_STRING("2.000000 GIP_IN type 06 len 14", buf);
}

static void test_format_worst_case_fits_declared_line(void)
{
    char buf[G920_HOSTLOG_LINE_MAX];
    g920_host_event_t ev = {
        .at_us = 0xFFFFFFFFu,
        .type = G920_HOST_EV_SET_ADDRESS,
        .answer = G920_HOST_ANSWER_STALL,
        .bm_request_type = 0xFF,
        .b_request = 0xFF,
        .w_value = 0xFFFF,
        .w_index = 0xFFFF,
        .w_length = 0xFFFF,
    };
    int len = g920_hostlog_format(buf, sizeof(buf), &ev);

    TEST_ASSERT_TRUE(len > 0);
    TEST_ASSERT_TRUE((size_t)len + 1u <= (size_t)G920_HOSTLOG_LINE_MAX);
    TEST_ASSERT_EQUAL_size_t((size_t)len, strlen(buf));
}

static void test_format_rejects_small_buffer(void)
{
    char buf[G920_HOSTLOG_LINE_MAX];
    g920_host_event_t ev = { .at_us = 0, .type = G920_HOST_EV_VBUS };

    TEST_ASSERT_EQUAL_INT(-1, g920_hostlog_format(buf, 8, &ev));
    TEST_ASSERT_EQUAL_INT(-1, g920_hostlog_format(NULL, 64, &ev));
    TEST_ASSERT_EQUAL_INT(-1, g920_hostlog_format(buf, 64, NULL));
}

static void test_event_names(void)
{
    TEST_ASSERT_EQUAL_STRING("VBUS", g920_host_event_name(G920_HOST_EV_VBUS));
    TEST_ASSERT_EQUAL_STRING("SET_CFG",
                             g920_host_event_name(G920_HOST_EV_SET_CONFIG));
    TEST_ASSERT_EQUAL_STRING("?",
                             g920_host_event_name((g920_host_event_type_t)99));
}


/*
 * Windows и Xbox настраивают устройство **дважды**, с переэнумерацией
 * между. Отсчёт T2 от первой настройки давал 301 мс вместо 8.6 — число,
 * описывающее привычку хоста переподключать устройство, а не его задержку.
 */
static void test_t2_counts_from_the_last_configuration(void)
{
    g920_hostlog_init(&trace, storage, CAPACITY);
    g920_hostlog_mark_origin(&trace, 0);

    /* Первая настройка, потом весь второй проход энумерации. */
    g920_hostlog_add_setup(&trace, 100000, 0x00, G920_USB_REQ_SET_CONFIGURATION,
                           1, 0, 0, G920_HOST_ANSWER_NONE);
    g920_hostlog_add_setup(&trace, 200000, 0x80, G920_USB_REQ_GET_DESCRIPTOR,
                           0x0100, 0, 18, G920_HOST_ANSWER_DATA);
    /* Вторая настройка — от неё и считаем. */
    g920_hostlog_add_setup(&trace, 300000, 0x00, G920_USB_REQ_SET_CONFIGURATION,
                           1, 0, 0, G920_HOST_ANSWER_NONE);
    g920_hostlog_add(&trace, 310000, G920_HOST_EV_IN_POLLED);

    uint32_t t2 = 0;

    TEST_ASSERT_TRUE(g920_hostlog_t2_us(&trace, &t2));
    /* 10 мс от последней настройки, а не 210 мс от первой. */
    TEST_ASSERT_EQUAL_UINT32(10000, t2);
}

/*
 * `bRequest` уникален только среди стандартных запросов. Vendor-код MS OS
 * в M4 станет параметром личности, и запрос с кодом 9 не должен лечь в
 * журнал как SET_CONFIGURATION: он испортил бы и T2, и фингерпринт.
 */
static void test_vendor_request_is_not_mistaken_for_a_standard_one(void)
{
    g920_hostlog_init(&trace, storage, CAPACITY);
    g920_hostlog_mark_origin(&trace, 0);

    /* 0xC0 = vendor | device. Код совпадает с SET_CONFIGURATION. */
    g920_hostlog_add_setup(&trace, 1000, 0xC0, 0x09, 0, 4, 40,
                           G920_HOST_ANSWER_DATA);
    g920_hostlog_add(&trace, 2000, G920_HOST_EV_IN_POLLED);

    {
        uint32_t t2 = 0;

        /* Настройки не было — значит и T2 нет. */
        TEST_ASSERT_FALSE(g920_hostlog_t2_us(&trace, &t2));
    }
    TEST_ASSERT_FALSE(g920_hostlog_saw_event(&trace, G920_HOST_EV_SET_CONFIG));

    /* А настоящий стандартный запрос с тем же кодом — распознаётся. */
    g920_hostlog_add_setup(&trace, 3000, 0x00, G920_USB_REQ_SET_CONFIGURATION,
                           1, 0, 0, G920_HOST_ANSWER_NONE);
    TEST_ASSERT_TRUE(g920_hostlog_saw_event(&trace, G920_HOST_EV_SET_CONFIG));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_t2_counts_from_the_last_configuration);
    RUN_TEST(test_vendor_request_is_not_mistaken_for_a_standard_one);

    RUN_TEST(test_origin_is_vbus_not_bus_reset);
    RUN_TEST(test_event_before_origin_sets_it_and_flags_it);
    RUN_TEST(test_explicit_origin_clears_implicit_flag);
    RUN_TEST(test_time_going_backwards_clamps_to_zero);

    RUN_TEST(test_overflow_keeps_the_head_and_counts_the_tail);
    RUN_TEST(test_no_storage_is_survivable);

    RUN_TEST(test_t1_is_first_host_activity_after_vbus);
    RUN_TEST(test_t1_absent_while_host_is_silent);
    RUN_TEST(test_t2_is_measured_from_set_configuration);
    RUN_TEST(test_t2_absent_without_polling);
    RUN_TEST(test_t2_ignores_polling_before_configuration);

    RUN_TEST(test_request_counts_and_descriptor_probes);
    RUN_TEST(test_stalls_are_counted);
    RUN_TEST(test_saw_event);
    RUN_TEST(test_gip_events);

    RUN_TEST(test_reset_clears_records_and_origin);

    RUN_TEST(test_format_setup_line);
    RUN_TEST(test_format_marks_stall);
    RUN_TEST(test_format_simple_and_gip);
    RUN_TEST(test_format_worst_case_fits_declared_line);
    RUN_TEST(test_format_rejects_small_buffer);
    RUN_TEST(test_event_names);

    return UNITY_END();
}
