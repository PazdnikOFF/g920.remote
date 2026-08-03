#include <stdint.h>
#include <string.h>

#include <unity.h>

#include "g920/link_stats.h"

/*
 * Этот модуль производит **все** числа вехи M3 — RTT, джиттер, долю потерь.
 * Он и вынесен-то в common ровно затем, чтобы его можно было проверить на
 * хосте, а тестов у него до сих пор не было ни одного: ошибку в оценке
 * джиттера по логу на плате не заметить.
 */

static g920_link_stats_t stats;

void setUp(void)
{
    g920_link_stats_init(&stats);
}

void tearDown(void) { }

static void test_fresh_stats_are_empty(void)
{
    TEST_ASSERT_EQUAL_UINT32(0, stats.sent);
    TEST_ASSERT_EQUAL_UINT32(0, stats.received);
    TEST_ASSERT_EQUAL_UINT32(0, g920_link_stats_avg_rtt_us(&stats));
    /* Ни одного кадра — ни одной потери. Ноль на ноль здесь не «0%», а
     * «нечего считать», и путать их нельзя. */
    TEST_ASSERT_EQUAL_UINT32(0, g920_link_stats_loss_permille(&stats));
}

static void test_rtt_min_avg_max(void)
{
    g920_link_stats_on_rtt(&stats, 2300);
    g920_link_stats_on_rtt(&stats, 2500);
    g920_link_stats_on_rtt(&stats, 16000);

    TEST_ASSERT_EQUAL_UINT32(2300, stats.rtt_min_us);
    TEST_ASSERT_EQUAL_UINT32(16000, stats.rtt_max_us);
    TEST_ASSERT_EQUAL_UINT32((2300 + 2500 + 16000) / 3,
                             g920_link_stats_avg_rtt_us(&stats));
}

static void test_jitter_is_smoothed_not_peak(void)
{
    /* Ровный поток: разброса нет. */
    for (int i = 0; i < 100; i++) {
        g920_link_stats_on_rtt(&stats, 2300);
    }
    TEST_ASSERT_EQUAL_UINT32(0, stats.jitter_us);

    /*
     * Один выброс не обязан навсегда испортить оценку: по RFC 3550
     * J += (|D| − J)/16, то есть шаг в одну шестнадцатую от разницы, а не
     * «макс минус мин».
     */
    g920_link_stats_on_rtt(&stats, 18300);
    TEST_ASSERT_EQUAL_UINT32(16000u / 16u, stats.jitter_us);

    /* И возвращается обратно, а не залипает на пике. */
    for (int i = 0; i < 200; i++) {
        g920_link_stats_on_rtt(&stats, 2300);
    }
    TEST_ASSERT_TRUE(stats.jitter_us < 100);
}

static void test_jitter_sees_systematic_shake(void)
{
    /* Систематическое дрожание обязано быть видно, в отличие от выброса. */
    for (int i = 0; i < 200; i++) {
        g920_link_stats_on_rtt(&stats, (i % 2 == 0) ? 2000 : 6000);
    }
    TEST_ASSERT_TRUE(stats.jitter_us > 3000);
}

static void test_loss_is_counted_against_sent(void)
{
    for (int i = 0; i < 1000; i++) {
        g920_link_stats_on_sent(&stats, true);
    }
    g920_link_stats_add_rx(&stats, 968, 0, 0);

    /* 32 из 1000 — 3.2%, то есть 32 промилле. */
    TEST_ASSERT_EQUAL_UINT32(32, g920_link_stats_loss_permille(&stats));
}

static void test_more_received_than_sent_is_not_negative_loss(void)
{
    /*
     * Развёртка идёт по кругу, и ступень после перегрузки ловит хвост чужой
     * очереди: принято больше, чем отправлено. Это не «−4% потерь», это
     * ноль.
     */
    g920_link_stats_on_sent(&stats, true);
    g920_link_stats_add_rx(&stats, 5, 0, 0);
    TEST_ASSERT_EQUAL_UINT32(0, g920_link_stats_loss_permille(&stats));
}

static void test_send_failures_are_separate_from_loss(void)
{
    /* Отказ драйвера — не потеря радио: кадр не вышел в эфир вообще. */
    g920_link_stats_on_sent(&stats, true);
    g920_link_stats_on_sent(&stats, false);

    TEST_ASSERT_EQUAL_UINT32(2, stats.sent);
    TEST_ASSERT_EQUAL_UINT32(1, stats.send_failed);
}

static void test_format_has_every_number(void)
{
    char line[G920_LINK_STATS_STR_MAX];

    g920_link_stats_on_sent(&stats, true);
    g920_link_stats_on_sent(&stats, false);
    g920_link_stats_on_rtt(&stats, 2300);
    g920_link_stats_add_rx(&stats, 1, 2, 3);

    TEST_ASSERT_TRUE(g920_link_stats_format(line, sizeof(line), &stats) > 0);
    TEST_ASSERT_EQUAL_STRING(
        "sent 2 rx 1 loss 50.0% rtt 2300/2300/2300us jit 0us dup 2 stale 3 "
        "txfail 1",
        line);
}

static void test_format_refuses_a_small_buffer(void)
{
    char line[16];

    /* Обрезанная строка замера хуже отсутствующей: её прочитают как
     * настоящую. */
    TEST_ASSERT_EQUAL_INT(-1, g920_link_stats_format(line, sizeof(line),
                                                     &stats));
    TEST_ASSERT_EQUAL_INT(-1, g920_link_stats_format(NULL, 128, &stats));
    TEST_ASSERT_EQUAL_INT(-1, g920_link_stats_format(line, sizeof(line), NULL));
}

static void test_format_survives_huge_counters(void)
{
    char line[G920_LINK_STATS_STR_MAX];
    int written;

    /*
     * Писатели раньше не знали границы вовсе: при разросшихся счётчиках это
     * переполнение стека главной задачи. Худший случай — все поля по
     * десять знаков.
     */
    stats.sent = UINT32_MAX;
    stats.received = UINT32_MAX;
    stats.duplicates = UINT32_MAX;
    stats.stale = UINT32_MAX;
    stats.send_failed = UINT32_MAX;
    stats.rtt_min_us = UINT32_MAX;
    stats.rtt_max_us = UINT32_MAX;
    stats.rtt_sum_us = UINT32_MAX;
    stats.rtt_count = 1;
    stats.jitter_us = UINT32_MAX;

    written = g920_link_stats_format(line, sizeof(line), &stats);
    /* Либо строка целиком, либо честный отказ — но не порча памяти. */
    if (written > 0) {
        TEST_ASSERT_TRUE((size_t)written < sizeof(line));
    } else {
        TEST_ASSERT_EQUAL_INT(-1, written);
    }
}

static void test_bad_arguments(void)
{
    g920_link_stats_init(NULL);
    g920_link_stats_on_sent(NULL, true);
    g920_link_stats_on_rtt(NULL, 1);
    g920_link_stats_add_rx(NULL, 1, 1, 1);
    TEST_ASSERT_EQUAL_UINT32(0, g920_link_stats_avg_rtt_us(NULL));
    TEST_ASSERT_EQUAL_UINT32(0, g920_link_stats_loss_permille(NULL));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_fresh_stats_are_empty);
    RUN_TEST(test_rtt_min_avg_max);
    RUN_TEST(test_jitter_is_smoothed_not_peak);
    RUN_TEST(test_jitter_sees_systematic_shake);
    RUN_TEST(test_loss_is_counted_against_sent);
    RUN_TEST(test_more_received_than_sent_is_not_negative_loss);
    RUN_TEST(test_send_failures_are_separate_from_loss);
    RUN_TEST(test_format_has_every_number);
    RUN_TEST(test_format_refuses_a_small_buffer);
    RUN_TEST(test_format_survives_huge_counters);
    RUN_TEST(test_bad_arguments);

    return UNITY_END();
}
