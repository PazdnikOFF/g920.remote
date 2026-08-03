#include <stdint.h>
#include <string.h>

#include <unity.h>

#include "g920/link.h"
#include "g920/peering.h"
#include "g920/version.h"

/*
 * Политика пиринга жила в прошивке под `#if ESP_PLATFORM` и не проверялась
 * ничем — и оказалась сломана так, что заметить это могли только глаза на
 * живом столе: пир принимался только пока его нет, а отпускался только
 * функцией, которую никто не вызывал. Сменить собеседника в работе было
 * нельзя, а вся защита от подмены модуля лежала на недостижимой ветке.
 */

#define TX ((uint8_t)G920_ROLE_TX)
#define RX ((uint8_t)G920_ROLE_RX)

static const uint8_t MAC_A[6] = { 0xAA, 0, 0, 0, 0, 1 };
static const uint8_t MAC_B[6] = { 0xBB, 0, 0, 0, 0, 2 };

static g920_peering_t p;

/* Порог освобождения в микросекундах. */
#define RELEASE_US ((uint64_t)G920_PEER_RELEASE_MS * 1000u)

void setUp(void)
{
    g920_peering_init(&p, 0);
}

void tearDown(void) { }

static void test_first_caller_is_adopted(void)
{
    TEST_ASSERT_EQUAL_INT(G920_PEERING_ADOPTED,
                          g920_peering_on_discover(&p, MAC_A, TX, RX, 1000));
    TEST_ASSERT_TRUE(g920_peering_has_peer(&p));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(MAC_A, g920_peering_peer(&p), 6);
    TEST_ASSERT_EQUAL_UINT32(1, g920_peering_adoptions(&p));
}

static void test_same_role_is_never_a_pair(void)
{
    /* Два TX в одной комнате находят друг друга по первому же вызову.
     * Спарившись, они зажгут зелёный и не передадут ни одного кадра. */
    TEST_ASSERT_EQUAL_INT(G920_PEERING_IGNORE,
                          g920_peering_on_discover(&p, MAC_A, RX, RX, 1000));
    TEST_ASSERT_FALSE(g920_peering_has_peer(&p));

    /* Роль не заявлена — тоже мимо: молчаливое «наверное, подойдёт» здесь
     * стоит того же. */
    TEST_ASSERT_EQUAL_INT(
        G920_PEERING_IGNORE,
        g920_peering_on_discover(&p, MAC_A, (uint8_t)G920_ROLE_UNKNOWN, RX,
                                 1000));
    TEST_ASSERT_FALSE(g920_peering_has_peer(&p));
}

static void test_own_peer_calling_gets_an_answer(void)
{
    (void)g920_peering_on_discover(&p, MAC_A, TX, RX, 1000);
    /* Свой зовёт — отвечаем, но пира не «переприниманием». */
    TEST_ASSERT_EQUAL_INT(G920_PEERING_REPLY,
                          g920_peering_on_discover(&p, MAC_A, TX, RX, 2000));
    TEST_ASSERT_EQUAL_UINT32(1, g920_peering_adoptions(&p));
}

static void test_stranger_gets_no_answer_while_peer_is_alive(void)
{
    (void)g920_peering_on_discover(&p, MAC_A, TX, RX, 1000);

    /*
     * Вот ради чего всё это. Ответить чужому значит сказать ему «мы пара»:
     * он зажжёт зелёный и погонит кадры, которые мы всё равно выбросим по
     * MAC. Именно так выглядела дыра шестого вердикта — руль молчит, оба
     * индикатора зелёные, в логе ничего.
     */
    TEST_ASSERT_EQUAL_INT(G920_PEERING_IGNORE,
                          g920_peering_on_discover(&p, MAC_B, TX, RX, 2000));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(MAC_A, g920_peering_peer(&p), 6);
    TEST_ASSERT_EQUAL_UINT32(1, g920_peering_rejected(&p));
}

static void test_stranger_is_adopted_after_the_peer_goes_quiet(void)
{
    (void)g920_peering_on_discover(&p, MAC_A, TX, RX, 1000);

    /* Замена платы: старый модуль больше не говорит. Пока не истёк порог —
     * место занято. */
    TEST_ASSERT_EQUAL_INT(
        G920_PEERING_IGNORE,
        g920_peering_on_discover(&p, MAC_B, TX, RX, 1000 + RELEASE_US / 2));

    /* А после порога новый принимается — иначе комплект чинится только
     * перепрошивкой. */
    TEST_ASSERT_EQUAL_INT(
        G920_PEERING_ADOPTED,
        g920_peering_on_discover(&p, MAC_B, TX, RX, 1000 + RELEASE_US + 1));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(MAC_B, g920_peering_peer(&p), 6);
    TEST_ASSERT_EQUAL_UINT32(2, g920_peering_adoptions(&p));
}

static void test_traffic_from_the_peer_keeps_it_alive(void)
{
    uint64_t now = 1000;

    (void)g920_peering_on_discover(&p, MAC_A, TX, RX, now);

    /* Поток кадров двигает таймер тишины: пир не должен отпускаться, пока
     * говорит. */
    for (int i = 0; i < 100; i++) {
        now += RELEASE_US / 2;
        TEST_ASSERT_EQUAL_INT(G920_PEERING_KNOWN,
                              g920_peering_on_frame(&p, MAC_A, now));
        TEST_ASSERT_FALSE(g920_peering_silent(&p, now));
    }

    /* А чужой кадр таймер не двигает: он не доказывает, что наш жив. */
    TEST_ASSERT_EQUAL_INT(G920_PEERING_IGNORE,
                          g920_peering_on_frame(&p, MAC_B, now + 1));
    TEST_ASSERT_TRUE(g920_peering_silent(&p, now + RELEASE_US + 1));
}

static void test_frames_from_strangers_are_counted(void)
{
    (void)g920_peering_on_discover(&p, MAC_A, TX, RX, 1000);

    for (int i = 0; i < 7; i++) {
        TEST_ASSERT_EQUAL_INT(G920_PEERING_IGNORE,
                              g920_peering_on_frame(&p, MAC_B, 2000));
    }
    /* Молчаливый отказ — тот самый симптом, за который веха блокировалась
     * дважды. Считаем, чтобы было о чём написать в лог. */
    TEST_ASSERT_EQUAL_UINT32(7, g920_peering_rejected(&p));
}

static void test_peer_from_nvs_does_not_hold_the_place(void)
{
    g920_peering_preset(&p, MAC_A);

    TEST_ASSERT_TRUE(g920_peering_has_peer(&p));
    /*
     * Сохранённый MAC — это не подтверждение, что модуль жив. Пока он не
     * сказал ни слова, он считается молчащим, и живой сосед подключится
     * сразу, не дожидаясь порога.
     */
    TEST_ASSERT_TRUE(g920_peering_silent(&p, 0));
    TEST_ASSERT_EQUAL_INT(G920_PEERING_ADOPTED,
                          g920_peering_on_discover(&p, MAC_B, TX, RX, 1));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(MAC_B, g920_peering_peer(&p), 6);
}

static void test_preset_peer_that_answers_holds_the_place(void)
{
    g920_peering_preset(&p, MAC_A);

    /* Он ответил — значит жив, и место за ним. */
    TEST_ASSERT_EQUAL_INT(G920_PEERING_REPLY,
                          g920_peering_on_discover(&p, MAC_A, TX, RX, 1000));
    TEST_ASSERT_FALSE(g920_peering_silent(&p, 1000));
    TEST_ASSERT_EQUAL_INT(G920_PEERING_IGNORE,
                          g920_peering_on_discover(&p, MAC_B, TX, RX, 1001));
}

static void test_release_frees_the_place(void)
{
    (void)g920_peering_on_discover(&p, MAC_A, TX, RX, 1000);
    g920_peering_release(&p);

    TEST_ASSERT_FALSE(g920_peering_has_peer(&p));
    TEST_ASSERT_NULL(g920_peering_peer(&p));
    TEST_ASSERT_EQUAL_INT(G920_PEERING_ADOPTED,
                          g920_peering_on_discover(&p, MAC_B, TX, RX, 1001));
}

static void test_clock_going_backwards_does_not_break_the_link(void)
{
    (void)g920_peering_on_discover(&p, MAC_A, TX, RX, 1000000);

    /* Часы дёрнулись назад — это не повод рвать связь. */
    TEST_ASSERT_FALSE(g920_peering_silent(&p, 1));
    TEST_ASSERT_EQUAL_INT(G920_PEERING_IGNORE,
                          g920_peering_on_discover(&p, MAC_B, TX, RX, 1));
}

/*
 * `g920_peering_quiet_for` — единственная функция модуля, у которой не было
 * своего теста: её проверяли только через `g920_peering_silent`, то есть на
 * одном пороге из двух. А порогов ровно два и они разные по смыслу: по
 * G920_LINK_SILENCE_MS (2 с) пира **зовут снова**, по release_ms (10 с) —
 * **отпускают**. Перепутать их значит либо звать вечно того, кого пора
 * забыть, либо отпускать живого при первой заминке.
 */
static void test_quiet_for_answers_each_threshold_separately(void)
{
    const uint64_t T0 = 1000000;

    (void)g920_peering_on_discover(&p, MAC_A, TX, RX, T0);

    /* Только что говорил — молчащим не считается ни по одному порогу. */
    TEST_ASSERT_FALSE(g920_peering_quiet_for(&p, T0, G920_LINK_SILENCE_MS));
    TEST_ASSERT_FALSE(g920_peering_silent(&p, T0));

    /* Две секунды тишины: звать пора, отпускать — ещё нет. */
    TEST_ASSERT_TRUE(g920_peering_quiet_for(
        &p, T0 + (uint64_t)G920_LINK_SILENCE_MS * 1000u,
        G920_LINK_SILENCE_MS));
    TEST_ASSERT_FALSE(
        g920_peering_silent(&p, T0 + (uint64_t)G920_LINK_SILENCE_MS * 1000u));

    /* Ровно на пороге — уже молчит: граница включительная, иначе «десять
     * секунд» означало бы «десять с чем-то». */
    TEST_ASSERT_TRUE(g920_peering_quiet_for(&p, T0 + RELEASE_US,
                                            G920_PEER_RELEASE_MS));
    TEST_ASSERT_FALSE(g920_peering_quiet_for(&p, T0 + RELEASE_US - 1,
                                             G920_PEER_RELEASE_MS));

    /* Пира нет вовсе — молчит по определению, а не «ещё не пора». */
    g920_peering_release(&p);
    TEST_ASSERT_TRUE(g920_peering_quiet_for(&p, T0, G920_LINK_SILENCE_MS));
    TEST_ASSERT_TRUE(g920_peering_quiet_for(NULL, T0, G920_LINK_SILENCE_MS));
}

static void test_action_names_and_bad_arguments(void)
{
    TEST_ASSERT_EQUAL_STRING("ADOPTED",
                             g920_peering_action_name(G920_PEERING_ADOPTED));
    TEST_ASSERT_EQUAL_STRING("IGNORE",
                             g920_peering_action_name(G920_PEERING_IGNORE));
    TEST_ASSERT_EQUAL_STRING(
        "?", g920_peering_action_name((g920_peering_action_t)42));

    TEST_ASSERT_EQUAL_INT(G920_PEERING_IGNORE,
                          g920_peering_on_discover(NULL, MAC_A, TX, RX, 0));
    TEST_ASSERT_EQUAL_INT(G920_PEERING_IGNORE,
                          g920_peering_on_discover(&p, NULL, TX, RX, 0));
    TEST_ASSERT_EQUAL_INT(G920_PEERING_IGNORE,
                          g920_peering_on_frame(NULL, MAC_A, 0));
    TEST_ASSERT_EQUAL_INT(G920_PEERING_IGNORE,
                          g920_peering_on_frame(&p, NULL, 0));
    TEST_ASSERT_TRUE(g920_peering_silent(NULL, 0));
    TEST_ASSERT_FALSE(g920_peering_has_peer(NULL));
    TEST_ASSERT_NULL(g920_peering_peer(NULL));
    TEST_ASSERT_EQUAL_UINT32(0, g920_peering_rejected(NULL));
    TEST_ASSERT_EQUAL_UINT32(0, g920_peering_adoptions(NULL));
    g920_peering_init(NULL, 0);
    g920_peering_preset(NULL, MAC_A);
    g920_peering_preset(&p, NULL);
    g920_peering_release(NULL);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_first_caller_is_adopted);
    RUN_TEST(test_same_role_is_never_a_pair);
    RUN_TEST(test_own_peer_calling_gets_an_answer);
    RUN_TEST(test_stranger_gets_no_answer_while_peer_is_alive);
    RUN_TEST(test_stranger_is_adopted_after_the_peer_goes_quiet);
    RUN_TEST(test_traffic_from_the_peer_keeps_it_alive);
    RUN_TEST(test_frames_from_strangers_are_counted);
    RUN_TEST(test_peer_from_nvs_does_not_hold_the_place);
    RUN_TEST(test_preset_peer_that_answers_holds_the_place);
    RUN_TEST(test_release_frees_the_place);
    RUN_TEST(test_clock_going_backwards_does_not_break_the_link);
    RUN_TEST(test_quiet_for_answers_each_threshold_separately);
    RUN_TEST(test_action_names_and_bad_arguments);

    return UNITY_END();
}
