#include <stdint.h>
#include <string.h>

#include <unity.h>

#include "g920/link_rx.h"
#include "g920/version.h"

/*
 * Семь вердиктов подряд находили дефект в приёмном пути прошивки — там, где
 * тестов не было. Этот файл существует ровно затем, чтобы восьмого такого
 * не случилось: он гоняет ту же функцию, которую исполняет прошивка.
 */

#define TX ((uint8_t)G920_ROLE_TX)
#define RX ((uint8_t)G920_ROLE_RX)

static const uint8_t MAC_A[6] = { 0xAA, 0, 0, 0, 0, 1 };
static const uint8_t MAC_B[6] = { 0xBB, 0, 0, 0, 0, 2 };

static g920_link_rx_t st;
/* Исход, который тесту не важен: чтобы не заводить его в каждом. */
static g920_link_rx_outcome_t scratch;

void setUp(void)
{
    /* Мы — RX, собеседник — TX. */
    g920_link_rx_init(&st, RX, 0);
}

void tearDown(void) { }

static g920_frame_t make(uint8_t type, uint16_t sequence, uint8_t epoch,
                         const uint8_t *payload, uint16_t length)
{
    g920_frame_t f;

    memset(&f, 0, sizeof(f));
    f.version = G920_LINK_PROTO_VERSION;
    f.type = type;
    f.sequence = sequence;
    f.epoch = epoch;
    f.payload = payload;
    f.length = length;
    return f;
}

static const uint8_t CALL[2] = { G920_LINK_DISCOVER_REQUEST,
                                 (uint8_t)G920_ROLE_TX };
static const uint8_t ANSWER[2] = { G920_LINK_DISCOVER_REPLY,
                                   (uint8_t)G920_ROLE_TX };

/* Спаривает нас с MAC_A и возвращает время. */
static uint64_t pair_with_a(void)
{
    g920_frame_t call = make(G920_FRAME_DISCOVER, 1, 7, CALL, 2);
    g920_link_rx_outcome_t out;

    g920_link_rx_on_frame(&st, &call, MAC_A, 1000, &out);
    TEST_ASSERT_TRUE(out.adopt_peer);
    TEST_ASSERT_TRUE(out.send_reply);
    TEST_ASSERT_EQUAL_UINT8(G920_FRAME_DISCOVER, out.reply.type);
    return 1000;
}

/* --- пиринг --------------------------------------------------------------- */

static void test_call_adopts_and_answers(void)
{
    (void)pair_with_a();
    TEST_ASSERT_TRUE(g920_link_rx_has_peer(&st));
    TEST_ASSERT_TRUE(g920_link_rx_peer_alive(&st));
}

static void test_answer_is_not_answered(void)
{
    g920_frame_t answer = make(G920_FRAME_DISCOVER, 1, 7, ANSWER, 2);
    g920_link_rx_outcome_t out;

    g920_link_rx_on_frame(&st, &answer, MAC_A, 1000, &out);
    /* Обмен из двух шагов: отвечать на ответ значит не кончить никогда. */
    TEST_ASSERT_TRUE(out.adopt_peer);
    TEST_ASSERT_FALSE(out.send_reply);
}

static void test_stranger_is_silently_ignored_while_peer_lives(void)
{
    g920_frame_t call = make(G920_FRAME_DISCOVER, 1, 9, CALL, 2);
    g920_link_rx_outcome_t out;

    (void)pair_with_a();
    g920_link_rx_on_frame(&st, &call, MAC_B, 2000, &out);

    /* Ответить чужому значит сказать ему «мы пара»: он зажжёт зелёный и
     * погонит кадры, которые мы всё равно выбросим. */
    TEST_ASSERT_FALSE(out.adopt_peer);
    TEST_ASSERT_FALSE(out.send_reply);
}

static void test_peer_can_be_replaced_after_it_goes_quiet(void)
{
    uint64_t later = 1000 + (uint64_t)G920_PEER_RELEASE_MS * 1000u + 1;
    g920_frame_t call = make(G920_FRAME_DISCOVER, 1, 9, CALL, 2);
    g920_link_rx_outcome_t out;

    (void)pair_with_a();
    /* Замена второй платы. Пока эта ветка была недостижима, комплект
     * чинился только перепрошивкой. */
    TEST_ASSERT_TRUE(g920_link_rx_peer_expired(&st, later));

    g920_link_rx_on_frame(&st, &call, MAC_B, later, &out);
    TEST_ASSERT_TRUE(out.adopt_peer);
    TEST_ASSERT_TRUE(out.send_reply);
}

static void test_new_peer_starts_the_session_clean(void)
{
    g920_frame_t input = make(G920_FRAME_INPUT, 5000, 200, NULL, 0);
    g920_frame_t call = make(G920_FRAME_DISCOVER, 1, 3, CALL, 2);
    uint64_t later = 1000 + (uint64_t)G920_PEER_RELEASE_MS * 1000u + 1;
    g920_link_rx_outcome_t out;

    (void)pair_with_a();
    g920_link_rx_on_frame(&st, &input, MAC_A, 1001, &scratch);

    /* Новый собеседник с меньшими номерами и меньшей эпохой. Без сброса
     * состояния каждый его кадр уходил бы как опоздавший — навсегда. */
    g920_link_rx_on_frame(&st, &call, MAC_B, later, &scratch);
    input = make(G920_FRAME_INPUT, 1, 3, NULL, 0);
    g920_link_rx_on_frame(&st, &input, MAC_B, later + 1, &out);
    TEST_ASSERT_TRUE(out.deliver);
}

static void test_same_role_never_pairs(void)
{
    const uint8_t call_rx[2] = { G920_LINK_DISCOVER_REQUEST,
                                 (uint8_t)G920_ROLE_RX };
    g920_frame_t call = make(G920_FRAME_DISCOVER, 1, 7, call_rx, 2);
    g920_link_rx_outcome_t out;

    g920_link_rx_on_frame(&st, &call, MAC_A, 1, &out);
    TEST_ASSERT_FALSE(out.adopt_peer);
    TEST_ASSERT_FALSE(out.send_reply);
    TEST_ASSERT_FALSE(g920_link_rx_has_peer(&st));
}

static void test_short_discover_is_ignored(void)
{
    const uint8_t one[1] = { G920_LINK_DISCOVER_REQUEST };
    g920_frame_t call = make(G920_FRAME_DISCOVER, 1, 7, one, 1);
    g920_link_rx_outcome_t out;

    g920_link_rx_on_frame(&st, &call, MAC_A, 1, &out);
    /* Вызов без роли — от чужой прошивки: отвечать нечего. */
    TEST_ASSERT_FALSE(out.adopt_peer);
    TEST_ASSERT_FALSE(out.send_reply);
}

/* --- подтверждения -------------------------------------------------------- */

static void test_ack_closes_the_record_and_nothing_else(void)
{
    g920_frame_t ack;
    g920_link_rx_outcome_t out;

    (void)pair_with_a();
    ack = make(G920_FRAME_ACK, 42, 5, NULL, 0);
    g920_link_rx_on_frame(&st, &ack, MAC_A, 1001, &out);

    /*
     * Именно это и потерялось при переписывании приёмного пути: без
     * close_reliable подтверждение не закрывает запись, кадр сгорает все 15
     * попыток и выбрасывается — то есть надёжной доставки нет вовсе.
     */
    TEST_ASSERT_TRUE(out.close_reliable);
    /* И ничего больше: наверх ACK не идёт. */
    TEST_ASSERT_FALSE(out.deliver);
    TEST_ASSERT_FALSE(out.adopt_peer);
    TEST_ASSERT_FALSE(out.send_reply);
}

static void test_ack_is_never_acked(void)
{
    g920_frame_t ack;
    g920_link_rx_outcome_t out;

    (void)pair_with_a();
    ack = make(G920_FRAME_ACK, 42, 5, NULL, 0);
    g920_link_rx_on_frame(&st, &ack, MAC_A, 1001, &out);

    /*
     * Подтверждение на подтверждение — это самоподдерживающийся обмен на
     * скорости радио. Пока ACK числился «надёжным по остаточному
     * принципу», такой шторм был в одном шаге.
     */
    TEST_ASSERT_FALSE((out.send_reply && out.reply.type == G920_FRAME_ACK));
}

static void test_ack_does_not_touch_the_peer_session(void)
{
    g920_frame_t ack;
    g920_frame_t input;
    g920_link_rx_outcome_t out;
    uint32_t sessions_before;

    (void)pair_with_a();
    input = make(G920_FRAME_INPUT, 100, 7, NULL, 0);
    g920_link_rx_on_frame(&st, &input, MAC_A, 1001, &scratch);
    sessions_before = st.rx.sessions;

    /*
     * Номер и эпоха в ACK — из **нашего** пространства: это наш же кадр,
     * вернувшийся отметкой о доставке. Прогонять его через трекер сессии
     * значит травить сессию собеседника чужими числами: считались бы
     * ложные перезапуски, а метка номера уезжала бы куда попало.
     */
    ack = make(G920_FRAME_ACK, 1, 200, NULL, 0);
    g920_link_rx_on_frame(&st, &ack, MAC_A, 1002, &scratch);
    TEST_ASSERT_EQUAL_UINT32(sessions_before, st.rx.sessions);

    /* Поток собеседника после этого идёт как ни в чём не бывало. */
    input = make(G920_FRAME_INPUT, 101, 7, NULL, 0);
    g920_link_rx_on_frame(&st, &input, MAC_A, 1003, &out);
    TEST_ASSERT_TRUE(out.deliver);
}

static void test_ack_from_a_stranger_is_dropped(void)
{
    g920_frame_t ack;
    g920_link_rx_outcome_t out;

    (void)pair_with_a();
    ack = make(G920_FRAME_ACK, 42, 5, NULL, 0);
    g920_link_rx_on_frame(&st, &ack, MAC_B, 1001, &out);

    /* Чужое подтверждение закрыло бы нашу запись молча: номера у всех
     * стартуют с единицы. */
    TEST_ASSERT_FALSE(out.close_reliable);
}

/* --- надёжная и свежая дисциплины ---------------------------------------- */

static void test_reliable_frame_is_acked_and_delivered(void)
{
    g920_frame_t auth;
    g920_link_rx_outcome_t out;

    /* Своя эпоха заведомо не равна эпохе кадра: иначе «повторяет чужую» и
     * «ставит свою» дают один и тот же байт, и тест ничего не различает. */
    g920_link_rx_set_epoch(&st, 200);
    (void)pair_with_a();
    auth = make(G920_FRAME_AUTH, 1, 7, NULL, 0);
    g920_link_rx_on_frame(&st, &auth, MAC_A, 1001, &out);

    TEST_ASSERT_TRUE((out.send_reply && out.reply.type == G920_FRAME_ACK));
    TEST_ASSERT_TRUE(out.deliver);
    TEST_ASSERT_EQUAL_INT(G920_RX_NEW_SESSION, out.verdict);

    /*
     * Раскладка самого подтверждения. Раньше её проверял test_proto на
     * отдельной сборке ACK — на той, по которой исполнение не шло. Теперь
     * точка сборки одна, и проверка стоит на ней.
     *
     * Номер в ACK — номер подтверждаемого кадра, отдельного поля нет.
     * Эпоха — тоже подтверждаемого, а не своя: иначе подтверждение,
     * разошедшееся с перезагрузкой, закрыло бы запись уже другой сессии.
     */
    TEST_ASSERT_EQUAL_UINT16(1, out.reply.sequence);
    TEST_ASSERT_EQUAL_UINT8(7, out.reply.epoch);
    TEST_ASSERT_EQUAL_UINT16(0, out.reply.length);
    TEST_ASSERT_EQUAL_UINT8(G920_LINK_PROTO_VERSION, out.reply.version);
}

static void test_repeat_is_acked_but_not_delivered(void)
{
    g920_frame_t auth;
    g920_link_rx_outcome_t out;

    (void)pair_with_a();
    auth = make(G920_FRAME_AUTH, 1, 7, NULL, 0);
    g920_link_rx_on_frame(&st, &auth, MAC_A, 1001, &scratch);

    g920_link_rx_on_frame(&st, &auth, MAC_A, 1002, &out);
    /* Повтор подтверждаем — потерялось как раз подтверждение; наверх не
     * отдаём — сборщик фрагментов GIP умеет повтор только заметить. */
    TEST_ASSERT_TRUE((out.send_reply && out.reply.type == G920_FRAME_ACK));
    TEST_ASSERT_FALSE(out.deliver);
}

static void test_frame_of_a_foreign_session_is_not_acked(void)
{
    g920_frame_t auth;
    g920_link_rx_outcome_t out;

    (void)pair_with_a();
    auth = make(G920_FRAME_AUTH, 10, 7, NULL, 0);
    g920_link_rx_on_frame(&st, &auth, MAC_A, 1001, &scratch);
    auth = make(G920_FRAME_AUTH, 1, 8, NULL, 0);
    g920_link_rx_on_frame(&st, &auth, MAC_A, 1002, &scratch); /* новая сессия */

    /* Заблудившийся кадр прошлой эпохи. Подтвердить его значит сказать
     * «доставлено» про то, чего наверх не отдали. */
    auth = make(G920_FRAME_AUTH, 11, 7, NULL, 0);
    g920_link_rx_on_frame(&st, &auth, MAC_A, 1003, &out);
    TEST_ASSERT_EQUAL_INT(G920_RX_FOREIGN, out.verdict);
    TEST_ASSERT_FALSE((out.send_reply && out.reply.type == G920_FRAME_ACK));
    TEST_ASSERT_FALSE(out.deliver);
}

static void test_fresh_frame_is_delivered_without_an_ack(void)
{
    g920_frame_t input;
    g920_link_rx_outcome_t out;

    (void)pair_with_a();
    input = make(G920_FRAME_INPUT, 1, 7, NULL, 0);
    g920_link_rx_on_frame(&st, &input, MAC_A, 1001, &out);

    /* Свежая дисциплина подтверждений не требует: пока ретрай долетит,
     * состояние уже другое. */
    TEST_ASSERT_TRUE(out.deliver);
    TEST_ASSERT_FALSE((out.send_reply && out.reply.type == G920_FRAME_ACK));
}

static void test_stale_fresh_frame_is_dropped(void)
{
    g920_frame_t input;
    g920_link_rx_outcome_t out;

    (void)pair_with_a();
    input = make(G920_FRAME_INPUT, 10, 7, NULL, 0);
    g920_link_rx_on_frame(&st, &input, MAC_A, 1001, &scratch);

    input = make(G920_FRAME_INPUT, 9, 7, NULL, 0);
    g920_link_rx_on_frame(&st, &input, MAC_A, 1002, &out);
    /* Отдать хосту вчерашнее положение руля после сегодняшнего — это
     * залипший орган управления. */
    TEST_ASSERT_FALSE(out.deliver);
    TEST_ASSERT_EQUAL_INT(G920_RX_STALE, out.verdict);
}

static void test_frame_from_a_stranger_never_reaches_the_app(void)
{
    g920_frame_t input;
    g920_link_rx_outcome_t out;

    (void)pair_with_a();
    input = make(G920_FRAME_INPUT, 1, 7, NULL, 0);
    g920_link_rx_on_frame(&st, &input, MAC_B, 1001, &out);

    TEST_ASSERT_FALSE(out.deliver);
    TEST_ASSERT_FALSE((out.send_reply && out.reply.type == G920_FRAME_ACK));
    /* И отказ не молчаливый: счётчик растёт, прошивке есть о чём писать. */
    TEST_ASSERT_EQUAL_UINT32(1, g920_peering_rejected(&st.peering));
}

/* --- состояния пира ------------------------------------------------------- */

static void test_peer_from_nvs_is_known_but_not_alive(void)
{
    g920_link_rx_preset_peer(&st, MAC_A);

    /*
     * Адрес есть, а собеседника может не быть вовсе. Склейка этих двух
     * состояний дважды давала зелёный индикатор при мёртвом линке, поэтому
     * ответа два, а не один.
     */
    TEST_ASSERT_TRUE(g920_link_rx_has_peer(&st));
    TEST_ASSERT_FALSE(g920_link_rx_peer_alive(&st));
    /* Отпускать его нечему — он и так никого не держит. */
    TEST_ASSERT_FALSE(g920_link_rx_peer_expired(&st, 0));

    /* А заговорил — стал живым. */
    {
        g920_frame_t input = make(G920_FRAME_INPUT, 1, 7, NULL, 0);

        g920_link_rx_on_frame(&st, &input, MAC_A, 1000, &scratch);
        TEST_ASSERT_TRUE(g920_link_rx_peer_alive(&st));
    }
}

static void test_release_makes_room(void)
{
    g920_frame_t call = make(G920_FRAME_DISCOVER, 1, 9, CALL, 2);

    (void)pair_with_a();
    g920_link_rx_release_peer(&st);

    TEST_ASSERT_FALSE(g920_link_rx_has_peer(&st));
    g920_link_rx_on_frame(&st, &call, MAC_B, 1001, &scratch);
    TEST_ASSERT_TRUE(scratch.adopt_peer);
}

static void test_bad_arguments(void)
{
    g920_frame_t input = make(G920_FRAME_INPUT, 1, 7, NULL, 0);
    g920_link_rx_outcome_t out;

    g920_link_rx_on_frame(NULL, &input, MAC_A, 0, &out);
    TEST_ASSERT_FALSE(out.deliver);
    g920_link_rx_on_frame(&st, NULL, MAC_A, 0, &out);
    TEST_ASSERT_FALSE(out.deliver);
    g920_link_rx_on_frame(&st, &input, NULL, 0, &out);
    TEST_ASSERT_FALSE(out.deliver);
    g920_link_rx_on_frame(&st, &input, MAC_A, 0, NULL);

    TEST_ASSERT_FALSE(g920_link_rx_has_peer(NULL));
    TEST_ASSERT_FALSE(g920_link_rx_peer_alive(NULL));
    TEST_ASSERT_FALSE(g920_link_rx_peer_expired(NULL, 0));
    g920_link_rx_init(NULL, RX, 0);
    g920_link_rx_preset_peer(NULL, MAC_A);
    g920_link_rx_release_peer(NULL);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_call_adopts_and_answers);
    RUN_TEST(test_answer_is_not_answered);
    RUN_TEST(test_stranger_is_silently_ignored_while_peer_lives);
    RUN_TEST(test_peer_can_be_replaced_after_it_goes_quiet);
    RUN_TEST(test_new_peer_starts_the_session_clean);
    RUN_TEST(test_same_role_never_pairs);
    RUN_TEST(test_short_discover_is_ignored);

    RUN_TEST(test_ack_closes_the_record_and_nothing_else);
    RUN_TEST(test_ack_is_never_acked);
    RUN_TEST(test_ack_does_not_touch_the_peer_session);
    RUN_TEST(test_ack_from_a_stranger_is_dropped);

    RUN_TEST(test_reliable_frame_is_acked_and_delivered);
    RUN_TEST(test_repeat_is_acked_but_not_delivered);
    RUN_TEST(test_frame_of_a_foreign_session_is_not_acked);
    RUN_TEST(test_fresh_frame_is_delivered_without_an_ack);
    RUN_TEST(test_stale_fresh_frame_is_dropped);
    RUN_TEST(test_frame_from_a_stranger_never_reaches_the_app);

    RUN_TEST(test_peer_from_nvs_is_known_but_not_alive);
    RUN_TEST(test_release_makes_room);
    RUN_TEST(test_bad_arguments);

    return UNITY_END();
}
