#include <stdint.h>
#include <string.h>

#include <unity.h>

#include "g920/queue.h"

static g920_fresh_queue_t fresh;
static g920_reliable_queue_t reliable;
static uint8_t out_buf[G920_RELIABLE_PAYLOAD_MAX];
static g920_frame_t out;

void setUp(void)
{
    g920_fresh_init(&fresh);
    g920_reliable_init(&reliable, 0, 0);
    memset(out_buf, 0, sizeof(out_buf));
    memset(&out, 0, sizeof(out));
}

void tearDown(void) { }

/* --- свежая: вытесняет ---------------------------------------------------- */

static void test_fresh_newest_evicts_older(void)
{
    const uint8_t first[] = { 1, 1, 1 };
    const uint8_t second[] = { 2, 2, 2 };

    TEST_ASSERT_TRUE(
        g920_fresh_push(&fresh, G920_FRAME_INPUT, 1, first, sizeof(first)));
    TEST_ASSERT_TRUE(
        g920_fresh_push(&fresh, G920_FRAME_INPUT, 2, second, sizeof(second)));

    /* Промежуточное состояние выброшено, не дождавшись отправки: копить
     * очередь здесь значит копить задержку. */
    TEST_ASSERT_EQUAL_UINT32(1, g920_fresh_evicted(&fresh));

    TEST_ASSERT_EQUAL_INT(
        1, g920_fresh_pop(&fresh, &out, out_buf, sizeof(out_buf)));
    TEST_ASSERT_EQUAL_UINT16(2, out.sequence);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(second, out.payload, sizeof(second));
    TEST_ASSERT_TRUE(g920_fresh_empty(&fresh));
}

static void test_fresh_queue_never_grows(void)
{
    const uint8_t data[] = { 0xAA };

    for (uint16_t i = 0; i < 1000; i++) {
        TEST_ASSERT_TRUE(
            g920_fresh_push(&fresh, G920_FRAME_INPUT, i, data, sizeof(data)));
    }
    /* Тысяча отчётов — один кадр на выходе. Это и есть коалесценция. */
    TEST_ASSERT_EQUAL_UINT32(999, g920_fresh_evicted(&fresh));
    TEST_ASSERT_EQUAL_INT(
        1, g920_fresh_pop(&fresh, &out, out_buf, sizeof(out_buf)));
    TEST_ASSERT_EQUAL_UINT16(999, out.sequence);
    TEST_ASSERT_EQUAL_INT(
        0, g920_fresh_pop(&fresh, &out, out_buf, sizeof(out_buf)));
}

static void test_input_and_ffb_do_not_evict_each_other(void)
{
    const uint8_t in_data[] = { 0x11 };
    const uint8_t ffb_data[] = { 0x22 };

    TEST_ASSERT_TRUE(g920_fresh_push(&fresh, G920_FRAME_INPUT, 1, in_data, 1));
    TEST_ASSERT_TRUE(g920_fresh_push(&fresh, G920_FRAME_FFB, 2, ffb_data, 1));
    /* Разные потоки — разные слоты: свежий отчёт руля не должен
     * выбрасывать свежие силы. */
    TEST_ASSERT_EQUAL_UINT32(0, g920_fresh_evicted(&fresh));

    /* INPUT приоритетнее — уходит первым. */
    TEST_ASSERT_EQUAL_INT(
        1, g920_fresh_pop(&fresh, &out, out_buf, sizeof(out_buf)));
    TEST_ASSERT_EQUAL_UINT8(G920_FRAME_INPUT, out.type);
    TEST_ASSERT_EQUAL_INT(
        1, g920_fresh_pop(&fresh, &out, out_buf, sizeof(out_buf)));
    TEST_ASSERT_EQUAL_UINT8(G920_FRAME_FFB, out.type);
    TEST_ASSERT_EQUAL_INT(
        0, g920_fresh_pop(&fresh, &out, out_buf, sizeof(out_buf)));
}

static void test_fresh_refuses_reliable_types(void)
{
    const uint8_t data[] = { 1 };

    /* Перепутанная очередь — это молча отключённые ретраи у AUTH. */
    TEST_ASSERT_FALSE(g920_fresh_push(&fresh, G920_FRAME_AUTH, 1, data, 1));
    TEST_ASSERT_FALSE(g920_fresh_push(&fresh, G920_FRAME_DESCRIPTOR, 1, data, 1));
    TEST_ASSERT_FALSE(g920_fresh_push(&fresh, G920_FRAME_CONTROL, 1, data, 1));
    TEST_ASSERT_FALSE(g920_fresh_push(&fresh, G920_FRAME_ACK, 1, data, 1));
    TEST_ASSERT_TRUE(g920_fresh_empty(&fresh));
}

static void test_fresh_bad_arguments(void)
{
    uint8_t big[G920_FRESH_PAYLOAD_MAX + 1] = { 0 };

    TEST_ASSERT_FALSE(g920_fresh_push(&fresh, G920_FRAME_INPUT, 1, big,
                                      sizeof(big)));
    TEST_ASSERT_FALSE(g920_fresh_push(&fresh, G920_FRAME_INPUT, 1, NULL, 4));
    TEST_ASSERT_FALSE(g920_fresh_push(NULL, G920_FRAME_INPUT, 1, big, 1));

    TEST_ASSERT_EQUAL_INT(-1, g920_fresh_pop(NULL, &out, out_buf, 64));
    TEST_ASSERT_EQUAL_INT(-1, g920_fresh_pop(&fresh, NULL, out_buf, 64));
    TEST_ASSERT_EQUAL_INT(-1, g920_fresh_pop(&fresh, &out, NULL, 64));

    /* Тесный буфер — отказ, а не обрезанная нагрузка. */
    TEST_ASSERT_TRUE(g920_fresh_push(&fresh, G920_FRAME_INPUT, 1, big, 10));
    TEST_ASSERT_EQUAL_INT(-1, g920_fresh_pop(&fresh, &out, out_buf, 9));
}

static void test_fresh_zero_length_payload(void)
{
    TEST_ASSERT_TRUE(g920_fresh_push(&fresh, G920_FRAME_INPUT, 5, NULL, 0));
    TEST_ASSERT_EQUAL_INT(
        1, g920_fresh_pop(&fresh, &out, out_buf, sizeof(out_buf)));
    TEST_ASSERT_EQUAL_UINT16(0, out.length);
    TEST_ASSERT_NULL(out.payload);
}

/* --- надёжная: отказывает, а не вытесняет -------------------------------- */

static void test_reliable_refuses_instead_of_evicting(void)
{
    const uint8_t data[] = { 1, 2, 3 };

    for (int i = 0; i < G920_RELIABLE_DEPTH; i++) {
        TEST_ASSERT_TRUE(g920_reliable_push(&reliable, G920_FRAME_AUTH,
                                            (uint16_t)i, data, sizeof(data)));
    }
    TEST_ASSERT_TRUE(g920_reliable_full(&reliable));

    /* Вот главное отличие от свежей очереди: здесь ничего не выбрасывается,
     * вызывающий обязан притормозить. */
    TEST_ASSERT_FALSE(
        g920_reliable_push(&reliable, G920_FRAME_AUTH, 99, data, sizeof(data)));
    TEST_ASSERT_EQUAL_UINT8(G920_RELIABLE_DEPTH,
                            g920_reliable_pending(&reliable));

    /* И первый кадр никуда не делся. */
    TEST_ASSERT_EQUAL_INT(1, g920_reliable_due(&reliable, 0, &out, out_buf,
                                               sizeof(out_buf)));
    TEST_ASSERT_EQUAL_UINT16(0, out.sequence);
}

static void test_reliable_takes_only_retriable_types(void)
{
    const uint8_t data[] = { 1 };

    TEST_ASSERT_FALSE(
        g920_reliable_push(&reliable, G920_FRAME_INPUT, 1, data, 1));
    TEST_ASSERT_FALSE(g920_reliable_push(&reliable, G920_FRAME_FFB, 1, data, 1));

    /* ACK и DISCOVER по классу доставки надёжны, но повторять их нельзя:
     * подтверждения на подтверждение никто не пришлёт, и кадр будет
     * пересылаться до самого отказа. */
    TEST_ASSERT_FALSE(g920_reliable_push(&reliable, G920_FRAME_ACK, 1, data, 1));
    TEST_ASSERT_FALSE(
        g920_reliable_push(&reliable, G920_FRAME_DISCOVER, 1, data, 1));
    TEST_ASSERT_EQUAL_UINT8(0, g920_reliable_pending(&reliable));

    TEST_ASSERT_TRUE(g920_reliable_push(&reliable, G920_FRAME_AUTH, 1, data, 1));
    TEST_ASSERT_TRUE(
        g920_reliable_push(&reliable, G920_FRAME_CONTROL, 2, data, 1));
    TEST_ASSERT_TRUE(
        g920_reliable_push(&reliable, G920_FRAME_DESCRIPTOR, 3, data, 1));
}

static void test_reliable_is_fifo(void)
{
    const uint8_t data[] = { 1 };

    g920_reliable_push(&reliable, G920_FRAME_AUTH, 10, data, 1);
    g920_reliable_push(&reliable, G920_FRAME_AUTH, 11, data, 1);
    g920_reliable_push(&reliable, G920_FRAME_AUTH, 12, data, 1);

    /* Окно в один кадр: следующий уходит только после подтверждения
     * предыдущего. Это и гарантирует порядок на проводе. */
    for (uint16_t expect = 10; expect <= 12; expect++) {
        TEST_ASSERT_EQUAL_INT(1, g920_reliable_due(&reliable, 0, &out, out_buf,
                                                   sizeof(out_buf)));
        TEST_ASSERT_EQUAL_UINT16(expect, out.sequence);

        /* Пока не подтвердили — второй кадр в эфир не пойдёт. */
        TEST_ASSERT_EQUAL_INT(0, g920_reliable_due(&reliable, 0, &out, out_buf,
                                                   sizeof(out_buf)));
        TEST_ASSERT_TRUE(g920_reliable_ack(&reliable, expect, 0));
    }
}

static void test_reliable_order_survives_slot_reuse(void)
{
    const uint8_t data[] = { 1 };

    /* Освободившийся слот не должен пускать новый кадр вперёд старых. */
    g920_reliable_push(&reliable, G920_FRAME_AUTH, 1, data, 1);
    g920_reliable_push(&reliable, G920_FRAME_AUTH, 2, data, 1);
    g920_reliable_push(&reliable, G920_FRAME_AUTH, 3, data, 1);
    TEST_ASSERT_EQUAL_INT(1, g920_reliable_due(&reliable, 0, &out, out_buf,
                                               sizeof(out_buf)));
    TEST_ASSERT_EQUAL_UINT16(1, out.sequence);
    TEST_ASSERT_TRUE(g920_reliable_ack(&reliable, 1, 0)); /* слот 0 свободен */
    g920_reliable_push(&reliable, G920_FRAME_AUTH, 4, data, 1); /* ляжет в 0 */

    TEST_ASSERT_EQUAL_INT(1, g920_reliable_due(&reliable, 0, &out, out_buf,
                                               sizeof(out_buf)));
    TEST_ASSERT_EQUAL_UINT16(2, out.sequence);
}

/* --- ретраи --------------------------------------------------------------- */

static void test_no_retry_before_timeout(void)
{
    const uint8_t data[] = { 1 };

    g920_reliable_push(&reliable, G920_FRAME_AUTH, 7, data, 1);
    TEST_ASSERT_EQUAL_INT(1, g920_reliable_due(&reliable, 0, &out, out_buf,
                                               sizeof(out_buf)));
    /* Первая отправка — без флага повтора. */
    TEST_ASSERT_EQUAL_UINT8(0, out.flags);

    /* До истечения таймера отправлять нечего. */
    TEST_ASSERT_EQUAL_INT(0, g920_reliable_due(&reliable, 1, &out, out_buf,
                                               sizeof(out_buf)));
    TEST_ASSERT_EQUAL_INT(
        0, g920_reliable_due(&reliable, G920_RELIABLE_RETRY_US - 1, &out,
                             out_buf, sizeof(out_buf)));
}

static void test_retry_after_timeout_is_marked(void)
{
    const uint8_t data[] = { 1 };

    g920_reliable_push(&reliable, G920_FRAME_AUTH, 7, data, 1);
    g920_reliable_due(&reliable, 0, &out, out_buf, sizeof(out_buf));

    TEST_ASSERT_EQUAL_INT(
        1, g920_reliable_due(&reliable, G920_RELIABLE_RETRY_US, &out, out_buf,
                             sizeof(out_buf)));
    TEST_ASSERT_EQUAL_UINT8(G920_FRAME_FLAG_RETRY, out.flags);
    TEST_ASSERT_EQUAL_UINT16(7, out.sequence);
    TEST_ASSERT_EQUAL_UINT32(1, g920_reliable_retries(&reliable));
}

static void test_ack_stops_retries(void)
{
    const uint8_t data[] = { 1 };

    g920_reliable_push(&reliable, G920_FRAME_AUTH, 7, data, 1);
    g920_reliable_due(&reliable, 0, &out, out_buf, sizeof(out_buf));

    TEST_ASSERT_TRUE(g920_reliable_ack(&reliable, 7, 0));
    TEST_ASSERT_EQUAL_UINT8(0, g920_reliable_pending(&reliable));
    TEST_ASSERT_EQUAL_INT(
        0, g920_reliable_due(&reliable, 1000000, &out, out_buf,
                             sizeof(out_buf)));
    /* Повторный ACK на тот же кадр — не находка, но и не сбой. */
    TEST_ASSERT_FALSE(g920_reliable_ack(&reliable, 7, 0));
}

static void test_ack_closes_only_a_frame_that_went_on_air(void)
{
    const uint8_t data[] = { 1 };

    g920_reliable_push(&reliable, G920_FRAME_AUTH, 10, data, 1);
    g920_reliable_push(&reliable, G920_FRAME_AUTH, 11, data, 1);
    g920_reliable_push(&reliable, G920_FRAME_AUTH, 12, data, 1);

    /*
     * Подтверждение на кадр, который ещё не уходил в эфир, — не наше: его
     * никто не мог видеть. Такой ACK приходит либо из прошлой сессии, либо
     * от чужого комплекта, и закрыть запись он не должен. Номера у трёх
     * типов общие, а ACK типа не несёт, поэтому единственная имеющаяся
     * защита — «отправляли ли мы это вообще».
     */
    TEST_ASSERT_FALSE(g920_reliable_ack(&reliable, 11, 0));
    TEST_ASSERT_EQUAL_UINT8(3, g920_reliable_pending(&reliable));

    /* Окно в один кадр: в эфире всегда первый по постановке. */
    TEST_ASSERT_EQUAL_INT(1, g920_reliable_due(&reliable, 0, &out, out_buf,
                                               sizeof(out_buf)));
    TEST_ASSERT_EQUAL_UINT16(10, out.sequence);
    TEST_ASSERT_TRUE(g920_reliable_ack(&reliable, 10, 0));

    TEST_ASSERT_EQUAL_INT(1, g920_reliable_due(&reliable, 0, &out, out_buf,
                                               sizeof(out_buf)));
    TEST_ASSERT_EQUAL_UINT16(11, out.sequence);
}

static void test_gives_up_after_max_attempts(void)
{
    const uint8_t data[] = { 1 };
    uint64_t now = 0;
    int sends = 0;

    g920_reliable_push(&reliable, G920_FRAME_AUTH, 7, data, 1);

    for (int i = 0; i < 20; i++) {
        if (g920_reliable_due(&reliable, now, &out, out_buf, sizeof(out_buf))
            == 1) {
            sends++;
        }
        now += G920_RELIABLE_RETRY_US;
        g920_reliable_expire(&reliable, now);
    }

    /* Ровно столько попыток, сколько объявлено, — не больше и не меньше. */
    TEST_ASSERT_EQUAL_INT(G920_RELIABLE_MAX_ATTEMPTS, sends);
    TEST_ASSERT_EQUAL_UINT32(1, g920_reliable_gave_up(&reliable));
    TEST_ASSERT_EQUAL_UINT8(0, g920_reliable_pending(&reliable));
    /* Место освободилось — линк не заклинило. */
    TEST_ASSERT_TRUE(g920_reliable_push(&reliable, G920_FRAME_AUTH, 8, data, 1));
}

static void test_last_attempt_gets_its_own_timeout(void)
{
    const uint8_t data[] = { 1 };
    uint64_t now = 0;

    g920_reliable_init(&reliable, 1000, 2);
    g920_reliable_push(&reliable, G920_FRAME_AUTH, 7, data, 1);

    g920_reliable_due(&reliable, now, &out, out_buf, sizeof(out_buf));
    now += 1000;
    g920_reliable_due(&reliable, now, &out, out_buf, sizeof(out_buf));

    /* Сразу после последней отправки бросать нельзя: подтверждение на неё
     * ещё может прийти. */
    TEST_ASSERT_EQUAL_UINT8(0, g920_reliable_expire(&reliable, now));
    TEST_ASSERT_EQUAL_UINT8(1, g920_reliable_pending(&reliable));

    TEST_ASSERT_TRUE(g920_reliable_ack(&reliable, 7, 0));
    TEST_ASSERT_EQUAL_UINT32(0, g920_reliable_gave_up(&reliable));
}

static void test_custom_parameters_are_honoured(void)
{
    const uint8_t data[] = { 1 };

    g920_reliable_init(&reliable, 500, 2);
    g920_reliable_push(&reliable, G920_FRAME_AUTH, 1, data, 1);

    g920_reliable_due(&reliable, 0, &out, out_buf, sizeof(out_buf));
    TEST_ASSERT_EQUAL_INT(0, g920_reliable_due(&reliable, 499, &out, out_buf,
                                               sizeof(out_buf)));
    TEST_ASSERT_EQUAL_INT(1, g920_reliable_due(&reliable, 500, &out, out_buf,
                                               sizeof(out_buf)));
    /* Две попытки — и всё. */
    TEST_ASSERT_EQUAL_INT(0, g920_reliable_due(&reliable, 5000, &out, out_buf,
                                               sizeof(out_buf)));
}

static void test_reliable_bad_arguments(void)
{
    static uint8_t big[G920_RELIABLE_PAYLOAD_MAX + 1];
    const uint8_t data[] = { 1 };

    memset(big, 0, sizeof(big));
    TEST_ASSERT_FALSE(
        g920_reliable_push(&reliable, G920_FRAME_AUTH, 1, big, sizeof(big)));
    TEST_ASSERT_FALSE(g920_reliable_push(&reliable, G920_FRAME_AUTH, 1, NULL, 4));
    TEST_ASSERT_FALSE(g920_reliable_push(NULL, G920_FRAME_AUTH, 1, data, 1));

    TEST_ASSERT_EQUAL_INT(-1, g920_reliable_due(NULL, 0, &out, out_buf, 64));
    TEST_ASSERT_EQUAL_INT(-1, g920_reliable_due(&reliable, 0, NULL, out_buf, 64));
    TEST_ASSERT_EQUAL_INT(-1, g920_reliable_due(&reliable, 0, &out, NULL, 64));
    TEST_ASSERT_FALSE(g920_reliable_ack(NULL, 1, 0));
}


/*
 * Модель линка с потерями переехала в test_link_model: там два узла, кадры
 * байтами и настоящий приёмный путь `g920_link_rx_on_frame`. Здешняя версия
 * дёргала куски по отдельности и склеивала их сама — то есть проверяла
 * тракт, которого в прошивке нет, и ACK в ней не существовал как кадр.
 * Ровно этой болезнью веха блокировалась восемь раз.
 *
 * Туда же ушёл и перезапуск приёмника: он оставался здесь склейкой
 * `g920_reliable_due` + `g920_peer_rx_accept` + `g920_reliable_ack` — той
 * самой конструкцией, про удаление которой этот комментарий уже сообщал.
 * Комментарий, описывающий не то дерево, что рядом с ним, — это не
 * неточность, а способ не заметить дыру: девятый вердикт нашёл её именно
 * так. Ниже в файле остаётся только то, что проверяет **одну** структуру:
 * очередь без канала и приёмник без очереди.
 */

static void test_ack_from_a_past_session_does_not_close_the_record(void)
{
    const uint8_t data[] = { 1 };

    g920_reliable_set_epoch(&reliable, 7);
    g920_reliable_push(&reliable, G920_FRAME_AUTH, 3, data, 1);
    TEST_ASSERT_EQUAL_INT(1, g920_reliable_due(&reliable, 0, &out, out_buf,
                                               sizeof(out_buf)));

    /*
     * Подтверждение с чужой эпохой — эхо прошлой сессии: номера начались
     * заново, и номер 3 в нём означал совсем другое сообщение. Закрыть нашу
     * запись оно не должно, иначе кадр считается доставленным, не будучи
     * отданным наверх ни разу.
     */
    TEST_ASSERT_FALSE(g920_reliable_ack(&reliable, 3, 6));
    TEST_ASSERT_EQUAL_UINT8(1, g920_reliable_pending(&reliable));

    TEST_ASSERT_TRUE(g920_reliable_ack(&reliable, 3, 7));
    TEST_ASSERT_EQUAL_UINT8(0, g920_reliable_pending(&reliable));
}

/* --- приёмник: точечные случаи ------------------------------------------- */

static void test_rx_suppresses_only_the_repeat(void)
{
    g920_peer_rx_t rx;

    g920_peer_rx_init(&rx);
    TEST_ASSERT_EQUAL_INT(G920_RX_NEW_SESSION,
                          g920_peer_rx_accept(&rx, 5, G920_FRAME_AUTH, 1));
    TEST_ASSERT_EQUAL_INT(G920_RX_NEW,
                          g920_peer_rx_accept(&rx, 5, G920_FRAME_AUTH, 2));
    TEST_ASSERT_EQUAL_INT(G920_RX_DUPLICATE,
                          g920_peer_rx_accept(&rx, 5, G920_FRAME_AUTH, 2));
    /* Опоздавший в надёжной дисциплине — то же самое: этот кадр уже
     * отдавали, окно отправки один кадр. */
    TEST_ASSERT_EQUAL_INT(G920_RX_DUPLICATE,
                          g920_peer_rx_accept(&rx, 5, G920_FRAME_AUTH, 1));
    TEST_ASSERT_EQUAL_INT(G920_RX_NEW,
                          g920_peer_rx_accept(&rx, 5, G920_FRAME_AUTH, 3));
    TEST_ASSERT_EQUAL_UINT32(3, rx.delivered);
    TEST_ASSERT_EQUAL_UINT32(2, rx.duplicates);
    TEST_ASSERT_EQUAL_UINT32(0, rx.gaps);
}

static void test_rx_counts_the_gap_left_by_a_dropped_frame(void)
{
    g920_peer_rx_t rx;

    g920_peer_rx_init(&rx);
    (void)g920_peer_rx_accept(&rx, 1, G920_FRAME_AUTH, 10);
    TEST_ASSERT_EQUAL_INT(G920_RX_NEW,
                          g920_peer_rx_accept(&rx, 1, G920_FRAME_AUTH, 12));
    /* Кадр 11 отправитель бросил, исчерпав попытки. Восстановить его нечем,
     * но в M8 по этому признаку решается, рвать ли обмен. */
    TEST_ASSERT_EQUAL_UINT32(1, rx.gaps);
}

static void test_rx_survives_sequence_wrap(void)
{
    g920_peer_rx_t rx;

    g920_peer_rx_init(&rx);
    (void)g920_peer_rx_accept(&rx, 1, G920_FRAME_AUTH, 65535);
    /* 65535 → 0 это шаг вперёд, а не откат на 65535 назад. */
    TEST_ASSERT_EQUAL_INT(G920_RX_NEW,
                          g920_peer_rx_accept(&rx, 1, G920_FRAME_AUTH, 0));
    TEST_ASSERT_EQUAL_UINT32(0, rx.gaps);
    TEST_ASSERT_EQUAL_UINT32(0, rx.duplicates);
}

static void test_rx_same_number_in_a_new_session_is_not_a_duplicate(void)
{
    g920_peer_rx_t rx;

    g920_peer_rx_init(&rx);
    (void)g920_peer_rx_accept(&rx, 1, G920_FRAME_AUTH, 7);
    /* Тот же номер, но эпоха вперёд — это не повтор, а первый кадр новой
     * жизни собеседника. */
    TEST_ASSERT_EQUAL_INT(G920_RX_NEW_SESSION,
                          g920_peer_rx_accept(&rx, 2, G920_FRAME_AUTH, 7));
    TEST_ASSERT_EQUAL_UINT32(0, rx.duplicates);
    /* Считаются перезапуски, а не сессии: первый кадр перезапуском не был. */
    TEST_ASSERT_EQUAL_UINT32(1, rx.sessions);
}

static void test_rx_epoch_wraps_forward_at_255(void)
{
    g920_peer_rx_t rx;

    g920_peer_rx_init(&rx);
    (void)g920_peer_rx_accept(&rx, 255, G920_FRAME_AUTH, 100);
    /* 255 → 0 — шаг вперёд, а не откат на 255 назад: сравнение по модулю
     * 256 со знаком. Иначе после 256-й загрузки линк вставал бы колом. */
    TEST_ASSERT_EQUAL_INT(G920_RX_NEW_SESSION,
                          g920_peer_rx_accept(&rx, 0, G920_FRAME_AUTH, 1));
    TEST_ASSERT_EQUAL_UINT8(0, rx.epoch);
}

static void test_rx_ignores_a_single_straggler_from_the_past(void)
{
    g920_peer_rx_t rx;

    g920_peer_rx_init(&rx);
    (void)g920_peer_rx_accept(&rx, 5, G920_FRAME_AUTH, 100);
    TEST_ASSERT_EQUAL_INT(G920_RX_NEW_SESSION,
                          g920_peer_rx_accept(&rx, 6, G920_FRAME_AUTH, 1));

    /*
     * Заблудившийся кадр прошлой эпохи. Принять его как новую сессию значило
     * бы утянуть метку назад, и дальше законные кадры шестой эпохи пошли бы
     * наверх повторно, каждый как «ещё одна новая сессия».
     */
    TEST_ASSERT_EQUAL_INT(G920_RX_FOREIGN,
                          g920_peer_rx_accept(&rx, 5, G920_FRAME_AUTH, 101));
    TEST_ASSERT_EQUAL_UINT8(6, rx.epoch);
    TEST_ASSERT_EQUAL_UINT32(1, rx.foreign);

    /* И подтверждать его нельзя: наверх он не пошёл. */
    TEST_ASSERT_FALSE(g920_rx_should_ack(G920_RX_FOREIGN));
    TEST_ASSERT_FALSE(g920_rx_should_deliver(G920_RX_FOREIGN));
}

static void test_rx_accepts_a_persistent_backward_epoch(void)
{
    g920_peer_rx_t rx;

    g920_peer_rx_init(&rx);
    (void)g920_peer_rx_accept(&rx, 200, G920_FRAME_INPUT, 5000);

    /*
     * У собеседника счётчик загрузок откатился — стёрся NVS, не сохранилась
     * запись, подменили модуль. Отказывать навсегда нельзя: цена ошибки
     * источника эпохи не должна быть пожизненной тишиной. Первые кадры
     * отбрасываются как чужие, но настойчивый поток принимается.
     *
     * 200 → 150 это откат на 50. Сравнение идёт по модулю 256 и выбирает
     * кратчайшее направление, поэтому «откатом» считается разница до 128:
     * 200 → 3 было бы **шагом вперёд** на 59, и это не ошибка, а свойство
     * кольцевого счётчика.
     */
    for (int i = 1; i < G920_RX_BACKWARD_FRAMES; i++) {
        TEST_ASSERT_EQUAL_INT(
            G920_RX_FOREIGN,
            g920_peer_rx_accept(&rx, 150, G920_FRAME_INPUT, (uint16_t)i));
    }
    TEST_ASSERT_EQUAL_INT(
        G920_RX_NEW_SESSION,
        g920_peer_rx_accept(&rx, 150, G920_FRAME_INPUT,
                            (uint16_t)G920_RX_BACKWARD_FRAMES));
    TEST_ASSERT_EQUAL_UINT8(150, rx.epoch);

    /* И дальше поток идёт как обычно, с чистого листа. */
    TEST_ASSERT_EQUAL_INT(
        G920_RX_NEW,
        g920_peer_rx_accept(&rx, 150, G920_FRAME_INPUT,
                            (uint16_t)(G920_RX_BACKWARD_FRAMES + 1)));
}

static void test_rx_live_session_cancels_backward_candidate(void)
{
    g920_peer_rx_t rx;

    g920_peer_rx_init(&rx);
    (void)g920_peer_rx_accept(&rx, 10, G920_FRAME_INPUT, 1);

    /* Два заблудившихся из прошлой эпохи вперемешку с живыми кадрами
     * текущей: накопиться до порога они не должны. */
    for (int round = 0; round < 5; round++) {
        TEST_ASSERT_EQUAL_INT(
            G920_RX_FOREIGN, g920_peer_rx_accept(&rx, 9, G920_FRAME_INPUT, 1));
        TEST_ASSERT_EQUAL_INT(
            G920_RX_FOREIGN, g920_peer_rx_accept(&rx, 9, G920_FRAME_INPUT, 2));
        TEST_ASSERT_EQUAL_INT(G920_RX_NEW,
                              g920_peer_rx_accept(&rx, 10, G920_FRAME_INPUT,
                                                  (uint16_t)(2 + round)));
    }
    TEST_ASSERT_EQUAL_UINT8(10, rx.epoch);
    TEST_ASSERT_EQUAL_UINT32(0, rx.sessions);
}

static void test_rx_streams_do_not_interfere(void)
{
    g920_peer_rx_t rx;

    g920_peer_rx_init(&rx);
    /* INPUT и FFB — разные потоки с общей нумерацией у отправителя. Один
     * трекер на двоих отбрасывал бы половину кадров как опоздавшие. */
    TEST_ASSERT_EQUAL_INT(G920_RX_NEW_SESSION,
                          g920_peer_rx_accept(&rx, 1, G920_FRAME_INPUT, 100));
    TEST_ASSERT_EQUAL_INT(G920_RX_NEW,
                          g920_peer_rx_accept(&rx, 1, G920_FRAME_FFB, 5));
    TEST_ASSERT_EQUAL_INT(G920_RX_NEW,
                          g920_peer_rx_accept(&rx, 1, G920_FRAME_INPUT, 101));
    TEST_ASSERT_EQUAL_INT(G920_RX_NEW,
                          g920_peer_rx_accept(&rx, 1, G920_FRAME_FFB, 6));
}

static void test_rx_fresh_restart_is_not_a_flood_of_stale(void)
{
    g920_peer_rx_t rx;

    g920_peer_rx_init(&rx);
    for (uint16_t seq = 1; seq <= 1000; seq++) {
        TEST_ASSERT_EQUAL_INT(
            (seq == 1) ? G920_RX_NEW_SESSION : G920_RX_NEW,
            g920_peer_rx_accept(&rx, 3, G920_FRAME_INPUT, seq));
    }

    /*
     * Собеседник перезагрузился. Без эпохи следующая тысяча кадров пошла бы
     * как опоздавшие — то есть руль на несколько секунд перестал бы отдавать
     * хосту новые положения, а FFB не доходил бы до руля с уже приложенными
     * силами. Тише и опаснее, чем оборванный auth.
     */
    TEST_ASSERT_EQUAL_INT(G920_RX_NEW_SESSION,
                          g920_peer_rx_accept(&rx, 4, G920_FRAME_INPUT, 1));
    TEST_ASSERT_EQUAL_INT(G920_RX_NEW,
                          g920_peer_rx_accept(&rx, 4, G920_FRAME_INPUT, 2));
    TEST_ASSERT_EQUAL_UINT32(1, rx.sessions);
}

static void test_rx_keeps_dropping_stale_within_a_session(void)
{
    g920_peer_rx_t rx;

    g920_peer_rx_init(&rx);
    TEST_ASSERT_EQUAL_INT(G920_RX_NEW_SESSION,
                          g920_peer_rx_accept(&rx, 1, G920_FRAME_INPUT, 10));
    /* Внутри сессии всё как было: опоздавший и повтор наверх не идут. */
    TEST_ASSERT_EQUAL_INT(G920_RX_STALE,
                          g920_peer_rx_accept(&rx, 1, G920_FRAME_INPUT, 9));
    TEST_ASSERT_EQUAL_INT(G920_RX_DUPLICATE,
                          g920_peer_rx_accept(&rx, 1, G920_FRAME_INPUT, 10));
    TEST_ASSERT_EQUAL_INT(G920_RX_NEW,
                          g920_peer_rx_accept(&rx, 1, G920_FRAME_INPUT, 11));
    TEST_ASSERT_EQUAL_UINT32(1, rx.stale);
    TEST_ASSERT_EQUAL_UINT32(1, rx.duplicates);
}

static void test_rx_verdict_policy(void)
{
    /* Дубликат подтверждаем — потерялось как раз подтверждение. Чужую
     * сессию нет: сказать «доставлено» про неотданное значит потерять
     * сообщение молча. */
    TEST_ASSERT_TRUE(g920_rx_should_ack(G920_RX_NEW));
    TEST_ASSERT_TRUE(g920_rx_should_ack(G920_RX_NEW_SESSION));
    TEST_ASSERT_TRUE(g920_rx_should_ack(G920_RX_DUPLICATE));
    TEST_ASSERT_FALSE(g920_rx_should_ack(G920_RX_STALE));
    TEST_ASSERT_FALSE(g920_rx_should_ack(G920_RX_FOREIGN));

    TEST_ASSERT_TRUE(g920_rx_should_deliver(G920_RX_NEW));
    TEST_ASSERT_TRUE(g920_rx_should_deliver(G920_RX_NEW_SESSION));
    TEST_ASSERT_FALSE(g920_rx_should_deliver(G920_RX_DUPLICATE));
    TEST_ASSERT_FALSE(g920_rx_should_deliver(G920_RX_STALE));
    TEST_ASSERT_FALSE(g920_rx_should_deliver(G920_RX_FOREIGN));

    TEST_ASSERT_EQUAL_STRING("NEW", g920_rx_verdict_name(G920_RX_NEW));
    TEST_ASSERT_EQUAL_STRING("FOREIGN", g920_rx_verdict_name(G920_RX_FOREIGN));
    TEST_ASSERT_EQUAL_STRING("?", g920_rx_verdict_name((g920_rx_verdict_t)99));
}

static void test_rx_bad_arguments(void)
{
    /* Без состояния наверх не отдаём и не подтверждаем: подавить лишнее
     * безопаснее, чем соврать про доставку. */
    TEST_ASSERT_EQUAL_INT(G920_RX_FOREIGN,
                          g920_peer_rx_accept(NULL, 1, G920_FRAME_AUTH, 1));
    g920_peer_rx_init(NULL);
}

/* --- две дисциплины бок о бок --------------------------------------------- */

static void test_auth_keeps_flowing_while_input_floods(void)
{
    /* Сценарий M8: поток отчётов идёт полным ходом, а security-обмен обязан
     * дойти. Свежая очередь при этом не растёт, а надёжная не теряется. */
    const uint8_t auth[] = { 0x06, 0x30, 0x02, 0x0E };
    const uint8_t input[] = { 0x20, 0x00 };
    uint64_t now = 0;
    bool auth_sent = false;

    TEST_ASSERT_TRUE(
        g920_reliable_push(&reliable, G920_FRAME_AUTH, 1, auth, sizeof(auth)));

    for (uint16_t i = 0; i < 500; i++) {
        g920_fresh_push(&fresh, G920_FRAME_INPUT, i, input, sizeof(input));

        /* Такт линка: сначала надёжное, потом свежее. */
        if (g920_reliable_due(&reliable, now, &out, out_buf, sizeof(out_buf))
            == 1) {
            TEST_ASSERT_EQUAL_UINT8(G920_FRAME_AUTH, out.type);
            auth_sent = true;
            g920_reliable_ack(&reliable, out.sequence, 0);
        }
        g920_fresh_pop(&fresh, &out, out_buf, sizeof(out_buf));
        now += 1000;
    }

    TEST_ASSERT_TRUE(auth_sent);
    TEST_ASSERT_EQUAL_UINT8(0, g920_reliable_pending(&reliable));
    TEST_ASSERT_EQUAL_UINT32(0, g920_reliable_gave_up(&reliable));
    /* Очередь отчётов так и не выросла — в ней всё это время был один слот. */
    TEST_ASSERT_TRUE(g920_fresh_empty(&fresh));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_fresh_newest_evicts_older);
    RUN_TEST(test_fresh_queue_never_grows);
    RUN_TEST(test_input_and_ffb_do_not_evict_each_other);
    RUN_TEST(test_fresh_refuses_reliable_types);
    RUN_TEST(test_fresh_bad_arguments);
    RUN_TEST(test_fresh_zero_length_payload);

    RUN_TEST(test_reliable_refuses_instead_of_evicting);
    RUN_TEST(test_reliable_takes_only_retriable_types);
    RUN_TEST(test_reliable_is_fifo);
    RUN_TEST(test_reliable_order_survives_slot_reuse);

    RUN_TEST(test_no_retry_before_timeout);
    RUN_TEST(test_retry_after_timeout_is_marked);
    RUN_TEST(test_ack_stops_retries);
    RUN_TEST(test_ack_closes_only_a_frame_that_went_on_air);
    RUN_TEST(test_gives_up_after_max_attempts);
    RUN_TEST(test_last_attempt_gets_its_own_timeout);
    RUN_TEST(test_custom_parameters_are_honoured);
    RUN_TEST(test_reliable_bad_arguments);


    RUN_TEST(test_rx_suppresses_only_the_repeat);
    RUN_TEST(test_rx_counts_the_gap_left_by_a_dropped_frame);
    RUN_TEST(test_rx_survives_sequence_wrap);
    RUN_TEST(test_rx_same_number_in_a_new_session_is_not_a_duplicate);
    RUN_TEST(test_rx_epoch_wraps_forward_at_255);
    RUN_TEST(test_rx_ignores_a_single_straggler_from_the_past);
    RUN_TEST(test_rx_accepts_a_persistent_backward_epoch);
    RUN_TEST(test_rx_live_session_cancels_backward_candidate);
    RUN_TEST(test_rx_streams_do_not_interfere);
    RUN_TEST(test_rx_fresh_restart_is_not_a_flood_of_stale);
    RUN_TEST(test_rx_keeps_dropping_stale_within_a_session);
    RUN_TEST(test_rx_verdict_policy);
    RUN_TEST(test_ack_from_a_past_session_does_not_close_the_record);
    RUN_TEST(test_rx_bad_arguments);

    RUN_TEST(test_auth_keeps_flowing_while_input_floods);

    return UNITY_END();
}
