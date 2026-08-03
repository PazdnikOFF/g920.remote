#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <unity.h>

#include "g920/link_rx.h"
#include "g920/proto.h"
#include "g920/queue.h"
#include "g920/version.h"

/*
 * Двухузловая модель линка: два состояния, канал с потерями, кадры —
 * байтами.
 *
 * ── Зачем именно так ───────────────────────────────────────────────────
 *
 * Критерий готовности M3 из ROADMAP: «надёжный канал переживает потерю 20%
 * кадров… с настоящей приёмной половиной (не её копией в тесте)». Прошлая
 * модель этого не выполняла: она дёргала `g920_peer_rx_accept`,
 * `g920_rx_should_ack` и `g920_reliable_ack` по отдельности и **сама
 * склеивала** их в тесте — то есть проверяла тракт, которого в прошивке
 * нет. Следствие проверялось на живом дереве: ACK нигде не существовал как
 * кадр, поэтому склейка в прошивке не была покрыта ничем, и один
 * перепутанный аргумент убивал надёжную доставку при зелёных тестах.
 *
 * Здесь узел делает ровно то же, что прошивка: собирает кадр в байты,
 * отдаёт в канал, принятое разбирает и прогоняет через
 * `g920_link_rx_on_frame`, а исход **исполняет** — шлёт готовый ответный
 * кадр, закрывает запись, отдаёт наверх. Ни одного решения в самом тесте.
 */

#define TX_ROLE ((uint8_t)G920_ROLE_TX)
#define RX_ROLE ((uint8_t)G920_ROLE_RX)

typedef struct {
    uint8_t mac[6];
    uint8_t role;
    uint8_t epoch;
    g920_link_rx_t rx;
    g920_reliable_queue_t reliable;
    uint16_t next_sequence;

    /* Что нам отдали наверх — по одному счётчику на всё, что важно. */
    uint32_t delivered;
    uint32_t delivered_seq[4096];
    uint32_t new_sessions;
} node_t;

static node_t a; /* TX */
static node_t b; /* RX */

/* --- канал ---------------------------------------------------------------- */

static uint32_t rnd_state;

static uint32_t rnd(void)
{
    rnd_state ^= rnd_state << 13;
    rnd_state ^= rnd_state >> 17;
    rnd_state ^= rnd_state << 5;
    return rnd_state;
}

static uint32_t loss_pct;
static uint64_t now_us;

static void node_init(node_t *n, const uint8_t *mac, uint8_t role,
                      uint8_t epoch)
{
    memset(n, 0, sizeof(*n));
    memcpy(n->mac, mac, 6);
    n->role = role;
    n->epoch = epoch;
    n->next_sequence = 1;
    g920_link_rx_init(&n->rx, role, 0);
    g920_link_rx_set_epoch(&n->rx, epoch);
    g920_reliable_init(&n->reliable, 0, 0);
    g920_reliable_set_epoch(&n->reliable, epoch);
}

static void deliver(node_t *to, const node_t *from, const uint8_t *bytes,
                    size_t len);

/* Кладёт кадр в эфир: собирает в байты и, если повезло, отдаёт получателю. */
static void transmit(node_t *from, node_t *to, const g920_frame_t *frame)
{
    uint8_t bytes[G920_FRAME_MAX];
    int built = g920_frame_build(bytes, sizeof(bytes), frame);

    TEST_ASSERT_TRUE(built > 0);
    if ((rnd() % 100u) < loss_pct) {
        return; /* потерялся */
    }
    deliver(to, from, bytes, (size_t)built);
}

/* Приём: разбор, решение, исполнение. Ровно как в прошивке. */
static void deliver(node_t *to, const node_t *from, const uint8_t *bytes,
                    size_t len)
{
    g920_frame_t frame;
    g920_link_rx_outcome_t out;

    if (g920_frame_parse(&frame, bytes, len) != G920_PROTO_OK) {
        return;
    }
    g920_link_rx_on_frame(&to->rx, &frame, from->mac, now_us, &out);

    if (out.close_reliable) {
        (void)g920_reliable_ack(&to->reliable, out.ack_sequence,
                                out.ack_epoch);
    }
    if (out.reset_reliable) {
        g920_reliable_init(&to->reliable, 0, 0);
        g920_reliable_set_epoch(&to->reliable, to->epoch);
    }
    if (out.send_reply) {
        /* Ответ идёт обратно тем же каналом и теми же потерями. */
        transmit(to, (node_t *)from, &out.reply);
    }
    if (out.deliver) {
        if (to->delivered < 4096) {
            to->delivered_seq[to->delivered] = frame.sequence;
        }
        to->delivered++;
        if (out.verdict == G920_RX_NEW_SESSION) {
            to->new_sessions++;
        }
    }
}

/* Такт линка: один надёжный кадр в эфир, отказ от безнадёжных. */
static void tick(node_t *from, node_t *to)
{
    g920_frame_t frame;
    uint8_t buf[64];

    if (g920_reliable_due(&from->reliable, now_us, &frame, buf, sizeof(buf))
        == 1) {
        frame.epoch = from->epoch;
        transmit(from, to, &frame);
    }
    (void)g920_reliable_expire(&from->reliable, now_us);
}

static void pair_nodes(void)
{
    uint8_t call[2] = { G920_LINK_DISCOVER_REQUEST, 0 };
    g920_frame_t frame;

    memset(&frame, 0, sizeof(frame));
    frame.version = G920_LINK_PROTO_VERSION;
    frame.type = G920_FRAME_DISCOVER;
    frame.sequence = 1;
    frame.epoch = a.epoch;
    call[1] = a.role;
    frame.payload = call;
    frame.length = 2;

    /* Пиринг без потерь: он не предмет этого теста. */
    {
        uint32_t saved = loss_pct;

        loss_pct = 0;
        transmit(&a, &b, &frame);
        loss_pct = saved;
    }
    TEST_ASSERT_TRUE(g920_link_rx_peer_alive(&a.rx));
    TEST_ASSERT_TRUE(g920_link_rx_peer_alive(&b.rx));
}

void setUp(void)
{
    static const uint8_t MAC_A[6] = { 0xAA, 0, 0, 0, 0, 1 };
    static const uint8_t MAC_B[6] = { 0xBB, 0, 0, 0, 0, 2 };

    rnd_state = 20260801u;
    loss_pct = 0;
    now_us = 0;
    node_init(&a, MAC_A, TX_ROLE, 5);
    node_init(&b, MAC_B, RX_ROLE, 9);
}

void tearDown(void) { }

/* --- собственно проверки -------------------------------------------------- */

static void test_pairing_makes_both_sides_alive(void)
{
    pair_nodes();
    TEST_ASSERT_EQUAL_UINT32(0, a.delivered);
    TEST_ASSERT_EQUAL_UINT32(0, b.delivered);
}

/* Гонит count надёжных сообщений от a к b. Возвращает «брошено». */
static uint32_t run_reliable(uint32_t count, uint32_t loss)
{
    const uint8_t payload[] = { 0xA5, 0x5A };
    uint32_t queued = 0;

    loss_pct = loss;
    for (uint32_t step = 0; step < count * 400u + 1000u; step++) {
        if (queued < count
            && g920_reliable_push(&a.reliable, G920_FRAME_AUTH,
                                  a.next_sequence, payload,
                                  sizeof(payload))) {
            a.next_sequence++;
            queued++;
        }
        tick(&a, &b);
        now_us += 1000;
        if (queued == count && g920_reliable_pending(&a.reliable) == 0) {
            break;
        }
    }
    return g920_reliable_gave_up(&a.reliable);
}

static void test_reliable_survives_20_percent_loss(void)
{
    uint32_t gave_up;

    pair_nodes();
    gave_up = run_reliable(200, 20);

    /*
     * «Готово когда» вехи. Ни одного брошенного кадра и ни одной повторной
     * выдачи наверх: подтверждения ходят по тому же каналу с теми же
     * потерями, и склейка «вердикт → ACK → закрытие записи» — та самая,
     * что исполняет прошивка.
     */
    TEST_ASSERT_EQUAL_UINT32(0, gave_up);
    TEST_ASSERT_EQUAL_UINT32(200, b.delivered);
    /* Номера строго по порядку, без повторов и перестановок. */
    for (uint32_t i = 0; i < 200; i++) {
        TEST_ASSERT_EQUAL_UINT32(i + 1, b.delivered_seq[i]);
    }
    /* Ровно одна сессия: перезапуска не было. */
    TEST_ASSERT_EQUAL_UINT32(1, b.new_sessions);
}

static void test_margin_beyond_the_milestone_requirement(void)
{
    uint32_t gave_up;

    /*
     * За границей требования вехи запас есть, но не бесконечный: при 35% в
     * обе стороны (42% успеха на попытку) одиночные отказы появляются.
     * Записано честно — ранняя формулировка «двукратный запас» была
     * артефактом слабого генератора.
     */
    pair_nodes();
    gave_up = run_reliable(200, 35);
    TEST_ASSERT_TRUE(gave_up <= 4);
    /* Что дошло — дошло по разу и по порядку. */
    for (uint32_t i = 1; i < b.delivered; i++) {
        TEST_ASSERT_TRUE(b.delivered_seq[i] > b.delivered_seq[i - 1]);
    }
}

/*
 * Перебор бюджета попыток — тот самый, которым обоснованы
 * G920_RELIABLE_MAX_ATTEMPTS и таблица в queue.h. Он существует потому, что
 * таблица там стояла от прошлой, удалённой модели: числа, на которые
 * ссылается умолчание, обязаны быть воспроизводимы тем же деревом, иначе
 * умолчание не обосновано ничем, а выглядит обоснованным.
 *
 * Зерно и порядок вызовов фиксированы, поэтому строки сравнимы между собой:
 * меняется только число попыток.
 */
static uint32_t sweep(uint8_t attempts, uint32_t loss, uint32_t count)
{
    setUp();
    pair_nodes();
    /*
     * Параметры — **после** пиринга, и это не мелочь: усыновление пира
     * сбрасывает очередь повторов (она адресована прежнему собеседнику), а
     * сброс берёт умолчания. Заданный до пиринга бюджет попыток стирался
     * молча, и все три строки таблицы выходили одинаковыми — перебор
     * измерял умолчание, называя его перебором.
     */
    g920_reliable_init(&a.reliable, 0, attempts);
    g920_reliable_set_epoch(&a.reliable, a.epoch);
    return run_reliable(count, loss);
}

static void test_attempt_budget_table_is_reproducible(void)
{
    const uint32_t COUNT = 2000;
    uint32_t at20[3];
    uint32_t at35[3];
    const uint8_t ATTEMPTS[3] = { 5, 10, 15 };

    for (int i = 0; i < 3; i++) {
        at20[i] = sweep(ATTEMPTS[i], 20, COUNT);
        at35[i] = sweep(ATTEMPTS[i], 35, COUNT);
    }

    /* Числа, попадающие в таблицу queue.h. */
    printf("\n  attempts  gave up @20%%  @35%%  (of %u)\n", (unsigned)COUNT);
    for (int i = 0; i < 3; i++) {
        printf("  %8u  %12u  %4u\n", (unsigned)ATTEMPTS[i],
               (unsigned)at20[i], (unsigned)at35[i]);
    }

    /* Требование вехи: при 20% в обе стороны умолчание не бросает ничего. */
    TEST_ASSERT_EQUAL_UINT32(0, at20[2]);
    /* Настойчивость помогает монотонно — иначе таблица описывала бы шум. */
    TEST_ASSERT_TRUE(at20[0] >= at20[1]);
    TEST_ASSERT_TRUE(at20[1] >= at20[2]);
    TEST_ASSERT_TRUE(at35[0] >= at35[1]);
    TEST_ASSERT_TRUE(at35[1] >= at35[2]);
    /* За границей требования запаса нет: на 35% отказы остаются и у
     * пятнадцати попыток. Ровный ноль здесь однажды уже оказался артефактом
     * слабого генератора, и записывать его снова незачем. */
    TEST_ASSERT_TRUE(at35[2] > 0);
}

static void test_dead_channel_gives_up_instead_of_hanging(void)
{
    uint32_t gave_up;

    pair_nodes();
    gave_up = run_reliable(5, 100);

    TEST_ASSERT_TRUE(gave_up > 0);
    TEST_ASSERT_EQUAL_UINT32(0, b.delivered);
}

static void test_sender_restart_mid_exchange(void)
{
    pair_nodes();
    TEST_ASSERT_EQUAL_UINT32(0, run_reliable(50, 20));
    TEST_ASSERT_EQUAL_UINT32(50, b.delivered);

    /*
     * Руль перезагрузился: эпоха вперёд, номера с единицы. Без эпохи
     * приёмник принял бы их за опоздавшие и молча выбросил, подтверждая.
     */
    a.epoch = 6;
    a.next_sequence = 1;
    g920_link_rx_set_epoch(&a.rx, a.epoch);
    g920_reliable_init(&a.reliable, 0, 0);
    g920_reliable_set_epoch(&a.reliable, a.epoch);

    TEST_ASSERT_EQUAL_UINT32(0, run_reliable(50, 20));
    TEST_ASSERT_EQUAL_UINT32(100, b.delivered);
    /* Вторая сессия замечена, и это единственный повод для повторной
     * нумерации. */
    TEST_ASSERT_EQUAL_UINT32(2, b.new_sessions);
}

/*
 * Перезагрузка приёмника. Донгл питается от шины USB, и засыпание хоста
 * перезапускает его штатно — это не авария. Приёмное состояние чистое,
 * эпоха своя новая, адрес пира поднят из NVS (но пир ещё не сказал ни
 * слова). Отправитель об этом не узнаёт ничем: встречной эпохи в протоколе
 * нет, и это записано в queue.h как принятая цена.
 *
 * Счётчики узла (delivered, new_sessions) — ведомость теста, а не состояние
 * платы: настоящий донгл теряет их вместе со всем остальным, но нам нужно
 * сосчитать выдачи по обе стороны перезапуска.
 */
static void restart_receiver(uint8_t new_epoch)
{
    b.epoch = new_epoch;
    g920_link_rx_init(&b.rx, b.role, 0);
    g920_link_rx_set_epoch(&b.rx, b.epoch);
    g920_link_rx_preset_peer(&b.rx, a.mac);
    g920_reliable_init(&b.reliable, 0, 0);
    g920_reliable_set_epoch(&b.reliable, b.epoch);
}

static void test_receiver_restart_mid_exchange(void)
{
    const uint8_t payload[] = { 0x11, 0x22 };
    uint32_t queued = 0;
    bool restarted = false;

    /*
     * ROADMAP требует перезапуск **любой** из сторон. Отправитель —
     * в test_sender_restart_mid_exchange; здесь перезагружается приёмник, и
     * до сих пор этот случай закрывался ручной склейкой кусков API в
     * test_queue: без кадра ACK, без g920_link_rx_on_frame и без потерь.
     */
    pair_nodes();
    TEST_ASSERT_EQUAL_UINT32(0, run_reliable(50, 20));
    TEST_ASSERT_EQUAL_UINT32(50, b.delivered);

    loss_pct = 20;
    for (uint32_t step = 0; step < 20000u; step++) {
        if (queued < 20
            && g920_reliable_push(&a.reliable, G920_FRAME_AUTH,
                                  a.next_sequence, payload,
                                  sizeof(payload))) {
            a.next_sequence++;
            queued++;
        }
        tick(&a, &b);
        now_us += 1000;
        /* Ровно посреди обмена, а не в паузе между сообщениями: интересен
         * кадр, который уже доехал, но чьё подтверждение ещё в пути. */
        if (!restarted && queued >= 10) {
            restart_receiver(200);
            restarted = true;
        }
        if (queued == 20 && g920_reliable_pending(&a.reliable) == 0) {
            break;
        }
    }

    TEST_ASSERT_TRUE(restarted);
    /* Отправитель не бросил ни одного кадра: перезапуск собеседника — это
     * не потеря связи, ретраи доезжают в чистое состояние. */
    TEST_ASSERT_EQUAL_UINT32(0, g920_reliable_gave_up(&a.reliable));
    /*
     * Всё отправленное доехало, но не обязательно по разу: доставка «не
     * реже одного раза». Повторно уйти наверх может **не больше одного**
     * сообщения — окно отправки один кадр, и в полёте на момент перезапуска
     * больше одного быть не может. Верхняя граница здесь и есть та цена,
     * которую queue.h называет принятой.
     */
    TEST_ASSERT_TRUE(b.delivered >= 70);
    TEST_ASSERT_TRUE(b.delivered <= 71);
    /* Первая сессия — пиринг, вторая — после перезагрузки. */
    TEST_ASSERT_EQUAL_UINT32(2, b.new_sessions);

    /* И связь не оборвана навсегда: обмен продолжается как обычно. */
    {
        uint32_t before = b.delivered;

        TEST_ASSERT_EQUAL_UINT32(0, run_reliable(10, 20));
        TEST_ASSERT_EQUAL_UINT32(before + 10, b.delivered);
        TEST_ASSERT_EQUAL_UINT32(2, b.new_sessions);
    }
}

static void test_peer_swap_mid_reliable_exchange(void)
{
    static const uint8_t MAC_C[6] = { 0xCC, 0, 0, 0, 0, 3 };
    node_t c;
    uint8_t call[2] = { G920_LINK_DISCOVER_REQUEST, TX_ROLE };
    g920_frame_t frame;
    uint32_t delivered_before;

    pair_nodes();
    TEST_ASSERT_EQUAL_UINT32(0, run_reliable(20, 0));
    delivered_before = b.delivered;

    /* Замену платы пиринг и очередь повторов до сих пор проверялись
     * порознь — а ломается ровно их стык. */
    node_init(&c, MAC_C, TX_ROLE, 200);
    now_us += (uint64_t)G920_PEER_RELEASE_MS * 1000u + 1;

    memset(&frame, 0, sizeof(frame));
    frame.version = G920_LINK_PROTO_VERSION;
    frame.type = G920_FRAME_DISCOVER;
    frame.sequence = 1;
    frame.epoch = c.epoch;
    frame.payload = call;
    frame.length = 2;
    loss_pct = 0;
    transmit(&c, &b, &frame);

    TEST_ASSERT_TRUE(g920_link_rx_peer_alive(&c.rx));

    /* Новая плата шлёт с номерами от единицы и меньшей эпохой. */
    {
        const uint8_t payload[] = { 1 };
        uint32_t queued = 0;

        for (uint32_t step = 0; step < 10000u; step++) {
            if (queued < 10
                && g920_reliable_push(&c.reliable, G920_FRAME_AUTH,
                                      c.next_sequence, payload, 1)) {
                c.next_sequence++;
                queued++;
            }
            tick(&c, &b);
            now_us += 1000;
            if (queued == 10 && g920_reliable_pending(&c.reliable) == 0) {
                break;
            }
        }
        TEST_ASSERT_EQUAL_UINT32(0, g920_reliable_gave_up(&c.reliable));
    }
    TEST_ASSERT_EQUAL_UINT32(delivered_before + 10, b.delivered);
}

static void test_stranger_cannot_break_a_live_pair(void)
{
    static const uint8_t MAC_C[6] = { 0xCC, 0, 0, 0, 0, 3 };
    node_t c;
    uint8_t call[2] = { G920_LINK_DISCOVER_REQUEST, TX_ROLE };
    g920_frame_t frame;

    pair_nodes();
    node_init(&c, MAC_C, TX_ROLE, 200);

    memset(&frame, 0, sizeof(frame));
    frame.version = G920_LINK_PROTO_VERSION;
    frame.type = G920_FRAME_DISCOVER;
    frame.sequence = 1;
    frame.epoch = c.epoch;
    frame.payload = call;
    frame.length = 2;
    loss_pct = 0;
    transmit(&c, &b, &frame);

    /* Второй комплект в комнате: пока свой пир жив, чужому не отвечают, и
     * он остаётся без пары — а не отбирает нашего. */
    TEST_ASSERT_FALSE(g920_link_rx_peer_alive(&c.rx));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(a.mac, g920_peering_peer(&b.rx.peering), 6);

    /* И обмен со своим продолжается как ни в чём не бывало. */
    TEST_ASSERT_EQUAL_UINT32(0, run_reliable(10, 20));
    TEST_ASSERT_EQUAL_UINT32(10, b.delivered);
}

static void test_fresh_stream_survives_loss_without_retries(void)
{
    g920_frame_t frame;
    uint8_t payload[8] = { 0 };

    pair_nodes();
    loss_pct = 20;

    memset(&frame, 0, sizeof(frame));
    frame.version = G920_LINK_PROTO_VERSION;
    frame.type = G920_FRAME_INPUT;
    frame.epoch = a.epoch;
    frame.payload = payload;
    frame.length = sizeof(payload);

    for (uint16_t seq = 1; seq <= 1000; seq++) {
        frame.sequence = seq;
        transmit(&a, &b, &frame);
        now_us += 1000;
    }

    /* Свежая дисциплина к потерям устойчива по построению: ретраев нет,
     * дошло примерно 80%, и ни один кадр не отдан дважды. */
    TEST_ASSERT_TRUE(b.delivered > 700);
    TEST_ASSERT_TRUE(b.delivered < 900);
    for (uint32_t i = 1; i < b.delivered && i < 4096; i++) {
        TEST_ASSERT_TRUE(b.delivered_seq[i] > b.delivered_seq[i - 1]);
    }
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_pairing_makes_both_sides_alive);
    RUN_TEST(test_reliable_survives_20_percent_loss);
    RUN_TEST(test_margin_beyond_the_milestone_requirement);
    RUN_TEST(test_attempt_budget_table_is_reproducible);
    RUN_TEST(test_dead_channel_gives_up_instead_of_hanging);
    RUN_TEST(test_sender_restart_mid_exchange);
    RUN_TEST(test_receiver_restart_mid_exchange);
    RUN_TEST(test_peer_swap_mid_reliable_exchange);
    RUN_TEST(test_stranger_cannot_break_a_live_pair);
    RUN_TEST(test_fresh_stream_survives_loss_without_retries);

    return UNITY_END();
}
