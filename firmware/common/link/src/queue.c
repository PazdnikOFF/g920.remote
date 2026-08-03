#include "g920/queue.h"

#include <string.h>

#include "g920/version.h"
#include "g920/hot.h"

/* --- свежая дисциплина --------------------------------------------------- */

void g920_fresh_init(g920_fresh_queue_t *queue)
{
    if (queue == NULL) {
        return;
    }
    memset(queue, 0, sizeof(*queue));
}

/*
 * Слот на тип: INPUT и FFB не должны вытеснять друг друга. Таблица одна на
 * очередь отправки и на трекеры приёма — разъехавшись, они дали бы ровно
 * ту «половину кадров как опоздавшие», против которой заведён тест
 * test_rx_streams_do_not_interfere.
 */
static int fresh_slot_of(g920_frame_type_t type)
{
    switch (type) {
    case G920_FRAME_INPUT:
        return 0;
    case G920_FRAME_FFB:
        return 1;
    default:
        return -1;
    }
}

G920_HOT bool g920_fresh_push(g920_fresh_queue_t *queue, g920_frame_type_t type,
                     uint16_t sequence, const uint8_t *payload,
                     uint16_t length)
{
    int index;
    g920_fresh_slot_t *slot;

    if (queue == NULL) {
        return false;
    }
    /* Тип не свежей дисциплины сюда попасть не может: перепутанная очередь
     * это молча отключённые ретраи у AUTH. */
    index = fresh_slot_of(type);
    if (index < 0) {
        return false;
    }
    if (length > G920_FRESH_PAYLOAD_MAX) {
        return false;
    }
    if (payload == NULL && length != 0) {
        return false;
    }

    slot = &queue->slots[index];
    if (slot->occupied) {
        /* Предыдущее состояние так и не уехало и уже неактуально. */
        queue->evicted++;
    }
    slot->occupied = true;
    slot->type = (uint8_t)type;
    slot->sequence = sequence;
    slot->length = length;
    if (length != 0) {
        memcpy(slot->payload, payload, length);
    }
    return true;
}

G920_HOT int g920_fresh_pop(g920_fresh_queue_t *queue, g920_frame_t *out, uint8_t *buf,
                   size_t size)
{
    int best = -1;

    if (queue == NULL || out == NULL || buf == NULL) {
        return -1;
    }

    for (int i = 0; i < G920_FRESH_STREAMS; i++) {
        if (!queue->slots[i].occupied) {
            continue;
        }
        if (best < 0
            || g920_frame_priority((g920_frame_type_t)queue->slots[i].type)
                   < g920_frame_priority(
                       (g920_frame_type_t)queue->slots[best].type)) {
            best = i;
        }
    }
    if (best < 0) {
        return 0;
    }
    if (size < queue->slots[best].length) {
        return -1;
    }

    g920_fresh_slot_t *slot = &queue->slots[best];

    if (slot->length != 0) {
        memcpy(buf, slot->payload, slot->length);
    }
    out->version = G920_LINK_PROTO_VERSION;
    out->type = slot->type;
    out->sequence = slot->sequence;
    out->flags = 0;
    out->length = slot->length;
    out->payload = (slot->length != 0) ? buf : NULL;

    slot->occupied = false;
    return 1;
}

bool g920_fresh_empty(const g920_fresh_queue_t *queue)
{
    if (queue == NULL) {
        return true;
    }
    for (int i = 0; i < G920_FRESH_STREAMS; i++) {
        if (queue->slots[i].occupied) {
            return false;
        }
    }
    return true;
}

uint32_t g920_fresh_evicted(const g920_fresh_queue_t *queue)
{
    return (queue != NULL) ? queue->evicted : 0;
}

/* --- надёжная дисциплина -------------------------------------------------- */

void g920_reliable_init(g920_reliable_queue_t *queue, uint32_t retry_us,
                        uint8_t max_attempts)
{
    if (queue == NULL) {
        return;
    }
    memset(queue, 0, sizeof(*queue));
    queue->retry_us = (retry_us != 0) ? retry_us : G920_RELIABLE_RETRY_US;
    queue->max_attempts =
        (max_attempts != 0) ? max_attempts : G920_RELIABLE_MAX_ATTEMPTS;
}

void g920_reliable_set_epoch(g920_reliable_queue_t *queue, uint8_t epoch)
{
    if (queue != NULL) {
        queue->epoch = epoch;
    }
}

bool g920_reliable_push(g920_reliable_queue_t *queue, g920_frame_type_t type,
                        uint16_t sequence, const uint8_t *payload,
                        uint16_t length)
{
    if (queue == NULL) {
        return false;
    }
    /*
     * Белый список, а не «всё надёжное». По классу доставки надёжны также
     * ACK и DISCOVER, но повторять их нельзя: подтверждение на подтверждение
     * никто не пришлёт, и кадр будет пересылаться до отказа.
     */
    if (type != G920_FRAME_AUTH && type != G920_FRAME_CONTROL
        && type != G920_FRAME_DESCRIPTOR) {
        return false;
    }
    if (length > G920_RELIABLE_PAYLOAD_MAX) {
        return false;
    }
    if (payload == NULL && length != 0) {
        return false;
    }

    for (int i = 0; i < G920_RELIABLE_DEPTH; i++) {
        g920_reliable_entry_t *e = &queue->entries[i];

        if (e->occupied) {
            continue;
        }
        e->occupied = true;
        e->type = (uint8_t)type;
        e->sequence = sequence;
        e->length = length;
        e->attempts = 0;
        e->sent_at_us = 0;
        if (length != 0) {
            memcpy(e->payload, payload, length);
        }
        queue->tickets[i] = queue->ticket++;
        return true;
    }

    /* Мест нет. Вытеснять нечего: здесь важна полнота, а не свежесть. */
    return false;
}

int g920_reliable_due(g920_reliable_queue_t *queue, uint64_t now_us,
                      g920_frame_t *out, uint8_t *buf, size_t size)
{
    int best = -1;

    if (queue == NULL || out == NULL || buf == NULL) {
        return -1;
    }

    /*
     * Окно отправки — ровно один кадр.
     *
     * Ретрай по построению порождает и дубликат (кадр дошёл, подтверждение
     * потерялось), и перестановку (третья попытка первого сообщения
     * приходит после второго). Разрешить нескольким кадрам быть в полёте
     * значит переложить разбор обоих беспорядков на приёмник — то есть
     * завести окно приёма, буфер пересборки и всё, что к ним прилагается.
     *
     * При окне в один кадр порядок на проводе гарантирован отправителем, а
     * приёмнику остаётся отличить повтор от нового — одно сравнение.
     *
     * Цена: одно сообщение за круг, около 430 в секунду при RTT 2.3 мс.
     * Надёжная дисциплина возит auth (десятки сообщений) и дескрипторы
     * (однократно) — этого хватает с запасом. Отчёты и силы едут свежей
     * дисциплиной и сюда не попадают.
     */
    for (int i = 0; i < G920_RELIABLE_DEPTH; i++) {
        if (!queue->entries[i].occupied) {
            continue;
        }
        if (best < 0 || queue->tickets[i] < queue->tickets[best]) {
            best = i;
        }
    }
    if (best < 0) {
        return 0;
    }
    {
        const g920_reliable_entry_t *e = &queue->entries[best];

        if (e->attempts >= queue->max_attempts) {
            return 0; /* исчерпан; выбросит g920_reliable_expire */
        }
        if (e->attempts != 0
            && (now_us < e->sent_at_us
                || (now_us - e->sent_at_us) < queue->retry_us)) {
            return 0; /* ждём подтверждения на предыдущую попытку */
        }
    }

    g920_reliable_entry_t *e = &queue->entries[best];

    if (size < e->length) {
        return -1;
    }
    if (e->length != 0) {
        memcpy(buf, e->payload, e->length);
    }
    out->version = G920_LINK_PROTO_VERSION;
    out->type = e->type;
    out->sequence = e->sequence;
    out->flags = (e->attempts != 0) ? G920_FRAME_FLAG_RETRY : 0;
    out->length = e->length;
    out->payload = (e->length != 0) ? buf : NULL;

    if (e->attempts != 0) {
        queue->retries++;
    }
    e->attempts++;
    e->sent_at_us = now_us;
    return 1;
}

bool g920_reliable_ack(g920_reliable_queue_t *queue, uint16_t sequence,
                       uint8_t epoch)
{
    if (queue == NULL) {
        return false;
    }
    /* Подтверждение чужой эпохи — эхо прошлой сессии: наши номера начались
     * заново, и закрыть оно могло бы совсем не ту запись. */
    if (epoch != queue->epoch) {
        return false;
    }
    for (int i = 0; i < G920_RELIABLE_DEPTH; i++) {
        g920_reliable_entry_t *e = &queue->entries[i];

        /* Подтвердить можно только то, что уже уходило в эфир. Номера у
         * трёх типов общие, и ACK типа не несёт: без этой проверки
         * подтверждение закрывало бы ещё не отправленную запись, чей номер
         * случайно совпал после переполнения счётчика. */
        if (e->occupied && e->attempts != 0 && e->sequence == sequence) {
            e->occupied = false;
            return true;
        }
    }
    return false;
}

uint8_t g920_reliable_expire(g920_reliable_queue_t *queue, uint64_t now_us)
{
    uint8_t dropped = 0;

    if (queue == NULL) {
        return 0;
    }
    for (int i = 0; i < G920_RELIABLE_DEPTH; i++) {
        g920_reliable_entry_t *e = &queue->entries[i];

        if (!e->occupied || e->attempts < queue->max_attempts) {
            continue;
        }
        /* Последней попытке тоже полагается таймаут: иначе кадр
         * выбрасывался бы, не дождавшись подтверждения на неё. */
        if (now_us < e->sent_at_us
            || (now_us - e->sent_at_us) < queue->retry_us) {
            continue;
        }
        e->occupied = false;
        queue->gave_up++;
        dropped++;
    }
    return dropped;
}

uint8_t g920_reliable_pending(const g920_reliable_queue_t *queue)
{
    uint8_t n = 0;

    if (queue == NULL) {
        return 0;
    }
    for (int i = 0; i < G920_RELIABLE_DEPTH; i++) {
        if (queue->entries[i].occupied) {
            n++;
        }
    }
    return n;
}

bool g920_reliable_full(const g920_reliable_queue_t *queue)
{
    return g920_reliable_pending(queue) >= G920_RELIABLE_DEPTH;
}

uint32_t g920_reliable_retries(const g920_reliable_queue_t *queue)
{
    return (queue != NULL) ? queue->retries : 0;
}

uint32_t g920_reliable_gave_up(const g920_reliable_queue_t *queue)
{
    return (queue != NULL) ? queue->gave_up : 0;
}

/* --- приём: одно состояние на пира --------------------------------------- */

void g920_peer_rx_init(g920_peer_rx_t *rx)
{
    if (rx == NULL) {
        return;
    }
    memset(rx, 0, sizeof(*rx));
    for (int i = 0; i < G920_FRESH_STREAMS; i++) {
        g920_seq_tracker_init(&rx->fresh[i]);
    }
}

/* Новая сессия собеседника: всё, что помнили о прошлой, больше не значит
 * ничего. */
static void adopt_session(g920_peer_rx_t *rx, uint8_t epoch, bool first)
{
    for (int i = 0; i < G920_FRESH_STREAMS; i++) {
        g920_seq_tracker_init(&rx->fresh[i]);
    }
    rx->reliable_started = false;
    rx->reliable_last = 0;
    rx->epoch = epoch;
    rx->started = true;
    rx->backward_epoch = 0;
    rx->backward_seen = 0;
    if (!first) {
        rx->sessions++;
    }
}

/*
 * Чужая эпоха. Возвращает true, если сессию приняли; false — кадр пока
 * чужой.
 */
static bool consider_foreign_epoch(g920_peer_rx_t *rx, uint8_t epoch)
{
    if ((int8_t)(epoch - rx->epoch) > 0) {
        /* Счётчик загрузок вырос: собеседник перезагрузился. */
        adopt_session(rx, epoch, false);
        return true;
    }

    /*
     * Откат. Одиночка — это заблудившийся кадр прошлой сессии, и принимать
     * его нельзя. Но настойчивый поток означает, что счётчик собеседника
     * действительно откатился, и отказывать навсегда нельзя тоже.
     */
    if (rx->backward_epoch != epoch || rx->backward_seen == 0) {
        rx->backward_epoch = epoch;
        rx->backward_seen = 1;
    } else if (rx->backward_seen < 0xFF) {
        rx->backward_seen++;
    }
    if (rx->backward_seen >= G920_RX_BACKWARD_FRAMES) {
        adopt_session(rx, epoch, false);
        return true;
    }
    return false;
}

G920_HOT g920_rx_verdict_t g920_peer_rx_accept(g920_peer_rx_t *rx,
                                               uint8_t epoch,
                                               g920_frame_type_t type,
                                               uint16_t sequence)
{
    bool new_session = false;
    int stream;

    if (rx == NULL) {
        return G920_RX_FOREIGN; /* нет состояния — наверх не отдаём */
    }

    if (!rx->started) {
        adopt_session(rx, epoch, true);
        new_session = true;
    } else if (epoch != rx->epoch) {
        if (!consider_foreign_epoch(rx, epoch)) {
            rx->foreign++;
            return G920_RX_FOREIGN;
        }
        new_session = true;
    } else if (rx->backward_seen != 0) {
        /* Пришёл кадр текущей сессии — значит она жива, и накопленные
         * кандидаты на откат больше не в счёт. */
        rx->backward_seen = 0;
        rx->backward_epoch = 0;
    }

    stream = fresh_slot_of(type);
    if (stream >= 0) {
        g920_seq_verdict_t verdict =
            g920_seq_track(&rx->fresh[stream], sequence);

        if (verdict == G920_SEQ_DUPLICATE) {
            rx->duplicates++;
            return G920_RX_DUPLICATE;
        }
        if (verdict == G920_SEQ_STALE) {
            rx->stale++;
            return G920_RX_STALE;
        }
        rx->delivered++;
        return new_session ? G920_RX_NEW_SESSION : G920_RX_NEW;
    }

    /* Надёжная дисциплина: окно отправки один кадр, поэтому «уже видели» —
     * это ровно «не новее последнего». */
    if (rx->reliable_started && !g920_seq_newer(sequence, rx->reliable_last)) {
        rx->duplicates++;
        return G920_RX_DUPLICATE;
    }
    if (rx->reliable_started
        && (uint16_t)(rx->reliable_last + 1u) != sequence) {
        /* Отправитель бросил кадр, исчерпав попытки. Восстановить нечем, но
         * молчать нельзя. */
        rx->gaps++;
    }
    rx->reliable_last = sequence;
    rx->reliable_started = true;
    rx->delivered++;
    return new_session ? G920_RX_NEW_SESSION : G920_RX_NEW;
}

bool g920_rx_should_ack(g920_rx_verdict_t verdict)
{
    /*
     * Дубликат подтверждаем: раз кадр пришёл повторно, потерялось как раз
     * подтверждение. Чужую сессию — нет: сказать «доставлено» про то, чего
     * не отдали наверх, значит потерять сообщение молча.
     */
    return verdict == G920_RX_NEW || verdict == G920_RX_NEW_SESSION
           || verdict == G920_RX_DUPLICATE;
}

bool g920_rx_should_deliver(g920_rx_verdict_t verdict)
{
    return verdict == G920_RX_NEW || verdict == G920_RX_NEW_SESSION;
}

const char *g920_rx_verdict_name(g920_rx_verdict_t verdict)
{
    switch (verdict) {
    case G920_RX_NEW:
        return "NEW";
    case G920_RX_NEW_SESSION:
        return "NEW_SESSION";
    case G920_RX_DUPLICATE:
        return "DUPLICATE";
    case G920_RX_STALE:
        return "STALE";
    case G920_RX_FOREIGN:
        return "FOREIGN";
    default:
        return "?";
    }
}
