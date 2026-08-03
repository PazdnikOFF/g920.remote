#include "g920/proto.h"

#include <string.h>

#include "g920/version.h"
#include "g920/hot.h"

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* --- кадр ---------------------------------------------------------------- */

G920_HOT int g920_frame_build(uint8_t *buf, size_t size, const g920_frame_t *frame)
{
    size_t total;

    if (buf == NULL || frame == NULL) {
        return -1;
    }
    if (!g920_frame_type_valid(frame->type)) {
        return -1;
    }
    if (frame->length > G920_FRAME_PAYLOAD_MAX) {
        return -1;
    }
    if (frame->payload == NULL && frame->length != 0) {
        return -1;
    }

    total = (size_t)G920_FRAME_HEADER_SIZE + frame->length;
    if (size < total) {
        return -1;
    }

    buf[0] = frame->version;
    buf[1] = frame->type;
    put_u16(buf + 2, frame->sequence);
    buf[4] = frame->flags;
    buf[5] = frame->epoch;
    put_u16(buf + 6, frame->length);
    if (frame->length != 0) {
        memcpy(buf + G920_FRAME_HEADER_SIZE, frame->payload, frame->length);
    }
    return (int)total;
}

G920_HOT g920_proto_status_t g920_frame_parse(g920_frame_t *out, const uint8_t *buf,
                                     size_t size)
{
    uint16_t length;

    if (out == NULL || buf == NULL) {
        return G920_PROTO_BAD_ARG;
    }
    if (size < G920_FRAME_HEADER_SIZE) {
        return G920_PROTO_TRUNCATED;
    }

    /*
     * Версия проверяется раньше всего остального: кадр чужой версии нельзя
     * разбирать по нашей раскладке даже «на всякий случай». Ноль означает
     * «протокола нет» и несовместим ни с чем, включая себя, —
     * см. g920_link_proto_compatible().
     */
    if (!g920_link_proto_compatible(G920_LINK_PROTO_VERSION, buf[0])) {
        return G920_PROTO_BAD_VERSION;
    }
    if (!g920_frame_type_valid(buf[1])) {
        return G920_PROTO_BAD_TYPE;
    }

    length = get_u16(buf + 6);
    if (length > G920_FRAME_PAYLOAD_MAX) {
        return G920_PROTO_BAD_LENGTH;
    }
    if (size < (size_t)G920_FRAME_HEADER_SIZE + length) {
        return G920_PROTO_TRUNCATED;
    }

    out->version = buf[0];
    out->type = buf[1];
    out->sequence = get_u16(buf + 2);
    out->flags = buf[4];
    out->epoch = buf[5];
    out->length = length;
    out->payload = (length != 0) ? (buf + G920_FRAME_HEADER_SIZE) : NULL;
    return G920_PROTO_OK;
}

/* --- свойства типов ------------------------------------------------------ */

G920_HOT g920_delivery_t g920_frame_delivery(g920_frame_type_t type)
{
    switch (type) {
    case G920_FRAME_INPUT:
    case G920_FRAME_FFB:
        return G920_DELIVERY_FRESH;
    case G920_FRAME_ACK:
    case G920_FRAME_DISCOVER:
        return G920_DELIVERY_SERVICE;
    default:
        /* AUTH, CONTROL, DESCRIPTOR. */
        return G920_DELIVERY_RELIABLE;
    }
}

G920_HOT uint8_t g920_frame_priority(g920_frame_type_t type)
{
    switch (type) {
    case G920_FRAME_ACK:
        return 0;
    case G920_FRAME_AUTH:
        return 1;
    case G920_FRAME_CONTROL:
        return 2;
    case G920_FRAME_DESCRIPTOR:
        return 3;
    case G920_FRAME_INPUT:
        return 4;
    case G920_FRAME_FFB:
        return 5;
    case G920_FRAME_DISCOVER:
        return 6; /* хозяйственный кадр, вперёд данных не лезет */
    default:
        return 0xFF;
    }
}

bool g920_frame_type_valid(uint8_t type)
{
    return type >= (uint8_t)G920_FRAME_ACK
           && type <= (uint8_t)G920_FRAME_DISCOVER;
}

const char *g920_frame_type_name(g920_frame_type_t type)
{
    switch (type) {
    case G920_FRAME_ACK:
        return "ACK";
    case G920_FRAME_AUTH:
        return "AUTH";
    case G920_FRAME_CONTROL:
        return "CONTROL";
    case G920_FRAME_DESCRIPTOR:
        return "DESCRIPTOR";
    case G920_FRAME_INPUT:
        return "INPUT";
    case G920_FRAME_FFB:
        return "FFB";
    case G920_FRAME_DISCOVER:
        return "DISCOVER";
    case G920_FRAME_NONE:
    default:
        return "?";
    }
}

const char *g920_proto_status_name(g920_proto_status_t status)
{
    switch (status) {
    case G920_PROTO_OK:
        return "OK";
    case G920_PROTO_TRUNCATED:
        return "TRUNCATED";
    case G920_PROTO_BAD_VERSION:
        return "BAD_VERSION";
    case G920_PROTO_BAD_TYPE:
        return "BAD_TYPE";
    case G920_PROTO_BAD_LENGTH:
        return "BAD_LENGTH";
    case G920_PROTO_BAD_ARG:
        return "BAD_ARG";
    default:
        return "?";
    }
}

/* --- счётчик последовательности ------------------------------------------ */

G920_HOT int16_t g920_seq_diff(uint16_t a, uint16_t b)
{
    /* Разность в беззнаковом с приведением к знаковому: так переполнение
     * счётчика перестаёт быть особым случаем. 65535 → 0 это +1, а не
     * −65535. */
    return (int16_t)((uint16_t)(b - a));
}

bool g920_seq_newer(uint16_t candidate, uint16_t reference)
{
    return g920_seq_diff(reference, candidate) > 0;
}

void g920_seq_tracker_init(g920_seq_tracker_t *tracker)
{
    if (tracker == NULL) {
        return;
    }
    tracker->last = 0;
    tracker->started = false;
}

G920_HOT g920_seq_verdict_t g920_seq_track(g920_seq_tracker_t *tracker,
                                  uint16_t sequence)
{
    int16_t diff;

    if (tracker == NULL) {
        return G920_SEQ_STALE;
    }
    if (!tracker->started) {
        tracker->started = true;
        tracker->last = sequence;
        return G920_SEQ_NEW;
    }

    diff = g920_seq_diff(tracker->last, sequence);
    if (diff == 0) {
        return G920_SEQ_DUPLICATE;
    }
    if (diff < 0) {
        /* Опоздавший. Метку не двигаем: иначе один заблудившийся кадр
         * отбросил бы счётчик назад и следующие нормальные кадры пошли
         * бы как дубликаты. */
        return G920_SEQ_STALE;
    }
    tracker->last = sequence;
    return G920_SEQ_NEW;
}

const char *g920_seq_verdict_name(g920_seq_verdict_t verdict)
{
    switch (verdict) {
    case G920_SEQ_NEW:
        return "NEW";
    case G920_SEQ_DUPLICATE:
        return "DUPLICATE";
    case G920_SEQ_STALE:
        return "STALE";
    default:
        return "?";
    }
}
