#include "g920/gip_control.h"

g920_gip_status_t g920_gip_control_parse(g920_gip_control_t *out,
                                         const uint8_t *payload, size_t size)
{
    if (out == NULL || payload == NULL) {
        return G920_GIP_BAD_ARG;
    }
    if (size < G920_GIP_CONTROL_PAYLOAD_SIZE) {
        return G920_GIP_TRUNCATED;
    }

    out->control_code = payload[0];
    out->ref_message_type = payload[1];
    out->ref_message_flags = payload[2];
    out->fragment_offset = (uint32_t)payload[3] | ((uint32_t)payload[4] << 8)
                           | ((uint32_t)payload[5] << 16)
                           | ((uint32_t)payload[6] << 24);
    out->remaining_buffer =
        (uint16_t)((uint16_t)payload[7] | ((uint16_t)payload[8] << 8));
    return G920_GIP_OK;
}

int g920_gip_control_build(uint8_t *buf, size_t size,
                           const g920_gip_control_t *control)
{
    if (buf == NULL || control == NULL
        || size < G920_GIP_CONTROL_PAYLOAD_SIZE) {
        return -1;
    }

    buf[0] = control->control_code;
    buf[1] = control->ref_message_type;
    /* Чистим за вызывающего: в ссылку идут только System и Index. */
    buf[2] = (uint8_t)(control->ref_message_flags & G920_GIP_REF_FLAGS_MASK);
    buf[3] = (uint8_t)(control->fragment_offset & 0xFFu);
    buf[4] = (uint8_t)((control->fragment_offset >> 8) & 0xFFu);
    buf[5] = (uint8_t)((control->fragment_offset >> 16) & 0xFFu);
    buf[6] = (uint8_t)((control->fragment_offset >> 24) & 0xFFu);
    buf[7] = (uint8_t)(control->remaining_buffer & 0xFFu);
    buf[8] = (uint8_t)((control->remaining_buffer >> 8) & 0xFFu);
    return G920_GIP_CONTROL_PAYLOAD_SIZE;
}

int g920_gip_build_ack(uint8_t *buf, size_t size,
                       const g920_gip_header_t *acked, uint32_t fragment_offset,
                       uint16_t remaining_buffer)
{
    g920_gip_header_t header;
    g920_gip_control_t control;
    int written;
    size_t pos;

    if (buf == NULL || acked == NULL) {
        return -1;
    }

    header.message_type = G920_GIP_MSG_PROTOCOL_CONTROL;
    /* Сам ACK — одиночное системное сообщение и подтверждения не просит. */
    header.flags = (uint8_t)(G920_GIP_FLAG_SYSTEM
                             | (acked->flags & G920_GIP_EXPANSION_MASK));
    header.sequence = acked->sequence;
    header.payload_length = G920_GIP_CONTROL_PAYLOAD_SIZE;
    header.length_bytes = 1;
    header.tlo = 0;
    header.tlo_bytes = 0;
    header.header_length = 0;

    written = g920_gip_header_build(buf, size, &header);
    if (written < 0) {
        return -1;
    }
    pos = (size_t)written;

    control.control_code = G920_GIP_CONTROL_ACK;
    control.ref_message_type = acked->message_type;
    control.ref_message_flags = acked->flags; /* маска накладывается в build */
    control.fragment_offset = fragment_offset;
    control.remaining_buffer = remaining_buffer;

    written = g920_gip_control_build(buf + pos, size - pos, &control);
    if (written < 0) {
        return -1;
    }
    return (int)(pos + (size_t)written);
}

bool g920_gip_should_request_ack(uint32_t fragment_index, bool is_last,
                                 uint32_t every_n)
{
    if (fragment_index == 0 || is_last) {
        return true;
    }
    if (every_n == 0) {
        return false;
    }
    return (fragment_index % every_n) == 0;
}
