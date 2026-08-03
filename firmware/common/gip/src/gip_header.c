#include "g920/gip.h"
#include "g920/hot.h"

#define CONTINUE_BIT 0x80u
#define VALUE_MASK 0x7Fu
#define VALUE_BITS 7

uint8_t g920_gip_varint_width(uint32_t value)
{
    uint8_t width = 1;

    while (value > VALUE_MASK && width < G920_GIP_VARINT_MAX_BYTES) {
        value >>= VALUE_BITS;
        width++;
    }
    return width;
}

G920_HOT g920_gip_status_t g920_gip_varint_decode(uint32_t *value, uint8_t *bytes,
                                         const uint8_t *buf, size_t size)
{
    uint32_t result = 0;
    uint8_t used = 0;

    if (value == NULL || buf == NULL) {
        return G920_GIP_BAD_ARG;
    }

    for (;;) {
        uint8_t byte;

        if (used >= size) {
            return G920_GIP_TRUNCATED;
        }
        if (used >= G920_GIP_VARINT_MAX_BYTES) {
            /* Спека ограничивает расширение четырьмя байтами. Дальше —
             * либо мусор, либо не GIP; додумывать нельзя. */
            return G920_GIP_MALFORMED;
        }

        byte = buf[used];
        result |= (uint32_t)(byte & VALUE_MASK) << (VALUE_BITS * used);
        used++;

        if ((byte & CONTINUE_BIT) == 0) {
            break;
        }
    }

    *value = result;
    if (bytes != NULL) {
        *bytes = used;
    }
    return G920_GIP_OK;
}

G920_HOT int g920_gip_varint_encode(uint8_t *buf, size_t size, uint32_t value,
                           uint8_t width)
{
    uint8_t needed;

    if (buf == NULL || value > G920_GIP_VARINT_MAX) {
        return -1;
    }

    needed = g920_gip_varint_width(value);
    if (width == 0) {
        width = needed;
    }
    /* Уже уместившееся значение можно записать шире — так спека добивает
     * заголовок до чётного размера. Уже́ — нельзя. */
    if (width < needed || width > G920_GIP_VARINT_MAX_BYTES) {
        return -1;
    }
    if (size < width) {
        return -1;
    }

    for (uint8_t i = 0; i < width; i++) {
        uint8_t byte = (uint8_t)((value >> (VALUE_BITS * i)) & VALUE_MASK);

        if (i + 1 < width) {
            byte |= CONTINUE_BIT;
        }
        buf[i] = byte;
    }
    return (int)width;
}

G920_HOT g920_gip_status_t g920_gip_header_parse(g920_gip_header_t *out,
                                        const uint8_t *buf, size_t size)
{
    g920_gip_status_t status;
    size_t pos = 3;
    uint8_t used = 0;

    if (out == NULL || buf == NULL) {
        return G920_GIP_BAD_ARG;
    }
    if (size < G920_GIP_HEADER_MIN) {
        return G920_GIP_TRUNCATED;
    }

    out->message_type = buf[0];
    out->flags = buf[1];
    out->sequence = buf[2];
    out->tlo = 0;
    out->tlo_bytes = 0;

    status = g920_gip_varint_decode(&out->payload_length, &used, buf + pos,
                                    size - pos);
    if (status != G920_GIP_OK) {
        return status;
    }
    out->length_bytes = used;
    pos += used;

    if ((out->flags & G920_GIP_FLAG_FRAGMENT) != 0) {
        status = g920_gip_varint_decode(&out->tlo, &used, buf + pos,
                                        size - pos);
        if (status != G920_GIP_OK) {
            return status;
        }
        out->tlo_bytes = used;
        pos += used;
    }

    out->header_length = (uint8_t)pos;
    return G920_GIP_OK;
}

G920_HOT int g920_gip_header_build(uint8_t *buf, size_t size,
                          const g920_gip_header_t *header)
{
    size_t pos = 3;
    int written;

    if (buf == NULL || header == NULL || size < G920_GIP_HEADER_MIN) {
        return -1;
    }

    buf[0] = header->message_type;
    buf[1] = header->flags;
    buf[2] = header->sequence;

    written = g920_gip_varint_encode(buf + pos, size - pos,
                                     header->payload_length,
                                     header->length_bytes);
    if (written < 0) {
        return -1;
    }
    pos += (size_t)written;

    if ((header->flags & G920_GIP_FLAG_FRAGMENT) != 0) {
        if (pos >= size) {
            return -1;
        }
        written = g920_gip_varint_encode(buf + pos, size - pos, header->tlo,
                                         header->tlo_bytes);
        if (written < 0) {
            return -1;
        }
        pos += (size_t)written;
    }

    return (int)pos;
}

g920_gip_class_t g920_gip_data_class(const g920_gip_header_t *header)
{
    return (g920_gip_class_t)((header->message_type >> 5) & 0x07u);
}

uint8_t g920_gip_message_number(const g920_gip_header_t *header)
{
    return (uint8_t)(header->message_type & 0x1Fu);
}

uint8_t g920_gip_expansion_index(const g920_gip_header_t *header)
{
    return (uint8_t)(header->flags & G920_GIP_EXPANSION_MASK);
}

bool g920_gip_is_fragment(const g920_gip_header_t *header)
{
    return (header->flags & G920_GIP_FLAG_FRAGMENT) != 0;
}

bool g920_gip_is_initial_fragment(const g920_gip_header_t *header)
{
    /* Бит InitFrag осмыслен только вместе с Fragment — таблица 4-3. */
    return g920_gip_is_fragment(header)
           && (header->flags & G920_GIP_FLAG_INIT_FRAG) != 0;
}

bool g920_gip_is_system(const g920_gip_header_t *header)
{
    return (header->flags & G920_GIP_FLAG_SYSTEM) != 0;
}

bool g920_gip_wants_ack(const g920_gip_header_t *header)
{
    return (header->flags & G920_GIP_FLAG_ACME) != 0;
}

const char *g920_gip_status_name(g920_gip_status_t status)
{
    switch (status) {
    case G920_GIP_OK:
        return "OK";
    case G920_GIP_TRUNCATED:
        return "TRUNCATED";
    case G920_GIP_MALFORMED:
        return "MALFORMED";
    case G920_GIP_BAD_ARG:
        return "BAD_ARG";
    default:
        return "?";
    }
}
