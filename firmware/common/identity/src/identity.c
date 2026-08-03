#include "g920/identity.h"

#include <string.h>

/* Стандартные смещения дескриптора устройства USB. Не про G920. */
#define USB_DEVICE_DESC_SIZE 18
#define USB_DESC_TYPE_DEVICE 0x01
#define USB_DEV_ID_VENDOR 8
#define USB_DEV_ID_PRODUCT 10
#define USB_DEV_BCD_DEVICE 12

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16)
           | ((uint32_t)p[3] << 24);
}

static void write_header(g920_identity_t *identity)
{
    uint8_t *h = identity->buffer;

    put_u32(h + 0, G920_IDENTITY_MAGIC);
    put_u16(h + 4, (uint16_t)G920_IDENTITY_FORMAT_VERSION);
    put_u16(h + 6, identity->sections);
    put_u16(h + 8, identity->fingerprint.vendor_id);
    put_u16(h + 10, identity->fingerprint.product_id);
    put_u16(h + 12, identity->fingerprint.device_release);
    put_u32(h + 14, (uint32_t)identity->used);
}

g920_identity_status_t g920_identity_init(g920_identity_t *identity,
                                          uint8_t *buffer, size_t capacity)
{
    if (identity == NULL || buffer == NULL) {
        return G920_IDENTITY_BAD_ARG;
    }
    if (capacity < G920_IDENTITY_HEADER_SIZE) {
        return G920_IDENTITY_NO_SPACE;
    }

    memset(identity, 0, sizeof(*identity));
    identity->buffer = buffer;
    identity->capacity = capacity;
    identity->used = G920_IDENTITY_HEADER_SIZE;
    write_header(identity);
    return G920_IDENTITY_OK;
}

void g920_identity_set_fingerprint(g920_identity_t *identity,
                                   g920_identity_fingerprint_t fingerprint)
{
    if (identity == NULL || identity->buffer == NULL) {
        return;
    }
    identity->fingerprint = fingerprint;
    write_header(identity);
}

g920_identity_status_t g920_identity_add(g920_identity_t *identity,
                                         g920_identity_section_t type,
                                         uint8_t index, const void *data,
                                         uint16_t length)
{
    uint8_t *p;

    if (identity == NULL || identity->buffer == NULL) {
        return G920_IDENTITY_BAD_ARG;
    }
    if (data == NULL && length != 0) {
        return G920_IDENTITY_BAD_ARG;
    }
    if (identity->sections == 0xFFFFu) {
        return G920_IDENTITY_NO_SPACE;
    }
    if (identity->capacity - identity->used
        < (size_t)G920_IDENTITY_SECTION_HEADER_SIZE + length) {
        return G920_IDENTITY_NO_SPACE;
    }

    p = identity->buffer + identity->used;
    p[0] = (uint8_t)type;
    p[1] = index;
    put_u16(p + 2, length);
    if (length != 0) {
        /* Байты копируются как есть. Что внутри — не наше дело (И2). */
        memcpy(p + G920_IDENTITY_SECTION_HEADER_SIZE, data, length);
    }

    identity->used += (size_t)G920_IDENTITY_SECTION_HEADER_SIZE + length;
    identity->sections++;
    write_header(identity);
    return G920_IDENTITY_OK;
}

g920_identity_status_t g920_identity_parse(g920_identity_t *identity,
                                           uint8_t *buffer, size_t size)
{
    uint32_t total;
    uint16_t declared;
    uint16_t counted = 0;
    size_t pos = G920_IDENTITY_HEADER_SIZE;

    if (identity == NULL || buffer == NULL) {
        return G920_IDENTITY_BAD_ARG;
    }
    if (size == 0) {
        return G920_IDENTITY_EMPTY;
    }
    if (size < G920_IDENTITY_HEADER_SIZE) {
        return G920_IDENTITY_BAD_LENGTH;
    }
    if (get_u32(buffer) != G920_IDENTITY_MAGIC) {
        return G920_IDENTITY_BAD_MAGIC;
    }
    if (get_u16(buffer + 4) != G920_IDENTITY_FORMAT_VERSION) {
        /* Раскладка контейнера другая — разбирать по нашей нельзя.
         * Новые *типы секций* версию не меняют, см. заголовок. */
        return G920_IDENTITY_BAD_VERSION;
    }

    total = get_u32(buffer + 14);
    if (total < G920_IDENTITY_HEADER_SIZE || total > size) {
        return G920_IDENTITY_BAD_LENGTH;
    }
    declared = get_u16(buffer + 6);

    while (pos < total) {
        uint16_t length;

        if (total - pos < (uint32_t)G920_IDENTITY_SECTION_HEADER_SIZE) {
            return G920_IDENTITY_BAD_LENGTH;
        }
        length = get_u16(buffer + pos + 2);
        if (total - pos - G920_IDENTITY_SECTION_HEADER_SIZE < length) {
            return G920_IDENTITY_BAD_LENGTH;
        }
        /* Тип не проверяем намеренно: незнакомая секция от прошивки новее
         * нашей обязана пережить разбор, а не уронить его. */
        pos += (size_t)G920_IDENTITY_SECTION_HEADER_SIZE + length;
        counted++;
    }

    if (counted != declared) {
        return G920_IDENTITY_BAD_LENGTH;
    }

    identity->buffer = buffer;
    identity->capacity = size;
    identity->used = total;
    identity->sections = counted;
    identity->fingerprint.vendor_id = get_u16(buffer + 8);
    identity->fingerprint.product_id = get_u16(buffer + 10);
    identity->fingerprint.device_release = get_u16(buffer + 12);
    return G920_IDENTITY_OK;
}

const uint8_t *g920_identity_find(const g920_identity_t *identity,
                                  g920_identity_section_t type, uint8_t index,
                                  uint16_t *out_length)
{
    size_t pos = G920_IDENTITY_HEADER_SIZE;

    if (identity == NULL || identity->buffer == NULL) {
        return NULL;
    }
    while (pos + G920_IDENTITY_SECTION_HEADER_SIZE <= identity->used) {
        const uint8_t *p = identity->buffer + pos;
        uint16_t length = get_u16(p + 2);

        if (p[0] == (uint8_t)type && p[1] == index) {
            if (out_length != NULL) {
                *out_length = length;
            }
            return (length != 0) ? p + G920_IDENTITY_SECTION_HEADER_SIZE : NULL;
        }
        pos += (size_t)G920_IDENTITY_SECTION_HEADER_SIZE + length;
    }
    return NULL;
}

uint16_t g920_identity_section_count(const g920_identity_t *identity)
{
    return (identity != NULL) ? identity->sections : 0;
}

size_t g920_identity_size(const g920_identity_t *identity)
{
    return (identity != NULL) ? identity->used : 0;
}

bool g920_identity_empty(const g920_identity_t *identity)
{
    return identity == NULL || identity->sections == 0;
}

bool g920_identity_matches(const g920_identity_t *identity,
                           g920_identity_fingerprint_t fingerprint)
{
    if (identity == NULL || g920_identity_empty(identity)) {
        return false;
    }
    return identity->fingerprint.vendor_id == fingerprint.vendor_id
           && identity->fingerprint.product_id == fingerprint.product_id
           && identity->fingerprint.device_release == fingerprint.device_release;
}

bool g920_identity_fingerprint_from_device_descriptor(
    g920_identity_fingerprint_t *out, const uint8_t *descriptor, size_t size)
{
    if (out == NULL || descriptor == NULL || size < USB_DEVICE_DESC_SIZE) {
        return false;
    }
    if (descriptor[0] != USB_DEVICE_DESC_SIZE
        || descriptor[1] != USB_DESC_TYPE_DEVICE) {
        return false;
    }
    out->vendor_id = get_u16(descriptor + USB_DEV_ID_VENDOR);
    out->product_id = get_u16(descriptor + USB_DEV_ID_PRODUCT);
    out->device_release = get_u16(descriptor + USB_DEV_BCD_DEVICE);
    return true;
}

const char *g920_identity_status_name(g920_identity_status_t status)
{
    switch (status) {
    case G920_IDENTITY_OK:
        return "OK";
    case G920_IDENTITY_EMPTY:
        return "EMPTY";
    case G920_IDENTITY_BAD_MAGIC:
        return "BAD_MAGIC";
    case G920_IDENTITY_BAD_VERSION:
        return "BAD_VERSION";
    case G920_IDENTITY_BAD_LENGTH:
        return "BAD_LENGTH";
    case G920_IDENTITY_NO_SPACE:
        return "NO_SPACE";
    case G920_IDENTITY_BAD_ARG:
        return "BAD_ARG";
    default:
        return "?";
    }
}
