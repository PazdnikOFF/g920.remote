#include "g920/store.h"

#include <stdbool.h>
#include <string.h>

#include "g920/log.h"

#define TAG "store"

#define RECORD_MAX (G920_STORE_HEADER_SIZE + G920_STORE_PAYLOAD_MAX)

/* --- заголовок --------------------------------------------------------- */

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

int g920_store_header_pack(uint8_t *buf, size_t size,
                           const g920_store_header_t *header)
{
    if (buf == NULL || header == NULL || size < G920_STORE_HEADER_SIZE) {
        return -1;
    }
    put_u32(buf + 0, header->magic);
    put_u16(buf + 4, header->kind);
    put_u16(buf + 6, header->version);
    put_u32(buf + 8, header->length);
    return G920_STORE_HEADER_SIZE;
}

g920_store_status_t g920_store_header_unpack(g920_store_header_t *out,
                                             const uint8_t *buf, size_t size)
{
    if (out == NULL || buf == NULL) {
        return G920_STORE_BAD_ARG;
    }
    if (size < G920_STORE_HEADER_SIZE) {
        return G920_STORE_BAD_LENGTH;
    }
    out->magic = get_u32(buf + 0);
    out->kind = get_u16(buf + 4);
    out->version = get_u16(buf + 6);
    out->length = get_u32(buf + 8);

    if (out->magic != G920_STORE_MAGIC) {
        return G920_STORE_BAD_MAGIC;
    }
    return G920_STORE_OK;
}

g920_store_status_t g920_store_header_check(const g920_store_header_t *header,
                                            g920_store_kind_t kind,
                                            uint16_t version)
{
    if (header == NULL) {
        return G920_STORE_BAD_ARG;
    }
    if (header->magic != G920_STORE_MAGIC) {
        return G920_STORE_BAD_MAGIC;
    }
    if (header->kind != (uint16_t)kind) {
        return G920_STORE_WRONG_KIND;
    }
    if (version == G920_STORE_ANY_VERSION) {
        return G920_STORE_OK;
    }
    if (header->version < version) {
        return G920_STORE_VERSION_OLDER;
    }
    if (header->version > version) {
        return G920_STORE_VERSION_NEWER;
    }
    return G920_STORE_OK;
}

int g920_store_key_valid(const char *key)
{
    size_t len = 0;

    if (key == NULL || key[0] == '\0') {
        return 0;
    }
    while (key[len] != '\0') {
        unsigned char c = (unsigned char)key[len];

        if (c <= 0x20 || c >= 0x7F) {
            return 0;
        }
        len++;
        if (len > G920_STORE_KEY_MAX) {
            return 0;
        }
    }
    return 1;
}

const char *g920_store_status_name(g920_store_status_t status)
{
    switch (status) {
    case G920_STORE_OK:
        return "OK";
    case G920_STORE_EMPTY:
        return "EMPTY";
    case G920_STORE_BAD_MAGIC:
        return "BAD_MAGIC";
    case G920_STORE_WRONG_KIND:
        return "WRONG_KIND";
    case G920_STORE_VERSION_OLDER:
        return "VERSION_OLDER";
    case G920_STORE_VERSION_NEWER:
        return "VERSION_NEWER";
    case G920_STORE_BAD_LENGTH:
        return "BAD_LENGTH";
    case G920_STORE_TOO_SMALL:
        return "TOO_SMALL";
    case G920_STORE_BAD_ARG:
        return "BAD_ARG";
    case G920_STORE_IO_ERROR:
        return "IO_ERROR";
    default:
        return "?";
    }
}

/* --- бэкенд: плата (NVS) или хост (память процесса) -------------------- */
/*
 * Ниже — только доступ к байтам по ключу. Вся логика заголовков и версий
 * общая и лежит выше, поэтому проверяется хостовыми тестами целиком, а на
 * плате остаётся ровно вызов nvs_*.
 */

static g920_store_status_t backend_init(void);
static g920_store_status_t backend_set(const char *key, const uint8_t *data,
                                       size_t len);
static g920_store_status_t backend_get(const char *key, uint8_t *buf,
                                       size_t capacity, size_t *out_len);
static g920_store_status_t backend_erase(const char *key);
static g920_store_status_t backend_erase_all(void);

#if defined(ESP_PLATFORM)

#include "nvs.h"
#include "nvs_flash.h"

#define NVS_NAMESPACE "g920"

static nvs_handle_t s_nvs;
static bool s_open;

static g920_store_status_t backend_init(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES
        || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* Раздел кончился или остался от другой версии NVS. Пересобираем:
         * всё, что там лежало, всё равно нечитаемо. */
        G920_LOGW(TAG, "nvs re-init: %s", esp_err_to_name(err));
        if (nvs_flash_erase() != ESP_OK) {
            return G920_STORE_IO_ERROR;
        }
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return G920_STORE_IO_ERROR;
    }
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &s_nvs) != ESP_OK) {
        return G920_STORE_IO_ERROR;
    }
    s_open = true;
    return G920_STORE_OK;
}

static g920_store_status_t backend_set(const char *key, const uint8_t *data,
                                       size_t len)
{
    if (!s_open) {
        return G920_STORE_IO_ERROR;
    }
    if (nvs_set_blob(s_nvs, key, data, len) != ESP_OK) {
        return G920_STORE_IO_ERROR;
    }
    return nvs_commit(s_nvs) == ESP_OK ? G920_STORE_OK : G920_STORE_IO_ERROR;
}

static g920_store_status_t backend_get(const char *key, uint8_t *buf,
                                       size_t capacity, size_t *out_len)
{
    size_t len = capacity;
    esp_err_t err;

    if (!s_open) {
        return G920_STORE_IO_ERROR;
    }
    err = nvs_get_blob(s_nvs, key, buf, &len);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return G920_STORE_EMPTY;
    }
    if (err == ESP_ERR_NVS_INVALID_LENGTH) {
        return G920_STORE_TOO_SMALL;
    }
    if (err != ESP_OK) {
        return G920_STORE_IO_ERROR;
    }
    *out_len = len;
    return G920_STORE_OK;
}

static g920_store_status_t backend_erase(const char *key)
{
    esp_err_t err;

    if (!s_open) {
        return G920_STORE_IO_ERROR;
    }
    err = nvs_erase_key(s_nvs, key);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return G920_STORE_EMPTY;
    }
    if (err != ESP_OK) {
        return G920_STORE_IO_ERROR;
    }
    return nvs_commit(s_nvs) == ESP_OK ? G920_STORE_OK : G920_STORE_IO_ERROR;
}

static g920_store_status_t backend_erase_all(void)
{
    if (!s_open) {
        return G920_STORE_IO_ERROR;
    }
    if (nvs_erase_all(s_nvs) != ESP_OK) {
        return G920_STORE_IO_ERROR;
    }
    return nvs_commit(s_nvs) == ESP_OK ? G920_STORE_OK : G920_STORE_IO_ERROR;
}

#else /* хост */

/*
 * Хранилище в памяти процесса. Нужно не «чтобы тесты собрались», а чтобы
 * логика версий и миграции проверялась целиком, включая круг
 * запись → перезапуск прошивки → чтение чужой версией.
 */
#define HOST_SLOTS 16

static struct {
    char key[G920_STORE_KEY_MAX + 1];
    uint8_t data[RECORD_MAX];
    size_t len;
    bool used;
} s_slots[HOST_SLOTS];

static int find_slot(const char *key)
{
    for (int i = 0; i < HOST_SLOTS; i++) {
        if (s_slots[i].used && strcmp(s_slots[i].key, key) == 0) {
            return i;
        }
    }
    return -1;
}

static g920_store_status_t backend_init(void)
{
    return G920_STORE_OK;
}

static g920_store_status_t backend_set(const char *key, const uint8_t *data,
                                       size_t len)
{
    int slot = find_slot(key);

    if (slot < 0) {
        for (int i = 0; i < HOST_SLOTS; i++) {
            if (!s_slots[i].used) {
                slot = i;
                break;
            }
        }
    }
    if (slot < 0) {
        return G920_STORE_IO_ERROR;
    }
    if (len > sizeof(s_slots[0].data)) {
        return G920_STORE_BAD_LENGTH;
    }

    memcpy(s_slots[slot].key, key, strlen(key) + 1);
    memcpy(s_slots[slot].data, data, len);
    s_slots[slot].len = len;
    s_slots[slot].used = true;
    return G920_STORE_OK;
}

static g920_store_status_t backend_get(const char *key, uint8_t *buf,
                                       size_t capacity, size_t *out_len)
{
    int slot = find_slot(key);

    if (slot < 0) {
        return G920_STORE_EMPTY;
    }
    if (s_slots[slot].len > capacity) {
        return G920_STORE_TOO_SMALL;
    }
    memcpy(buf, s_slots[slot].data, s_slots[slot].len);
    *out_len = s_slots[slot].len;
    return G920_STORE_OK;
}

static g920_store_status_t backend_erase(const char *key)
{
    int slot = find_slot(key);

    if (slot < 0) {
        return G920_STORE_EMPTY;
    }
    s_slots[slot].used = false;
    s_slots[slot].len = 0;
    return G920_STORE_OK;
}

static g920_store_status_t backend_erase_all(void)
{
    for (int i = 0; i < HOST_SLOTS; i++) {
        s_slots[i].used = false;
        s_slots[i].len = 0;
    }
    return G920_STORE_OK;
}

#endif /* ESP_PLATFORM */

/* --- общая логика ------------------------------------------------------ */

g920_store_status_t g920_store_init(void)
{
    return backend_init();
}

g920_store_status_t g920_store_write(const char *key, g920_store_kind_t kind,
                                     uint16_t version, const void *payload,
                                     size_t len)
{
    uint8_t record[RECORD_MAX];
    g920_store_header_t header;

    if (!g920_store_key_valid(key) || version == G920_STORE_ANY_VERSION) {
        return G920_STORE_BAD_ARG;
    }
    if (payload == NULL && len != 0) {
        return G920_STORE_BAD_ARG;
    }
    if (len > G920_STORE_PAYLOAD_MAX) {
        return G920_STORE_BAD_LENGTH;
    }

    header.magic = G920_STORE_MAGIC;
    header.kind = (uint16_t)kind;
    header.version = version;
    header.length = (uint32_t)len;

    if (g920_store_header_pack(record, sizeof(record), &header) < 0) {
        return G920_STORE_BAD_ARG;
    }
    if (len != 0) {
        memcpy(record + G920_STORE_HEADER_SIZE, payload, len);
    }
    return backend_set(key, record, G920_STORE_HEADER_SIZE + len);
}

g920_store_status_t g920_store_read(const char *key, g920_store_kind_t kind,
                                    uint16_t version, void *payload,
                                    size_t capacity, size_t *out_len,
                                    uint16_t *out_version)
{
    uint8_t record[RECORD_MAX];
    g920_store_header_t header;
    g920_store_status_t status;
    size_t record_len = 0;

    if (!g920_store_key_valid(key)) {
        return G920_STORE_BAD_ARG;
    }
    if (payload == NULL && capacity != 0) {
        return G920_STORE_BAD_ARG;
    }

    status = backend_get(key, record, sizeof(record), &record_len);
    if (status != G920_STORE_OK) {
        return status;
    }

    status = g920_store_header_unpack(&header, record, record_len);
    if (status != G920_STORE_OK) {
        return status;
    }
    /* Заявленная длина обязана совпасть с фактической: расхождение значит,
     * что запись обрезана или писал кто-то другой. */
    if ((size_t)header.length + G920_STORE_HEADER_SIZE != record_len) {
        return G920_STORE_BAD_LENGTH;
    }

    if (out_version != NULL) {
        *out_version = header.version;
    }

    status = g920_store_header_check(&header, kind, version);
    if (status != G920_STORE_OK) {
        return status;
    }

    if ((size_t)header.length > capacity) {
        return G920_STORE_TOO_SMALL;
    }
    if (header.length != 0) {
        memcpy(payload, record + G920_STORE_HEADER_SIZE, header.length);
    }
    if (out_len != NULL) {
        *out_len = (size_t)header.length;
    }
    return G920_STORE_OK;
}

g920_store_status_t g920_store_erase(const char *key)
{
    if (!g920_store_key_valid(key)) {
        return G920_STORE_BAD_ARG;
    }
    return backend_erase(key);
}

g920_store_status_t g920_store_erase_all(void)
{
    return backend_erase_all();
}
