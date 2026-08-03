#include "g920/trace.h"

#include <stdio.h>
#include <string.h>

#include "g920/hot.h"
#include "g920/log.h"
#include "g920/timestamp.h"

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void put_u64(uint8_t *p, uint64_t v)
{
    for (int i = 0; i < 8; i++) {
        p[i] = (uint8_t)((v >> (8 * i)) & 0xFFu);
    }
}

static uint64_t get_u64(const uint8_t *p)
{
    uint64_t v = 0;

    for (int i = 0; i < 8; i++) {
        v |= (uint64_t)p[i] << (8 * i);
    }
    return v;
}

void g920_trace_init(g920_trace_t *trace, void *storage, size_t size,
                     g920_trace_policy_t policy)
{
    if (trace == NULL) {
        return;
    }
    memset(trace, 0, sizeof(*trace));
    trace->bytes = (uint8_t *)storage;
    trace->size = (storage != NULL) ? size : 0;
    trace->wrap_at = trace->size;
    trace->policy = (uint8_t)policy;
}

void g920_trace_reset(g920_trace_t *trace)
{
    if (trace == NULL) {
        return;
    }
    trace->head = 0;
    trace->tail = 0;
    trace->used = 0;
    trace->wrap_at = trace->size;
    trace->count = 0;
    /* Счётчики за жизнь не трогаем: «сколько всего прошло» — это не то же
     * самое, что «сколько лежит сейчас». */
}

/* Выбрасывает самую старую запись. false — выбрасывать нечего. */
static bool evict_oldest(g920_trace_t *trace)
{
    uint16_t length;
    size_t record;

    if (trace->count == 0) {
        return false;
    }
    length = get_u16(trace->bytes + trace->tail + 10);
    record = (size_t)G920_TRACE_HEADER_SIZE + length;

    trace->tail += record;
    trace->used -= record;
    trace->count--;
    trace->evicted++;

    if (trace->tail == trace->wrap_at) {
        trace->used -= trace->size - trace->wrap_at;
        trace->tail = 0;
        trace->wrap_at = trace->size;
    }
    return true;
}

G920_HOT bool g920_trace_write(g920_trace_t *trace, uint64_t now_us,
                               g920_trace_kind_t kind, const void *data,
                               uint16_t length)
{
    size_t record;
    uint8_t *at;

    if (trace == NULL || trace->bytes == NULL) {
        return false;
    }
    if (trace->dumping) {
        /* Выгрузка идёт через лог, а лог может быть подключён сюда же.
         * Молча игнорируем — иначе дамп никогда не кончится. */
        return false;
    }
    if (data == NULL && length != 0) {
        return false;
    }
    if (length > G920_TRACE_PAYLOAD_MAX) {
        trace->refused++;
        return false;
    }

    record = (size_t)G920_TRACE_HEADER_SIZE + length;
    if (record > trace->size) {
        /* В буфер не влезет никогда, сколько ни выбрасывай. */
        trace->refused++;
        return false;
    }

    for (;;) {
        size_t free_bytes = trace->size - trace->used;
        size_t gap = (trace->head + record > trace->size)
                         ? trace->size - trace->head
                         : 0;

        if (free_bytes >= gap + record) {
            if (gap != 0) {
                /* Запись не режется переносом: хвост остаётся дыркой,
                 * читатель её перепрыгнет. */
                trace->wrap_at = trace->head;
                trace->used += gap;
                trace->head = 0;
            }
            break;
        }
        if (trace->policy == (uint8_t)G920_TRACE_KEEP_OLDEST) {
            /* Полный дамп: заполнились и молчим. Потеря видна счётчиком. */
            trace->refused++;
            return false;
        }
        if (!evict_oldest(trace)) {
            trace->refused++;
            return false;
        }
    }

    at = trace->bytes + trace->head;
    put_u64(at, now_us);
    at[8] = (uint8_t)kind;
    at[9] = 0; /* флаги, пока не заняты */
    put_u16(at + 10, length);
    if (length != 0) {
        memcpy(at + G920_TRACE_HEADER_SIZE, data, length);
    }

    trace->head += record;
    if (trace->head == trace->size) {
        /*
         * Записи легли ровно до конца — перенос без дырки. Оставить head
         * равным size нельзя: следующая запись посчитала бы дырку нулевой и
         * пошла писать за границу буфера. Читателю ничего сообщать не надо,
         * он прыгает на ноль по wrap_at, а тот и так равен size.
         */
        trace->head = 0;
    }
    trace->used += record;
    trace->count++;
    trace->written++;
    return true;
}

/* --- чтение --------------------------------------------------------------- */

void g920_trace_rewind(const g920_trace_t *trace, g920_trace_cursor_t *cursor)
{
    if (cursor == NULL) {
        return;
    }
    if (trace == NULL) {
        cursor->offset = 0;
        cursor->remaining = 0;
        return;
    }
    cursor->offset = trace->tail;
    cursor->remaining = trace->count;
}

bool g920_trace_next(const g920_trace_t *trace, g920_trace_cursor_t *cursor,
                     g920_trace_record_t *out)
{
    const uint8_t *at;

    if (trace == NULL || cursor == NULL || out == NULL) {
        return false;
    }
    if (cursor->remaining == 0) {
        return false;
    }
    if (cursor->offset == trace->wrap_at) {
        cursor->offset = 0;
    }

    at = trace->bytes + cursor->offset;
    out->at_us = get_u64(at);
    out->kind = at[8];
    out->flags = at[9];
    out->length = get_u16(at + 10);
    out->payload = (out->length != 0) ? (at + G920_TRACE_HEADER_SIZE) : NULL;

    cursor->offset += (size_t)G920_TRACE_HEADER_SIZE + out->length;
    cursor->remaining--;
    return true;
}

uint32_t g920_trace_count(const g920_trace_t *trace)
{
    return (trace != NULL) ? trace->count : 0;
}

uint32_t g920_trace_written(const g920_trace_t *trace)
{
    return (trace != NULL) ? trace->written : 0;
}

uint32_t g920_trace_evicted(const g920_trace_t *trace)
{
    return (trace != NULL) ? trace->evicted : 0;
}

uint32_t g920_trace_refused(const g920_trace_t *trace)
{
    return (trace != NULL) ? trace->refused : 0;
}

size_t g920_trace_used(const g920_trace_t *trace)
{
    return (trace != NULL) ? trace->used : 0;
}

bool g920_trace_empty(const g920_trace_t *trace)
{
    return (trace == NULL) || trace->count == 0;
}

/* --- вывод ---------------------------------------------------------------- */

const char *g920_trace_kind_name(g920_trace_kind_t kind)
{
    switch (kind) {
    case G920_TRACE_LOG:
        return "LOG";
    case G920_TRACE_USB_IN:
        return "USB_IN";
    case G920_TRACE_USB_OUT:
        return "USB_OUT";
    case G920_TRACE_USB_SETUP:
        return "SETUP";
    case G920_TRACE_GIP_IN:
        return "GIP_IN";
    case G920_TRACE_GIP_OUT:
        return "GIP_OUT";
    case G920_TRACE_LINK_TX:
        return "LINK_TX";
    case G920_TRACE_LINK_RX:
        return "LINK_RX";
    case G920_TRACE_MARK:
        return "MARK";
    case G920_TRACE_NONE:
    default:
        return "?";
    }
}

int g920_trace_format(char *buf, size_t size,
                      const g920_trace_record_t *record)
{
    int written;

    if (buf == NULL || record == NULL || size == 0) {
        return -1;
    }
    /* Формат метки тот же, что у лога и hostlog: трассы предстоит сводить
     * одним разборщиком. */
    written = snprintf(buf, size, "%6llu.%06llu %s %u B",
                       (unsigned long long)(record->at_us / 1000000ull),
                       (unsigned long long)(record->at_us % 1000000ull),
                       g920_trace_kind_name((g920_trace_kind_t)record->kind),
                       (unsigned)record->length);
    if (written < 0 || (size_t)written >= size) {
        return -1;
    }
    return written;
}

void g920_trace_dump(g920_trace_t *trace, const char *tag, bool with_payload)
{
    g920_trace_cursor_t cursor;
    g920_trace_record_t record;
    char line[G920_TRACE_LINE_MAX];

    if (trace == NULL) {
        return;
    }
    /* Пока идёт выгрузка, запись в трассу игнорируется: лог может быть
     * подключён приёмником сюда же. */
    trace->dumping = true;

    G920_LOGI(tag, "trace: %u records, %u B used, %u evicted, %u refused",
              (unsigned)trace->count, (unsigned)trace->used,
              (unsigned)trace->evicted, (unsigned)trace->refused);

    g920_trace_rewind(trace, &cursor);
    while (g920_trace_next(trace, &cursor, &record)) {
        if (g920_trace_format(line, sizeof(line), &record) > 0) {
            G920_LOGI(tag, "%s", line);
        }
        if (with_payload && record.length != 0) {
            g920_log_hexdump(G920_LOG_INFO, tag, record.payload,
                             record.length);
        }
    }

    trace->dumping = false;
}

void g920_trace_log_sink(void *ctx, const char *text, size_t len)
{
    g920_trace_t *trace = (g920_trace_t *)ctx;

    if (trace == NULL || text == NULL) {
        return;
    }
    if (len > G920_TRACE_PAYLOAD_MAX) {
        len = G920_TRACE_PAYLOAD_MAX;
    }
    (void)g920_trace_write(trace, g920_timestamp_us(), G920_TRACE_LOG, text,
                           (uint16_t)len);
}
