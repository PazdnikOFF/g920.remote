#include "g920/log.h"

#include <stdio.h>

#include "g920/hexdump.h"
#include "g920/timestamp.h"
#include "g920/version.h"

/* Префикс + сообщение + '\n' + '\0'. */
#define LINE_MAX (G920_LOG_PREFIX_MAX + G920_LOG_MSG_MAX + 2)

static void console_sink(void *ctx, const char *text, size_t len);

static g920_log_sink_t s_sink = console_sink;
static void *s_sink_ctx = NULL;
static g920_log_level_t s_level = G920_LOG_INFO;

/*
 * Приёмник по умолчанию — консоль. На плате это UART0: так настроено в
 * sdkconfig.defaults обеих прошивок, и вторичная консоль на USB-Serial-JTAG
 * там же выключена. На хосте — тот же stdout, отдельная ветка не нужна.
 */
static void console_sink(void *ctx, const char *text, size_t len)
{
    (void)ctx;
    (void)fwrite(text, 1, len, stdout);
}

void g920_log_set_sink(g920_log_sink_t sink, void *ctx)
{
    s_sink = (sink != NULL) ? sink : console_sink;
    s_sink_ctx = (sink != NULL) ? ctx : NULL;
}

void g920_log_set_level(g920_log_level_t level)
{
    s_level = level;
}

g920_log_level_t g920_log_get_level(void)
{
    return s_level;
}

char g920_log_level_char(g920_log_level_t level)
{
    switch (level) {
    case G920_LOG_ERROR:
        return 'E';
    case G920_LOG_WARN:
        return 'W';
    case G920_LOG_INFO:
        return 'I';
    case G920_LOG_DEBUG:
        return 'D';
    default:
        return '?';
    }
}

int g920_log_prefix(char *buf, size_t size, g920_log_level_t level, uint64_t us,
                    const char *tag)
{
    char tmp[G920_LOG_PREFIX_MAX];
    const char *role = g920_role_name(g920_build_role());
    size_t pos = 0;
    int written;

    if (buf == NULL) {
        return -1;
    }
    if (tag == NULL) {
        tag = "?";
    }

    tmp[pos++] = g920_log_level_char(level);
    tmp[pos++] = ' ';

    written = g920_timestamp_format(tmp + pos, sizeof(tmp) - pos, us);
    if (written < 0) {
        return -1;
    }
    pos += (size_t)written;

    tmp[pos++] = ' ';
    for (size_t i = 0; role[i] != '\0'; i++) {
        tmp[pos++] = role[i];
    }
    tmp[pos++] = ' ';
    for (size_t i = 0; i < G920_LOG_TAG_MAX && tag[i] != '\0'; i++) {
        tmp[pos++] = tag[i];
    }
    tmp[pos++] = ':';
    tmp[pos++] = ' ';

    if (size < pos + 1u) {
        return -1;
    }
    for (size_t i = 0; i < pos; i++) {
        buf[i] = tmp[i];
    }
    buf[pos] = '\0';
    return (int)pos;
}

static void emit(g920_log_level_t level, const char *tag, const char *body,
                 size_t body_len)
{
    char line[LINE_MAX];
    int prefix_len = g920_log_prefix(line, sizeof(line), level,
                                     g920_timestamp_us(), tag);
    size_t pos;

    if (prefix_len < 0) {
        return;
    }
    pos = (size_t)prefix_len;

    if (body_len > G920_LOG_MSG_MAX) {
        body_len = G920_LOG_MSG_MAX;
    }
    for (size_t i = 0; i < body_len; i++) {
        line[pos++] = body[i];
    }
    line[pos++] = '\n';

    s_sink(s_sink_ctx, line, pos);
}

void g920_logv(g920_log_level_t level, const char *tag, const char *fmt,
               va_list args)
{
    char body[G920_LOG_MSG_MAX + 1];
    int n;

    if (level > s_level || fmt == NULL) {
        return;
    }

    n = vsnprintf(body, sizeof(body), fmt, args);
    if (n < 0) {
        return;
    }
    if ((size_t)n >= sizeof(body)) {
        /* Хвост потерян — говорим об этом прямо в строке, а не молчим. */
        n = (int)sizeof(body) - 1;
        body[n - 1] = '~';
    }

    emit(level, tag, body, (size_t)n);
}

void g920_logf(g920_log_level_t level, const char *tag, const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    g920_logv(level, tag, fmt, args);
    va_end(args);
}

void g920_log_hexdump(g920_log_level_t level, const char *tag,
                      const void *data, size_t len)
{
    const uint8_t *bytes = (const uint8_t *)data;
    char line[G920_HEXDUMP_LINE_MAX];

    if (level > s_level) {
        return;
    }
    if (bytes == NULL && len != 0) {
        g920_logf(level, tag, "hexdump: NULL, %u bytes claimed",
                  (unsigned)len);
        return;
    }

    g920_logf(level, tag, "hexdump: %u bytes", (unsigned)len);

    for (size_t offset = 0; offset < len;
         offset += G920_HEXDUMP_BYTES_PER_LINE) {
        size_t chunk = len - offset;
        int line_len;

        if (chunk > G920_HEXDUMP_BYTES_PER_LINE) {
            chunk = G920_HEXDUMP_BYTES_PER_LINE;
        }
        line_len = g920_hexdump_line(line, sizeof(line), offset,
                                     bytes + offset, chunk);
        if (line_len < 0) {
            return;
        }
        emit(level, tag, line, (size_t)line_len);
    }
}
