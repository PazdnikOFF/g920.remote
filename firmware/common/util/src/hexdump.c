#include "g920/hexdump.h"

#define GROUP_SIZE 8
#define OFFSET_DIGITS 8

static const char HEX[] = "0123456789abcdef";

size_t g920_hexdump_line_count(size_t length)
{
    return (length + G920_HEXDUMP_BYTES_PER_LINE - 1)
           / G920_HEXDUMP_BYTES_PER_LINE;
}

int g920_hexdump_line(char *buf, size_t size, size_t offset, const void *data,
                      size_t len)
{
    const uint8_t *bytes = (const uint8_t *)data;
    char tmp[G920_HEXDUMP_LINE_MAX];
    size_t pos = 0;

    if (buf == NULL || len > G920_HEXDUMP_BYTES_PER_LINE) {
        return -1;
    }
    if (bytes == NULL && len != 0) {
        return -1;
    }
    /* Смещение печатается ровно восемью цифрами, иначе поедет ширина строки
     * и столбец ASCII перестанет быть столбцом. 4 ГБ дампа — заведомо больше
     * всего, что влезет на TF-карту за сессию.
     * На платах size_t 32-битный, там проверка тавтология — отсюда #if. */
#if SIZE_MAX > 0xFFFFFFFFu
    if ((uint64_t)offset > 0xFFFFFFFFull) {
        return -1;
    }
#endif

    for (int shift = (OFFSET_DIGITS - 1) * 4; shift >= 0; shift -= 4) {
        tmp[pos++] = HEX[(offset >> shift) & 0xFu];
    }
    tmp[pos++] = ' ';
    tmp[pos++] = ' ';

    for (size_t i = 0; i < G920_HEXDUMP_BYTES_PER_LINE; i++) {
        if (i == GROUP_SIZE) {
            tmp[pos++] = ' ';
        }
        if (i < len) {
            tmp[pos++] = HEX[(bytes[i] >> 4) & 0xFu];
            tmp[pos++] = HEX[bytes[i] & 0xFu];
        } else {
            tmp[pos++] = ' ';
            tmp[pos++] = ' ';
        }
        tmp[pos++] = ' ';
    }

    /* Ещё один пробел перед ASCII — ровно как у `hexdump -C`. */
    tmp[pos++] = ' ';
    tmp[pos++] = '|';
    for (size_t i = 0; i < len; i++) {
        tmp[pos++] = (bytes[i] >= 0x20 && bytes[i] < 0x7F) ? (char)bytes[i]
                                                           : '.';
    }
    tmp[pos++] = '|';

    if (size < pos + 1u) {
        return -1;
    }
    for (size_t i = 0; i < pos; i++) {
        buf[i] = tmp[i];
    }
    buf[pos] = '\0';
    return (int)pos;
}
