#include "g920/link_stats.h"

#include <string.h>

#define JITTER_SHIFT 4 /* деление на 16 из RFC 3550 */

void g920_link_stats_init(g920_link_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    memset(stats, 0, sizeof(*stats));
    stats->rtt_min_us = UINT32_MAX;
}

void g920_link_stats_on_sent(g920_link_stats_t *stats, bool ok)
{
    if (stats == NULL) {
        return;
    }
    stats->sent++;
    if (!ok) {
        stats->send_failed++;
    }
}

void g920_link_stats_on_rtt(g920_link_stats_t *stats, uint32_t rtt_us)
{
    if (stats == NULL) {
        return;
    }

    if (stats->rtt_count != 0) {
        uint32_t d = (rtt_us > stats->rtt_last_us)
                         ? (rtt_us - stats->rtt_last_us)
                         : (stats->rtt_last_us - rtt_us);

        /* J += (|D| − J) / 16, целочисленно и без знака. */
        if (d > stats->jitter_us) {
            stats->jitter_us += (d - stats->jitter_us) >> JITTER_SHIFT;
        } else {
            stats->jitter_us -= (stats->jitter_us - d) >> JITTER_SHIFT;
        }
    }

    if (rtt_us < stats->rtt_min_us) {
        stats->rtt_min_us = rtt_us;
    }
    if (rtt_us > stats->rtt_max_us) {
        stats->rtt_max_us = rtt_us;
    }
    stats->rtt_last_us = rtt_us;
    stats->rtt_sum_us += rtt_us;
    stats->rtt_count++;
}

void g920_link_stats_add_rx(g920_link_stats_t *stats, uint32_t delivered,
                            uint32_t duplicates, uint32_t stale)
{
    if (stats == NULL) {
        return;
    }
    /*
     * Отброшенные кадры приложение не видит: их фильтрует линк. Числа
     * приходят снимком его счётчиков — иначе «потеряно 3%» считалось бы по
     * тому, что и так дошло.
     */
    stats->received += delivered;
    stats->duplicates += duplicates;
    stats->stale += stale;
}

uint32_t g920_link_stats_avg_rtt_us(const g920_link_stats_t *stats)
{
    if (stats == NULL || stats->rtt_count == 0) {
        return 0;
    }
    return (uint32_t)(stats->rtt_sum_us / stats->rtt_count);
}

uint32_t g920_link_stats_loss_permille(const g920_link_stats_t *stats)
{
    uint32_t lost;

    if (stats == NULL || stats->sent == 0) {
        return 0;
    }
    if (stats->received >= stats->sent) {
        return 0;
    }
    lost = stats->sent - stats->received;
    return (uint32_t)(((uint64_t)lost * 1000u) / stats->sent);
}

/*
 * Писатели не пишут за границу и не молчат об этом: позиция растёт дальше
 * ёмкости, и вызывающий по ней видит, что строка не поместилась. Раньше
 * границы не было вовсе — при разросшихся счётчиках это переполнение стека
 * главной задачи.
 */
static size_t put_dec(char *buf, size_t cap, size_t pos, uint32_t value)
{
    char digits[10];
    int n = 0;

    do {
        digits[n++] = (char)('0' + (value % 10u));
        value /= 10u;
    } while (value != 0u);
    while (n > 0) {
        n--;
        if (pos < cap) {
            buf[pos] = digits[n];
        }
        pos++;
    }
    return pos;
}

static size_t put_str(char *buf, size_t cap, size_t pos, const char *text)
{
    for (size_t i = 0; text[i] != '\0'; i++) {
        if (pos < cap) {
            buf[pos] = text[i];
        }
        pos++;
    }
    return pos;
}

static size_t put_char(char *buf, size_t cap, size_t pos, char c)
{
    if (pos < cap) {
        buf[pos] = c;
    }
    return pos + 1;
}

/* "12.3" из промилле: целая часть и десятая доля процента. */
static size_t put_permille_as_percent(char *buf, size_t cap, size_t pos,
                                      uint32_t permille)
{
    pos = put_dec(buf, cap, pos, permille / 10u);
    pos = put_char(buf, cap, pos, '.');
    return put_dec(buf, cap, pos, permille % 10u);
}

int g920_link_stats_format(char *buf, size_t size,
                           const g920_link_stats_t *stats)
{
    char tmp[G920_LINK_STATS_STR_MAX];
    size_t pos = 0;
    uint32_t rtt_min;

    if (buf == NULL || stats == NULL) {
        return -1;
    }
    rtt_min = (stats->rtt_count != 0) ? stats->rtt_min_us : 0;

    pos = put_str(tmp, sizeof(tmp), pos, "sent ");
    pos = put_dec(tmp, sizeof(tmp), pos, stats->sent);
    pos = put_str(tmp, sizeof(tmp), pos, " rx ");
    pos = put_dec(tmp, sizeof(tmp), pos, stats->received);
    pos = put_str(tmp, sizeof(tmp), pos, " loss ");
    pos = put_permille_as_percent(tmp, sizeof(tmp), pos,
                                  g920_link_stats_loss_permille(stats));
    pos = put_str(tmp, sizeof(tmp), pos, "% rtt ");
    pos = put_dec(tmp, sizeof(tmp), pos, rtt_min);
    pos = put_char(tmp, sizeof(tmp), pos, '/');
    pos = put_dec(tmp, sizeof(tmp), pos, g920_link_stats_avg_rtt_us(stats));
    pos = put_char(tmp, sizeof(tmp), pos, '/');
    pos = put_dec(tmp, sizeof(tmp), pos, stats->rtt_max_us);
    pos = put_str(tmp, sizeof(tmp), pos, "us jit ");
    pos = put_dec(tmp, sizeof(tmp), pos, stats->jitter_us);
    pos = put_str(tmp, sizeof(tmp), pos, "us dup ");
    pos = put_dec(tmp, sizeof(tmp), pos, stats->duplicates);
    pos = put_str(tmp, sizeof(tmp), pos, " stale ");
    pos = put_dec(tmp, sizeof(tmp), pos, stats->stale);
    pos = put_str(tmp, sizeof(tmp), pos, " txfail ");
    pos = put_dec(tmp, sizeof(tmp), pos, stats->send_failed);

    /* Не поместилось ни в свой буфер, ни в чужой — обрезанная строка замера
     * хуже отсутствующей: её прочитают как настоящую. */
    if (pos > sizeof(tmp) || size < pos + 1u) {
        return -1;
    }
    memcpy(buf, tmp, pos);
    buf[pos] = '\0';
    return (int)pos;
}
