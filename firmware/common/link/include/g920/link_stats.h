/*
 * Статистика линка: RTT, джиттер, потери.
 *
 * Веха M3 требует не «работает», а цифры: RTT, джиттер, доля потерь,
 * устойчивая частота кадров, деградация при загруженном Wi-Fi рядом.
 * По ним в M5 выбирается частота отчётов, которая сейчас записана как
 * «200–500 Гц по результатам M3», то есть не выбрана.
 *
 * Считается здесь, а не в прошивке, потому что это чистая арифметика и её
 * можно проверить на хосте. Ошибиться в оценке джиттера легко, а заметить
 * ошибку по логу на плате — почти нельзя.
 */

#ifndef G920_LINK_STATS_H
#define G920_LINK_STATS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "g920/proto.h"
#include "g920/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t sent;
    uint32_t received;
    uint32_t duplicates;
    uint32_t stale;
    uint32_t send_failed;

    uint32_t rtt_min_us;
    uint32_t rtt_max_us;
    uint32_t rtt_last_us;
    uint64_t rtt_sum_us;
    uint32_t rtt_count;

    /*
     * Джиттер по RFC 3550: J += (|D| − J) / 16, где D — разность соседних
     * задержек. Экспоненциальное сглаживание, а не разброс «макс минус
     * мин»: одиночный выброс не должен навсегда испортить оценку, а
     * систематическое дрожание обязано быть видно.
     */
    uint32_t jitter_us;
} g920_link_stats_t;

void g920_link_stats_init(g920_link_stats_t *stats);

void g920_link_stats_on_sent(g920_link_stats_t *stats, bool ok);

/* Кадр вернулся: обновляет RTT и джиттер. */
void g920_link_stats_on_rtt(g920_link_stats_t *stats, uint32_t rtt_us);

/*
 * Снимок счётчиков приёмника линка. Отброшенные кадры приложение не видит —
 * их фильтрует линк, — поэтому доля потерь считается по его числам, а не по
 * тому, что дошло наверх.
 */
void g920_link_stats_add_rx(g920_link_stats_t *stats, uint32_t delivered,
                            uint32_t duplicates, uint32_t stale);

uint32_t g920_link_stats_avg_rtt_us(const g920_link_stats_t *stats);

/* Доля потерь в промилле от отправленного. */
uint32_t g920_link_stats_loss_permille(const g920_link_stats_t *stats);

/* Строка для лога и для записи в PROGRESS. Длина без '\0' либо -1. */
#define G920_LINK_STATS_STR_MAX 128
int g920_link_stats_format(char *buf, size_t size,
                           const g920_link_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* G920_LINK_STATS_H */
