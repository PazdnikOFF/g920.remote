/*
 * GIP: сборка фрагментированного сообщения.
 *
 * Источник — `H001419`, § 3.3 «Large Message Handling», таблицы 4-6 … 4-12.
 *
 * Правила, которые здесь реализованы:
 *
 *  - Все фрагменты одного сообщения несут один и тот же Sequence ID.
 *  - У первого фрагмента InitFrag=1, и поле TLO — полная длина сообщения.
 *  - У остальных InitFrag=0, и поле TLO — смещение фрагмента.
 *  - **Последний фрагмент ничем не помечен.** Отдельного бита «конец» в
 *    протоколе нет: сообщение считается собранным, когда принято ровно
 *    столько байт, сколько объявил первый фрагмент.
 *  - После этого отправитель шлёт нулевой «completion»-фрагмент, и только
 *    получив ACK, покрывающий все байты. Для собирающей стороны это
 *    подтверждение конца, а не источник данных.
 *
 * Буфер даёт вызывающий. Своего размера у сборщика нет намеренно: блоб
 * метаданных и security-обмен различаются на порядок, а на RX позже
 * появится PSRAM.
 *
 * Это сборка, а не перевод формата: собранные байты уезжают дальше как
 * есть. Инвариант И1 запрещает менять представление, а не склеивать
 * фрагменты — иначе auth passthrough в M8 был бы невозможен.
 */

#ifndef G920_GIP_REASM_H
#define G920_GIP_REASM_H

#include "g920/gip.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    /* Сообщение целиком уместилось в один пакет — фрагментации не было. */
    G920_GIP_REASM_SINGLE = 0,
    /* Фрагмент принят, ждём следующий. */
    G920_GIP_REASM_MORE,
    /* Принято ровно столько, сколько объявлено: сообщение собрано. */
    G920_GIP_REASM_DONE,
    /* Нулевой completion-фрагмент после уже собранного сообщения. */
    G920_GIP_REASM_COMPLETION,

    /* Фрагмент без начала: первый пакет потерян или мы включились посреди. */
    G920_GIP_REASM_ORPHAN,
    /* Смещение не там, где ждали: пакет потерян или переупорядочен. */
    G920_GIP_REASM_OUT_OF_ORDER,
    /* Sequence ID сменился, пока сборка не закончена. */
    G920_GIP_REASM_SEQUENCE_MISMATCH,
    /* Не влезает: либо больше объявленного, либо больше буфера. */
    G920_GIP_REASM_OVERFLOW,
    G920_GIP_REASM_BAD_ARG
} g920_gip_reasm_status_t;

typedef struct {
    uint8_t *buffer;
    size_t capacity;

    bool active;
    uint8_t message_type;
    uint8_t flags; /* флаги первого фрагмента, как пришли */
    uint8_t sequence;
    uint32_t total; /* объявленная полная длина */
    uint32_t received; /* сколько собрано подряд */
} g920_gip_reasm_t;

void g920_gip_reasm_init(g920_gip_reasm_t *reasm, uint8_t *buffer,
                         size_t capacity);

/* Бросить незаконченную сборку. */
void g920_gip_reasm_reset(g920_gip_reasm_t *reasm);

/*
 * Скармливает очередной пакет: разобранный заголовок и его полезную часть.
 *
 * При SINGLE и DONE собранное сообщение лежит в буфере, длина — в
 * g920_gip_reasm_length(). При SINGLE это просто скопированная нагрузка.
 */
g920_gip_reasm_status_t g920_gip_reasm_push(g920_gip_reasm_t *reasm,
                                            const g920_gip_header_t *header,
                                            const uint8_t *payload,
                                            size_t length);

size_t g920_gip_reasm_length(const g920_gip_reasm_t *reasm);
bool g920_gip_reasm_in_progress(const g920_gip_reasm_t *reasm);

/*
 * Сколько ещё места остаётся до конца объявленного сообщения. Это и есть
 * поле Remaining Buffer в ответном ACK.
 */
uint16_t g920_gip_reasm_remaining(const g920_gip_reasm_t *reasm);

const char *g920_gip_reasm_status_name(g920_gip_reasm_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* G920_GIP_REASM_H */
