/*
 * GIP: Protocol Control — механизм надёжной доставки.
 *
 * Источник — `H001419`, таблица 4-14 «Bidirectional GIP Message: Protocol
 * Control» и § 3.2 «Reliable Message Acknowledgement».
 *
 * Отправитель просит подтвердить сообщение или фрагмент, выставив ACME в
 * флагах заголовка. Получатель отвечает Protocol Control с ControlCode ACK.
 * Это нужно для крупных сообщений: метаданных (M4) и security (M8).
 *
 * Полезная нагрузка ровно 9 байт:
 *
 *   0     ControlCode       0x00 = ACK, других действующих значений нет
 *   1     RefMessageType    MessageType подтверждаемого сообщения
 *   2     RefMessageFlags   только биты System и Expansion Index, остальные ноль
 *   3..6  Fragment Offset   32 бита, little-endian
 *   7..8  Remaining Buffer  16 бит, little-endian
 *
 * Sequence ID ответа обязан совпадать с Sequence ID подтверждаемого
 * сообщения — это и есть привязка ACK к сообщению.
 */

#ifndef G920_GIP_CONTROL_H
#define G920_GIP_CONTROL_H

#include "g920/gip.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Номера системных сообщений, таблица 4-4. */
#define G920_GIP_MSG_PROTOCOL_CONTROL 0x01u

/*
 * ControlCode. По спеке 0x00 — единственное действующее значение; 0x01..0x07
 * были определены в прошлом (NACK, UNK, AB, MPER, STOP, START, ERR) и больше
 * не используются. Разбор их не отвергает: своё мнение о чужом трафике
 * ломает вербатимность (И1), а решать — дело вызывающего.
 */
#define G920_GIP_CONTROL_ACK 0x00u

#define G920_GIP_CONTROL_PAYLOAD_SIZE 9

/*
 * Маска для RefMessageFlags: System (бит 5) и Expansion Index (биты 2:0).
 * Всё остальное в ссылке обязано быть нулём — таблица 4-14.
 */
#define G920_GIP_REF_FLAGS_MASK 0x27u

typedef struct {
    uint8_t control_code;
    uint8_t ref_message_type;
    uint8_t ref_message_flags;
    /* Сколько байт сообщения принято подряд. Для не-последнего ACK это же
     * значение — желаемое начало следующего фрагмента. */
    uint32_t fragment_offset;
    /* Сколько места осталось у получателя. Обычно — сколько ещё осталось
     * дослать до полного сообщения. */
    uint16_t remaining_buffer;
} g920_gip_control_t;

g920_gip_status_t g920_gip_control_parse(g920_gip_control_t *out,
                                         const uint8_t *payload, size_t size);

/* Возвращает число записанных байт (всегда 9) либо -1. */
int g920_gip_control_build(uint8_t *buf, size_t size,
                           const g920_gip_control_t *control);

/*
 * Собирает готовый пакет ACK на принятое сообщение: заголовок плюс нагрузка.
 * Sequence ID берётся из подтверждаемого заголовка, RefMessageFlags —
 * из его флагов по маске.
 *
 * Возвращает длину пакета (13 байт) либо -1.
 */
int g920_gip_build_ack(uint8_t *buf, size_t size,
                       const g920_gip_header_t *acked, uint32_t fragment_offset,
                       uint16_t remaining_buffer);

/*
 * Просить ли подтверждение на этот фрагмент.
 *
 * Спека (§ 3.2): ACME обязателен на первом и последнем фрагменте, а из
 * средних подтверждают «обычно каждый четвёртый или пятый», чтобы уложиться
 * в 100 мс таймаута ACK у хоста. Периодичность вынесена параметром, а не
 * зашита: это единственное место, где выбор наш, и он должен быть виден.
 */
bool g920_gip_should_request_ack(uint32_t fragment_index, bool is_last,
                                 uint32_t every_n);

#ifdef __cplusplus
}
#endif

#endif /* G920_GIP_CONTROL_H */
