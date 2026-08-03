/*
 * GIP: заголовок сообщения.
 *
 * Источник — `docs/gip-official/txt/H001419 - Original GIP Spec.txt`,
 * раздел «Message Header», таблицы 4-1 … 4-10. Ничего сверх спеки здесь
 * нет и быть не должно.
 *
 * Заголовок нужен обеим сторонам: TX разбирает поток от руля, RX собирает и
 * разбирает поток от хоста. Это единственный крупный кусок протокольной
 * логики, честно общий для двух прошивок.
 *
 * Раскладка (таблица 4-1):
 *
 *   0  MessageType   [7:5] класс данных, [4:0] номер сообщения
 *   1  Flags         [7] Fragment [6] InitFrag [5] System [4] ACME
 *                    [3] резерв   [2:0] Expansion Index
 *   2  Sequence ID   счётчик, 0x00 зарезервирован
 *   3  Payload Length расширяемое поле: [7] признак продолжения, [6:0] данные
 *
 * При установленном Fragment сразу за полем длины идёт поле TLO: при
 * InitFrag=1 это полная длина сообщения, при InitFrag=0 — смещение
 * фрагмента. Оно тоже расширяемое.
 *
 * ── Почему хранится ширина полей ────────────────────────────────────────
 *
 * Спека разрешает записать одно и то же число разной длиной: длина 58
 * пишется как `3A` или как `BA 00`. Второй вариант — не избыточность, а
 * приём из спеки: заголовки вниз к устройству обязаны иметь чётный размер,
 * и его добивают именно расширением поля длины (таблицы 4-6 и 4-7).
 *
 * Значит разбор и сборка обязаны давать **побайтово тот же** заголовок.
 * Иначе туннель перестаёт быть вербатим — а это инвариант И1, ради
 * которого вся архитектура и построена. Поэтому в структуре лежит не только
 * значение, но и число байт, которым оно было записано.
 *
 * ── Чего здесь нет ──────────────────────────────────────────────────────
 *
 * Контрольной суммы. В GIP её нет: целостность обеспечивают CRC самого USB
 * на уровне пакета и Protocol Control с ACK на уровне протокола. Слова
 * «CRC» нет ни в одном из документов пакета `gip-official`.
 */

#ifndef G920_GIP_H
#define G920_GIP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Минимальный заголовок: тип, флаги, номер, однобайтная длина. */
#define G920_GIP_HEADER_MIN 4

/*
 * Расширяемое поле не длиннее четырёх байт (спека, § 3.3), значит потолок
 * заголовка — 3 + 4 + 4.
 */
#define G920_GIP_VARINT_MAX_BYTES 4
#define G920_GIP_VARINT_MAX 0x0FFFFFFFu
#define G920_GIP_HEADER_MAX (3 + 2 * G920_GIP_VARINT_MAX_BYTES)

/* Флаги, байт 1. */
#define G920_GIP_FLAG_FRAGMENT 0x80u
#define G920_GIP_FLAG_INIT_FRAG 0x40u
#define G920_GIP_FLAG_SYSTEM 0x20u
#define G920_GIP_FLAG_ACME 0x10u
#define G920_GIP_FLAG_RESERVED 0x08u
#define G920_GIP_EXPANSION_MASK 0x07u

/* Класс данных, старшие три бита MessageType. */
typedef enum {
    G920_GIP_CLASS_COMMAND = 0,
    G920_GIP_CLASS_LOW_LATENCY = 1,
    G920_GIP_CLASS_STANDARD_LATENCY = 2,
    G920_GIP_CLASS_AUDIO = 3
} g920_gip_class_t;

typedef enum {
    G920_GIP_OK = 0,
    G920_GIP_TRUNCATED, /* буфера не хватило, чтобы дочитать заголовок */
    G920_GIP_MALFORMED, /* поле длины тянется дольше четырёх байт */
    G920_GIP_BAD_ARG
} g920_gip_status_t;

typedef struct {
    /* Байты 0 и 1 хранятся целиком, а не разобранными на поля: разбирать
     * их на структуру и собирать заново — тот самый перевод представлений,
     * от которого предостерегает И1. Разбор — дело функций-геттеров. */
    uint8_t message_type;
    uint8_t flags;
    uint8_t sequence;

    uint32_t payload_length;

    /* Полная длина сообщения при InitFrag=1, смещение фрагмента иначе. */
    uint32_t tlo;

    /* Чем эти значения были записаны в исходных байтах, 1..4. */
    uint8_t length_bytes;
    uint8_t tlo_bytes; /* 0 — поля не было */

    uint8_t header_length;
} g920_gip_header_t;

/* --- расширяемые поля длины -------------------------------------------- */

/*
 * Читает расширяемое поле: младшие семь бит каждого байта складываются в
 * число, бит 7 означает «дальше есть ещё байт».
 * bytes — сколько байт занято, может быть NULL.
 */
g920_gip_status_t g920_gip_varint_decode(uint32_t *value, uint8_t *bytes,
                                         const uint8_t *buf, size_t size);

/*
 * Пишет расширяемое поле. width == 0 — минимальным числом байт;
 * иначе ровно width байт, дополняя нулями (так спека добивает заголовок до
 * чётного размера).
 * Возвращает число записанных байт либо -1.
 */
int g920_gip_varint_encode(uint8_t *buf, size_t size, uint32_t value,
                           uint8_t width);

/* Сколько байт нужно значению минимально: 1..4. */
uint8_t g920_gip_varint_width(uint32_t value);

/* --- заголовок ---------------------------------------------------------- */

g920_gip_status_t g920_gip_header_parse(g920_gip_header_t *out,
                                        const uint8_t *buf, size_t size);

/*
 * Собирает заголовок обратно. При тех же length_bytes / tlo_bytes, что
 * вернул разбор, результат обязан совпасть с исходником побайтово.
 * Возвращает число записанных байт либо -1.
 */
int g920_gip_header_build(uint8_t *buf, size_t size,
                          const g920_gip_header_t *header);

/* --- разбор полей ------------------------------------------------------- */

g920_gip_class_t g920_gip_data_class(const g920_gip_header_t *header);
uint8_t g920_gip_message_number(const g920_gip_header_t *header);
uint8_t g920_gip_expansion_index(const g920_gip_header_t *header);

bool g920_gip_is_fragment(const g920_gip_header_t *header);
bool g920_gip_is_initial_fragment(const g920_gip_header_t *header);
bool g920_gip_is_system(const g920_gip_header_t *header);
bool g920_gip_wants_ack(const g920_gip_header_t *header);

const char *g920_gip_status_name(g920_gip_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* G920_GIP_H */
