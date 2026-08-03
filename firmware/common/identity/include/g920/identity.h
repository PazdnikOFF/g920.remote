/*
 * Личность руля: контейнер снятого с настоящего устройства.
 *
 * Инвариант И2: в прошивке RX не должно быть ни одного захардкоженного
 * дескриптора, VID/PID, строки или блоба метаданных G920. Всё приходит от TX
 * и лежит в NVS. Этот модуль — то, чем оно приходит и лежит.
 *
 * ── Главное свойство: модуль не смотрит внутрь ─────────────────────────
 *
 * Секции хранятся как непрозрачные байты. Ни одна функция здесь не
 * разбирает дескриптор, не проверяет его на осмысленность и не знает, что
 * такое G920. Единственное исключение — вытаскивание VID/PID/bcdDevice из
 * дескриптора устройства, и то по стандартным смещениям USB, а не по
 * знанию конкретного руля: без них нечем отличить один руль от другого.
 *
 * Отсюда же следует, что модуль можно написать до того, как снят дамп M1:
 * содержимое секций на формат не влияет.
 *
 * ── Незнакомые секции переживают разбор ────────────────────────────────
 *
 * Тип секции — это данные, а не часть формата. Блоб, записанный прошивкой
 * новее нашей, разбирается целиком: незнакомые секции просто пропускаются
 * по длине и остаются в буфере. Поэтому добавление нового типа секции
 * **не требует поднимать версию формата**, и разные прошивки TX и RX не
 * разъезжаются на каждом шаге.
 *
 * Версия формата поднимается только при смене раскладки самого контейнера.
 *
 * ── Раскладка ──────────────────────────────────────────────────────────
 *
 *   заголовок, 18 байт:
 *     0..3   магия "G9ID"
 *     4..5   версия формата контейнера
 *     6..7   число секций
 *     8..9   idVendor
 *     10..11 idProduct
 *     12..13 bcdDevice
 *     14..17 общая длина вместе с заголовком
 *
 *   далее секции подряд:
 *     0      тип
 *     1      индекс (для строковых дескрипторов — их номер)
 *     2..3   длина данных
 *     4..    данные
 *
 * Буфер и есть сериализованная форма: отдельного «упаковать» нет, потому
 * что перекладывание из одного представления в другое — ровно то, чего
 * требует избегать И1.
 */

#ifndef G920_IDENTITY_H
#define G920_IDENTITY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* "G9ID" в порядке байтов записи. */
#define G920_IDENTITY_MAGIC 0x44493947u
#define G920_IDENTITY_FORMAT_VERSION 1u

#define G920_IDENTITY_HEADER_SIZE 18
#define G920_IDENTITY_SECTION_HEADER_SIZE 4

/*
 * Виды секций. Значения фиксированы навсегда: они уезжают по радио и лежат
 * во флеше. Новые добавлять только в конец — старая прошивка их пропустит,
 * а не подавится.
 */
typedef enum {
    G920_ID_DEVICE_DESC = 1,
    G920_ID_CONFIG_DESC = 2,
    G920_ID_STRING_DESC = 3, /* индекс = номер строкового дескриптора */
    G920_ID_HID_REPORT_DESC = 4, /* снимается в режиме c262 */
    G920_ID_GIP_METADATA = 5, /* блоб сообщения 0x04 */
    G920_ID_GIP_HELLO = 6 /* содержимое Hello, сообщение 0x02 */
} g920_identity_section_t;

typedef enum {
    G920_IDENTITY_OK = 0,
    G920_IDENTITY_EMPTY, /* буфер пуст: личности ещё нет */
    G920_IDENTITY_BAD_MAGIC,
    G920_IDENTITY_BAD_VERSION,
    G920_IDENTITY_BAD_LENGTH, /* длины не сходятся: блоб битый или обрезан */
    G920_IDENTITY_NO_SPACE,
    G920_IDENTITY_BAD_ARG
} g920_identity_status_t;

/*
 * Отпечаток руля. По нему и только по нему решается, годится ли кэш:
 * консоль кэширует метаданные по VID/PID/Revision, и расхождение между
 * заявленной личностью и тем, за что ручается security-обмен, — потенциально
 * нерешаемый баг.
 */
typedef struct {
    uint16_t vendor_id;
    uint16_t product_id;
    uint16_t device_release;
} g920_identity_fingerprint_t;

typedef struct {
    uint8_t *buffer;
    size_t capacity;
    size_t used;
    uint16_t sections;
    g920_identity_fingerprint_t fingerprint;
} g920_identity_t;

/* Готовит пустой контейнер поверх буфера вызывающего. */
g920_identity_status_t g920_identity_init(g920_identity_t *identity,
                                          uint8_t *buffer, size_t capacity);

void g920_identity_set_fingerprint(g920_identity_t *identity,
                                   g920_identity_fingerprint_t fingerprint);

/* Добавляет секцию. Данные копируются как есть, без разбора. */
g920_identity_status_t g920_identity_add(g920_identity_t *identity,
                                         g920_identity_section_t type,
                                         uint8_t index, const void *data,
                                         uint16_t length);

/*
 * Разбирает готовый блоб (из NVS или из линка) поверх того же буфера.
 * Незнакомые типы секций не мешают: они пропускаются по длине.
 */
g920_identity_status_t g920_identity_parse(g920_identity_t *identity,
                                           uint8_t *buffer, size_t size);

/*
 * Ищет секцию. Возвращает указатель внутрь буфера либо NULL.
 * Копии нет: вызывающий отдаёт эти байты хосту вербатим.
 */
const uint8_t *g920_identity_find(const g920_identity_t *identity,
                                  g920_identity_section_t type, uint8_t index,
                                  uint16_t *out_length);

uint16_t g920_identity_section_count(const g920_identity_t *identity);
size_t g920_identity_size(const g920_identity_t *identity);
bool g920_identity_empty(const g920_identity_t *identity);

/*
 * Годится ли эта личность для руля с таким отпечатком. false означает
 * «руль сменили» — кэш надо выбросить и снимать заново.
 */
bool g920_identity_matches(const g920_identity_t *identity,
                           g920_identity_fingerprint_t fingerprint);

/*
 * Достаёт отпечаток из дескриптора устройства по стандартным смещениям
 * USB. Единственное место, где модуль вообще заглядывает в байты, и оно
 * не знает ничего про G920.
 */
bool g920_identity_fingerprint_from_device_descriptor(
    g920_identity_fingerprint_t *out, const uint8_t *descriptor, size_t size);

const char *g920_identity_status_name(g920_identity_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* G920_IDENTITY_H */
