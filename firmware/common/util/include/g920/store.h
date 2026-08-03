/*
 * Постоянное хранилище с версионированием структур — общий код обеих
 * прошивок.
 *
 * Что здесь лежит по ходу проекта:
 *   - MAC пира линка после пиринга по кнопке (M3);
 *   - клонированная личность руля: дескрипторы, строки, HID report
 *     descriptor, блоб метаданных GIP (M4);
 *   - кэш вердикта детектора хоста (M6).
 *
 * Почему не просто nvs_set_blob:
 *
 * Все три структуры переживут не одну переделку формата, а прошивку на TX и
 * на RX перезальют в разное время. Блоб, записанный прошивкой другой версии,
 * прочитанный как текущая структура — это молчаливая порча: личность руля
 * разъедется с тем, за что ручается security-обмен, и отлаживать это придётся
 * по симптомам на консоли. Поэтому у каждой записи есть заголовок с магией,
 * видом и версией схемы, и чтение чужой версии — явная ошибка, а не мусор в
 * полях.
 *
 * Правило по версиям:
 *   - версия старше текущей → G920_STORE_VERSION_OLDER, вызывающий волен
 *     мигрировать (прочитать с G920_STORE_ANY_VERSION и переписать);
 *   - версия новее текущей → G920_STORE_VERSION_NEWER, запись не трогаем.
 *     Её написала прошивка, которая знает больше нас; перезаписать её значит
 *     потерять данные при откате на старую прошивку.
 */

#ifndef G920_STORE_H
#define G920_STORE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* "G920" в порядке байтов записи. */
#define G920_STORE_MAGIC 0x30323947u

#define G920_STORE_HEADER_SIZE 12

/* Ограничение NVS на длину ключа — 15 значащих символов. */
#define G920_STORE_KEY_MAX 15

/*
 * Потолок полезной нагрузки одной записи. Личность руля в M4 — единицы
 * килобайт, и она поедет отдельными записями по частям, а не одним блобом:
 * иначе каждая правка дескриптора переписывает всё.
 *
 * Значение упирается в стек: запись целиком собирается в буфере на стеке
 * вызывающей задачи, а у задач FreeRTOS его единицы килобайт.
 */
#define G920_STORE_PAYLOAD_MAX 512

/* Читать запись любой версии — для миграции. */
#define G920_STORE_ANY_VERSION 0xFFFFu

typedef enum {
    G920_STORE_OK = 0,
    G920_STORE_EMPTY, /* ключа нет — не ошибка, а «ещё не писали» */
    G920_STORE_BAD_MAGIC,
    G920_STORE_WRONG_KIND,
    G920_STORE_VERSION_OLDER,
    G920_STORE_VERSION_NEWER,
    G920_STORE_BAD_LENGTH, /* запись повреждена: длина не сходится */
    G920_STORE_TOO_SMALL, /* буфер вызывающего мал под нагрузку */
    G920_STORE_BAD_ARG,
    G920_STORE_IO_ERROR
} g920_store_status_t;

/*
 * Вид структуры. Значения фиксированы навсегда: они лежат во флеше.
 * Новые виды добавлять только в конец.
 */
typedef enum {
    G920_STORE_KIND_PEER = 1, /* MAC пира линка, M3 */
    G920_STORE_KIND_IDENTITY = 2, /* личность руля, M4 */
    G920_STORE_KIND_VERDICT = 3, /* вердикт детектора хоста, M6 */
    G920_STORE_KIND_EPOCH = 4 /* счётчик загрузок = эпоха сессии линка, M3 */
} g920_store_kind_t;

typedef struct {
    uint32_t magic;
    uint16_t kind;
    uint16_t version;
    uint32_t length; /* длина полезной нагрузки без заголовка */
} g920_store_header_t;

/* --- заголовок: чистые функции, проверяются на хосте ------------------- */

/*
 * Пишет заголовок в буфер в фиксированном порядке байтов (little-endian) —
 * порядок задан явно, чтобы запись не зависела от платформы.
 * Возвращает число записанных байт либо -1.
 */
int g920_store_header_pack(uint8_t *buf, size_t size,
                           const g920_store_header_t *header);

/* Разбирает заголовок. Проверяет только магию и размер буфера. */
g920_store_status_t g920_store_header_unpack(g920_store_header_t *out,
                                             const uint8_t *buf, size_t size);

/*
 * Сверяет разобранный заголовок с ожиданиями вызывающего.
 * version == G920_STORE_ANY_VERSION отключает проверку версии.
 */
g920_store_status_t g920_store_header_check(const g920_store_header_t *header,
                                            g920_store_kind_t kind,
                                            uint16_t version);

/* Годен ли ключ: 1..G920_STORE_KEY_MAX печатных символов без пробелов. */
int g920_store_key_valid(const char *key);

const char *g920_store_status_name(g920_store_status_t status);

/* --- хранилище --------------------------------------------------------- */

/*
 * Поднимает NVS. На плате — с обычной пересборкой раздела, если страницы
 * кончились или раздел от прошлой версии NVS. На хосте — хранилище в памяти
 * процесса, ровно для тестов и инструментов разбора трасс.
 */
g920_store_status_t g920_store_init(void);

g920_store_status_t g920_store_write(const char *key, g920_store_kind_t kind,
                                     uint16_t version, const void *payload,
                                     size_t len);

/*
 * Читает запись. out_len и out_version можно передать NULL.
 * При G920_STORE_VERSION_OLDER / VERSION_NEWER нагрузка не копируется, но
 * out_version заполняется — по нему вызывающий решает, мигрировать или
 * сдаться.
 */
g920_store_status_t g920_store_read(const char *key, g920_store_kind_t kind,
                                    uint16_t version, void *payload,
                                    size_t capacity, size_t *out_len,
                                    uint16_t *out_version);

g920_store_status_t g920_store_erase(const char *key);

/* Полная очистка: сброс пиринга и кэша личности (M10). */
g920_store_status_t g920_store_erase_all(void);

#ifdef __cplusplus
}
#endif

#endif /* G920_STORE_H */
