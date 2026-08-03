/*
 * Журнал событий хоста — сырьё для опознания платформы.
 *
 * RX втыкают в разные хосты (M2), и от каждого снимается: какие
 * control-запросы пришли, в каком порядке, с какими интервалами, на что
 * хост получил STALL, когда впервые дёрнул IN endpoint.
 *
 * ⚠ Эти трассы дают **признаки платформы и тайминги энумерации**, но не
 * пороги детектора M6: те названы теми же буквами T1/T2 и означают другие
 * величины (GIP-тишина и простой IN в HID-режиме). Подробнее — у
 * g920_hostlog_t1_us и в docs/HOSTS.md.
 *
 * Отсюда три решения.
 *
 * ── Отсчёт от VBUS, а не от bus reset ──────────────────────────────────
 *
 * Инвариант И4: опознание привязано к появлению питания. Bus reset,
 * suspend/resume и переэнумерация личность не сбрасывают. Точка отсчёта
 * журнала — та же: VBUS. Иначе холодный старт с выключенного хоста, ради
 * которого и меряется T1, попадёт в журнал уже с середины.
 *
 * ── Переполнение теряет хвост, а не голову ─────────────────────────────
 *
 * Кольцевой буфер, затирающий старое, здесь был бы прямо вреден: в
 * фингерпринте энумерации ценно именно **начало** — порядок первых
 * GET_DESCRIPTOR, момент SET_ADDRESS, длины первых запросов. Журнал
 * заполняется и останавливается, считая потерянные события. Лучше знать,
 * что хвост потерян, чем тихо лишиться головы.
 *
 * ── Время относительное и 32-битное ────────────────────────────────────
 *
 * Микросекунды от точки отсчёта. 32 бита — это 71 минута, энумерации
 * хватает с запасом, зато запись вдвое компактнее и трассы разных
 * платформ сравниваются напрямую, без приведения к общему нулю.
 */

#ifndef G920_HOSTLOG_H
#define G920_HOSTLOG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    G920_HOST_EV_NONE = 0,
    G920_HOST_EV_VBUS, /* появилось питание — точка отсчёта (И4) */
    G920_HOST_EV_BUS_RESET,
    G920_HOST_EV_SETUP, /* control-запрос со всеми полями */
    /*
     * ⚠ **Недостижим на TinyUSB.** Стандартный SET_ADDRESS обрабатывает
     * ядро стека, колбэка приложению оно не даёт, и в живых трассах M2
     * этого события нет ни одного. Тип оставлен потому, что журнал
     * читается и разбирается общим кодом, а в M1 тот же формат поедет для
     * трассы **руля**, снятой USB-хостом, — там SET_ADDRESS виден.
     * Считать его признаком платформы в M6 нельзя.
     */
    G920_HOST_EV_SET_ADDRESS,
    G920_HOST_EV_SET_CONFIG,
    G920_HOST_EV_IN_POLLED, /* хост дёрнул IN endpoint */
    G920_HOST_EV_GIP_IN, /* принято GIP-сообщение от хоста */
    G920_HOST_EV_GIP_OUT, /* отправлено GIP-сообщение хосту */
    G920_HOST_EV_SUSPEND,
    G920_HOST_EV_RESUME
} g920_host_event_type_t;

typedef enum {
    G920_HOST_ANSWER_NONE = 0,
    G920_HOST_ANSWER_DATA,
    /* STALL — не «ошибка», а осмысленный ответ, и он различает платформы:
     * Windows спрашивает Device Qualifier, консоль нет. */
    G920_HOST_ANSWER_STALL
} g920_host_answer_t;

typedef struct {
    uint32_t at_us; /* от точки отсчёта */
    uint8_t type;
    uint8_t answer;
    /* Поля SETUP-пакета. Для остальных событий нули; для GIP в b_request
     * лежит MessageType, в w_length — длина полезной части. */
    uint8_t bm_request_type;
    uint8_t b_request;
    uint16_t w_value;
    uint16_t w_index;
    uint16_t w_length;
} g920_host_event_t;

typedef struct {
    g920_host_event_t *events;
    uint16_t capacity;
    uint16_t count;
    uint16_t dropped;
    uint64_t origin_us;
    bool origin_set;
    /* Точку отсчёта поставило первое же событие, а не явный VBUS. Значит
     * начало трассы могло быть пропущено — при разборе замеров это надо
     * знать. */
    bool origin_implicit;
} g920_hostlog_t;

/* Стандартные bRequest, по которым различаются стеки хостов. */
#define G920_USB_REQ_GET_DESCRIPTOR 0x06u
#define G920_USB_REQ_SET_ADDRESS 0x05u
#define G920_USB_REQ_SET_CONFIGURATION 0x09u

void g920_hostlog_init(g920_hostlog_t *log, g920_host_event_t *storage,
                       uint16_t capacity);

/*
 * Бросает записи, но не хранилище.
 *
 * ⚠ **Вызывающего в прошивке пока нет, и это осознанно.** Журнал
 * сбрасывается при переопознании хоста, а переопознание появляется в M6.
 * Звать его в M2 неоткуда: заглушка живёт от загрузки до отключения, и
 * сброс посреди этого стёр бы ровно ту трассу, ради которой веха сделана.
 * Если функция доживёт до M6 без вызывающего — её надо удалить, а не
 * оставлять «на будущее»: соседняя веха уже блокировалась за такое.
 */
void g920_hostlog_reset(g920_hostlog_t *log);

/* Явно поставить точку отсчёта — вызывается по появлению VBUS. */
void g920_hostlog_mark_origin(g920_hostlog_t *log, uint64_t now_us);

/*
 * Добавляют событие. Возвращают false, если журнал полон: событие
 * потеряно и учтено в dropped.
 */
bool g920_hostlog_add(g920_hostlog_t *log, uint64_t now_us,
                      g920_host_event_type_t type);

bool g920_hostlog_add_setup(g920_hostlog_t *log, uint64_t now_us,
                            uint8_t bm_request_type, uint8_t b_request,
                            uint16_t w_value, uint16_t w_index,
                            uint16_t w_length, g920_host_answer_t answer);

bool g920_hostlog_add_gip(g920_hostlog_t *log, uint64_t now_us, bool incoming,
                          uint8_t message_type, uint16_t payload_length);

uint16_t g920_hostlog_count(const g920_hostlog_t *log);
uint16_t g920_hostlog_dropped(const g920_hostlog_t *log);
const g920_host_event_t *g920_hostlog_at(const g920_hostlog_t *log,
                                         uint16_t index);

/* --- замеры ------------------------------------------------------------- */

/*
 * T1 — от точки отсчёта до первой активности хоста. Меряется в худшем
 * случае: воткнуть в выключённый хост и включить его.
 * false, если активности ещё не было.
 */
bool g920_hostlog_t1_us(const g920_hostlog_t *log, uint32_t *out);

/*
 * T2 — от SET_CONFIGURATION до первого опроса IN endpoint. По нему в M6
 * работает откат: в HID-режиме нет опроса дольше T2 → возвращаемся в GIP.
 * false, если одного из двух событий ещё не было.
 */
bool g920_hostlog_t2_us(const g920_hostlog_t *log, uint32_t *out);

/* --- признаки для фингерпринта ----------------------------------------- */

uint16_t g920_hostlog_count_request(const g920_hostlog_t *log,
                                    uint8_t b_request);

/* Спрашивал ли хост дескриптор такого типа (старший байт wValue). */
bool g920_hostlog_saw_descriptor(const g920_hostlog_t *log,
                                 uint8_t descriptor_type);

/* Сколько раз хост получил STALL. */
uint16_t g920_hostlog_count_stalls(const g920_hostlog_t *log);

bool g920_hostlog_saw_event(const g920_hostlog_t *log,
                            g920_host_event_type_t type);

/* --- вывод -------------------------------------------------------------- */

/* Максимум символов в строке события вместе с '\0'. */
#define G920_HOSTLOG_LINE_MAX 64

/*
 * Одна строка трассы, без перевода строки:
 *   "  0.001234 SETUP 80 06 0100 0000 len 40 STALL"
 * Возвращает длину без '\0' либо -1.
 */
int g920_hostlog_format(char *buf, size_t size, const g920_host_event_t *event);

const char *g920_host_event_name(g920_host_event_type_t type);

/* Выгружает журнал в лог целиком, по строке на событие. */
void g920_hostlog_dump(const g920_hostlog_t *log, const char *tag);

#ifdef __cplusplus
}
#endif

#endif /* G920_HOSTLOG_H */
