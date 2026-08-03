#include "g920/hostlog.h"

#include <string.h>

#include "g920/log.h"
#include "g920/timestamp.h"

static const char HEX[] = "0123456789abcdef";

void g920_hostlog_init(g920_hostlog_t *log, g920_host_event_t *storage,
                       uint16_t capacity)
{
    if (log == NULL) {
        return;
    }
    memset(log, 0, sizeof(*log));
    log->events = storage;
    log->capacity = (storage != NULL) ? capacity : 0;
}

void g920_hostlog_reset(g920_hostlog_t *log)
{
    if (log == NULL) {
        return;
    }
    log->count = 0;
    log->dropped = 0;
    log->origin_us = 0;
    log->origin_set = false;
    log->origin_implicit = false;
}

void g920_hostlog_mark_origin(g920_hostlog_t *log, uint64_t now_us)
{
    if (log == NULL) {
        return;
    }
    log->origin_us = now_us;
    log->origin_set = true;
    log->origin_implicit = false;
}

static uint32_t relative(g920_hostlog_t *log, uint64_t now_us)
{
    uint64_t delta;

    if (!log->origin_set) {
        /* Событие пришло раньше, чем отметили VBUS. Ставим отсчёт здесь и
         * помечаем, что начало трассы могло быть пропущено. */
        log->origin_us = now_us;
        log->origin_set = true;
        log->origin_implicit = true;
    }
    if (now_us < log->origin_us) {
        return 0;
    }
    delta = now_us - log->origin_us;
    return (delta > 0xFFFFFFFFull) ? 0xFFFFFFFFu : (uint32_t)delta;
}

static g920_host_event_t *reserve(g920_hostlog_t *log)
{
    if (log->count >= log->capacity) {
        /* Хвост теряем сознательно: голова трассы ценнее. */
        if (log->dropped < 0xFFFFu) {
            log->dropped++;
        }
        return NULL;
    }
    return &log->events[log->count++];
}

bool g920_hostlog_add(g920_hostlog_t *log, uint64_t now_us,
                      g920_host_event_type_t type)
{
    g920_host_event_t *ev;
    uint32_t at;

    if (log == NULL || log->events == NULL) {
        return false;
    }

    /* Отсчёт ставим до резервирования: даже потерянное событие сдвигает
     * точку отсчёта, если она ещё не стояла. */
    at = relative(log, now_us);
    if (type == G920_HOST_EV_VBUS) {
        at = 0;
    }

    ev = reserve(log);
    if (ev == NULL) {
        return false;
    }
    memset(ev, 0, sizeof(*ev));
    ev->at_us = at;
    ev->type = (uint8_t)type;
    return true;
}

bool g920_hostlog_add_setup(g920_hostlog_t *log, uint64_t now_us,
                            uint8_t bm_request_type, uint8_t b_request,
                            uint16_t w_value, uint16_t w_index,
                            uint16_t w_length, g920_host_answer_t answer)
{
    g920_host_event_t *ev;
    uint32_t at;
    g920_host_event_type_t type = G920_HOST_EV_SETUP;

    if (log == NULL || log->events == NULL) {
        return false;
    }

    /*
     * SET_ADDRESS и SET_CONFIGURATION — обычные control-запросы, но по ним
     * считается T2 и строится фингерпринт, поэтому у них свой тип.
     *
     * Смотрим и на тип запроса, а не только на код: `bRequest` уникален
     * лишь среди стандартных. Vendor-код MS OS сейчас `0x90` и не
     * сталкивается ни с чем, но в M4 он станет параметром личности, и
     * vendor-запрос с кодом 5 или 9 лёг бы в журнал как SET_ADDRESS или
     * SET_CONFIG — то есть испортил бы и T2, и фингерпринт.
     */
    if ((bm_request_type & 0x60u) == 0x00u
        && b_request == G920_USB_REQ_SET_ADDRESS) {
        type = G920_HOST_EV_SET_ADDRESS;
    } else if ((bm_request_type & 0x60u) == 0x00u
               && b_request == G920_USB_REQ_SET_CONFIGURATION) {
        type = G920_HOST_EV_SET_CONFIG;
    }

    at = relative(log, now_us);
    ev = reserve(log);
    if (ev == NULL) {
        return false;
    }
    ev->at_us = at;
    ev->type = (uint8_t)type;
    ev->answer = (uint8_t)answer;
    ev->bm_request_type = bm_request_type;
    ev->b_request = b_request;
    ev->w_value = w_value;
    ev->w_index = w_index;
    ev->w_length = w_length;
    return true;
}

bool g920_hostlog_add_gip(g920_hostlog_t *log, uint64_t now_us, bool incoming,
                          uint8_t message_type, uint16_t payload_length)
{
    g920_host_event_t *ev;
    uint32_t at;

    if (log == NULL || log->events == NULL) {
        return false;
    }

    at = relative(log, now_us);
    ev = reserve(log);
    if (ev == NULL) {
        return false;
    }
    memset(ev, 0, sizeof(*ev));
    ev->at_us = at;
    ev->type = (uint8_t)(incoming ? G920_HOST_EV_GIP_IN : G920_HOST_EV_GIP_OUT);
    ev->b_request = message_type;
    ev->w_length = payload_length;
    return true;
}

uint16_t g920_hostlog_count(const g920_hostlog_t *log)
{
    return (log != NULL) ? log->count : 0;
}

uint16_t g920_hostlog_dropped(const g920_hostlog_t *log)
{
    return (log != NULL) ? log->dropped : 0;
}

const g920_host_event_t *g920_hostlog_at(const g920_hostlog_t *log,
                                         uint16_t index)
{
    if (log == NULL || index >= log->count) {
        return NULL;
    }
    return &log->events[index];
}

/* --- замеры ------------------------------------------------------------- */

bool g920_hostlog_t1_us(const g920_hostlog_t *log, uint32_t *out)
{
    if (log == NULL || out == NULL) {
        return false;
    }
    for (uint16_t i = 0; i < log->count; i++) {
        /* Сам VBUS активностью хоста не является — это наша точка отсчёта. */
        if (log->events[i].type != G920_HOST_EV_VBUS) {
            *out = log->events[i].at_us;
            return true;
        }
    }
    return false;
}

bool g920_hostlog_t2_us(const g920_hostlog_t *log, uint32_t *out)
{
    bool have_config = false;
    uint32_t config_at = 0;

    if (log == NULL || out == NULL) {
        return false;
    }
    for (uint16_t i = 0; i < log->count; i++) {
        const g920_host_event_t *ev = &log->events[i];

        /*
         * Отсчёт — от **последней** настройки перед первым опросом, а не от
         * первой.
         *
         * Windows и Xbox настраивают устройство дважды, и между настройками
         * лежит целая переэнумерация. От первой выходило 301 мс, от
         * последней — 8.6; первое число описывает не задержку хоста, а его
         * привычку переподключать устройство, выбрав драйвер. Для отката по
         * тишине осмысленно второе: «сколько ждать опроса после того, как
         * хост закончил настраивать».
         *
         * Считалось от первой, а в таблицу эталонов выписывалось от
         * последней — то есть числа в документе не совпадали с тем, что
         * печатает прошивка.
         */
        if (ev->type == G920_HOST_EV_SET_CONFIG) {
            have_config = true;
            config_at = ev->at_us;
        } else if (have_config && ev->type == G920_HOST_EV_IN_POLLED) {
            *out = (ev->at_us >= config_at) ? (ev->at_us - config_at) : 0;
            return true;
        }
    }
    return false;
}

/* --- признаки ------------------------------------------------------------ */

uint16_t g920_hostlog_count_request(const g920_hostlog_t *log,
                                    uint8_t b_request)
{
    uint16_t n = 0;

    if (log == NULL) {
        return 0;
    }
    for (uint16_t i = 0; i < log->count; i++) {
        const g920_host_event_t *ev = &log->events[i];

        if ((ev->type == G920_HOST_EV_SETUP
             || ev->type == G920_HOST_EV_SET_ADDRESS
             || ev->type == G920_HOST_EV_SET_CONFIG)
            && ev->b_request == b_request) {
            n++;
        }
    }
    return n;
}

bool g920_hostlog_saw_descriptor(const g920_hostlog_t *log,
                                 uint8_t descriptor_type)
{
    if (log == NULL) {
        return false;
    }
    for (uint16_t i = 0; i < log->count; i++) {
        const g920_host_event_t *ev = &log->events[i];

        if (ev->type == G920_HOST_EV_SETUP
            && ev->b_request == G920_USB_REQ_GET_DESCRIPTOR
            && ((ev->w_value >> 8) & 0xFFu) == descriptor_type) {
            return true;
        }
    }
    return false;
}

uint16_t g920_hostlog_count_stalls(const g920_hostlog_t *log)
{
    uint16_t n = 0;

    if (log == NULL) {
        return 0;
    }
    for (uint16_t i = 0; i < log->count; i++) {
        if (log->events[i].answer == G920_HOST_ANSWER_STALL) {
            n++;
        }
    }
    return n;
}

bool g920_hostlog_saw_event(const g920_hostlog_t *log,
                            g920_host_event_type_t type)
{
    if (log == NULL) {
        return false;
    }
    for (uint16_t i = 0; i < log->count; i++) {
        if (log->events[i].type == (uint8_t)type) {
            return true;
        }
    }
    return false;
}

/* --- вывод -------------------------------------------------------------- */

const char *g920_host_event_name(g920_host_event_type_t type)
{
    switch (type) {
    case G920_HOST_EV_VBUS:
        return "VBUS";
    case G920_HOST_EV_BUS_RESET:
        return "RESET";
    case G920_HOST_EV_SETUP:
        return "SETUP";
    case G920_HOST_EV_SET_ADDRESS:
        return "SET_ADDR";
    case G920_HOST_EV_SET_CONFIG:
        return "SET_CFG";
    case G920_HOST_EV_IN_POLLED:
        return "IN_POLL";
    case G920_HOST_EV_GIP_IN:
        return "GIP_IN";
    case G920_HOST_EV_GIP_OUT:
        return "GIP_OUT";
    case G920_HOST_EV_SUSPEND:
        return "SUSPEND";
    case G920_HOST_EV_RESUME:
        return "RESUME";
    case G920_HOST_EV_NONE:
    default:
        return "?";
    }
}

static size_t put_hex8(char *buf, size_t pos, uint8_t value)
{
    buf[pos++] = HEX[(value >> 4) & 0xFu];
    buf[pos++] = HEX[value & 0xFu];
    return pos;
}

static size_t put_hex16(char *buf, size_t pos, uint16_t value)
{
    pos = put_hex8(buf, pos, (uint8_t)((value >> 8) & 0xFFu));
    return put_hex8(buf, pos, (uint8_t)(value & 0xFFu));
}

static size_t put_dec(char *buf, size_t pos, uint16_t value)
{
    char digits[5];
    int n = 0;

    do {
        digits[n++] = (char)('0' + (value % 10u));
        value = (uint16_t)(value / 10u);
    } while (value != 0u);
    while (n > 0) {
        buf[pos++] = digits[--n];
    }
    return pos;
}

static size_t put_str(char *buf, size_t pos, const char *text)
{
    for (size_t i = 0; text[i] != '\0'; i++) {
        buf[pos++] = text[i];
    }
    return pos;
}

int g920_hostlog_format(char *buf, size_t size, const g920_host_event_t *event)
{
    char tmp[G920_HOSTLOG_LINE_MAX];
    size_t pos = 0;
    int written;

    if (buf == NULL || event == NULL) {
        return -1;
    }

    written = g920_timestamp_format(tmp, sizeof(tmp), event->at_us);
    if (written < 0) {
        return -1;
    }
    pos = (size_t)written;
    tmp[pos++] = ' ';
    pos = put_str(tmp, pos, g920_host_event_name(
                                (g920_host_event_type_t)event->type));

    switch (event->type) {
    case G920_HOST_EV_SETUP:
    case G920_HOST_EV_SET_ADDRESS:
    case G920_HOST_EV_SET_CONFIG:
        tmp[pos++] = ' ';
        pos = put_hex8(tmp, pos, event->bm_request_type);
        tmp[pos++] = ' ';
        pos = put_hex8(tmp, pos, event->b_request);
        tmp[pos++] = ' ';
        pos = put_hex16(tmp, pos, event->w_value);
        tmp[pos++] = ' ';
        pos = put_hex16(tmp, pos, event->w_index);
        pos = put_str(tmp, pos, " len ");
        pos = put_dec(tmp, pos, event->w_length);
        if (event->answer == G920_HOST_ANSWER_STALL) {
            pos = put_str(tmp, pos, " STALL");
        }
        break;
    case G920_HOST_EV_GIP_IN:
    case G920_HOST_EV_GIP_OUT:
        pos = put_str(tmp, pos, " type ");
        pos = put_hex8(tmp, pos, event->b_request);
        pos = put_str(tmp, pos, " len ");
        pos = put_dec(tmp, pos, event->w_length);
        break;
    default:
        break;
    }

    if (size < pos + 1u) {
        return -1;
    }
    for (size_t i = 0; i < pos; i++) {
        buf[i] = tmp[i];
    }
    buf[pos] = '\0';
    return (int)pos;
}

void g920_hostlog_dump(const g920_hostlog_t *log, const char *tag)
{
    char line[G920_HOSTLOG_LINE_MAX];

    if (log == NULL) {
        return;
    }

    G920_LOGI(tag, "host trace: %u events%s", (unsigned)log->count,
              log->origin_implicit ? ", origin implicit" : "");
    /* Через аксессор, а не в структуру напрямую: иначе `g920_hostlog_at`
     * остаётся без единого вызывающего вне тестов, а обход журнала
     * оказывается написан дважды. */
    for (uint16_t i = 0; i < log->count; i++) {
        const g920_host_event_t *event = g920_hostlog_at(log, i);

        if (event == NULL
            || g920_hostlog_format(line, sizeof(line), event) < 0) {
            continue;
        }
        G920_LOGI(tag, "%s", line);
    }
    if (log->dropped != 0) {
        G920_LOGW(tag, "host trace: %u events dropped, buffer full",
                  (unsigned)log->dropped);
    }
}
