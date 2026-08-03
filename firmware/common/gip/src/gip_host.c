/*
 * Последовательность хоста: Hello → запрос метаданных → Start.
 * Разбор решений — в g920/gip_host.h.
 */

#include <string.h>

#include "g920/gip_host.h"

/* --- сборка исходящих ------------------------------------------------------ */

static int build_command_flags(g920_gip_host_packet_t *out,
                               uint8_t message_type, uint8_t flags,
                               uint8_t sequence, const uint8_t *payload,
                               uint8_t payload_length, const char *what)
{
    g920_gip_header_t header;
    int written;

    memset(&header, 0, sizeof(header));
    header.message_type = message_type;
    header.flags = flags;
    header.sequence = sequence;
    header.payload_length = payload_length;
    header.length_bytes = 1;
    header.tlo_bytes = 0;

    written = g920_gip_header_build(out->data, sizeof(out->data), &header);
    if (written < 0 || (size_t)written + payload_length > sizeof(out->data)) {
        return -1;
    }
    if (payload_length > 0 && payload != NULL) {
        memcpy(out->data + written, payload, payload_length);
    }

    out->length = (uint8_t)((size_t)written + payload_length);
    out->what = what;
    return 0;
}

/*
 * Системные команды: флаги 0x20 — одиночный пакет первичному устройству,
 * подтверждения не требует. Так предписано таблицам 4-26 и 4-31.
 */
static int build_command(g920_gip_host_packet_t *out, uint8_t message_type,
                         uint8_t sequence, const uint8_t *payload,
                         uint8_t payload_length, const char *what)
{
    return build_command_flags(out, message_type, G920_GIP_FLAG_SYSTEM,
                               sequence, payload, payload_length, what);
}

/*
 * Sequence ID команд: счётчик, перескакивающий ноль. Спека (таблица 4-31)
 * оставляет 0x00 за особыми случаями, и занять его обычной командой значит
 * сказать устройству не то, что имелось в виду.
 */
static uint8_t next_sequence(g920_gip_host_t *host)
{
    host->command_sequence++;
    if (host->command_sequence == 0) {
        host->command_sequence = 1;
    }
    return host->command_sequence;
}

/*
 * Запрос начальных отчётов, H001861 § «Initial Reports Request».
 *
 * Флаги здесь **0x00, а не 0x20**: это не системное сообщение, а запрос к
 * прикладной части руля. Длина нагрузки всегда 3, даже когда значащий байт
 * один — так предписано спекой явно, «unused bytes will always be 0x00».
 */
static int build_initial_reports_request(g920_gip_host_t *host,
                                         g920_gip_host_packet_t *out)
{
    const uint8_t payload[3] = { G920_GIP_WHEEL_INITIAL_REPORTS, 0x00, 0x00 };

    if (build_command_flags(out, G920_GIP_MSG_WHEEL_REQUEST, 0x00,
                            next_sequence(host), payload, sizeof(payload),
                            "initial reports request")
        < 0) {
        return 0;
    }
    host->initial_reports_asked = true;
    return 1;
}

static int build_metadata_request(g920_gip_host_t *host,
                                  g920_gip_host_packet_t *out, uint64_t now_ms)
{
    if (build_command(out, G920_GIP_MSG_METADATA,
                      G920_GIP_METADATA_REQUEST_SEQ, NULL, 0,
                      "metadata request")
        < 0) {
        return 0;
    }
    host->metadata_requests++;
    host->metadata_asked_ms = now_ms;
    host->state = G920_GIP_HOST_METADATA;
    return 1;
}

static int build_start(g920_gip_host_t *host, g920_gip_host_packet_t *out)
{
    const uint8_t state = G920_GIP_STATE_START;

    if (build_command(out, G920_GIP_MSG_SET_STATE, next_sequence(host), &state,
                      1, "set device state: start")
        < 0) {
        return 0;
    }
    host->state = G920_GIP_HOST_ACTIVE;
    return 1;
}

/* --- приём ---------------------------------------------------------------- */

void g920_gip_host_init(g920_gip_host_t *host, uint8_t *buffer, size_t capacity)
{
    if (host == NULL) {
        return;
    }
    memset(host, 0, sizeof(*host));
    g920_gip_reasm_init(&host->reasm, buffer, capacity);
}

/*
 * Подтверждение принятого сообщения, если оно об этом просило.
 *
 * Смещение — сколько байт сообщения принято **подряд**: для одиночного это
 * его длина, для фрагмента — то, что уже собрано. Остаток буфера берётся у
 * сборщика: он один знает, сколько ещё ждать до объявленной длины.
 */
static int maybe_ack(g920_gip_host_t *host, const g920_gip_header_t *header,
                     bool fragmented, g920_gip_host_packet_t *out)
{
    uint32_t offset;
    uint16_t remaining;
    int written;

    if (!g920_gip_wants_ack(header)) {
        return 0;
    }

    if (fragmented) {
        offset = (uint32_t)g920_gip_reasm_length(&host->reasm);
        remaining = g920_gip_reasm_remaining(&host->reasm);
    } else {
        offset = header->payload_length;
        remaining = 0;
    }

    written = g920_gip_build_ack(out->data, sizeof(out->data), header, offset,
                                 remaining);
    if (written <= 0) {
        return 0;
    }
    out->length = (uint8_t)written;
    out->what = "ack";
    return 1;
}

int g920_gip_host_on_packet(g920_gip_host_t *host, const uint8_t *data,
                            size_t length, uint64_t now_ms,
                            g920_gip_host_packet_t *out, int out_max)
{
    g920_gip_header_t header;
    const uint8_t *payload;
    size_t payload_length;
    bool fragmented;
    int count = 0;

    if (host == NULL || data == NULL || out == NULL
        || out_max < G920_GIP_HOST_OUT_MAX) {
        return -1;
    }
    if (g920_gip_header_parse(&header, data, length) != G920_GIP_OK) {
        return 0;
    }

    payload = data + header.header_length;
    payload_length = (length > header.header_length)
                         ? length - header.header_length
                         : 0;
    fragmented = g920_gip_is_fragment(&header);

    switch (g920_gip_message_number(&header)) {
    case G920_GIP_MSG_HELLO:
        host->hellos++;
        /*
         * Hello в любом состоянии значит одно: устройство считает себя
         * только что появившимся. Спорить с ним нечем — своё состояние
         * сбрасываем и начинаем заново. Так же поступает и линуксовый
         * драйвер xone, встретив announce посреди работы.
         */
        g920_gip_reasm_reset(&host->reasm);
        host->metadata_ready = false;
        host->metadata_length = 0;
        host->metadata_requests = 0;
        host->state = G920_GIP_HOST_ARRIVAL;
        count += maybe_ack(host, &header, false, &out[count]);
        count += build_metadata_request(host, &out[count], now_ms);
        break;

    case G920_GIP_MSG_METADATA: {
        g920_gip_reasm_status_t status = g920_gip_reasm_push(
            &host->reasm, &header, payload, payload_length);

        /* Подтверждение идёт после сборки: в нём то самое смещение, которое
         * сборщик только что досчитал. */
        count += maybe_ack(host, &header, fragmented, &out[count]);

        if (status == G920_GIP_REASM_SINGLE || status == G920_GIP_REASM_DONE) {
            host->metadata_ready = true;
            host->metadata_length = g920_gip_reasm_length(&host->reasm);
            host->metadata_done_ms = now_ms;
        }

        /*
         * Start — только когда транзакция закрыта.
         *
         * Метаданные одним пакетом закрывать нечем, там Start сразу. А
         * фрагментированные закрываются нулевым completion-фрагментом, и
         * ранний Start живой руль отвергает: он возвращается в Arrival и
         * начинает знакомство заново (проверено 02.08.2026).
         */
        if (host->metadata_ready && host->state == G920_GIP_HOST_METADATA
            && (status == G920_GIP_REASM_SINGLE
                || status == G920_GIP_REASM_COMPLETION)
            && count < out_max) {
            count += build_start(host, &out[count]);
        }
        break;
    }

    default:
        /*
         * Всё остальное — ввод, статус, security. Толковать это здесь не
         * дело модуля (И1): пакет уже разобран и залогирован вызывающим.
         * Подтверждаем, если попросили, и считаем ввод для отчёта.
         */
        if (host->state == G920_GIP_HOST_ACTIVE) {
            host->inputs++;
        }
        count += maybe_ack(host, &header, fragmented, &out[count]);
        break;
    }

    return count;
}

int g920_gip_host_tick(g920_gip_host_t *host, uint64_t now_ms,
                       g920_gip_host_packet_t *out, int out_max)
{
    if (host == NULL || out == NULL || out_max < 1) {
        return -1;
    }
    if (host->state == G920_GIP_HOST_ACTIVE) {
        /* Руль запущен — теперь можно спросить его о себе. Один раз: спека
         * не обещает повторного ответа, а долбить запросами работающее
         * устройство значит мерить не его, а свою настойчивость. */
        if (!host->initial_reports_asked) {
            return build_initial_reports_request(host, out);
        }
        return 0;
    }
    if (host->state != G920_GIP_HOST_METADATA) {
        return 0;
    }
    if (host->metadata_ready) {
        /*
         * Метаданные собраны, но completion-фрагмент не пришёл. Ждать его
         * вечно нельзя: у нас уже есть всё, ради чего затевался обмен, а
         * устройство без Start так и останется в Idle.
         */
        if (now_ms - host->metadata_done_ms >= G920_GIP_COMPLETION_WAIT_MS) {
            return build_start(host, out);
        }
        return 0;
    }
    if (host->metadata_requests >= G920_GIP_METADATA_RETRY_MAX) {
        return 0;
    }
    if (now_ms - host->metadata_asked_ms < G920_GIP_METADATA_RETRY_MS) {
        return 0;
    }
    /* Сборка могла остановиться на половине потерянного сообщения — новый
     * ответ придёт с начала, и старый хвост в буфере стал бы мусором. */
    g920_gip_reasm_reset(&host->reasm);
    return build_metadata_request(host, out, now_ms);
}

const uint8_t *g920_gip_host_metadata(const g920_gip_host_t *host,
                                      size_t *length)
{
    if (host == NULL || !host->metadata_ready) {
        if (length != NULL) {
            *length = 0;
        }
        return NULL;
    }
    if (length != NULL) {
        *length = host->metadata_length;
    }
    return host->reasm.buffer;
}

const char *g920_gip_host_state_name(g920_gip_host_state_t state)
{
    switch (state) {
    case G920_GIP_HOST_ARRIVAL:
        return "arrival";
    case G920_GIP_HOST_METADATA:
        return "metadata";
    case G920_GIP_HOST_ACTIVE:
        return "active";
    default:
        return "?";
    }
}
