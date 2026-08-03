#include "g920/gip_reasm.h"

#include <string.h>

void g920_gip_reasm_init(g920_gip_reasm_t *reasm, uint8_t *buffer,
                         size_t capacity)
{
    if (reasm == NULL) {
        return;
    }
    memset(reasm, 0, sizeof(*reasm));
    reasm->buffer = buffer;
    reasm->capacity = capacity;
}

void g920_gip_reasm_reset(g920_gip_reasm_t *reasm)
{
    if (reasm == NULL) {
        return;
    }
    reasm->active = false;
    reasm->total = 0;
    reasm->received = 0;
}

static g920_gip_reasm_status_t store(g920_gip_reasm_t *reasm,
                                     const uint8_t *payload, size_t length)
{
    if (length > reasm->capacity - reasm->received) {
        return G920_GIP_REASM_OVERFLOW;
    }
    if (length != 0) {
        memcpy(reasm->buffer + reasm->received, payload, length);
    }
    reasm->received += (uint32_t)length;
    return G920_GIP_REASM_MORE;
}

g920_gip_reasm_status_t g920_gip_reasm_push(g920_gip_reasm_t *reasm,
                                            const g920_gip_header_t *header,
                                            const uint8_t *payload,
                                            size_t length)
{
    g920_gip_reasm_status_t status;

    if (reasm == NULL || header == NULL || reasm->buffer == NULL) {
        return G920_GIP_REASM_BAD_ARG;
    }
    if (payload == NULL && length != 0) {
        return G920_GIP_REASM_BAD_ARG;
    }
    if (length != header->payload_length) {
        /* Заголовок и то, что реально пришло, обязаны сходиться. */
        return G920_GIP_REASM_BAD_ARG;
    }

    /* --- одиночное сообщение ------------------------------------------- */
    if (!g920_gip_is_fragment(header)) {
        if (length > reasm->capacity) {
            return G920_GIP_REASM_OVERFLOW;
        }
        g920_gip_reasm_reset(reasm);
        if (length != 0) {
            memcpy(reasm->buffer, payload, length);
        }
        reasm->message_type = header->message_type;
        reasm->flags = header->flags;
        reasm->sequence = header->sequence;
        reasm->total = (uint32_t)length;
        reasm->received = (uint32_t)length;
        return G920_GIP_REASM_SINGLE;
    }

    /* --- первый фрагмент ------------------------------------------------ */
    if (g920_gip_is_initial_fragment(header)) {
        if (header->tlo > reasm->capacity) {
            return G920_GIP_REASM_OVERFLOW;
        }
        if (length > header->tlo) {
            return G920_GIP_REASM_OVERFLOW;
        }
        /* Новый первый фрагмент всегда начинает сборку заново, даже если
         * предыдущая не закончилась: отправитель начал сначала. */
        g920_gip_reasm_reset(reasm);
        reasm->active = true;
        reasm->message_type = header->message_type;
        reasm->flags = header->flags;
        reasm->sequence = header->sequence;
        reasm->total = header->tlo;

        status = store(reasm, payload, length);
        if (status != G920_GIP_REASM_MORE) {
            return status;
        }
        if (reasm->received == reasm->total) {
            reasm->active = false;
            return G920_GIP_REASM_DONE;
        }
        return G920_GIP_REASM_MORE;
    }

    /* --- продолжение ---------------------------------------------------- */
    if (!reasm->active) {
        /* Нулевой фрагмент после собранного сообщения — это completion,
         * а не потеря: отправитель закрывает передачу. */
        if (length == 0 && reasm->total != 0
            && reasm->received == reasm->total
            && header->sequence == reasm->sequence) {
            return G920_GIP_REASM_COMPLETION;
        }
        return G920_GIP_REASM_ORPHAN;
    }
    if (header->sequence != reasm->sequence) {
        return G920_GIP_REASM_SEQUENCE_MISMATCH;
    }
    /* У не-первого фрагмента поле TLO — смещение, и оно обязано совпасть с
     * тем, сколько мы уже приняли подряд. */
    if (header->tlo != reasm->received) {
        return G920_GIP_REASM_OUT_OF_ORDER;
    }
    if (length > reasm->total - reasm->received) {
        return G920_GIP_REASM_OVERFLOW;
    }

    status = store(reasm, payload, length);
    if (status != G920_GIP_REASM_MORE) {
        return status;
    }
    if (reasm->received == reasm->total) {
        /* Отдельного бита «последний фрагмент» в GIP нет — конец виден
         * только по добранной длине. */
        reasm->active = false;
        return G920_GIP_REASM_DONE;
    }
    return G920_GIP_REASM_MORE;
}

size_t g920_gip_reasm_length(const g920_gip_reasm_t *reasm)
{
    return (reasm != NULL) ? (size_t)reasm->received : 0;
}

bool g920_gip_reasm_in_progress(const g920_gip_reasm_t *reasm)
{
    return (reasm != NULL) && reasm->active;
}

uint16_t g920_gip_reasm_remaining(const g920_gip_reasm_t *reasm)
{
    uint32_t left;

    if (reasm == NULL || reasm->received >= reasm->total) {
        return 0;
    }
    left = reasm->total - reasm->received;
    return (left > 0xFFFFu) ? 0xFFFFu : (uint16_t)left;
}

const char *g920_gip_reasm_status_name(g920_gip_reasm_status_t status)
{
    switch (status) {
    case G920_GIP_REASM_SINGLE:
        return "SINGLE";
    case G920_GIP_REASM_MORE:
        return "MORE";
    case G920_GIP_REASM_DONE:
        return "DONE";
    case G920_GIP_REASM_COMPLETION:
        return "COMPLETION";
    case G920_GIP_REASM_ORPHAN:
        return "ORPHAN";
    case G920_GIP_REASM_OUT_OF_ORDER:
        return "OUT_OF_ORDER";
    case G920_GIP_REASM_SEQUENCE_MISMATCH:
        return "SEQUENCE_MISMATCH";
    case G920_GIP_REASM_OVERFLOW:
        return "OVERFLOW";
    case G920_GIP_REASM_BAD_ARG:
        return "BAD_ARG";
    default:
        return "?";
    }
}
