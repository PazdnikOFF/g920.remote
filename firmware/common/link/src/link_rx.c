#include "g920/link_rx.h"

#include <string.h>

#include "g920/hot.h"
#include "g920/version.h"

void g920_link_rx_set_epoch(g920_link_rx_t *state, uint8_t epoch)
{
    if (state != NULL) {
        state->own_epoch = epoch;
    }
}

void g920_link_rx_init(g920_link_rx_t *state, uint8_t own_role,
                       uint32_t release_ms)
{
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
    g920_peering_init(&state->peering, release_ms);
    g920_peer_rx_init(&state->rx);
    state->own_role = own_role;
}

void g920_link_rx_preset_peer(g920_link_rx_t *state, const uint8_t *mac)
{
    if (state == NULL) {
        return;
    }
    g920_peering_preset(&state->peering, mac);
    g920_peer_rx_init(&state->rx);
}

void g920_link_rx_release_peer(g920_link_rx_t *state)
{
    if (state == NULL) {
        return;
    }
    g920_peering_release(&state->peering);
    g920_peer_rx_init(&state->rx);
}

bool g920_link_rx_peer_expired(const g920_link_rx_t *state, uint64_t now_us)
{
    if (state == NULL || !state->peering.has_peer) {
        return false;
    }
    /*
     * Пир из NVS, который ещё не сказал ни слова, не «просрочен» — его
     * просто некому отпускать: адрес взят с прошлой загрузки, а не выдан
     * живым собеседником. Место он при этом не занимает, см.
     * g920_peering_on_discover.
     */
    if (!state->peering.heard) {
        return false;
    }
    return g920_peering_silent(&state->peering, now_us);
}

bool g920_link_rx_has_peer(const g920_link_rx_t *state)
{
    return (state != NULL) && state->peering.has_peer;
}

bool g920_link_rx_peer_alive(const g920_link_rx_t *state)
{
    return (state != NULL) && state->peering.has_peer && state->peering.heard;
}

static void make_reply(g920_link_rx_outcome_t *out, uint8_t type,
                       uint16_t sequence, uint8_t epoch, uint16_t length)
{
    out->reply.version = G920_LINK_PROTO_VERSION;
    out->reply.type = type;
    out->reply.sequence = sequence;
    out->reply.flags = 0;
    out->reply.epoch = epoch;
    out->reply.length = length;
    out->reply.payload = (length != 0) ? out->reply_payload : NULL;
    out->send_reply = true;
}

G920_HOT void g920_link_rx_on_frame(g920_link_rx_t *state,
                                    const g920_frame_t *frame,
                                    const uint8_t *src, uint64_t now_us,
                                    g920_link_rx_outcome_t *out)
{
    g920_peering_action_t action;
    uint8_t previous[G920_PEER_MAC_LEN];
    bool had_peer;

    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->verdict = G920_RX_FOREIGN;

    if (state == NULL || frame == NULL || src == NULL) {
        return;
    }

    had_peer = state->peering.has_peer;
    if (had_peer) {
        memcpy(previous, state->peering.peer, G920_PEER_MAC_LEN);
    }

    if (frame->type == G920_FRAME_DISCOVER) {
        /* Нагрузка вызова: вид (запрос/ответ) и роль. Без роли не отличить
         * TX от TX, и два передатчика спарятся друг с другом. */
        if (frame->length != 2) {
            return;
        }
        action = g920_peering_on_discover(&state->peering, src,
                                          frame->payload[1], state->own_role,
                                          now_us);
        if (action == G920_PEERING_IGNORE) {
            /*
             * Чужой вызов при живом пире. Молчим: ответ означал бы «мы
             * пара», и на том конце зажёгся бы зелёный индикатор при линке,
             * все кадры которого мы всё равно отбрасываем.
             */
            return;
        }
        if (action == G920_PEERING_ADOPTED) {
            out->adopt_peer = true;
            /* Новый собеседник — чистый лист обеих дисциплин: его эпоха не
             * обязана быть больше прежней. */
            g920_peer_rx_init(&state->rx);
            /* И очередь повторов: она адресована прежнему. */
            out->reset_reliable = true;
            if (had_peer
                && memcmp(previous, src, G920_PEER_MAC_LEN) != 0) {
                /* Прежний адрес — из таблицы радио: она конечна, а после
                 * усыновления удалить его будет уже нечем, он затёрт. */
                out->drop_peer = true;
                memcpy(out->dropped, previous, G920_PEER_MAC_LEN);
            }
        }
        /* На ответ не отвечаем: обмен из двух шагов, иначе он не кончится. */
        if (frame->payload[0] == G920_LINK_DISCOVER_REQUEST) {
            out->reply_payload[0] = G920_LINK_DISCOVER_REPLY;
            out->reply_payload[1] = state->own_role;
            make_reply(out, (uint8_t)G920_FRAME_DISCOVER, frame->sequence,
                       state->own_epoch, 2);
        }
        return;
    }

    action = g920_peering_on_frame(&state->peering, src, now_us);
    if (action != G920_PEERING_KNOWN) {
        /* Не от нашего собеседника. Счётчик отказов ведёт политика пиринга:
         * молчаливый отказ здесь — тот самый симптом «руль молчит, всё
         * зелёное». */
        return;
    }

    if (g920_frame_delivery((g920_frame_type_t)frame->type)
        == G920_DELIVERY_SERVICE) {
        /*
         * ACK. В сессию собеседника он не входит: его номер и эпоха — из
         * **нашего** пространства, это наш же кадр, вернувшийся отметкой о
         * доставке. Прогонять его через трекер сессии значит травить его
         * чужими номерами, а подтверждать — устроить обмен подтверждениями
         * без конца.
         */
        if (frame->type == G920_FRAME_ACK) {
            out->close_reliable = true;
            out->ack_sequence = frame->sequence;
            out->ack_epoch = frame->epoch;
        }
        return;
    }

    out->verdict = g920_peer_rx_accept(&state->rx, frame->epoch,
                                       (g920_frame_type_t)frame->type,
                                       frame->sequence);

    if (g920_frame_delivery((g920_frame_type_t)frame->type)
            == G920_DELIVERY_RELIABLE
        && g920_rx_should_ack(out->verdict)) {
        /*
         * Подтверждаем и дубликат: раз он пришёл повторно, значит
         * потерялось как раз подтверждение. Но не чужую сессию — сказать
         * «доставлено» про неотданное наверх нельзя.
         *
         * **Эпоха в подтверждении — подтверждаемого кадра, а не своя.**
         * Правило живёт здесь, вместе с проверкой: пока номер и эпоху
         * подставляла прошивка, один перепутанный аргумент убивал надёжную
         * доставку при зелёных тестах.
         */
        make_reply(out, (uint8_t)G920_FRAME_ACK, frame->sequence,
                   frame->epoch, 0);
    }
    out->deliver = g920_rx_should_deliver(out->verdict);
}
