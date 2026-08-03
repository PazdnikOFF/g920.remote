#include "g920/peering.h"

#include <string.h>

#include "g920/hot.h"
#include "g920/version.h"

void g920_peering_init(g920_peering_t *p, uint32_t release_ms)
{
    if (p == NULL) {
        return;
    }
    memset(p, 0, sizeof(*p));
    p->release_ms = (release_ms != 0) ? release_ms : G920_PEER_RELEASE_MS;
}

void g920_peering_preset(g920_peering_t *p, const uint8_t *mac)
{
    if (p == NULL || mac == NULL) {
        return;
    }
    memcpy(p->peer, mac, G920_PEER_MAC_LEN);
    p->has_peer = true;
    /*
     * Сохранённый MAC — это не подтверждение, что модуль жив. Пока он не
     * сказал ни слова, считаем его молчащим: иначе он занимал бы место и не
     * отпускался, а живой сосед не смог бы к нам подключиться.
     */
    p->heard = false;
    p->last_rx_us = 0;
}

void g920_peering_release(g920_peering_t *p)
{
    if (p == NULL) {
        return;
    }
    p->has_peer = false;
    p->heard = false;
    p->last_rx_us = 0;
    memset(p->peer, 0, sizeof(p->peer));
}

bool g920_peering_quiet_for(const g920_peering_t *p, uint64_t now_us,
                            uint32_t ms)
{
    if (p == NULL || !p->has_peer) {
        return true;
    }
    if (!p->heard) {
        /* Поднят из NVS и молчит: место занимать не должен. */
        return true;
    }
    if (now_us < p->last_rx_us) {
        return false; /* часы дёрнулись назад — не повод рвать связь */
    }
    return (now_us - p->last_rx_us) >= (uint64_t)ms * 1000u;
}

bool g920_peering_silent(const g920_peering_t *p, uint64_t now_us)
{
    return g920_peering_quiet_for(p, now_us, (p != NULL) ? p->release_ms : 0);
}

static bool is_peer(const g920_peering_t *p, const uint8_t *src)
{
    return p->has_peer && memcmp(p->peer, src, G920_PEER_MAC_LEN) == 0;
}

static void adopt(g920_peering_t *p, const uint8_t *src, uint64_t now_us)
{
    memcpy(p->peer, src, G920_PEER_MAC_LEN);
    p->has_peer = true;
    p->heard = true;
    p->last_rx_us = now_us;
    p->adoptions++;
}

g920_peering_action_t g920_peering_on_discover(g920_peering_t *p,
                                               const uint8_t *src,
                                               uint8_t their_role,
                                               uint8_t own_role,
                                               uint64_t now_us)
{
    if (p == NULL || src == NULL) {
        return G920_PEERING_IGNORE;
    }
    /*
     * Роль — первый фильтр. Два TX в одной комнате находят друг друга по
     * первому же вызову, и без этого линк «поднимается» с зелёным
     * индикатором и без единого кадра данных.
     */
    if (their_role == (uint8_t)G920_ROLE_UNKNOWN || their_role == own_role) {
        p->rejected_role++;
        return G920_PEERING_IGNORE;
    }

    if (is_peer(p, src)) {
        p->heard = true;
        p->last_rx_us = now_us;
        return G920_PEERING_REPLY;
    }

    if (!p->has_peer || g920_peering_silent(p, now_us)) {
        /* Своего нет или он давно молчит — принимаем нового. */
        adopt(p, src, now_us);
        return G920_PEERING_ADOPTED;
    }

    /*
     * Свой пир жив, а зовёт кто-то другой. **Молчим.** Ответить значило бы
     * сказать чужому модулю «мы пара», не будучи ею: он бы зажёг зелёный и
     * гнал кадры, которые мы всё равно отбрасываем по MAC.
     */
    p->rejected_busy++;
    return G920_PEERING_IGNORE;
}

G920_HOT g920_peering_action_t g920_peering_on_frame(g920_peering_t *p,
                                                     const uint8_t *src,
                                                     uint64_t now_us)
{
    if (p == NULL || src == NULL) {
        return G920_PEERING_IGNORE;
    }
    if (!is_peer(p, src)) {
        /* Считаем: молчаливый отказ здесь — это ровно тот симптом, за
         * который веха блокировалась дважды. */
        p->rejected_frames++;
        return G920_PEERING_IGNORE;
    }
    p->heard = true;
    p->last_rx_us = now_us;
    return G920_PEERING_KNOWN;
}

bool g920_peering_has_peer(const g920_peering_t *p)
{
    return (p != NULL) && p->has_peer;
}

const uint8_t *g920_peering_peer(const g920_peering_t *p)
{
    return (p != NULL && p->has_peer) ? p->peer : NULL;
}

uint32_t g920_peering_rejected(const g920_peering_t *p)
{
    return (p != NULL)
               ? p->rejected_role + p->rejected_busy + p->rejected_frames
               : 0;
}

uint32_t g920_peering_adoptions(const g920_peering_t *p)
{
    return (p != NULL) ? p->adoptions : 0;
}

const char *g920_peering_action_name(g920_peering_action_t action)
{
    switch (action) {
    case G920_PEERING_IGNORE:
        return "IGNORE";
    case G920_PEERING_KNOWN:
        return "KNOWN";
    case G920_PEERING_ADOPTED:
        return "ADOPTED";
    case G920_PEERING_REPLY:
        return "REPLY";
    default:
        return "?";
    }
}
