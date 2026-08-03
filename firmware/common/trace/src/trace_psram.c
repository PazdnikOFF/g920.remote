/*
 * Хранилище трассы в PSRAM — только на плате.
 *
 * Вынесено отдельным файлом, потому что сам `trace.c` обязан собираться и на
 * хосте: логика кольца проверяется тестами, а тесты про PSRAM ничего не
 * знают и знать не должны.
 */

#include "g920/trace.h"

#if defined(ESP_PLATFORM)

#include "esp_heap_caps.h"

#include "g920/log.h"

#define TAG "trace"

bool g920_trace_init_psram(g920_trace_t *trace, size_t size,
                           g920_trace_policy_t policy)
{
    void *storage;

    if (trace == NULL || size == 0) {
        return false;
    }

    storage = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (storage == NULL) {
        /*
         * Тихо взять внутреннюю память вместо PSRAM было бы худшим из
         * решений: буфер вышел бы в сотни раз меньше запрошенного, трасса
         * начала бы затираться на первых же секундах, и выглядело бы это
         * как «радио теряет кадры». Отказ громкий, решение — за прошивкой.
         */
        G920_LOGE(TAG, "no psram for %u B trace buffer", (unsigned)size);
        g920_trace_init(trace, NULL, 0, policy);
        return false;
    }

    g920_trace_init(trace, storage, size, policy);
    G920_LOGI(TAG, "trace buffer %u B in psram", (unsigned)size);
    return true;
}

/*
 * Кольцо лога — статическое, и это не лень.
 *
 * Приёмник лога хранит указатель на контекст и живёт до конца работы, то
 * есть буфер обязан пережить вызывающего. Отдать его на откуп прошивке
 * значило бы завести в каждой из двух по своей копии одного и того же
 * объекта и по своему шансу отдать приёмнику указатель на стек.
 */
static g920_trace_t s_log_ring;
static bool s_log_ring_ready;

bool g920_trace_log_to_psram(size_t size)
{
    if (s_log_ring_ready) {
        return true;
    }
    if (!g920_trace_init_psram(&s_log_ring, size, G920_TRACE_KEEP_NEWEST)) {
        /*
         * Лог остаётся в UART. Это хуже по времени, но неизмеримо лучше,
         * чем прошивка, которая молча перестала говорить: в такой не
         * отличить «всё хорошо» от «приёмник не завёлся».
         */
        G920_LOGE(TAG, "log stays on uart: no psram for %u B ring",
                  (unsigned)size);
        return false;
    }

    s_log_ring_ready = true;
    /* Строка о переезде печатается **до** подмены приёмника — иначе она
     * первой же и уедет в кольцо, и в UART не останется даже следа. */
    G920_LOGI(TAG, "log moves to a %u B psram ring, uart goes quiet",
              (unsigned)size);
    g920_log_set_sink(g920_trace_log_sink, &s_log_ring);
    return true;
}

g920_trace_t *g920_trace_log_ring(void)
{
    return s_log_ring_ready ? &s_log_ring : NULL;
}

#endif /* ESP_PLATFORM */
