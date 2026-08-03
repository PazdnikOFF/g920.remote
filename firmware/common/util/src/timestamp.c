#include "g920/timestamp.h"

#if defined(ESP_PLATFORM)
#include "esp_timer.h"
#else
#include <time.h>
#endif

#define US_PER_SEC 1000000ull

uint64_t g920_timestamp_us(void)
{
#if defined(ESP_PLATFORM)
    /* esp_timer_get_time() — микросекунды от старта, 64 бита, не
     * переполняется за срок жизни устройства. */
    return (uint64_t)esp_timer_get_time();
#else
    struct timespec ts;

    /* На хосте это нужно только тестам и инструментам разбора трасс. */
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0;
    }
    return (uint64_t)ts.tv_sec * US_PER_SEC + (uint64_t)(ts.tv_nsec / 1000);
#endif
}

int g920_timestamp_format(char *buf, size_t size, uint64_t us)
{
    char tmp[G920_TIMESTAMP_STR_MAX];
    char digits[20];
    uint64_t seconds = us / US_PER_SEC;
    uint32_t fraction = (uint32_t)(us % US_PER_SEC);
    size_t len = 0;
    int ndigits = 0;

    if (buf == NULL) {
        return -1;
    }

    do {
        digits[ndigits++] = (char)('0' + (seconds % 10u));
        seconds /= 10u;
    } while (seconds != 0u);
    while (ndigits > 0) {
        tmp[len++] = digits[--ndigits];
    }

    tmp[len++] = '.';
    for (int i = 5; i >= 0; i--) {
        tmp[len + (size_t)i] = (char)('0' + (fraction % 10u));
        fraction /= 10u;
    }
    len += 6;

    if (size < len + 1u) {
        return -1;
    }
    for (size_t i = 0; i < len; i++) {
        buf[i] = tmp[i];
    }
    buf[len] = '\0';
    return (int)len;
}
