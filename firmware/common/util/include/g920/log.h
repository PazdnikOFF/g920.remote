/*
 * Логирование — общий код обеих прошивок.
 *
 * Почему свой лог, а не ESP_LOGx напрямую:
 *
 *  - Вывод обязан идти в UART и только в UART. USB на обеих платах занят
 *    ролью (Host на TX, Device на RX), и на RX это физически разъём QWIIC
 *    (GPIO43/44 = UART0). Консоль прибита к UART0 в sdkconfig.defaults обеих
 *    прошивок, вторичная консоль на USB-Serial-JTAG отключена.
 *  - Метка времени нужна микросекундная. У ESP_LOGx она миллисекундная, а из
 *    миллисекунд не собрать ни T1/T2 в M2, ни джиттер в M3.
 *  - Логи TX и RX предстоит сводить одним разборщиком, поэтому строка должна
 *    быть одного формата на обеих сторонах и нести роль.
 *  - Тот же поток строк позже поедет на TF-карту и по радио — отсюда
 *    подменяемый приёмник вместо жёсткой печати в stdout.
 *
 * Формат строки:
 *
 *   I 12.345678 RX gip: hello, 20 bytes
 *   ^ ^         ^  ^     ^
 *   | |         |  |     сообщение
 *   | |         |  тег подсистемы
 *   | |         роль модуля (см. g920/version.h)
 *   | метка времени от старта, микросекунды
 *   уровень: E W I D
 */

#ifndef G920_LOG_H
#define G920_LOG_H

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    G920_LOG_ERROR = 0,
    G920_LOG_WARN = 1,
    G920_LOG_INFO = 2,
    G920_LOG_DEBUG = 3
} g920_log_level_t;

/*
 * Максимум символов в одном сообщении без префикса и '\n'. Длиннее —
 * обрезается, и обрезка помечается символом '~' в конце: молча потерянный
 * хвост в трассе хуже, чем видимая метка обрезки.
 */
#define G920_LOG_MSG_MAX 224

/* Тег длиннее обрезается: префикс должен быть фиксированной ширины, иначе
 * колонки в трассе разъедутся. */
#define G920_LOG_TAG_MAX 16

/* 'I' + ' ' + метка (21) + ' ' + роль (4) + ' ' + тег (16) + ": " + '\0'. */
#define G920_LOG_PREFIX_MAX 48

/*
 * Приёмник готовой строки. text не заканчивается '\0'-гарантией по длине —
 * длина передаётся отдельно. Вызывается по одной строке целиком, вместе с
 * завершающим '\n': приёмнику не нужно склеивать куски.
 */
typedef void (*g920_log_sink_t)(void *ctx, const char *text, size_t len);

/*
 * Подменить приёмник. sink == NULL возвращает вывод по умолчанию — в консоль,
 * то есть в UART0.
 */
void g920_log_set_sink(g920_log_sink_t sink, void *ctx);

void g920_log_set_level(g920_log_level_t level);
g920_log_level_t g920_log_get_level(void);

/* Односимвольная метка уровня: 'E', 'W', 'I', 'D'; для неизвестного — '?'. */
char g920_log_level_char(g920_log_level_t level);

/*
 * Собирает префикс "<уровень> <метка> <роль> <тег>: " в буфер.
 * Возвращает длину без '\0' либо -1, если буфер мал или buf == NULL.
 * Вынесено наружу, чтобы формат строки можно было проверить тестом, не
 * трогая часы.
 */
int g920_log_prefix(char *buf, size_t size, g920_log_level_t level,
                    uint64_t us, const char *tag);

void g920_logf(g920_log_level_t level, const char *tag, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

void g920_logv(g920_log_level_t level, const char *tag, const char *fmt,
               va_list args);

/*
 * Дамп буфера: строка-заголовок с длиной, дальше строки hexdump.
 * len == 0 печатает только заголовок — «пусто» тоже факт для трассы.
 */
void g920_log_hexdump(g920_log_level_t level, const char *tag,
                      const void *data, size_t len);

/*
 * Уровень **сборки** — отдельная величина от уровня времени выполнения.
 *
 * `g920_log_set_level` только отбрасывает готовую строку: формат уже
 * собран, аргументы уже вычислены, вызов уже сделан. В горячем пути это
 * стоит дорого — в M1 печать одного пакета управляла временем на шине, а в
 * M8 разовый дамп трассы ронял главный цикл в watchdog. На проде такой
 * платы быть не должно вовсе.
 *
 * Поэтому уровень сборки убирает вызов целиком, вместе с вычислением
 * аргументов: условие константное, и компилятор выбрасывает всю ветку.
 * Задаётся флагом сборки, значения — числа, чтобы их можно было передать
 * из `platformio.ini`:
 *
 *   -DG920_LOG_BUILD_LEVEL=G920_LOG_LEVEL_WARN
 *   -DG920_LOG_BUILD_LEVEL=-1        ; молчать совсем
 *
 * Ветка остаётся **синтаксически живой** нарочно, а не прячется под
 * `#ifdef`: так компилятор по-прежнему проверяет формат и типы аргументов,
 * и переменные, нужные только логу, не становятся «неиспользованными».
 * Прошивка, собранная без логов, не расходится с той, что с логами.
 */
#define G920_LOG_LEVEL_OFF (-1)
#define G920_LOG_LEVEL_ERROR 0
#define G920_LOG_LEVEL_WARN 1
#define G920_LOG_LEVEL_INFO 2
#define G920_LOG_LEVEL_DEBUG 3

#ifndef G920_LOG_BUILD_LEVEL
#define G920_LOG_BUILD_LEVEL G920_LOG_LEVEL_INFO
#endif

/*
 * Без приведений типа намеренно: обе стороны — целочисленные макросы, и в
 * таком виде условие годится и для `#if` препроцессора. С `(int)` внутри
 * `#if` компилятор говорит «missing binary operator before token "("» —
 * проверено.
 */
#define G920_LOG_ENABLED(level) ((level) <= (G920_LOG_BUILD_LEVEL))

#define G920_LOG_AT(level, call)                                              \
    do {                                                                      \
        if (G920_LOG_ENABLED(level)) {                                        \
            call;                                                             \
        }                                                                     \
    } while (0)

#define G920_LOGE(tag, ...)                                                   \
    G920_LOG_AT(G920_LOG_LEVEL_ERROR,                                         \
                g920_logf(G920_LOG_ERROR, (tag), __VA_ARGS__))
#define G920_LOGW(tag, ...)                                                   \
    G920_LOG_AT(G920_LOG_LEVEL_WARN,                                          \
                g920_logf(G920_LOG_WARN, (tag), __VA_ARGS__))
#define G920_LOGI(tag, ...)                                                   \
    G920_LOG_AT(G920_LOG_LEVEL_INFO,                                          \
                g920_logf(G920_LOG_INFO, (tag), __VA_ARGS__))
#define G920_LOGD(tag, ...)                                                   \
    G920_LOG_AT(G920_LOG_LEVEL_DEBUG,                                         \
                g920_logf(G920_LOG_DEBUG, (tag), __VA_ARGS__))

/*
 * Дамп буфера тем же выключателем. Он дороже строки на порядок — hexdump
 * шестнадцати байт это ещё одна строка, — и убирать его надо тем же
 * механизмом, а не забывать вручную у каждого вызова.
 */
#define G920_LOG_HEXDUMP_AT(build_level, level, tag, data, len)               \
    G920_LOG_AT(build_level, g920_log_hexdump((level), (tag), (data), (len)))
#define G920_LOGI_HEXDUMP(tag, data, len)                                     \
    G920_LOG_HEXDUMP_AT(G920_LOG_LEVEL_INFO, G920_LOG_INFO, (tag), (data),    \
                        (len))
#define G920_LOGD_HEXDUMP(tag, data, len)                                     \
    G920_LOG_HEXDUMP_AT(G920_LOG_LEVEL_DEBUG, G920_LOG_DEBUG, (tag), (data),  \
                        (len))

#ifdef __cplusplus
}
#endif

#endif /* G920_LOG_H */
