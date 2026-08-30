/*
 * Привязка к плате: индикатор.
 *
 * Плат в проекте три и они разные: у ESP32-S3-DevKitC-1 адресный WS2812, у
 * T-Dongle-S3 — APA102 на двух ногах, у Supermini — обычный светодиод на
 * GPIO. Прикладной код не должен об этом знать: он говорит «жёлтый, идёт
 * опознание», а как это зажечь — дело платы.
 *
 * Тип индикатора выбирается флагом сборки в platformio.ini:
 *
 *   -DG920_LED_WS2812=<gpio>
 *   -DG920_LED_APA102_DIN=<gpio> -DG920_LED_APA102_CLK=<gpio>
 *   -DG920_LED_GPIO=<gpio>
 *
 * Ни один не задан — индикатор молча отсутствует, прошивка живёт.
 */

#ifndef G920_BOARD_H
#define G920_BOARD_H

#include <stdbool.h>

#include "g920/rgb.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Готовит индикатор. false — индикатора нет или он не поднялся. */
bool g920_board_led_init(void);

/* Показать состояние. Без индикатора — тихо ничего. */
void g920_board_led_set(g920_indication_t state);

/* Имя типа индикатора для лога: "ws2812", "apa102", "gpio", "none". */
const char *g920_board_led_kind(void);

/*
 * Уйти в режим загрузки ПЗУ и ждать `esptool`.
 *
 * Не возвращается. Нужна плате с единственным USB-разъёмом, занятым ролью
 * устройства: подробности и оговорки — в `src/download.c`.
 */
void g920_board_enter_download_mode(void);

#ifdef __cplusplus
}
#endif

#endif /* G920_BOARD_H */
