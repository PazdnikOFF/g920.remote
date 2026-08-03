/*
 * Конфигурация TinyUSB для RX.
 *
 * Устройство GIP — это **свой класс**: vendor-specific 0xFF/0x47/0xD0 с
 * двумя interrupt-эндпоинтами. Ни один встроенный класс TinyUSB ему не
 * подходит, поэтому все они выключены, а наш регистрируется через
 * `usbd_app_driver_get_cb`. Выключены они не ради экономии флеша: включённый
 * класс объявляет свои эндпоинты и свои ответы на control-запросы, а в M2
 * предмет вехи ровно в том, что именно хост спросил и что именно получил.
 * Лишний отвечающий здесь портит наблюдение.
 *
 * ESP32-S3 — Full Speed (12 Мбит/с). Interrupt-эндпоинт на FS даёт максимум
 * 64 байта на пакет и `bInterval` в миллисекундах; и то и другое совпадает
 * с тем, что объявляет заглушка в `g920/gip_stub.h`.
 */

#ifndef G920_TUSB_CONFIG_H
#define G920_TUSB_CONFIG_H

/* CFG_TUSB_MCU приходит из CMakeLists самого компонента tinyusb. */

#define CFG_TUSB_OS OPT_OS_FREERTOS
#define CFG_TUSB_DEBUG 0

/*
 * Буферы DMA. Внутренняя память и выравнивание по 4 — требование dwc2:
 * PSRAM под дескрипторы и эндпоинты не годится.
 */
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))

#define CFG_TUD_ENABLED 1
#define CFG_TUD_MAX_SPEED OPT_MODE_FULL_SPEED

/* Нулевой эндпоинт: 64 байта — максимум для Full Speed. */
#define CFG_TUD_ENDPOINT0_SIZE 64

/* Ни одного встроенного класса — см. заголовок файла. */
#define CFG_TUD_CDC 0
#define CFG_TUD_MSC 0
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_AUDIO 0
#define CFG_TUD_VIDEO 0
#define CFG_TUD_VENDOR 0
#define CFG_TUD_USBTMC 0
#define CFG_TUD_DFU 0
#define CFG_TUD_DFU_RUNTIME 0
#define CFG_TUD_ECM_RNDIS 0
#define CFG_TUD_NCM 0
#define CFG_TUD_BTH 0

#endif /* G920_TUSB_CONFIG_H */
