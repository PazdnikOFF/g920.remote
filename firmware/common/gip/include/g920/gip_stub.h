/*
 * ═══════════════════════════════════════════════════════════════════════
 *  ДЕСКРИПТОРЫ GIP-ЗАГЛУШКИ. ВРЕМЕННЫЕ. НЕ ЛИЧНОСТЬ РУЛЯ.
 * ═══════════════════════════════════════════════════════════════════════
 *
 * Инвариант И2 запрещает зашивать личность G920 в прошивку RX: дескрипторы,
 * VID/PID, строки и блоб метаданных приходят от TX и лежат в NVS (M4).
 * Единственное исключение, которое И2 оговаривает явно, — дескрипторы
 * GIP-заглушки на этапе опознания хоста, до того как личность получена.
 * Это они и есть, и они обязаны быть помечены как временные.
 *
 * Что здесь НЕ лежит и лежать не должно:
 *   - VID/PID G920 — идентификаторы передаются параметром, значения по
 *     умолчанию заведомо не логитековские;
 *   - строковые дескрипторы — заглушка объявляет ноль строк намеренно;
 *   - блоб метаданных.
 *
 * Заглушка существует ради одного: заставить хост начать энумерацию и
 * показать, кто он такой. Как только детектор вынес вердикт и личность
 * приехала от TX, USB поднимается заново — уже настоящим устройством.
 *
 * ── Источники ───────────────────────────────────────────────────────────
 *
 * Класс интерфейса `0xFF / 0x47 / 0xD0`, два interrupt-эндпоинта по 64
 * байта, `bInterval` ≥ 4 — из брифинга, раздел «Дескрипторы USB».
 *
 * Формат MS OS Descriptor (строка 0xEE = `MSFT100`, Extended Compatible ID
 * = `XGIP10`) — стандартный майкрософтовский, раскладка байт общеизвестна.
 *
 * ⚠ Не подтверждено первоисточником из репозитория и ждёт дампа M1:
 *   - vendor code `0x90` в строке 0xEE;
 *   - класс на уровне устройства (здесь тот же `0xFF/0x47/0xD0`, брифинг
 *     оговаривает только интерфейс);
 *   - порядок эндпоинтов IN/OUT в конфигурации.
 * Настоящий руль — тоже GIP-устройство, и его дамп в M1 закроет все три.
 */

#ifndef G920_GIP_STUB_H
#define G920_GIP_STUB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Класс GIP-интерфейса. */
#define G920_GIP_USB_CLASS 0xFFu
#define G920_GIP_USB_SUBCLASS 0x47u
#define G920_GIP_USB_PROTOCOL 0xD0u

#define G920_GIP_USB_EP_SIZE 64u
#define G920_GIP_USB_EP_INTERVAL 4u
#define G920_GIP_USB_EP_IN 0x81u
#define G920_GIP_USB_EP_OUT 0x01u

#define G920_STUB_DEVICE_DESC_SIZE 18
#define G920_STUB_CONFIG_DESC_SIZE 32

/* MS OS Descriptor. */
#define G920_MSOS_STRING_INDEX 0xEEu
#define G920_MSOS_STRING_SIZE 18
#define G920_MSOS_VENDOR_CODE 0x90u
#define G920_MSOS_COMPAT_ID_SIZE 40

/*
 * bNumInterfaces в секции функции Extended Compatible ID (H001419):
 * число интерфейсов функции, а не резерв.
 *
 * У заглушки аудио нет и быть не может — она не личность руля. Объявляет
 * ли аудио-интерфейс настоящий G920, выясняет дамп M1; там же это значение
 * станет параметром, а не константой.
 *
 * Константы «с аудио» здесь намеренно нет: она не понадобится до M1, а
 * заведённая заранее была бы ровно тем неиспользуемым API, за которое этот
 * проект блокирует вехи.
 */
#define G920_MSOS_INTERFACES_NO_AUDIO 0x01u
#define G920_MSOS_INDEX_COMPAT_ID 0x0004u
#define G920_MSOS_INDEX_EXT_PROPERTIES 0x0005u

/* Тип дескриптора Device Qualifier — на него отвечаем STALL. */
#define G920_USB_DESC_DEVICE_QUALIFIER 0x06u

/*
 * Идентификаторы для заглушки. Значения по умолчанию — заведомо
 * не логитековские: заглушка не должна и не может выдавать себя за руль.
 */
typedef struct {
    uint16_t vendor_id;
    uint16_t product_id;
    uint16_t device_release;
} g920_stub_ids_t;

/* Все функции возвращают число записанных байт либо -1. */

int g920_stub_device_descriptor(uint8_t *buf, size_t size,
                                g920_stub_ids_t ids);

int g920_stub_config_descriptor(uint8_t *buf, size_t size);

/* Строковый дескриптор по индексу 0xEE: сигнатура MSFT100 и vendor code. */
int g920_msos_string_descriptor(uint8_t *buf, size_t size,
                                uint8_t vendor_code);

/* Extended Compatible ID: одна функция, совместимый идентификатор XGIP10. */
int g920_msos_compatible_id(uint8_t *buf, size_t size,
                            uint8_t first_interface);

/* --- политика ответов на control-запросы -------------------------------- */

/*
 * Device Qualifier обязан получить STALL. Иначе Windows покажет «This
 * device can perform faster»; консоли он не нужен вовсе.
 * w_value — поле GET_DESCRIPTOR: старший байт тип, младший индекс.
 */
bool g920_usb_should_stall_get_descriptor(uint16_t w_value);

/*
 * Extended Properties (wIndex 0x0005) обязан получить STALL, Extended
 * Compatible ID (0x0004) — нет. Молча игнорировать нельзя: хост ждёт
 * определённого ответа, и разница между STALL и тишиной видна в трассе.
 */
bool g920_usb_should_stall_ms_os(uint16_t w_index);

#ifdef __cplusplus
}
#endif

#endif /* G920_GIP_STUB_H */
