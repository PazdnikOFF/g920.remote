#include "g920/gip_stub.h"

#include <string.h>

#define DESC_TYPE_DEVICE 0x01u
#define DESC_TYPE_CONFIG 0x02u
#define DESC_TYPE_STRING 0x03u
#define DESC_TYPE_INTERFACE 0x04u
#define DESC_TYPE_ENDPOINT 0x05u

#define EP_ATTR_INTERRUPT 0x03u

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

int g920_stub_device_descriptor(uint8_t *buf, size_t size, g920_stub_ids_t ids)
{
    if (buf == NULL || size < G920_STUB_DEVICE_DESC_SIZE) {
        return -1;
    }

    buf[0] = G920_STUB_DEVICE_DESC_SIZE;
    buf[1] = DESC_TYPE_DEVICE;
    put_u16(buf + 2, 0x0200u); /* USB 2.0 */
    buf[4] = G920_GIP_USB_CLASS;
    buf[5] = G920_GIP_USB_SUBCLASS;
    buf[6] = G920_GIP_USB_PROTOCOL;
    buf[7] = 64; /* bMaxPacketSize0 */
    put_u16(buf + 8, ids.vendor_id);
    put_u16(buf + 10, ids.product_id);
    put_u16(buf + 12, ids.device_release);
    /*
     * Строк нет намеренно. Заглушка не имеет права называться рулём: имена
     * и серийник приезжают от TX вместе с остальной личностью (И2).
     */
    buf[14] = 0; /* iManufacturer */
    buf[15] = 0; /* iProduct */
    buf[16] = 0; /* iSerialNumber */
    buf[17] = 1; /* bNumConfigurations */
    return G920_STUB_DEVICE_DESC_SIZE;
}

int g920_stub_config_descriptor(uint8_t *buf, size_t size)
{
    size_t pos = 0;

    if (buf == NULL || size < G920_STUB_CONFIG_DESC_SIZE) {
        return -1;
    }

    /* Configuration */
    buf[pos++] = 9;
    buf[pos++] = DESC_TYPE_CONFIG;
    put_u16(buf + pos, G920_STUB_CONFIG_DESC_SIZE);
    pos += 2;
    buf[pos++] = 1; /* bNumInterfaces: аудио-интерфейс не заявляем */
    buf[pos++] = 1; /* bConfigurationValue */
    buf[pos++] = 0; /* iConfiguration: строк нет */
    buf[pos++] = 0x80u; /* питание от шины, удалённого пробуждения нет */
    buf[pos++] = 250; /* 500 мА */

    /* Interface #0 — обязателен для всех GIP-устройств. */
    buf[pos++] = 9;
    buf[pos++] = DESC_TYPE_INTERFACE;
    buf[pos++] = 0; /* bInterfaceNumber */
    buf[pos++] = 0; /* bAlternateSetting */
    buf[pos++] = 2; /* bNumEndpoints */
    buf[pos++] = G920_GIP_USB_CLASS;
    buf[pos++] = G920_GIP_USB_SUBCLASS;
    buf[pos++] = G920_GIP_USB_PROTOCOL;
    buf[pos++] = 0; /* iInterface */

    /* Interrupt IN */
    buf[pos++] = 7;
    buf[pos++] = DESC_TYPE_ENDPOINT;
    buf[pos++] = (uint8_t)G920_GIP_USB_EP_IN;
    buf[pos++] = EP_ATTR_INTERRUPT;
    put_u16(buf + pos, (uint16_t)G920_GIP_USB_EP_SIZE);
    pos += 2;
    buf[pos++] = (uint8_t)G920_GIP_USB_EP_INTERVAL;

    /* Interrupt OUT */
    buf[pos++] = 7;
    buf[pos++] = DESC_TYPE_ENDPOINT;
    buf[pos++] = (uint8_t)G920_GIP_USB_EP_OUT;
    buf[pos++] = EP_ATTR_INTERRUPT;
    put_u16(buf + pos, (uint16_t)G920_GIP_USB_EP_SIZE);
    pos += 2;
    buf[pos++] = (uint8_t)G920_GIP_USB_EP_INTERVAL;

    return (int)pos;
}

int g920_msos_string_descriptor(uint8_t *buf, size_t size, uint8_t vendor_code)
{
    static const char SIGNATURE[] = "MSFT100";
    size_t pos = 0;

    if (buf == NULL || size < G920_MSOS_STRING_SIZE) {
        return -1;
    }

    buf[pos++] = G920_MSOS_STRING_SIZE;
    buf[pos++] = DESC_TYPE_STRING;
    /* Сигнатура — UTF-16LE, как любой строковый дескриптор USB. */
    for (size_t i = 0; i < sizeof(SIGNATURE) - 1; i++) {
        buf[pos++] = (uint8_t)SIGNATURE[i];
        buf[pos++] = 0x00;
    }
    buf[pos++] = vendor_code;
    buf[pos++] = 0x00; /* bPad */
    return (int)pos;
}

int g920_msos_compatible_id(uint8_t *buf, size_t size, uint8_t first_interface)
{
    static const char COMPAT_ID[] = "XGIP10";

    if (buf == NULL || size < G920_MSOS_COMPAT_ID_SIZE) {
        return -1;
    }

    memset(buf, 0, G920_MSOS_COMPAT_ID_SIZE);

    /* Заголовок, 16 байт. */
    buf[0] = (uint8_t)G920_MSOS_COMPAT_ID_SIZE; /* dwLength, 32 бита LE */
    put_u16(buf + 4, 0x0100u); /* bcdVersion 1.00 */
    put_u16(buf + 6, G920_MSOS_INDEX_COMPAT_ID);
    buf[8] = 1; /* bCount: одна функция */
    /* buf[9..15] — резерв, уже ноль. */

    /* Секция функции, 24 байта. */
    buf[16] = first_interface;
    /*
     * bNumInterfaces, а **не резерв** — как было подписано здесь раньше.
     * По H001419 (таблица Extended Compatible ID, смещение 17) это число
     * интерфейсов функции: `0x01` — GIP-контроллер без аудио, `0x02` — с
     * аудио. Значение и тогда было верным, но подпись «резерв» означала,
     * что в M4 никто не вспомнит его поднять: объявляет ли G920
     * аудио-интерфейс — открытый вопрос, который закрывает дамп руля.
     */
    buf[17] = G920_MSOS_INTERFACES_NO_AUDIO;
    memcpy(buf + 18, COMPAT_ID, sizeof(COMPAT_ID) - 1);
    /* subCompatibleID (buf[26..33]) и резерв (buf[34..39]) — нули. */

    return G920_MSOS_COMPAT_ID_SIZE;
}

bool g920_usb_should_stall_get_descriptor(uint16_t w_value)
{
    return ((w_value >> 8) & 0xFFu) == G920_USB_DESC_DEVICE_QUALIFIER;
}

bool g920_usb_should_stall_ms_os(uint16_t w_index)
{
    return w_index == G920_MSOS_INDEX_EXT_PROPERTIES;
}
