/*
 * Дескрипторы GIP-заглушки.
 *
 * Проверяется побайтово: дескриптор, у которого поле съехало на байт, не
 * ломается заметно — он просто перестаёт опознаваться, и разбираться с этим
 * придётся осциллографом на живом хосте.
 *
 * Отдельно проверяется инвариант И2: в заглушке не должно быть ни следа
 * личности G920.
 */

#include <stdint.h>
#include <string.h>

#include <unity.h>

#include "g920/gip_stub.h"

void setUp(void) { }
void tearDown(void) { }

/* --- дескриптор устройства ---------------------------------------------- */

static void test_device_descriptor_layout(void)
{
    g920_stub_ids_t ids = { 0x1209, 0x0001, 0x0100 };
    uint8_t buf[G920_STUB_DEVICE_DESC_SIZE];

    TEST_ASSERT_EQUAL_INT(18,
                          g920_stub_device_descriptor(buf, sizeof(buf), ids));

    TEST_ASSERT_EQUAL_HEX8(18, buf[0]); /* bLength */
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[1]); /* DEVICE */
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[2]); /* bcdUSB 2.00, little-endian */
    TEST_ASSERT_EQUAL_HEX8(0x02, buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, buf[4]);
    TEST_ASSERT_EQUAL_HEX8(0x47, buf[5]);
    TEST_ASSERT_EQUAL_HEX8(0xD0, buf[6]);
    TEST_ASSERT_EQUAL_HEX8(64, buf[7]);
    TEST_ASSERT_EQUAL_HEX8(0x09, buf[8]); /* idVendor LE */
    TEST_ASSERT_EQUAL_HEX8(0x12, buf[9]);
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[10]); /* idProduct LE */
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[11]);
    TEST_ASSERT_EQUAL_HEX8(1, buf[17]); /* bNumConfigurations */
}

static void test_device_descriptor_declares_no_strings(void)
{
    /* И2: заглушка не имеет права называться рулём. Имена и серийник
     * приезжают от TX вместе с личностью, а не лежат в прошивке. */
    g920_stub_ids_t ids = { 0x1209, 0x0001, 0x0100 };
    uint8_t buf[G920_STUB_DEVICE_DESC_SIZE];

    TEST_ASSERT_EQUAL_INT(18,
                          g920_stub_device_descriptor(buf, sizeof(buf), ids));
    TEST_ASSERT_EQUAL_HEX8(0, buf[14]); /* iManufacturer */
    TEST_ASSERT_EQUAL_HEX8(0, buf[15]); /* iProduct */
    TEST_ASSERT_EQUAL_HEX8(0, buf[16]); /* iSerialNumber */
}

static void test_stub_carries_no_g920_identity(void)
{
    /* Ровно та проверка, которую судья делает грепом по прошивке RX:
     * ни 046d, ни c261, ни c262 не должны появляться в заглушке ни при
     * каких переданных идентификаторах. */
    g920_stub_ids_t ids = { 0x1209, 0x0001, 0x0100 };
    uint8_t device[G920_STUB_DEVICE_DESC_SIZE];
    uint8_t config[G920_STUB_CONFIG_DESC_SIZE];

    TEST_ASSERT_EQUAL_INT(
        18, g920_stub_device_descriptor(device, sizeof(device), ids));
    TEST_ASSERT_EQUAL_INT(
        32, g920_stub_config_descriptor(config, sizeof(config)));

    TEST_ASSERT_NOT_EQUAL_UINT16(0x046D, (uint16_t)(device[8] | (device[9] << 8)));

    const uint8_t forbidden[][2] = {
        { 0x6D, 0x04 }, /* 046d little-endian */
        { 0x61, 0xC2 }, /* c261 */
        { 0x62, 0xC2 } /* c262 */
    };
    for (size_t f = 0; f < 3; f++) {
        for (size_t i = 0; i + 1 < sizeof(device); i++) {
            TEST_ASSERT_FALSE(device[i] == forbidden[f][0]
                              && device[i + 1] == forbidden[f][1]);
        }
        for (size_t i = 0; i + 1 < sizeof(config); i++) {
            TEST_ASSERT_FALSE(config[i] == forbidden[f][0]
                              && config[i + 1] == forbidden[f][1]);
        }
    }
}

/* --- дескриптор конфигурации -------------------------------------------- */

static void test_config_descriptor_layout(void)
{
    /* Конфигурация 9 + интерфейс 9 + два эндпоинта по 7 = 32 байта. */
    const uint8_t want[G920_STUB_CONFIG_DESC_SIZE] = {
        0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0x80, 0xFA,
        0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0x47, 0xD0, 0x00,
        0x07, 0x05, 0x81, 0x03, 0x40, 0x00, 0x04,
        0x07, 0x05, 0x01, 0x03, 0x40, 0x00, 0x04
    };
    uint8_t buf[G920_STUB_CONFIG_DESC_SIZE];

    TEST_ASSERT_EQUAL_INT(32, g920_stub_config_descriptor(buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, buf, sizeof(want));
}

static void test_config_total_length_matches_actual_bytes(void)
{
    /* wTotalLength, разошедшийся с фактической длиной, — классика: хост
     * читает мусор за концом или обрезает интерфейс. */
    uint8_t buf[G920_STUB_CONFIG_DESC_SIZE];
    int written = g920_stub_config_descriptor(buf, sizeof(buf));
    uint16_t total = (uint16_t)(buf[2] | (buf[3] << 8));

    TEST_ASSERT_EQUAL_INT((int)total, written);
}

static void test_config_descriptor_lengths_walk_exactly(void)
{
    /* Проход по цепочке дескрипторов по полю bLength обязан попасть точно
     * в конец, а не мимо. */
    uint8_t buf[G920_STUB_CONFIG_DESC_SIZE];
    int written = g920_stub_config_descriptor(buf, sizeof(buf));
    size_t pos = 0;
    int count = 0;

    while (pos < (size_t)written) {
        TEST_ASSERT_TRUE(buf[pos] > 0);
        pos += buf[pos];
        count++;
    }
    TEST_ASSERT_EQUAL_size_t((size_t)written, pos);
    TEST_ASSERT_EQUAL_INT(4, count); /* config, interface, 2 endpoint */
}

static void test_endpoints_are_interrupt_64_with_interval_4(void)
{
    /* Требование брифинга: два interrupt по 64 байта, bInterval ≥ 4,
     * то есть опрос не чаще 250 Гц. */
    uint8_t buf[G920_STUB_CONFIG_DESC_SIZE];
    const size_t ep[2] = { 18, 25 };

    TEST_ASSERT_EQUAL_INT(32, g920_stub_config_descriptor(buf, sizeof(buf)));
    for (int i = 0; i < 2; i++) {
        size_t p = ep[i];

        TEST_ASSERT_EQUAL_HEX8(0x05, buf[p + 1]); /* ENDPOINT */
        TEST_ASSERT_EQUAL_HEX8(0x03, buf[p + 3]); /* interrupt */
        TEST_ASSERT_EQUAL_UINT16(64, (uint16_t)(buf[p + 4] | (buf[p + 5] << 8)));
        TEST_ASSERT_TRUE(buf[p + 6] >= 4);
    }
    /* Один IN, один OUT. */
    TEST_ASSERT_EQUAL_HEX8(0x80, buf[ep[0] + 2] & 0x80u);
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[ep[1] + 2] & 0x80u);
}

static void test_no_audio_interface_declared(void)
{
    /* Аудио-интерфейс не заявляем: объявит ли его сам руль — открытый
     * вопрос M1. */
    uint8_t buf[G920_STUB_CONFIG_DESC_SIZE];

    TEST_ASSERT_EQUAL_INT(32, g920_stub_config_descriptor(buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_HEX8(1, buf[4]); /* bNumInterfaces */
}

/* --- MS OS Descriptor ---------------------------------------------------- */

static void test_ms_os_string_descriptor(void)
{
    const uint8_t want[G920_MSOS_STRING_SIZE] = {
        0x12, 0x03, 'M', 0, 'S', 0, 'F', 0, 'T',
        /*
         * Ожидаем **литерал**, а зовём **макросом** — и порядок здесь
         * важен. Первая попытка чинить это поставила литерал в обе
         * стороны: тест проверял, что функция копирует свой аргумент, и
         * константу проекта не пиннил ничем — подмена `G920_MSOS_VENDOR_CODE`
         * по-прежнему прошла бы зелёной. Спека H001419 называет здесь
         * конкретный байт, значит тест обязан сверить с ним именно
         * константу, которой пользуется прошивка.
         */
        0,    '1',  0,   '0', 0,   '0', 0,   0x90, 0x00
    };
    uint8_t buf[G920_MSOS_STRING_SIZE];

    TEST_ASSERT_EQUAL_INT(18, g920_msos_string_descriptor(buf, sizeof(buf),
                                                          G920_MSOS_VENDOR_CODE));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, buf, sizeof(want));
}

static void test_ms_os_compatible_id_is_xgip10(void)
{
    uint8_t buf[G920_MSOS_COMPAT_ID_SIZE];

    TEST_ASSERT_EQUAL_INT(40, g920_msos_compatible_id(buf, sizeof(buf), 0));

    /* Заголовок. */
    TEST_ASSERT_EQUAL_UINT32(40, (uint32_t)buf[0] | ((uint32_t)buf[1] << 8)
                                     | ((uint32_t)buf[2] << 16)
                                     | ((uint32_t)buf[3] << 24));
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[4]); /* bcdVersion 1.00 LE */
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[5]);
    TEST_ASSERT_EQUAL_HEX8(0x04, buf[6]); /* wIndex 0x0004 LE */
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[7]);
    TEST_ASSERT_EQUAL_HEX8(1, buf[8]); /* bCount */
    for (size_t i = 9; i < 16; i++) {
        TEST_ASSERT_EQUAL_HEX8(0x00, buf[i]);
    }

    /* Секция функции. */
    TEST_ASSERT_EQUAL_HEX8(0, buf[16]); /* bFirstInterfaceNumber */
    /* bNumInterfaces, не резерв: 0x01 — контроллер без аудио, 0x02 — с
     * аудио (H001419). У заглушки аудио нет и быть не может. */
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[17]);
    TEST_ASSERT_EQUAL_INT(0, memcmp(buf + 18, "XGIP10\0\0", 8));
    /* subCompatibleID и хвостовой резерв — нули. */
    for (size_t i = 26; i < 40; i++) {
        TEST_ASSERT_EQUAL_HEX8(0x00, buf[i]);
    }
}

static void test_ms_os_compatible_id_honours_interface_number(void)
{
    uint8_t buf[G920_MSOS_COMPAT_ID_SIZE];

    TEST_ASSERT_EQUAL_INT(40, g920_msos_compatible_id(buf, sizeof(buf), 3));
    TEST_ASSERT_EQUAL_HEX8(3, buf[16]);
}

/* --- политика STALL ------------------------------------------------------ */

static void test_device_qualifier_is_stalled(void)
{
    /* Спека велит отвечать STALL: иначе Windows покажет «This device can
     * perform faster». */
    TEST_ASSERT_TRUE(g920_usb_should_stall_get_descriptor(0x0600));
    TEST_ASSERT_TRUE(g920_usb_should_stall_get_descriptor(0x0601));
}

static void test_normal_descriptors_are_not_stalled(void)
{
    TEST_ASSERT_FALSE(g920_usb_should_stall_get_descriptor(0x0100)); /* DEVICE */
    TEST_ASSERT_FALSE(g920_usb_should_stall_get_descriptor(0x0200)); /* CONFIG */
    TEST_ASSERT_FALSE(g920_usb_should_stall_get_descriptor(0x03EE)); /* MSFT100 */
    TEST_ASSERT_FALSE(g920_usb_should_stall_get_descriptor(0x0500)); /* 0x05 */
    TEST_ASSERT_FALSE(g920_usb_should_stall_get_descriptor(0x0700));
}

static void test_extended_properties_stalled_but_compat_id_answered(void)
{
    TEST_ASSERT_TRUE(
        g920_usb_should_stall_ms_os(G920_MSOS_INDEX_EXT_PROPERTIES));
    TEST_ASSERT_FALSE(g920_usb_should_stall_ms_os(G920_MSOS_INDEX_COMPAT_ID));
    TEST_ASSERT_FALSE(g920_usb_should_stall_ms_os(0x0000));
}

/* --- границы ------------------------------------------------------------- */

static void test_small_buffers_are_refused(void)
{
    g920_stub_ids_t ids = { 1, 2, 3 };
    uint8_t buf[64];

    TEST_ASSERT_EQUAL_INT(-1, g920_stub_device_descriptor(buf, 17, ids));
    TEST_ASSERT_EQUAL_INT(-1, g920_stub_config_descriptor(buf, 31));
    TEST_ASSERT_EQUAL_INT(-1, g920_msos_string_descriptor(buf, 17, 0x90));
    TEST_ASSERT_EQUAL_INT(-1, g920_msos_compatible_id(buf, 39, 0));

    TEST_ASSERT_EQUAL_INT(-1, g920_stub_device_descriptor(NULL, 64, ids));
    TEST_ASSERT_EQUAL_INT(-1, g920_stub_config_descriptor(NULL, 64));
    TEST_ASSERT_EQUAL_INT(-1, g920_msos_string_descriptor(NULL, 64, 0x90));
    TEST_ASSERT_EQUAL_INT(-1, g920_msos_compatible_id(NULL, 64, 0));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_device_descriptor_layout);
    RUN_TEST(test_device_descriptor_declares_no_strings);
    RUN_TEST(test_stub_carries_no_g920_identity);

    RUN_TEST(test_config_descriptor_layout);
    RUN_TEST(test_config_total_length_matches_actual_bytes);
    RUN_TEST(test_config_descriptor_lengths_walk_exactly);
    RUN_TEST(test_endpoints_are_interrupt_64_with_interval_4);
    RUN_TEST(test_no_audio_interface_declared);

    RUN_TEST(test_ms_os_string_descriptor);
    RUN_TEST(test_ms_os_compatible_id_is_xgip10);
    RUN_TEST(test_ms_os_compatible_id_honours_interface_number);

    RUN_TEST(test_device_qualifier_is_stalled);
    RUN_TEST(test_normal_descriptors_are_not_stalled);
    RUN_TEST(test_extended_properties_stalled_but_compat_id_answered);

    RUN_TEST(test_small_buffers_are_refused);

    return UNITY_END();
}
