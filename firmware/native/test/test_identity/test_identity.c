#include <stdint.h>
#include <string.h>

#include <unity.h>

#include "g920/identity.h"

#define BUF_SIZE 512

static uint8_t buffer[BUF_SIZE];
static g920_identity_t identity;

/*
 * Заведомо НЕ логитековские значения. Настоящие VID/PID появятся только из
 * дампа M1 и приедут по радио — в коде их быть не должно (И2).
 */
static const g920_identity_fingerprint_t FAKE = { 0x1209, 0x0001, 0x0100 };

void setUp(void)
{
    memset(buffer, 0, sizeof(buffer));
    TEST_ASSERT_EQUAL_INT(G920_IDENTITY_OK,
                          g920_identity_init(&identity, buffer, sizeof(buffer)));
}

void tearDown(void) { }

/* --- раскладка ----------------------------------------------------------- */

static void test_header_layout(void)
{
    g920_identity_set_fingerprint(&identity, FAKE);

    TEST_ASSERT_EQUAL_INT(0, memcmp(buffer, "G9ID", 4));
    TEST_ASSERT_EQUAL_HEX8(0x01, buffer[4]); /* версия формата, LE */
    TEST_ASSERT_EQUAL_HEX8(0x00, buffer[5]);
    TEST_ASSERT_EQUAL_HEX8(0x00, buffer[6]); /* секций пока нет */
    TEST_ASSERT_EQUAL_HEX8(0x09, buffer[8]); /* VID LE */
    TEST_ASSERT_EQUAL_HEX8(0x12, buffer[9]);
    TEST_ASSERT_EQUAL_HEX8(18, buffer[14]); /* общая длина */
    TEST_ASSERT_EQUAL_size_t(G920_IDENTITY_HEADER_SIZE,
                             g920_identity_size(&identity));
}

static void test_section_layout(void)
{
    const uint8_t desc[] = { 0x12, 0x01, 0x00, 0x02 };

    TEST_ASSERT_EQUAL_INT(
        G920_IDENTITY_OK,
        g920_identity_add(&identity, G920_ID_DEVICE_DESC, 0, desc,
                          sizeof(desc)));

    const uint8_t *p = buffer + G920_IDENTITY_HEADER_SIZE;

    TEST_ASSERT_EQUAL_HEX8(G920_ID_DEVICE_DESC, p[0]);
    TEST_ASSERT_EQUAL_HEX8(0, p[1]); /* индекс */
    TEST_ASSERT_EQUAL_HEX8(4, p[2]); /* длина LE */
    TEST_ASSERT_EQUAL_HEX8(0, p[3]);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(desc, p + 4, sizeof(desc));

    TEST_ASSERT_EQUAL_UINT16(1, g920_identity_section_count(&identity));
    TEST_ASSERT_EQUAL_size_t(G920_IDENTITY_HEADER_SIZE + 4 + sizeof(desc),
                             g920_identity_size(&identity));
}

/* --- сборка и поиск ------------------------------------------------------ */

static void test_find_returns_bytes_verbatim(void)
{
    const uint8_t device[] = { 0x12, 0x01, 0xAA, 0xBB };
    const uint8_t config[] = { 0x09, 0x02, 0x20, 0x00, 0x01 };
    const uint8_t hid[] = { 0x05, 0x01, 0x09, 0x04, 0xA1, 0x01 };
    uint16_t len = 0;
    const uint8_t *found;

    g920_identity_add(&identity, G920_ID_DEVICE_DESC, 0, device, sizeof(device));
    g920_identity_add(&identity, G920_ID_CONFIG_DESC, 0, config, sizeof(config));
    g920_identity_add(&identity, G920_ID_HID_REPORT_DESC, 0, hid, sizeof(hid));

    found = g920_identity_find(&identity, G920_ID_CONFIG_DESC, 0, &len);
    TEST_ASSERT_NOT_NULL(found);
    TEST_ASSERT_EQUAL_UINT16(sizeof(config), len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(config, found, sizeof(config));

    /* Копии нет: указатель ведёт внутрь буфера, эти байты уедут хосту
     * вербатим. */
    TEST_ASSERT_TRUE(found > buffer && found < buffer + sizeof(buffer));

    found = g920_identity_find(&identity, G920_ID_HID_REPORT_DESC, 0, &len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(hid, found, sizeof(hid));
}

static void test_string_descriptors_are_indexed(void)
{
    const uint8_t manufacturer[] = { 0x04, 0x03, 'A', 0 };
    const uint8_t product[] = { 0x04, 0x03, 'B', 0 };
    const uint8_t serial[] = { 0x04, 0x03, 'C', 0 };
    uint16_t len = 0;

    /* Строк у устройства несколько, и различаются они только индексом. */
    g920_identity_add(&identity, G920_ID_STRING_DESC, 1, manufacturer, 4);
    g920_identity_add(&identity, G920_ID_STRING_DESC, 2, product, 4);
    g920_identity_add(&identity, G920_ID_STRING_DESC, 3, serial, 4);

    TEST_ASSERT_EQUAL_UINT8(
        'B', g920_identity_find(&identity, G920_ID_STRING_DESC, 2, &len)[2]);
    TEST_ASSERT_EQUAL_UINT8(
        'C', g920_identity_find(&identity, G920_ID_STRING_DESC, 3, &len)[2]);
    TEST_ASSERT_NULL(g920_identity_find(&identity, G920_ID_STRING_DESC, 9, &len));
}

static void test_missing_section_is_null(void)
{
    uint16_t len = 123;

    TEST_ASSERT_NULL(
        g920_identity_find(&identity, G920_ID_GIP_METADATA, 0, &len));
    TEST_ASSERT_NULL(g920_identity_find(NULL, G920_ID_DEVICE_DESC, 0, &len));
}

static void test_zero_length_section(void)
{
    uint16_t len = 99;

    /* Пустая секция — это факт «спрашивали, ответа нет», а не отсутствие. */
    TEST_ASSERT_EQUAL_INT(
        G920_IDENTITY_OK,
        g920_identity_add(&identity, G920_ID_GIP_HELLO, 0, NULL, 0));
    TEST_ASSERT_NULL(g920_identity_find(&identity, G920_ID_GIP_HELLO, 0, &len));
    TEST_ASSERT_EQUAL_UINT16(0, len);
    TEST_ASSERT_EQUAL_UINT16(1, g920_identity_section_count(&identity));
}

/* --- круг через хранилище ------------------------------------------------ */

static void test_build_parse_roundtrip(void)
{
    const uint8_t device[] = { 0x12, 0x01, 0x00, 0x02, 0xFF, 0x47, 0xD0 };
    const uint8_t metadata[] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    uint8_t stored[BUF_SIZE];
    g920_identity_t restored;
    uint16_t len = 0;
    size_t size;

    g920_identity_set_fingerprint(&identity, FAKE);
    g920_identity_add(&identity, G920_ID_DEVICE_DESC, 0, device, sizeof(device));
    g920_identity_add(&identity, G920_ID_GIP_METADATA, 0, metadata,
                      sizeof(metadata));
    size = g920_identity_size(&identity);

    /* Так это ляжет в NVS и так же приедет по радио: буфер и есть
     * сериализованная форма, отдельного «упаковать» нет. */
    memcpy(stored, buffer, size);

    TEST_ASSERT_EQUAL_INT(G920_IDENTITY_OK,
                          g920_identity_parse(&restored, stored, size));
    TEST_ASSERT_EQUAL_UINT16(2, g920_identity_section_count(&restored));
    TEST_ASSERT_TRUE(g920_identity_matches(&restored, FAKE));

    TEST_ASSERT_EQUAL_UINT8_ARRAY(
        device, g920_identity_find(&restored, G920_ID_DEVICE_DESC, 0, &len),
        sizeof(device));
    TEST_ASSERT_EQUAL_UINT16(sizeof(device), len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(
        metadata, g920_identity_find(&restored, G920_ID_GIP_METADATA, 0, &len),
        sizeof(metadata));
}

static void test_unknown_section_type_survives_parsing(void)
{
    /* Блоб от прошивки новее нашей: тип 200 нам неизвестен. Разбор обязан
     * пройти целиком, иначе TX и RX разъедутся на каждом добавлении
     * секции. Именно поэтому новый тип не требует поднимать версию
     * формата. */
    const uint8_t future[] = { 0xDE, 0xAD };
    const uint8_t known[] = { 0x12, 0x01 };
    uint8_t stored[BUF_SIZE];
    g920_identity_t restored;
    uint16_t len = 0;
    size_t size;

    g920_identity_add(&identity, (g920_identity_section_t)200, 0, future,
                      sizeof(future));
    g920_identity_add(&identity, G920_ID_DEVICE_DESC, 0, known, sizeof(known));
    size = g920_identity_size(&identity);
    memcpy(stored, buffer, size);

    TEST_ASSERT_EQUAL_INT(G920_IDENTITY_OK,
                          g920_identity_parse(&restored, stored, size));
    TEST_ASSERT_EQUAL_UINT16(2, g920_identity_section_count(&restored));
    /* Знакомая секция за незнакомой всё равно находится. */
    TEST_ASSERT_EQUAL_UINT8_ARRAY(
        known, g920_identity_find(&restored, G920_ID_DEVICE_DESC, 0, &len),
        sizeof(known));
    /* И незнакомая никуда не делась. */
    TEST_ASSERT_EQUAL_UINT8_ARRAY(
        future,
        g920_identity_find(&restored, (g920_identity_section_t)200, 0, &len),
        sizeof(future));
}

static void test_parse_rejects_corrupted_blobs(void)
{
    const uint8_t data[] = { 1, 2, 3, 4 };
    uint8_t stored[BUF_SIZE];
    g920_identity_t restored;
    size_t size;

    g920_identity_add(&identity, G920_ID_DEVICE_DESC, 0, data, sizeof(data));
    size = g920_identity_size(&identity);
    memcpy(stored, buffer, size);

    TEST_ASSERT_EQUAL_INT(G920_IDENTITY_EMPTY,
                          g920_identity_parse(&restored, stored, 0));
    TEST_ASSERT_EQUAL_INT(G920_IDENTITY_BAD_LENGTH,
                          g920_identity_parse(&restored, stored, 10));

    stored[0] = 'X';
    TEST_ASSERT_EQUAL_INT(G920_IDENTITY_BAD_MAGIC,
                          g920_identity_parse(&restored, stored, size));
    memcpy(stored, buffer, size);

    stored[4] = 99; /* чужая версия контейнера */
    TEST_ASSERT_EQUAL_INT(G920_IDENTITY_BAD_VERSION,
                          g920_identity_parse(&restored, stored, size));
    memcpy(stored, buffer, size);

    /* Длина секции врёт — блоб обрезан. */
    stored[G920_IDENTITY_HEADER_SIZE + 2] = 0xFF;
    TEST_ASSERT_EQUAL_INT(G920_IDENTITY_BAD_LENGTH,
                          g920_identity_parse(&restored, stored, size));
    memcpy(stored, buffer, size);

    /* Заявлено секций больше, чем есть. */
    stored[6] = 5;
    TEST_ASSERT_EQUAL_INT(G920_IDENTITY_BAD_LENGTH,
                          g920_identity_parse(&restored, stored, size));
}

/* --- инвалидация при смене руля ------------------------------------------ */

static void test_cache_invalidated_when_wheel_changes(void)
{
    const uint8_t data[] = { 1 };
    g920_identity_fingerprint_t other = FAKE;

    g920_identity_set_fingerprint(&identity, FAKE);
    g920_identity_add(&identity, G920_ID_DEVICE_DESC, 0, data, 1);

    TEST_ASSERT_TRUE(g920_identity_matches(&identity, FAKE));

    /* Консоль кэширует метаданные по VID/PID/Revision. Разойтись здесь —
     * значит получить личность, за которую security-обмен не ручается. */
    other.product_id = (uint16_t)(FAKE.product_id + 1);
    TEST_ASSERT_FALSE(g920_identity_matches(&identity, other));

    other = FAKE;
    other.device_release = (uint16_t)(FAKE.device_release + 1);
    TEST_ASSERT_FALSE(g920_identity_matches(&identity, other));

    other = FAKE;
    other.vendor_id = (uint16_t)(FAKE.vendor_id + 1);
    TEST_ASSERT_FALSE(g920_identity_matches(&identity, other));
}

static void test_empty_identity_never_matches(void)
{
    /* Пустой кэш не должен выдавать себя за годный. */
    g920_identity_set_fingerprint(&identity, FAKE);
    TEST_ASSERT_TRUE(g920_identity_empty(&identity));
    TEST_ASSERT_FALSE(g920_identity_matches(&identity, FAKE));
    TEST_ASSERT_FALSE(g920_identity_matches(NULL, FAKE));
}

static void test_fingerprint_from_device_descriptor(void)
{
    /* Стандартный дескриптор устройства USB. Значения выдуманные. */
    const uint8_t desc[18] = {
        0x12, 0x01, 0x00, 0x02, 0xFF, 0x47, 0xD0, 0x40,
        0x09, 0x12, /* idVendor  0x1209 */
        0x34, 0x00, /* idProduct 0x0034 */
        0x00, 0x03, /* bcdDevice 0x0300 */
        0x00, 0x00, 0x00, 0x01
    };
    g920_identity_fingerprint_t fp;

    TEST_ASSERT_TRUE(g920_identity_fingerprint_from_device_descriptor(
        &fp, desc, sizeof(desc)));
    TEST_ASSERT_EQUAL_HEX16(0x1209, fp.vendor_id);
    TEST_ASSERT_EQUAL_HEX16(0x0034, fp.product_id);
    TEST_ASSERT_EQUAL_HEX16(0x0300, fp.device_release);
}

static void test_fingerprint_rejects_non_device_descriptor(void)
{
    uint8_t desc[18] = { 0x12, 0x01 };

    /* Конфигурационный дескриптор вместо устройства — не молча нули. */
    desc[1] = 0x02;
    TEST_ASSERT_FALSE(g920_identity_fingerprint_from_device_descriptor(
        NULL, desc, sizeof(desc)));
    g920_identity_fingerprint_t fp;

    TEST_ASSERT_FALSE(g920_identity_fingerprint_from_device_descriptor(
        &fp, desc, sizeof(desc)));
    desc[1] = 0x01;
    desc[0] = 0x09; /* не та длина */
    TEST_ASSERT_FALSE(g920_identity_fingerprint_from_device_descriptor(
        &fp, desc, sizeof(desc)));
    desc[0] = 0x12;
    TEST_ASSERT_FALSE(
        g920_identity_fingerprint_from_device_descriptor(&fp, desc, 17));
}

/* --- И2: ни следа G920 в коде -------------------------------------------- */

static void test_no_g920_identity_is_baked_in(void)
{
    /* Свежесобранный контейнер обязан быть пустым во всех смыслах: ни
     * VID/PID, ни строк, ни единой секции. Ровно то, что судья будет
     * искать грепом по прошивке RX. */
    uint8_t fresh[BUF_SIZE];
    g920_identity_t blank;

    memset(fresh, 0xCC, sizeof(fresh));
    g920_identity_init(&blank, fresh, sizeof(fresh));

    TEST_ASSERT_EQUAL_UINT16(0, g920_identity_section_count(&blank));
    TEST_ASSERT_EQUAL_HEX16(0, blank.fingerprint.vendor_id);
    TEST_ASSERT_EQUAL_HEX16(0, blank.fingerprint.product_id);

    const uint8_t forbidden[][2] = {
        { 0x6D, 0x04 }, /* 046d */
        { 0x61, 0xC2 }, /* c261 */
        { 0x62, 0xC2 } /* c262 */
    };
    for (size_t f = 0; f < 3; f++) {
        for (size_t i = 0; i + 1 < g920_identity_size(&blank); i++) {
            TEST_ASSERT_FALSE(fresh[i] == forbidden[f][0]
                              && fresh[i + 1] == forbidden[f][1]);
        }
    }
}

/* --- границы ------------------------------------------------------------- */

static void test_no_space_is_refused_not_truncated(void)
{
    uint8_t small[G920_IDENTITY_HEADER_SIZE + 8];
    g920_identity_t tight;
    const uint8_t data[8] = { 0 };

    TEST_ASSERT_EQUAL_INT(G920_IDENTITY_NO_SPACE,
                          g920_identity_init(&tight, small,
                                             G920_IDENTITY_HEADER_SIZE - 1));

    TEST_ASSERT_EQUAL_INT(G920_IDENTITY_OK,
                          g920_identity_init(&tight, small, sizeof(small)));
    /* 4 байта заголовка секции + 8 данных = 12, а места 8. */
    TEST_ASSERT_EQUAL_INT(
        G920_IDENTITY_NO_SPACE,
        g920_identity_add(&tight, G920_ID_DEVICE_DESC, 0, data, 8));
    /* Обрезанной секции не появилось. */
    TEST_ASSERT_EQUAL_UINT16(0, g920_identity_section_count(&tight));

    TEST_ASSERT_EQUAL_INT(
        G920_IDENTITY_OK,
        g920_identity_add(&tight, G920_ID_DEVICE_DESC, 0, data, 4));
}

static void test_bad_arguments(void)
{
    const uint8_t data[] = { 1 };

    TEST_ASSERT_EQUAL_INT(G920_IDENTITY_BAD_ARG,
                          g920_identity_init(NULL, buffer, sizeof(buffer)));
    TEST_ASSERT_EQUAL_INT(G920_IDENTITY_BAD_ARG,
                          g920_identity_init(&identity, NULL, 64));
    TEST_ASSERT_EQUAL_INT(
        G920_IDENTITY_BAD_ARG,
        g920_identity_add(NULL, G920_ID_DEVICE_DESC, 0, data, 1));
    TEST_ASSERT_EQUAL_INT(
        G920_IDENTITY_BAD_ARG,
        g920_identity_add(&identity, G920_ID_DEVICE_DESC, 0, NULL, 4));
    TEST_ASSERT_EQUAL_INT(G920_IDENTITY_BAD_ARG,
                          g920_identity_parse(NULL, buffer, 64));
    TEST_ASSERT_EQUAL_INT(G920_IDENTITY_BAD_ARG,
                          g920_identity_parse(&identity, NULL, 64));
}

static void test_status_names(void)
{
    TEST_ASSERT_EQUAL_STRING("OK",
                             g920_identity_status_name(G920_IDENTITY_OK));
    TEST_ASSERT_EQUAL_STRING(
        "BAD_VERSION", g920_identity_status_name(G920_IDENTITY_BAD_VERSION));
    TEST_ASSERT_EQUAL_STRING(
        "?", g920_identity_status_name((g920_identity_status_t)99));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_header_layout);
    RUN_TEST(test_section_layout);

    RUN_TEST(test_find_returns_bytes_verbatim);
    RUN_TEST(test_string_descriptors_are_indexed);
    RUN_TEST(test_missing_section_is_null);
    RUN_TEST(test_zero_length_section);

    RUN_TEST(test_build_parse_roundtrip);
    RUN_TEST(test_unknown_section_type_survives_parsing);
    RUN_TEST(test_parse_rejects_corrupted_blobs);

    RUN_TEST(test_cache_invalidated_when_wheel_changes);
    RUN_TEST(test_empty_identity_never_matches);
    RUN_TEST(test_fingerprint_from_device_descriptor);
    RUN_TEST(test_fingerprint_rejects_non_device_descriptor);

    RUN_TEST(test_no_g920_identity_is_baked_in);

    RUN_TEST(test_no_space_is_refused_not_truncated);
    RUN_TEST(test_bad_arguments);
    RUN_TEST(test_status_names);

    return UNITY_END();
}
