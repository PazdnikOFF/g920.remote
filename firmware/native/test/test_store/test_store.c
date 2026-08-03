#include <stdint.h>
#include <string.h>

#include <unity.h>

#include "g920/store.h"

/* Версии схемы «пира» — как они будут расти в M3. */
#define PEER_V1 1
#define PEER_V2 2

typedef struct {
    uint8_t mac[6];
    uint8_t channel;
} peer_v1_t;

typedef struct {
    uint8_t mac[6];
    uint8_t channel;
    uint8_t proto; /* добавилось во второй версии */
} peer_v2_t;

static const char *KEY = "peer";

void setUp(void)
{
    TEST_ASSERT_EQUAL_INT(G920_STORE_OK, g920_store_init());
    TEST_ASSERT_EQUAL_INT(G920_STORE_OK, g920_store_erase_all());
}

void tearDown(void) { }

/* --- заголовок ---------------------------------------------------------- */

static void test_header_pack_layout(void)
{
    g920_store_header_t h = { G920_STORE_MAGIC, G920_STORE_KIND_IDENTITY,
                              0x0201u, 0x00040302u };
    uint8_t buf[G920_STORE_HEADER_SIZE];
    /* "G920" плюс поля в little-endian — порядок задан явно, чтобы запись
     * не зависела от платформы. */
    const uint8_t want[G920_STORE_HEADER_SIZE] = {
        'G', '9', '2', '0', 0x02, 0x00, 0x01, 0x02, 0x02, 0x03, 0x04, 0x00
    };

    TEST_ASSERT_EQUAL_INT(G920_STORE_HEADER_SIZE,
                          g920_store_header_pack(buf, sizeof(buf), &h));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, buf, sizeof(want));
}

static void test_header_roundtrip(void)
{
    g920_store_header_t in = { G920_STORE_MAGIC, G920_STORE_KIND_PEER, 7, 42 };
    g920_store_header_t out;
    uint8_t buf[G920_STORE_HEADER_SIZE];

    TEST_ASSERT_EQUAL_INT(G920_STORE_HEADER_SIZE,
                          g920_store_header_pack(buf, sizeof(buf), &in));
    TEST_ASSERT_EQUAL_INT(
        G920_STORE_OK, g920_store_header_unpack(&out, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_UINT32(in.magic, out.magic);
    TEST_ASSERT_EQUAL_UINT16(in.kind, out.kind);
    TEST_ASSERT_EQUAL_UINT16(in.version, out.version);
    TEST_ASSERT_EQUAL_UINT32(in.length, out.length);
}

static void test_header_pack_rejects_bad_arguments(void)
{
    g920_store_header_t h = { G920_STORE_MAGIC, 1, 1, 0 };
    uint8_t buf[G920_STORE_HEADER_SIZE];

    TEST_ASSERT_EQUAL_INT(-1, g920_store_header_pack(NULL, 64, &h));
    TEST_ASSERT_EQUAL_INT(-1, g920_store_header_pack(buf, sizeof(buf), NULL));
    TEST_ASSERT_EQUAL_INT(-1,
                          g920_store_header_pack(buf, sizeof(buf) - 1, &h));
}

static void test_header_unpack_detects_foreign_bytes(void)
{
    g920_store_header_t out;
    uint8_t buf[G920_STORE_HEADER_SIZE] = { 0 };

    TEST_ASSERT_EQUAL_INT(G920_STORE_BAD_MAGIC,
                          g920_store_header_unpack(&out, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_INT(G920_STORE_BAD_LENGTH,
                          g920_store_header_unpack(&out, buf, sizeof(buf) - 1));
    TEST_ASSERT_EQUAL_INT(G920_STORE_BAD_ARG,
                          g920_store_header_unpack(NULL, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_INT(G920_STORE_BAD_ARG,
                          g920_store_header_unpack(&out, NULL, sizeof(buf)));
}

static void test_header_check_version_policy(void)
{
    g920_store_header_t h = { G920_STORE_MAGIC, G920_STORE_KIND_PEER, 2, 0 };

    TEST_ASSERT_EQUAL_INT(
        G920_STORE_OK, g920_store_header_check(&h, G920_STORE_KIND_PEER, 2));
    TEST_ASSERT_EQUAL_INT(
        G920_STORE_VERSION_NEWER,
        g920_store_header_check(&h, G920_STORE_KIND_PEER, 1));
    TEST_ASSERT_EQUAL_INT(
        G920_STORE_VERSION_OLDER,
        g920_store_header_check(&h, G920_STORE_KIND_PEER, 3));
    TEST_ASSERT_EQUAL_INT(
        G920_STORE_WRONG_KIND,
        g920_store_header_check(&h, G920_STORE_KIND_IDENTITY, 2));
    TEST_ASSERT_EQUAL_INT(
        G920_STORE_OK,
        g920_store_header_check(&h, G920_STORE_KIND_PEER,
                                G920_STORE_ANY_VERSION));

    h.magic = 0xDEADBEEFu;
    TEST_ASSERT_EQUAL_INT(
        G920_STORE_BAD_MAGIC,
        g920_store_header_check(&h, G920_STORE_KIND_PEER, 2));
}

/* --- ключи -------------------------------------------------------------- */

static void test_key_validation(void)
{
    TEST_ASSERT_TRUE(g920_store_key_valid("peer"));
    TEST_ASSERT_TRUE(g920_store_key_valid("a"));
    /* Ровно 15 символов — предел NVS. */
    TEST_ASSERT_TRUE(g920_store_key_valid("123456789012345"));

    TEST_ASSERT_FALSE(g920_store_key_valid("1234567890123456"));
    TEST_ASSERT_FALSE(g920_store_key_valid(""));
    TEST_ASSERT_FALSE(g920_store_key_valid(NULL));
    TEST_ASSERT_FALSE(g920_store_key_valid("has space"));
    TEST_ASSERT_FALSE(g920_store_key_valid("tab\there"));
}

static void test_status_names(void)
{
    TEST_ASSERT_EQUAL_STRING("OK", g920_store_status_name(G920_STORE_OK));
    TEST_ASSERT_EQUAL_STRING("EMPTY",
                             g920_store_status_name(G920_STORE_EMPTY));
    TEST_ASSERT_EQUAL_STRING(
        "VERSION_NEWER", g920_store_status_name(G920_STORE_VERSION_NEWER));
    TEST_ASSERT_EQUAL_STRING("?",
                             g920_store_status_name((g920_store_status_t)99));
}

/* --- круг записи и чтения ----------------------------------------------- */

static void test_read_missing_key_is_empty_not_error(void)
{
    peer_v1_t peer;

    /* «Ещё не писали» — обычное состояние на первой загрузке, а не сбой. */
    TEST_ASSERT_EQUAL_INT(G920_STORE_EMPTY,
                          g920_store_read(KEY, G920_STORE_KIND_PEER, PEER_V1,
                                          &peer, sizeof(peer), NULL, NULL));
}

static void test_write_then_read_roundtrip(void)
{
    peer_v1_t in = { { 0x24, 0x6F, 0x28, 0x11, 0x22, 0x33 }, 6 };
    peer_v1_t out;
    size_t len = 0;
    uint16_t version = 0;

    memset(&out, 0, sizeof(out));
    TEST_ASSERT_EQUAL_INT(G920_STORE_OK,
                          g920_store_write(KEY, G920_STORE_KIND_PEER, PEER_V1,
                                           &in, sizeof(in)));
    TEST_ASSERT_EQUAL_INT(G920_STORE_OK,
                          g920_store_read(KEY, G920_STORE_KIND_PEER, PEER_V1,
                                          &out, sizeof(out), &len, &version));
    TEST_ASSERT_EQUAL_size_t(sizeof(in), len);
    TEST_ASSERT_EQUAL_UINT16(PEER_V1, version);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(in.mac, out.mac, sizeof(in.mac));
    TEST_ASSERT_EQUAL_UINT8(in.channel, out.channel);
}

static void test_write_overwrites_same_key(void)
{
    peer_v1_t a = { { 1, 1, 1, 1, 1, 1 }, 1 };
    peer_v1_t b = { { 2, 2, 2, 2, 2, 2 }, 11 };
    peer_v1_t out;

    TEST_ASSERT_EQUAL_INT(G920_STORE_OK,
                          g920_store_write(KEY, G920_STORE_KIND_PEER, PEER_V1,
                                           &a, sizeof(a)));
    TEST_ASSERT_EQUAL_INT(G920_STORE_OK,
                          g920_store_write(KEY, G920_STORE_KIND_PEER, PEER_V1,
                                           &b, sizeof(b)));
    TEST_ASSERT_EQUAL_INT(G920_STORE_OK,
                          g920_store_read(KEY, G920_STORE_KIND_PEER, PEER_V1,
                                          &out, sizeof(out), NULL, NULL));
    TEST_ASSERT_EQUAL_UINT8(11, out.channel);
}

static void test_erase_and_erase_all(void)
{
    peer_v1_t in = { { 1, 2, 3, 4, 5, 6 }, 6 };
    peer_v1_t out;

    TEST_ASSERT_EQUAL_INT(G920_STORE_OK,
                          g920_store_write(KEY, G920_STORE_KIND_PEER, PEER_V1,
                                           &in, sizeof(in)));
    TEST_ASSERT_EQUAL_INT(G920_STORE_OK, g920_store_erase(KEY));
    TEST_ASSERT_EQUAL_INT(G920_STORE_EMPTY,
                          g920_store_read(KEY, G920_STORE_KIND_PEER, PEER_V1,
                                          &out, sizeof(out), NULL, NULL));
    TEST_ASSERT_EQUAL_INT(G920_STORE_EMPTY, g920_store_erase(KEY));

    TEST_ASSERT_EQUAL_INT(G920_STORE_OK,
                          g920_store_write("a", G920_STORE_KIND_PEER, PEER_V1,
                                           &in, sizeof(in)));
    TEST_ASSERT_EQUAL_INT(G920_STORE_OK,
                          g920_store_write("b", G920_STORE_KIND_PEER, PEER_V1,
                                           &in, sizeof(in)));
    TEST_ASSERT_EQUAL_INT(G920_STORE_OK, g920_store_erase_all());
    TEST_ASSERT_EQUAL_INT(G920_STORE_EMPTY,
                          g920_store_read("a", G920_STORE_KIND_PEER, PEER_V1,
                                          &out, sizeof(out), NULL, NULL));
    TEST_ASSERT_EQUAL_INT(G920_STORE_EMPTY,
                          g920_store_read("b", G920_STORE_KIND_PEER, PEER_V1,
                                          &out, sizeof(out), NULL, NULL));
}

static void test_zero_length_payload(void)
{
    size_t len = 123;

    /* Запись-флаг: важен сам факт, что она есть. */
    TEST_ASSERT_EQUAL_INT(G920_STORE_OK,
                          g920_store_write("flag", G920_STORE_KIND_VERDICT, 1,
                                           NULL, 0));
    TEST_ASSERT_EQUAL_INT(G920_STORE_OK,
                          g920_store_read("flag", G920_STORE_KIND_VERDICT, 1,
                                          NULL, 0, &len, NULL));
    TEST_ASSERT_EQUAL_size_t(0, len);
}

/* --- версии: то, ради чего всё это ------------------------------------- */

static void test_reading_older_version_is_refused_not_garbled(void)
{
    peer_v1_t old = { { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF }, 6 };
    peer_v2_t out;
    uint16_t found = 0;

    /* Так выглядит плата, которую прошили новой прошивкой поверх старой. */
    TEST_ASSERT_EQUAL_INT(G920_STORE_OK,
                          g920_store_write(KEY, G920_STORE_KIND_PEER, PEER_V1,
                                           &old, sizeof(old)));

    memset(&out, 0xA5, sizeof(out));
    TEST_ASSERT_EQUAL_INT(G920_STORE_VERSION_OLDER,
                          g920_store_read(KEY, G920_STORE_KIND_PEER, PEER_V2,
                                          &out, sizeof(out), NULL, &found));
    /* Версия сообщена — по ней вызывающий решает, мигрировать или стереть. */
    TEST_ASSERT_EQUAL_UINT16(PEER_V1, found);
    /* И ни байта чужой структуры в буфер не попало. */
    for (size_t i = 0; i < sizeof(out); i++) {
        TEST_ASSERT_EQUAL_UINT8(0xA5, ((const uint8_t *)&out)[i]);
    }
}

static void test_reading_newer_version_is_refused(void)
{
    peer_v2_t newer = { { 1, 2, 3, 4, 5, 6 }, 6, 2 };
    peer_v1_t out;
    uint16_t found = 0;

    /* Откат на старую прошивку: запись новее, чем мы понимаем. */
    TEST_ASSERT_EQUAL_INT(G920_STORE_OK,
                          g920_store_write(KEY, G920_STORE_KIND_PEER, PEER_V2,
                                           &newer, sizeof(newer)));
    TEST_ASSERT_EQUAL_INT(G920_STORE_VERSION_NEWER,
                          g920_store_read(KEY, G920_STORE_KIND_PEER, PEER_V1,
                                          &out, sizeof(out), NULL, &found));
    TEST_ASSERT_EQUAL_UINT16(PEER_V2, found);
}

static void test_migration_path(void)
{
    peer_v1_t old = { { 0x24, 0x6F, 0x28, 0x11, 0x22, 0x33 }, 6 };
    peer_v1_t raw;
    peer_v2_t migrated;
    peer_v2_t out;
    uint16_t found = 0;
    size_t len = 0;

    TEST_ASSERT_EQUAL_INT(G920_STORE_OK,
                          g920_store_write(KEY, G920_STORE_KIND_PEER, PEER_V1,
                                           &old, sizeof(old)));

    /* Штатный путь миграции: прочитать любой версией, посмотреть какая
     * оказалась, дополнить, переписать текущей. */
    TEST_ASSERT_EQUAL_INT(
        G920_STORE_OK,
        g920_store_read(KEY, G920_STORE_KIND_PEER, G920_STORE_ANY_VERSION,
                        &raw, sizeof(raw), &len, &found));
    TEST_ASSERT_EQUAL_UINT16(PEER_V1, found);
    TEST_ASSERT_EQUAL_size_t(sizeof(peer_v1_t), len);

    memcpy(migrated.mac, raw.mac, sizeof(raw.mac));
    migrated.channel = raw.channel;
    migrated.proto = 2;
    TEST_ASSERT_EQUAL_INT(G920_STORE_OK,
                          g920_store_write(KEY, G920_STORE_KIND_PEER, PEER_V2,
                                           &migrated, sizeof(migrated)));

    TEST_ASSERT_EQUAL_INT(G920_STORE_OK,
                          g920_store_read(KEY, G920_STORE_KIND_PEER, PEER_V2,
                                          &out, sizeof(out), NULL, NULL));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(old.mac, out.mac, sizeof(old.mac));
    TEST_ASSERT_EQUAL_UINT8(2, out.proto);
}

static void test_wrong_kind_is_refused(void)
{
    peer_v1_t in = { { 1, 2, 3, 4, 5, 6 }, 6 };
    peer_v1_t out;

    TEST_ASSERT_EQUAL_INT(G920_STORE_OK,
                          g920_store_write(KEY, G920_STORE_KIND_PEER, PEER_V1,
                                           &in, sizeof(in)));
    /* Совпадающие версия и размер не должны спасать от чужого вида. */
    TEST_ASSERT_EQUAL_INT(G920_STORE_WRONG_KIND,
                          g920_store_read(KEY, G920_STORE_KIND_IDENTITY,
                                          PEER_V1, &out, sizeof(out), NULL,
                                          NULL));
}

/* --- границы ------------------------------------------------------------ */

static void test_read_into_small_buffer_is_refused(void)
{
    peer_v2_t in = { { 1, 2, 3, 4, 5, 6 }, 6, 2 };
    uint8_t small[2];

    TEST_ASSERT_EQUAL_INT(G920_STORE_OK,
                          g920_store_write(KEY, G920_STORE_KIND_PEER, PEER_V1,
                                           &in, sizeof(in)));
    TEST_ASSERT_EQUAL_INT(G920_STORE_TOO_SMALL,
                          g920_store_read(KEY, G920_STORE_KIND_PEER, PEER_V1,
                                          small, sizeof(small), NULL, NULL));
}

static void test_payload_limit(void)
{
    static uint8_t big[G920_STORE_PAYLOAD_MAX + 1];
    static uint8_t back[G920_STORE_PAYLOAD_MAX];
    size_t len = 0;

    memset(big, 0x5A, sizeof(big));

    TEST_ASSERT_EQUAL_INT(G920_STORE_BAD_LENGTH,
                          g920_store_write(KEY, G920_STORE_KIND_IDENTITY, 1,
                                           big, sizeof(big)));
    TEST_ASSERT_EQUAL_INT(G920_STORE_OK,
                          g920_store_write(KEY, G920_STORE_KIND_IDENTITY, 1,
                                           big, G920_STORE_PAYLOAD_MAX));
    TEST_ASSERT_EQUAL_INT(G920_STORE_OK,
                          g920_store_read(KEY, G920_STORE_KIND_IDENTITY, 1,
                                          back, sizeof(back), &len, NULL));
    TEST_ASSERT_EQUAL_size_t(G920_STORE_PAYLOAD_MAX, len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(big, back, G920_STORE_PAYLOAD_MAX);
}

static void test_bad_arguments(void)
{
    peer_v1_t in = { { 1, 2, 3, 4, 5, 6 }, 6 };
    peer_v1_t out;

    TEST_ASSERT_EQUAL_INT(G920_STORE_BAD_ARG,
                          g920_store_write("has space", G920_STORE_KIND_PEER,
                                           PEER_V1, &in, sizeof(in)));
    TEST_ASSERT_EQUAL_INT(G920_STORE_BAD_ARG,
                          g920_store_write(NULL, G920_STORE_KIND_PEER, PEER_V1,
                                           &in, sizeof(in)));
    TEST_ASSERT_EQUAL_INT(G920_STORE_BAD_ARG,
                          g920_store_write(KEY, G920_STORE_KIND_PEER, PEER_V1,
                                           NULL, sizeof(in)));
    /* ANY_VERSION — только для чтения: записать «любую версию» нельзя. */
    TEST_ASSERT_EQUAL_INT(G920_STORE_BAD_ARG,
                          g920_store_write(KEY, G920_STORE_KIND_PEER,
                                           G920_STORE_ANY_VERSION, &in,
                                           sizeof(in)));
    TEST_ASSERT_EQUAL_INT(G920_STORE_BAD_ARG,
                          g920_store_read(NULL, G920_STORE_KIND_PEER, PEER_V1,
                                          &out, sizeof(out), NULL, NULL));
    TEST_ASSERT_EQUAL_INT(G920_STORE_BAD_ARG,
                          g920_store_read(KEY, G920_STORE_KIND_PEER, PEER_V1,
                                          NULL, sizeof(out), NULL, NULL));
    TEST_ASSERT_EQUAL_INT(G920_STORE_BAD_ARG, g920_store_erase(""));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_header_pack_layout);
    RUN_TEST(test_header_roundtrip);
    RUN_TEST(test_header_pack_rejects_bad_arguments);
    RUN_TEST(test_header_unpack_detects_foreign_bytes);
    RUN_TEST(test_header_check_version_policy);

    RUN_TEST(test_key_validation);
    RUN_TEST(test_status_names);

    RUN_TEST(test_read_missing_key_is_empty_not_error);
    RUN_TEST(test_write_then_read_roundtrip);
    RUN_TEST(test_write_overwrites_same_key);
    RUN_TEST(test_erase_and_erase_all);
    RUN_TEST(test_zero_length_payload);

    RUN_TEST(test_reading_older_version_is_refused_not_garbled);
    RUN_TEST(test_reading_newer_version_is_refused);
    RUN_TEST(test_migration_path);
    RUN_TEST(test_wrong_kind_is_refused);

    RUN_TEST(test_read_into_small_buffer_is_refused);
    RUN_TEST(test_payload_limit);
    RUN_TEST(test_bad_arguments);

    return UNITY_END();
}
