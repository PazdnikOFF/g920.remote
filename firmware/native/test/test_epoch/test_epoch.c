#include <stdint.h>

#include <unity.h>

#include "g920/epoch.h"

void setUp(void) { }
void tearDown(void) { }

/*
 * Эпоха — единственное, на что опирается весь приём: по ней приёмник
 * понимает, что собеседник перезагрузился. Пока её выбор жил в прошивке под
 * `#if ESP_PLATFORM`, он не проверялся ничем, и в нём сидели ровно те
 * ошибки, за которые веха блокировалась дважды: при отказе NVS эпоха молча
 * становилась константой, а при мусоре в RTC умела пойти назад.
 */

static void test_first_boot_starts_at_one(void)
{
    g920_epoch_input_t in = { true, true, 0, 0 };
    g920_epoch_result_t out = g920_epoch_next(&in);

    /* Пустая запись — это первая загрузка, а не отказ. */
    TEST_ASSERT_EQUAL_UINT32(1, out.counter);
    TEST_ASSERT_EQUAL_UINT8(1, out.epoch);
    TEST_ASSERT_FALSE(out.degraded);
}

static void test_counter_grows_every_boot(void)
{
    g920_epoch_input_t in = { true, false, 41, 41 };
    g920_epoch_result_t out = g920_epoch_next(&in);

    TEST_ASSERT_EQUAL_UINT32(42, out.counter);
    TEST_ASSERT_EQUAL_UINT8(42, out.epoch);
    TEST_ASSERT_FALSE(out.degraded);
}

static void test_nvs_failure_is_degraded_not_silent(void)
{
    g920_epoch_input_t in = { false, false, 0, 77 };
    g920_epoch_result_t out = g920_epoch_next(&in);

    /*
     * Раньше здесь возвращалась единица на каждой загрузке — то есть эпоха
     * переставала меняться, и приёмник снова начинал молча выбрасывать
     * законные кадры, подтверждая их. Теперь счётчик берётся из RTC, а флаг
     * degraded обязывает сказать об этом в лог.
     */
    TEST_ASSERT_EQUAL_UINT32(78, out.counter);
    TEST_ASSERT_TRUE(out.degraded);
}

static void test_counter_never_goes_backwards(void)
{
    /*
     * Сессия прошла на счётчике из RTC (запись в NVS не удалась), а
     * следующая загрузка прочитала из NVS старое маленькое значение. Взять
     * его значило бы откатить эпоху назад — приёмник это переживёт, но
     * заплатит кадрами.
     */
    g920_epoch_input_t in = { true, false, 5, 900 };
    g920_epoch_result_t out = g920_epoch_next(&in);

    TEST_ASSERT_EQUAL_UINT32(901, out.counter);
    TEST_ASSERT_FALSE(out.degraded);
}

static void test_stale_rtc_does_not_hold_the_counter_back(void)
{
    /* Обратный случай: RTC отстал (было отключение питания), NVS помнит
     * больше. Берём большее — то есть NVS. */
    g920_epoch_input_t in = { true, false, 900, 0 };
    g920_epoch_result_t out = g920_epoch_next(&in);

    TEST_ASSERT_EQUAL_UINT32(901, out.counter);
}

static void test_epoch_is_the_low_byte_and_wraps(void)
{
    g920_epoch_input_t in = { true, false, 255, 255 };
    g920_epoch_result_t out = g920_epoch_next(&in);

    /* 255 → 0 не особый случай: приёмник сравнивает эпохи по модулю 256 со
     * знаком, и ноль после 255 для него шаг вперёд. */
    TEST_ASSERT_EQUAL_UINT32(256, out.counter);
    TEST_ASSERT_EQUAL_UINT8(0, out.epoch);

    in.stored = 511;
    in.rtc = 511;
    out = g920_epoch_next(&in);
    TEST_ASSERT_EQUAL_UINT32(512, out.counter);
    TEST_ASSERT_EQUAL_UINT8(0, out.epoch);
}

static void test_two_consecutive_boots_never_share_an_epoch(void)
{
    uint32_t counter = 0;
    uint8_t previous = 0;

    /* Ради этого эпоха и есть счётчик, а не случайное число: у случайного
     * совпадение с прошлой раз в 256 загрузок, и ровно тогда, когда важно,
     * чтобы не совпало. */
    for (int boot = 0; boot < 1000; boot++) {
        g920_epoch_input_t in = { true, boot == 0, counter, counter };
        g920_epoch_result_t out = g920_epoch_next(&in);

        if (boot != 0) {
            TEST_ASSERT_NOT_EQUAL_UINT8(previous, out.epoch);
        }
        previous = out.epoch;
        counter = out.counter;
    }
}

static void test_bad_arguments(void)
{
    g920_epoch_result_t out = g920_epoch_next(NULL);

    /* Без входных данных честно только одно: считать, что не знаем ничего,
     * и сказать об этом. */
    TEST_ASSERT_TRUE(out.degraded);
    TEST_ASSERT_EQUAL_UINT8(1, out.epoch);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_first_boot_starts_at_one);
    RUN_TEST(test_counter_grows_every_boot);
    RUN_TEST(test_nvs_failure_is_degraded_not_silent);
    RUN_TEST(test_counter_never_goes_backwards);
    RUN_TEST(test_stale_rtc_does_not_hold_the_counter_back);
    RUN_TEST(test_epoch_is_the_low_byte_and_wraps);
    RUN_TEST(test_two_consecutive_boots_never_share_an_epoch);
    RUN_TEST(test_bad_arguments);

    return UNITY_END();
}
