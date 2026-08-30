/*
 * TX в роли USB Host — веха M1.
 *
 * Прошивка делает ровно два шага, и второй включается только после
 * удавшегося первого:
 *
 *   1. **увидеть руль** — поднять стек USB Host, дождаться подключения и
 *      выложить в лог всё, что хост узнал: дескриптор устройства,
 *      конфигурацию, интерфейсы, эндпоинты;
 *   2. **услышать руль** — захватить vendor-интерфейс `0xFF/0x47/0xD0` и
 *      непрерывно опрашивать его interrupt IN, печатая каждое
 *      GIP-сообщение сырыми байтами и разобранным заголовком.
 *
 * Второй шаг — не украшение. USB-устройство пассивно: без IN-токенов от
 * хоста руль молчит, и молчание это неотличимо от неисправности. Зато как
 * только опрос пошёл, руль заговорит **сам** — в состоянии Arrival он шлёт
 * Hello каждые 500 мс, пока хост не ответит (H001419, § таймингов).
 * Поэтому «поговорить» и «начать слушать» здесь одно и то же действие.
 *
 * OUT-эндпоинт открывается вместе с IN и стоит наготове, но по своей воле
 * прошивка в него **ничего не пишет**. Ответы хоста на Hello (ACK,
 * запрос метаданных, Set Device State) — следующий шаг вехи, и делать их
 * до того, как увидены живые Hello, значит отвечать вслепую.
 *
 * ⚠ **Мотор не трогать.** Ни одного сообщения обратной связи по силе в
 * этом файле нет и быть не должно: руль стоит на столе без присмотра, а
 * заклиненный мотор греется. FFB — веха M9, и она делается при человеке.
 *
 * Почему так узко. M1 — первое поднятие капризного стека, и при неудаче
 * список подозреваемых длинный: распайка, питание, земля, стек, руль. Чем
 * меньше делает прошивка, тем короче этот список.
 *
 * ⚠ Она же **проверяет сборку железа**. Если руль энумерировался — распайка
 * (D− на GPIO19, D+ на GPIO20, общая земля) и внешние 5 В на VBUS сделаны
 * верно. Это единственный способ их проверить: тестером видно только
 * обрыв, а не то, что хост и устройство договорились.
 *
 * Развод прошивок — флагом `G920_MODE_HOST`, как и на RX: при
 * `framework = espidf` PlatformIO собирает `src/` через CMake, и
 * `build_src_filter` там не действует.
 */

#ifdef G920_MODE_HOST

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "hal/usb_serial_jtag_ll.h"
#include "usb/usb_helpers.h"
#include "usb/usb_host.h"

#include "g920/board.h"
#include "g920/gip.h"
#include "g920/gip_control.h"
#include "g920/gip_host.h"
#include "g920/hexdump.h"
#include "g920/identity.h"
#include "g920/link.h"
#include "esp_log.h"
#include "esp_system.h"

#include "g920/log.h"
#include "g920/store.h"
#include "g920/timestamp.h"
#include "g920/trace.h"
#include "g920/version.h"

static const char *TAG = "boot";
static const char *M1 = "m1";

/*
 * Такт цикла. Два миллисекунды, а не десять: этим же циклом двигается
 * `g920_link_tick`, а он отдаёт по кадру за вызов. При такте 10 мс
 * пятнадцать попыток надёжной дисциплины растягиваются на 150 мс — больше
 * чужого ACK timeout в 100 мс, то есть security-обмен рвётся не по потерям,
 * а по нашей медлительности.
 */
#define TICK_MS 2

/* Клиент один, событий у него в очереди немного: подключили и отключили. */
#define CLIENT_EVENT_QUEUE 5

static usb_host_client_handle_t s_client;
static usb_device_handle_t s_device;
static volatile uint8_t s_pending_addr;
static volatile bool s_pending_new;
static volatile bool s_pending_gone;

/* Интерфейс GIP по MS-GIPUSB: тот же тройной класс, что RX объявляет о себе
 * в M2. Совпадение не случайно — это одно и то же место протокола с двух
 * сторон. */
#define GIP_IF_CLASS 0xFFu
#define GIP_IF_SUBCLASS 0x47u
#define GIP_IF_PROTOCOL 0xD0u

/* Interrupt-эндпоинты GIP — 64 байта. Размер берётся из дескриптора, а это
 * потолок буфера: чужое устройство с большим MPS будет отвергнуто громко, а
 * не переполнит приём молча. */
#define EP_BUF_MAX 64

static uint8_t s_intf_num;
static bool s_claimed;
static usb_transfer_t *s_in;
static usb_transfer_t *s_out;
static uint8_t s_ep_in;
static uint8_t s_ep_out;
static volatile bool s_in_running;
static volatile bool s_out_busy;
static volatile uint32_t s_in_msgs;
static volatile uint32_t s_in_errors;
static uint64_t s_claim_us;
static volatile uint64_t s_first_in_us;

/* --- проверка проводов до запуска стека ------------------------------------ */

/*
 * Читает D− и D+ как обычные входы и говорит, что на них видно.
 *
 * Зачем: когда руль не энумерируется, подозреваемых пятеро — питание,
 * земля, перепутанные D+/D−, обрыв и сам стек. Тестером видно обрыв, но не
 * видно, договорились ли хост с устройством. А вот это различимо
 * электрически и **до** того, как стек заберёт пины себе:
 *
 * USB-устройство подтягивает **D+ к 3.3 В через 1.5 кОм**, как только у
 * него появляется VBUS (для full speed; low speed тянет D−, но G920 —
 * full speed). Значит:
 *
 *   D+ высокий, D− низкий  → устройство есть, провода на своих местах;
 *   D− высокий, D+ низкий  → **провода перепутаны местами**;
 *   оба низкие             → нет VBUS, нет земли или обрыв;
 *   оба высокие            → похоже на замыкание или на чужую подтяжку.
 *
 * Читается один раз при загрузке: дальше пины забирает PHY, иGPIO-чтение
 * стало бы враньём. Перепаяли — сбросьте плату, и она скажет заново.
 */
#define USB_DM_GPIO 19
#define USB_DP_GPIO 20

/*
 * ⚠ Вся диагностика ниже собирается только с `-DG920_PRESTACK_PROBES`.
 *
 * Она крутит пины 19/20 как GPIO, а после этого USB-PHY не видит
 * статически подключённое устройство (разбор — у выбора пути в app_main).
 * В рабочей прошивке её нет не ради экономии, а чтобы дорога к стеку
 * физически не проходила через то, что его ломает.
 */
#ifdef G920_PRESTACK_PROBES
static int read_pin(int gpio, bool pull_up)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = pull_up ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = pull_up ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    if (gpio_config(&cfg) != ESP_OK) {
        return -1;
    }
    /* Внутренние подтяжки слабые (десятки кОм), линии ёмкостные — даём
     * время устояться. */
    vTaskDelay(pdMS_TO_TICKS(5));
    return gpio_get_level(gpio);
}

/*
 * Что означает пара «со слабой подтяжкой вниз» / «со слабой вверх»:
 *
 *   вниз 0, вверх 1 → провод **никуда не приходит**: подтяжки внутри
 *                     контроллера тянут его куда хотят, снаружи никто не
 *                     возражает. Обрыв, не тот контакт, не та розетка;
 *   вниз 0, вверх 0 → снаружи что-то **держит линию у земли**: провод
 *                     дошёл, но устройство на том конце не запитано (его
 *                     защитные диоды сидят на мёртвой шине) либо линия
 *                     закорочена на землю;
 *   вниз 1          → снаружи **активно тянут вверх**. Для D+ это и есть
 *                     подтяжка 1.5 кОм внутри устройства: оно запитано и
 *                     готово говорить.
 *
 * Различать первые два случая важно: «обрыв» и «нет питания» чинятся в
 * разных местах, а по одному чтению они выглядели одинаково.
 */
static const char *verdict(int down, int up)
{
    if (down < 0 || up < 0) {
        return "?";
    }
    if (down == 1) {
        return "подтянут снаружи вверх — устройство запитано";
    }
    if (up == 1) {
        return "болтается — до провода никто не достаёт";
    }
    return "держат у земли — провод дошёл, но питания на той стороне нет";
}

static void check_wires(void)
{
    /*
     * Сначала отцепить USB-Serial-JTAG от площадок — иначе всё, что ниже,
     * ложь.
     *
     * У S3 на GPIO19/20 висят **два** периферийных блока: USB-OTG и
     * USB-Serial-JTAG. Второй включён с самого сброса и владеет падами по
     * умолчанию; `gpio_config` рядом с ним не хозяин, и чтение показывает
     * состояние чужой периферии, а не то, что на проводах. Первая версия
     * этой проверки читала именно так и уверенно печатала «ничего не
     * подключено» — вывод, на который нельзя было опираться.
     *
     * Лог от этого не пострадает: консоль в проекте намеренно на UART0, а
     * не на USB-CDC — ровно потому, что USB занят ролью.
     */
    usb_serial_jtag_ll_phy_enable_pad(false);

    int dm_down = read_pin(USB_DM_GPIO, false);
    int dm_up = read_pin(USB_DM_GPIO, true);
    int dp_down = read_pin(USB_DP_GPIO, false);
    int dp_up = read_pin(USB_DP_GPIO, true);

    G920_LOGI(M1, "wire check: D- (gpio%d) down=%d up=%d -> %s", USB_DM_GPIO,
              dm_down, dm_up, verdict(dm_down, dm_up));
    G920_LOGI(M1, "wire check: D+ (gpio%d) down=%d up=%d -> %s", USB_DP_GPIO,
              dp_down, dp_up, verdict(dp_down, dp_up));

    if (dp_down == 1 && dm_down == 0) {
        G920_LOGI(M1, "wire check: full-speed device present, wiring looks ok");
    } else if (dm_down == 1 && dp_down == 0) {
        G920_LOGE(M1, "wire check: D+ and D- appear SWAPPED");
    } else if (dm_up == 1 && dp_up == 1) {
        G920_LOGE(M1,
                  "wire check: both lines float — nothing is connected to "
                  "gpio%d/gpio%d at all",
                  USB_DM_GPIO, USB_DP_GPIO);
    } else {
        G920_LOGE(M1,
                  "wire check: lines held low — wires reach something, but it "
                  "has no vbus");
    }
}

/*
 * Активная проба линии: подать уровень и посмотреть, что получилось.
 *
 * Слабая подтяжка (45 кОм) отвечает на вопрос «тянет ли кто-то снаружи», но
 * не отличает **обрыв** от **линии, посаженной на землю** через что-то
 * низкоомное: и там, и там с подтяжкой вниз будет ноль, а с подтяжкой вверх
 * единица только в первом случае — и то если сопротивление к земле велико.
 * Мультиметр здесь тоже плохой свидетель: его 10 МОм против наших 45 кОм
 * всегда покажут ноль на свободном проводе, и человек читает это как
 * «линию кто-то прижал».
 *
 * Проба разрешает спор: пин переводится в push-pull и **сам** выставляет
 * уровень. Если при выданной единице на пине читается ноль — снаружи держат
 * землю, и держат сильно. Если единица — снаружи никого, обрыв. Заодно
 * читается соседняя линия: подтянувшаяся вместе с ведомой значит, что D+ и
 * D− где-то соединены между собой (типовой случай — зарядка вместо хоста).
 *
 * Безопасно: 3.3 В — рабочий уровень самого USB, приёмопередатчику руля это
 * штатный сигнал. На замкнутой на землю линии пин отдаст десятки мА
 * миллисекунду, что в пределах допустимого для вывода S3.
 */
static void probe_line(const char *name, int gpio, int neighbour)
{
    gpio_config_t drive = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    /*
     * Сосед слушается **с подтяжкой вниз**, а не свободным входом.
     *
     * Первая версия слушала свободным, и проба сама себя подловила: вердикт
     * «линии соединены» прыгал от прогона к прогону на одном и том же
     * железе. Висящий вход КМОП рядом с активно перекладываемой дорожкой
     * набирает заряд через ёмкость связи и читается как единица. Подтяжка
     * вниз эту наводку стравливает, а настоящую перемычку push-pull выход
     * перебивает без труда: 45 кОм против единиц ом.
     */
    gpio_config_t listen = {
        .pin_bit_mask = 1ULL << neighbour,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    int high_readback;
    int neighbour_high;
    int low_readback;
    int neighbour_low;

    if (gpio_config(&drive) != ESP_OK || gpio_config(&listen) != ESP_OK) {
        G920_LOGE(M1, "probe %s: gpio config failed", name);
        return;
    }

    (void)gpio_set_level(gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
    high_readback = gpio_get_level(gpio);
    neighbour_high = gpio_get_level(neighbour);

    (void)gpio_set_level(gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    low_readback = gpio_get_level(gpio);
    neighbour_low = gpio_get_level(neighbour);

    gpio_reset_pin((gpio_num_t)gpio);
    gpio_reset_pin((gpio_num_t)neighbour);

    G920_LOGI(M1,
              "probe %s: driven high -> %d, driven low -> %d, neighbour %d/%d",
              name, high_readback, low_readback, neighbour_high,
              neighbour_low);

    /*
     * Перемычка **следует** за ведомой линией: сосед единица при нашей
     * единице и ноль при нашем нуле. Сосед, стоящий единицей в обеих
     * фазах, — не перемычка, а чужая подтяжка: у запитанного руля D+
     * поднят его собственными 1.5 кОм, и они сильнее нашей слушающей
     * подтяжки вниз. Первая версия читала соседа только в верхней фазе и
     * на каждом запуске с живым рулём кричала «линии соединены» — про
     * исправный стенд.
     */
    if (high_readback == 0) {
        G920_LOGE(M1, "probe %s: line is held at ground — short or a dead "
                      "transceiver on the far end",
                  name);
    } else if (neighbour_high == 1 && neighbour_low == 0) {
        G920_LOGE(M1, "probe %s: the other data line follows it — D+ and D- "
                      "are tied together somewhere (a charger does this)",
                  name);
    } else if (neighbour_high == 1) {
        G920_LOGI(M1, "probe %s: the other line is pulled up on its own — a "
                      "powered device holds it",
                  name);
    } else {
        G920_LOGI(M1, "probe %s: line is free — nothing loads it", name);
    }
}

static void probe_lines(void)
{
    probe_line("D+", USB_DP_GPIO, USB_DM_GPIO);
    probe_line("D-", USB_DM_GPIO, USB_DP_GPIO);
}

/*
 * Маяк: плата по очереди держит 3.3 В на D+ и на D−.
 *
 * Проба выше отвечает на вопрос «нагружен ли пин», но не отвечает на
 * главный: **доходит ли провод от пина до дальнего конца**. Прозвонка это
 * тоже не отвечает — она проверяет кусок меди, который человек держит в
 * руках, а не путь от гнезда до кристалла. А путь этот длинный: контакт
 * гнезда, провод, гребёнка, дорожка платы, пад.
 *
 * Маяк проверяет весь путь разом и с той стороны, где стоит человек с
 * тестером. Никакой договорённости о времени не нужно: фазы сменяются сами
 * с постоянным периодом, и на дальнем конце тестер видит, как 3.3 В
 * перебегают с одного контакта на другой каждые три секунды. Не перебегают
 * — провод не доходит, и искать надо между гнездом и ногой.
 *
 * Раз в цикл обе линии отпускаются на 700 мс: это окно, в котором видно
 * подтяжку устройства, если оно наконец появится. Маяк не мешает
 * обнаружению, он лишь откладывает его на секунды.
 *
 * 3.3 В на линии данных — штатный уровень USB (состояние J для full speed),
 * приёмопередатчику на том конце это привычный сигнал.
 */
/*
 * Маяк собирается только по флагу `G920_HOST_BEACON` и по умолчанию
 * выключен. Свою задачу — доказать, что провод доходит от ноги чипа до
 * дальнего конца, — он выполнил 02.08.2026, а в обычной работе он вреден:
 * человек с тестером видит на линиях наши 3.3 В и принимает их за признак
 * подключённого устройства. Понадобится снова после перепайки — собрать с
 * флагом.
 */
#ifdef G920_HOST_BEACON
#define BEACON_DRIVE_MS 3000
#define BEACON_IDLE_MS 700
#endif

/* Пауза между замерами в тихом режиме: линии всё это время отпущены. */
#define QUIET_POLL_MS 2000

/*
 * Отпускает обе линии: вход без подтяжек, высокоомно.
 *
 * Нужно, чтобы **человек с тестером мерил руль, а не плату**. Пока на пинах
 * висит подтяжка вниз 45 кОм, свободный провод читается нулём, а маяк и
 * вовсе сам выдаёт 3.3 В — оба раза человек видит своё измерение как
 * свойство руля, хотя это свойство прошивки. Между опросами линии отпущены,
 * и на них ровно то напряжение, которое создаёт та сторона.
 *
 * Подтяжка вниз включается только на время замера — пять миллисекунд раз в
 * две секунды.
 */
#ifndef G920_VBUS_GPIO /* зовётся только из wait_for_pullup */
static void release_lines(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << USB_DP_GPIO) | (1ULL << USB_DM_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    (void)gpio_config(&cfg);
}
#endif

#ifdef G920_HOST_BEACON
static void beacon_drive(int gpio, int level, uint32_t ms)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    if (gpio_config(&cfg) == ESP_OK) {
        (void)gpio_set_level(gpio, level);
    }
    vTaskDelay(pdMS_TO_TICKS(ms));
    gpio_reset_pin((gpio_num_t)gpio);
}
#endif /* G920_HOST_BEACON */
#endif /* G920_PRESTACK_PROBES */

/*
 * Ждёт, пока на D+ появится подтяжка устройства, и только потом отдаёт пины
 * стеку.
 *
 * Зачем ждать. Стек, запущенный на пустых проводах, видит на плавающих
 * линиях мусор, принимает его за подключение и раз в полсекунды печатает
 * `HUB: Root port reset failed`. Лог в этот момент состоит из одной
 * повторяющейся ошибки, за которой ничего не разглядеть, а главное —
 * ошибка эта **не про руль**, и человека она уводит искать неисправность
 * там, где её нет.
 *
 * Ожидание же делает прошивку пригодной для работы в одиночку: плата стоит
 * включённой, человек в любой момент подаёт питание на руль или на VBUS
 * розетки — и в логе появляется ровно та строка, ради которой всё
 * затевалось, без перепрошивки и без присутствия у стола.
 *
 * Ожидание **бесконечное**, и это выбор по итогам первого прогона: сначала
 * стоял таймаут в минуту, после которого стек поднимался всё равно. Прогон
 * показал две вещи. Признак на D+ точен — стек, поднятый на пустых
 * проводах, не нашёл ничего, то есть подтяжки не было и по его версии.
 * А цена таймаута — `Root port reset failed` раз в полсекунды навсегда, и
 * при работе без человека это часы нечитаемого лога.
 *
 * Обратная сторона названа честно: если руль когда-нибудь окажется на шине
 * без видимой подтяжки, прошивка его не заметит. Тогда виноват признак, и
 * чинить надо признак, а не глушить его таймаутом.
 */
/* Период опроса задаёт маяк: чтение идёт в его паузе. */
#define PULLUP_NOTE_MS 60000
/* Шаг мигания в паузе между замерами: линии всё это время отпущены. */
#define LED_SLICE_MS 250
/*
 * Пауза снятого VBUS. Не сотни миллисекунд, и это замер, а не
 * перестраховка: 22.08.2026 подтяжка руля переживала и 700 мс снятого
 * VBUS, и сброс платы. Руль тянет с VBUS микроамперы — заряд узла сидит
 * там секундами, и отключение, которого руль не заметил, не отключение.
 * Балластный резистор с VBUS на землю у розетки сделает разряд мгновенным —
 * тогда паузу можно будет вернуть к сотням миллисекунд.
 */
#define VBUS_OFF_MS 4000
#ifdef G920_VBUS_GPIO
/* Сам ключ описан ниже, у replug_wheel; нужен он и раньше знакомства. */
static void vbus_set(bool on);
#endif

#if defined(G920_VBUS_GPIO) && defined(G920_PRESTACK_PROBES)
/*
 * Слить VBUS перед подъёмом стека — и измерить, как долго он сливается.
 *
 * Ключ держит розетку обесточенной с самого сброса, но узел VBUS — это
 * ёмкость, из которой руль тянет микроамперы: заряд сидит там секундами.
 * 22.08.2026 подтяжка руля пережила и сброс платы, и 700 мс снятого
 * VBUS — руль «отключения» просто не видел. Поэтому здесь ждётся не время,
 * а факт: D+ отпущен — значит руль действительно остался без питания
 * розетки, и его следующее появление будет настоящим фронтом подключения.
 * Измеренное время печатается: это готовый ответ, нужен ли балластный
 * резистор на VBUS и какой.
 */
static void drain_vbus(void)
{
    uint32_t off_ms = 0;
    int dp;

    vbus_set(false);
    dp = read_pin(USB_DP_GPIO, false);
    while (dp != 0 && off_ms < 20000u) {
        vTaskDelay(pdMS_TO_TICKS(100));
        off_ms += 100;
        dp = read_pin(USB_DP_GPIO, false);
    }
    if (dp == 0) {
        G920_LOGI(M1, "vbus drained: D+ released %u ms into the off window",
                  (unsigned)off_ms);
        /* Хвост: подтяжка падает раньше, чем VBUS дотекает до нуля. */
        vTaskDelay(pdMS_TO_TICKS(500));
    } else {
        G920_LOGW(M1, "D+ is still pulled up %u ms into vbus off — the "
                      "switch is not actually cutting power to the socket "
                      "(wiring, or a second 5V path)",
                  (unsigned)off_ms);
    }
    /* Отпускаем пины: дальше ими распоряжается PHY. */
    gpio_reset_pin(USB_DM_GPIO);
    gpio_reset_pin(USB_DP_GPIO);
}
#endif

#if !defined(G920_VBUS_GPIO) && defined(G920_PRESTACK_PROBES)
static void wait_for_pullup(void)
{
    uint32_t waited_ms = 0;
    uint32_t since_note_ms = PULLUP_NOTE_MS;
    int last_dp = -2;
    int last_dm = -2;
#ifndef G920_HOST_BEACON
    bool blink = false;
#endif

    for (;;) {
        /* Со слабой подтяжкой вниз: единица здесь значит, что снаружи
         * тянут вверх, а это и есть 1.5 кОм внутри запитанного устройства. */
        int dp = read_pin(USB_DP_GPIO, false);
        int dm = read_pin(USB_DM_GPIO, false);

        /*
         * Любое изменение картины — сразу в лог, а не по расписанию.
         * Так плата становится тестером: человек у стола втыкает и
         * вытыкает, подаёт и снимает питание, и видит отклик за четверть
         * секунды, вместо того чтобы сверять показания мультиметра с
         * записями раз в минуту.
         */
        if (dp != last_dp || dm != last_dm) {
            if (last_dp != -2) {
                G920_LOGI(M1, "lines changed: D+ %d -> %d, D- %d -> %d",
                          last_dp, dp, last_dm, dm);
            }
            last_dp = dp;
            last_dm = dm;
            since_note_ms = PULLUP_NOTE_MS;
        }

        if (dp == 1 && dm == 0) {
            G920_LOGI(M1, "D+ pulled up after %u ms — device is powered",
                      (unsigned)waited_ms);
            break;
        }
        if (dm == 1 && dp == 0) {
            /*
             * Подтяжка на D− значит одно из двух, и различить их отсюда
             * нельзя: либо провода перепутаны местами, либо на шине
             * low-speed устройство — мышь или клавиатура тянут именно D−.
             * Второе случается ровно тогда, когда гнездо проверяют чужим
             * устройством, и назвать это «перепутаны провода» значило бы
             * соврать в самый неподходящий момент.
             */
            G920_LOGW(M1,
                      "D- pulled up: either D+/D- are swapped, or this is a "
                      "low-speed device (mouse, keyboard) — starting the host");
            break;
        }
        if (since_note_ms >= PULLUP_NOTE_MS) {
            since_note_ms = 0;
            /* Раз в минуту, пока подтяжки нет: провода могли перепаять, и
             * тогда сменится не уровень, а сама природа отказа. */
            probe_lines();
            G920_LOGI(M1,
                      "waiting %u s: D+ down=%d up=%d (%s), D- down=%d up=%d",
                      (unsigned)(waited_ms / 1000u), dp,
                      read_pin(USB_DP_GPIO, true),
                      verdict(dp, read_pin(USB_DP_GPIO, true)), dm,
                      read_pin(USB_DM_GPIO, true));
            /*
             * Подозреваемые названы поимённо и в порядке частоты, потому что
             * читать этот лог будут ровно тогда, когда подтяжки нет, а
             * догадываться придётся о железе.
             *
             * Общая земля стоит в списке не случайно и не третьей: 04.08.2026
             * стенд оживал от **касания земли type-c**, воткнутого в мак, —
             * то есть у D+/D− не было опоры, и всё, что видела прошивка,
             * было измерением относительно ничего. Отсюда же и весь
             * предыдущий морок: «пока не подключишь мак, руль не
             * калибруется» — мак давал не питание и не сброс, а землю.
             */
            G920_LOGW(M1, "no pull-up: check 5V on pin 1 of the socket, then "
                          "the common ground (board, external 5V and socket "
                          "must share one), then the D+ wire itself");
#ifdef G920_HOST_BEACON
            G920_LOGI(M1,
                      "beacon: 3.3V on D+ for %u s, then on D- for %u s, then "
                      "both released — look for it at the far end",
                      (unsigned)(BEACON_DRIVE_MS / 1000u),
                      (unsigned)(BEACON_DRIVE_MS / 1000u));
#else
            G920_LOGI(M1, "lines are released between polls — a meter now "
                          "reads the far end, not the board");
#endif
        }

#ifdef G920_HOST_BEACON
        beacon_drive(USB_DP_GPIO, 1, BEACON_DRIVE_MS);
        beacon_drive(USB_DM_GPIO, 1, BEACON_DRIVE_MS);
        vTaskDelay(pdMS_TO_TICKS(BEACON_IDLE_MS));
        waited_ms += 2 * BEACON_DRIVE_MS + BEACON_IDLE_MS;
        since_note_ms += 2 * BEACON_DRIVE_MS + BEACON_IDLE_MS;
#else
        release_lines();
        /*
         * Синий мигает всё это время — и это единственный канал наружу,
         * который тут вообще есть.
         *
         * Отказ, о котором речь, случается **когда никто не смотрит**: без
         * мака в передатчике нет и консоли, а лог уходит в UART, который
         * никуда не приходит. Индикатор различает три стадии, и различает их
         * без единого провода: синий — стек ещё не поднят, ждём подтяжку на
         * D+; жёлтый — стек поднят, устройства на шине нет; зелёный — руль
         * открыт. До этой правки первая стадия не показывала ничего, и
         * «плата не дошла до шины» выглядело так же, как «плата мертва».
         */
        for (uint32_t slice = 0; slice < QUIET_POLL_MS / LED_SLICE_MS;
             slice++) {
            blink = !blink;
            g920_board_led_set(blink ? G920_IND_BOOT : G920_IND_OFF);
            vTaskDelay(pdMS_TO_TICKS(LED_SLICE_MS));
        }
        waited_ms += QUIET_POLL_MS;
        since_note_ms += QUIET_POLL_MS;
#endif
    }

    /* Отпускаем пины: дальше ими распоряжается PHY. */
    gpio_reset_pin(USB_DM_GPIO);
    gpio_reset_pin(USB_DP_GPIO);
}
#endif /* !G920_VBUS_GPIO && G920_PRESTACK_PROBES */

/* --- вывод ---------------------------------------------------------------- */

/*
 * Дамп буфера построчно. Готовой функции «весь буфер» в `g920/hexdump.h`
 * нет намеренно — там форматируется одна строка, и это правильно: строки
 * должны идти через тот же приёмник лога, что и всё остальное.
 */
static void dump(const char *what, const uint8_t *data, size_t len)
{
    char line[G920_HEXDUMP_LINE_MAX];
    size_t lines = g920_hexdump_line_count(len);

    G920_LOGI(M1, "%s, %u bytes:", what, (unsigned)len);
    for (size_t i = 0; i < lines; i++) {
        size_t offset = i * G920_HEXDUMP_BYTES_PER_LINE;
        size_t chunk = len - offset;

        if (chunk > G920_HEXDUMP_BYTES_PER_LINE) {
            chunk = G920_HEXDUMP_BYTES_PER_LINE;
        }
        if (g920_hexdump_line(line, sizeof(line), offset, data + offset, chunk)
            > 0) {
            G920_LOGI(M1, "%s", line);
        }
    }
}

/*
 * Дескрипторы выкладываются **и разобранными, и сырыми байтами**.
 *
 * Разбор удобен человеку, но веха собирает первоисточник: в M4 эти самые
 * байты поедут на RX как личность руля, и сверять их придётся побайтово.
 * Печатать только разбор значило бы потерять то, ради чего дамп и делается
 * (И1: не пересобирать чужое представление).
 */
static void report_device(usb_device_handle_t dev)
{
    const usb_device_desc_t *device = NULL;
    const usb_config_desc_t *config = NULL;
    usb_device_info_t info;

    if (usb_host_device_info(dev, &info) == ESP_OK) {
        G920_LOGI(M1, "speed %s, addr %u, cfg %u, max ep0 %u",
                  (info.speed == USB_SPEED_LOW)    ? "low"
                  : (info.speed == USB_SPEED_FULL) ? "full"
                                                   : "high",
                  (unsigned)info.dev_addr,
                  (unsigned)info.bConfigurationValue,
                  (unsigned)info.bMaxPacketSize0);
    }

    if (usb_host_get_device_descriptor(dev, &device) == ESP_OK
        && device != NULL) {
        G920_LOGI(M1,
                  "device: vid %04x pid %04x, bcdDevice %04x, "
                  "class %02x/%02x/%02x, configs %u",
                  (unsigned)device->idVendor, (unsigned)device->idProduct,
                  (unsigned)device->bcdDevice, (unsigned)device->bDeviceClass,
                  (unsigned)device->bDeviceSubClass,
                  (unsigned)device->bDeviceProtocol,
                  (unsigned)device->bNumConfigurations);
        G920_LOGI(M1, "strings: iManufacturer %u, iProduct %u, iSerial %u",
                  (unsigned)device->iManufacturer,
                  (unsigned)device->iProduct,
                  (unsigned)device->iSerialNumber);
        dump("device descriptor", (const uint8_t *)device, device->bLength);
    } else {
        G920_LOGE(M1, "device descriptor unavailable");
    }

    if (usb_host_get_active_config_descriptor(dev, &config) == ESP_OK
        && config != NULL) {
        G920_LOGI(M1, "config: %u interfaces, wTotalLength %u, %u mA",
                  (unsigned)config->bNumInterfaces,
                  (unsigned)config->wTotalLength,
                  (unsigned)config->bMaxPower * 2u);
        /*
         * Целиком, вместе с интерфейсами и эндпоинтами: конфигурация — это
         * цепочка дескрипторов, и обрывать её по bLength первого значило бы
         * выбросить как раз то, что нужно (класс `0xFF/0x47/0xD0`,
         * `bInterval`, наличие аудио-интерфейса).
         */
        dump("config descriptor", (const uint8_t *)config,
             config->wTotalLength);
    } else {
        G920_LOGE(M1, "config descriptor unavailable");
    }
}

/* --- поток GIP: захват интерфейса и опрос IN ------------------------------ */

/*
 * Печать одного сообщения: разобранный заголовок и **всегда** сырые байты.
 *
 * Разбор здесь — удобство для человека, а предмет вехи это байты: в M4 они
 * поедут на RX как личность руля и сверяться будут побайтово (И1). Нагрузка
 * не толкуется вовсе: что означает третий байт Hello — вопрос M4, а не
 * прошивки-пробы.
 */
static bool log_message(const char *dir, const uint8_t *data, size_t len,
                        g920_gip_header_t *out)
{
    g920_gip_header_t header;
    g920_gip_status_t status;

    if (len == 0) {
        /* Не потеря: нулевой пакет в GIP осмыслен (завершающий фрагмент). */
        G920_LOGI(M1, "%s: zero-length packet", dir);
        return false;
    }

    status = g920_gip_header_parse(&header, data, len);
    if (status == G920_GIP_OK) {
        G920_LOGI(M1,
                  "%s: msg %02x (class %u, number %u), flags %02x%s%s%s%s, "
                  "seq %u, payload %u, header %u",
                  dir, (unsigned)header.message_type,
                  (unsigned)g920_gip_data_class(&header),
                  (unsigned)g920_gip_message_number(&header),
                  (unsigned)header.flags,
                  g920_gip_is_fragment(&header) ? " frag" : "",
                  g920_gip_is_initial_fragment(&header) ? " init" : "",
                  g920_gip_is_system(&header) ? " sys" : "",
                  g920_gip_wants_ack(&header) ? " acme" : "",
                  (unsigned)header.sequence, (unsigned)header.payload_length,
                  (unsigned)header.header_length);
    } else {
        G920_LOGW(M1, "%s: header does not parse (%s)", dir,
                  g920_gip_status_name(status));
    }
    dump(dir, data, len);

    if (status == G920_GIP_OK && out != NULL) {
        *out = header;
    }
    return status == G920_GIP_OK;
}

/*
 * Колбэк IN. Зовётся из usb_host_client_handle_events, то есть из главного
 * цикла — своей задачи у него нет, и гонок с печатью тоже.
 *
 * Каждое завершение — это реальные данные: на interrupt IN устройству
 * нечего сказать → оно отвечает NAK, а NAK транзакцию не завершает.
 * Поэтому пустого потока в логе не будет, и печатать можно всё подряд.
 */
static bool host_send(const char *what, const uint8_t *data, size_t len);

/*
 * Очередь исходящих.
 *
 * Прямо из колбэка отправлять нельзя: OUT-передача одна, её завершение
 * приходит тем же потоком событий, и ждать его внутри колбэка — это ждать
 * самого себя. А на одно принятое сообщение ответов бывает два сразу
 * (подтверждение и следующая команда). Поэтому колбэк только складывает,
 * а отправляет главный цикл.
 *
 * Четыре ячейки — с запасом: больше двух за раз последовательность не
 * рождает, а переполнение всё равно названо вслух, а не проглочено.
 */
#define OUT_QUEUE_LEN 16

/*
 * Своя запись очереди, а не `g920_gip_host_packet_t`.
 *
 * У того буфер 20 байт — ровно под то, что собирает последовательность
 * знакомства: заголовок плюс девятибайтное подтверждение. Через эту же
 * очередь идут сообщения консоли, а самое важное из них — Host Hello
 * аутентификации — весит 62 байта.
 *
 * Цена ошибки была велика и невидима: короткие управляющие сообщения
 * проходили, длинный Host Hello **молча отбрасывался** проверкой длины, и
 * руль выглядел как устройство, не умеющее аутентификацию. Он её умеет —
 * ему просто ни разу не задали вопрос.
 *
 * Отсюда размер: столько же, сколько эндпоинт, — больше в один пакет USB
 * всё равно не влезет.
 */
typedef struct {
    uint8_t data[EP_BUF_MAX];
    uint8_t length;
    const char *what;
} out_packet_t;

static out_packet_t s_out_queue[OUT_QUEUE_LEN];
static volatile uint8_t s_out_head;
static volatile uint8_t s_out_tail;
static volatile uint32_t s_out_dropped;
/*
 * Очередь к рулю живёт в двух задачах: наполняют её и колбэк линка (задача
 * Wi-Fi — силы, security), и главный цикл (ACK'и знакомства), разгружают
 * оба плюс завершение отправки. На двух ядрах S3 это настоящая гонка, а не
 * теоретическая: два производителя без замка пишут один слот, а два вызова
 * `try_send_next` сдают один и тот же transfer дважды. Спинлок, потому что
 * контексты — разные ядра, и запрета прерываний мало.
 */
static portMUX_TYPE s_out_lock = portMUX_INITIALIZER_UNLOCKED;

/* Был ли пир на прошлом такте — ловим появление, а не наличие. */
static bool s_peer_seen;
/* Переподключали ли руль по факту поднявшейся связки. Один раз за загрузку. */
static bool s_link_replug_done;

/*
 * Руль жив? Два независимых свидетельства, и достаточно любого: наше
 * знакомство дошло до active — или отчёты ввода идут прямо сейчас, кто бы
 * их ни вызвал. Второе важнее первого: рулём правит и консоль тоже.
 */
static bool wheel_is_alive(uint32_t now_ms);

/* Свой счёт номеров для типов, которые шлём рулю и мы, и консоль. */
static uint8_t s_seq_set_state;
static uint8_t s_seq_initial_reports;

static void queue_bytes(const uint8_t *data, size_t len, const char *what)
{
    uint8_t next;
    bool full = false;

    if (len == 0 || len > EP_BUF_MAX) {
        s_out_dropped++;
        G920_LOGE(M1, "out: %s is %u bytes, does not fit", what, (unsigned)len);
        return;
    }
    /*
     * Руля на шине нет — копить нечего и некому отдавать.
     *
     * Без этого сообщения консоли набиваются в очередь до переполнения, и
     * дальше каждое рождает строку `out queue full` в журнале: на живом
     * стенде 03.08.2026 это шло по пять строк в секунду и делало журнал
     * нечитаемым ровно тогда, когда он нужнее всего — при разборе «почему
     * руля нет». Отказ считается, но печатается редко.
     */
    if (s_device == NULL) {
        s_out_dropped++;
        if ((s_out_dropped % 200u) == 1u) {
            G920_LOGW(M1, "no wheel on the bus — dropping %s (%u so far)",
                      what, (unsigned)s_out_dropped);
        }
        return;
    }
    /* Индексы, слот и счётчики номеров — только под замком; печать —
     * только снаружи: строка в UART под спинлоком остановила бы второе
     * ядро на миллисекунды. */
    portENTER_CRITICAL_SAFE(&s_out_lock);
    next = (uint8_t)((s_out_head + 1) % OUT_QUEUE_LEN);
    if (next == s_out_tail) {
        full = true;
    } else {
    memcpy(s_out_queue[s_out_head].data, data, len);
    s_out_queue[s_out_head].length = (uint8_t)len;
    s_out_queue[s_out_head].what = what;
    /*
     * ⚠ Номера сообщений, которые шлём рулю **и мы, и консоль**, ставим
     * здесь — иначе в один пул пишут два источника.
     *
     * Эта болезнь за 03.08.2026 оказывалась причиной дважды, и оба раза
     * симптом был один: сообщение доезжает, а собеседник его не исполняет,
     * потому что видит откат номера назад и вправе считать это повтором.
     *
     * Здесь она проявляется так. Во время своей загрузки передатчик сам
     * спрашивает у руля начальные отчёты, и руль отвечает — в одном окне
     * тринадцать раз (`21=13`). Потом ту же просьбу шлёт консоль, со своим
     * счётом, начинающимся с малых номеров. Для руля это откат в том же
     * пуле, и он молчит. Итог измерен: до консоли не дошло **ни одной**
     * статической конфигурации, а без неё (`H001861`) ей нечем читать
     * отчёт состояния `0x20` — отсюда «работает только кнопка Xbox», ведь
     * `0x07` системное и метаданных не требует.
     *
     * Перенумеровываются только те типы, которые передатчик генерирует
     * сам: `0x05` Set Device State и `0x0A` Initial Reports Request.
     * Security (`0x06`) он только возит, источник у неё один — и номер в
     * ней связан с подтверждениями, которые консоль сверяет по нему же.
     *
     * Байт 2 — GIP Sequence ID (`H001419`, GIP Message Header). 0x00
     * зарезервирован.
     */
    if (len > 2) {
        uint8_t *out = s_out_queue[s_out_head].data;

        if (out[0] == 0x05u || out[0] == 0x0Au) {
            uint8_t *seq = (out[0] == 0x05u) ? &s_seq_set_state
                                             : &s_seq_initial_reports;

            (*seq)++;
            if (*seq == 0u) {
                *seq = 1u;
            }
            out[2] = *seq;
        }
    }
    s_out_head = next;
    }
    portEXIT_CRITICAL_SAFE(&s_out_lock);

    if (full) {
        s_out_dropped++;
        G920_LOGE(M1, "out queue full, dropped %s", what);
    }
}

static void queue_packet(const g920_gip_host_packet_t *packet)
{
    queue_bytes(packet->data, packet->length, packet->what);
}

/*
 * Последовательность обмена: Hello → запрос метаданных → Start.
 *
 * Логика живёт в `common/gip` и проверена юнит-тестами на векторах спеки —
 * здесь только связь с USB: что пришло, отдать ей; что она вернула,
 * положить в очередь.
 */
static g920_gip_host_t s_host;

/*
 * Блоб метаданных руля. 2 КБ — с запасом: в трассе спеки он 186 байт, у
 * настоящего устройства крупнее, но не на порядки. Переполнение сборщик
 * скажет вслух (OVERFLOW), а не обрежет молча.
 */
static uint8_t s_metadata[2048];
static bool s_metadata_logged;

/*
 * Личность руля: то, ради чего веха M1 и делалась.
 *
 * Контейнер собирается здесь, на TX, из снятого с живого устройства — и
 * никогда не пишется в прошивку RX руками. Это инвариант И2: у донгла не
 * должно быть ни одного захардкоженного байта G920, всё приходит от TX и
 * лежит в NVS. Отсюда же и порядок работ: сначала снять, потом передать.
 *
 * Секции ровно те, что предусмотрел `g920/identity.h`, и все они у нас
 * теперь есть: дескриптор устройства, конфигурация целиком, строки,
 * содержимое Hello и блоб метаданных. Не хватает только HID report
 * descriptor — он снимается в режиме `c262`, это отдельный шаг вехи.
 */
#define IDENTITY_BYTES 3072

static uint8_t s_identity_buffer[IDENTITY_BYTES];
static g920_identity_t s_identity;
static bool s_identity_saved;

/* Содержимое Hello: копится при первом же сообщении 0x02. Позже его не
 * достать — устройство шлёт Hello только в Arrival. */
static uint8_t s_hello[64];
static uint16_t s_hello_len;

/* Сколько сообщений какого номера пришло от руля. */
static volatile uint32_t s_wheel_msgs[64];

/*
 * Когда руль в последний раз прислал отчёт ввода.
 *
 * Это **единственный признак «руль жив», не зависящий от нашего разговора с
 * ним**, и он понадобился дорогой ценой. Рулём может править консоль через
 * донгл: тогда наш собственный `gip_host` так и остаётся в `arrival` — Hello
 * он не видел, — а руль при этом включён, откалиброван и работает. Судить о
 * руле по своему состоянию знакомства в этом случае значит объявить
 * исправное устройство мёртвым и начать его чинить. Ровно это и вышло
 * 04.08.2026: руль, подключённый к боксу и работающий, переподключался по
 * кругу и калибровался снова и снова.
 *
 * Отчёты ввода идут с частотой 250 Гц и не зависят ни от чьего знакомства:
 * есть они — устройство включено, и трогать его нельзя ничем.
 */
static volatile uint32_t s_last_input_ms;
#define WHEEL_ALIVE_MS 2000

/*
 * Буфер сообщений руля, ждущих отправки в линк.
 *
 * Надёжная дисциплина линка держит четыре кадра, а руль в разгар знакомства
 * сыплет фрагменты раз в 4 мс. Без буфера лишнее просто не влезало и
 * пропадало молча: 02.08.2026 руль отдал 48 фрагментов метаданных, а до
 * консоли доехало 13 — и та бесконечно просила их заново.
 *
 * Шестнадцать ячеек по 64 байта: это вся пачка фрагментов метаданных с
 * запасом. Переполнение считается и печатается — потеря в туннеле обязана
 * быть видимой, иначе она выглядит как молчание руля.
 */
#define TUNNEL_QUEUE_LEN 128

static struct {
    uint8_t data[EP_BUF_MAX];
    uint8_t length;
} s_tunnel_queue[TUNNEL_QUEUE_LEN];
static volatile uint8_t s_tunnel_head;
static volatile uint8_t s_tunnel_tail;
static volatile uint32_t s_tunnel_dropped;
static volatile uint32_t s_tunnel_sent;
static uint16_t s_tunnel_seq;
/*
 * Просьба переподнять руль по USB. Ставится из колбэка линка (задача Wi-Fi),
 * исполняется главным циклом: трогать стек хоста из чужой задачи нельзя, а
 * передёргивание порта к тому же спит 300 мс.
 */
static volatile bool s_relaunch_wheel;
/* Когда переподнимали в последний раз: второе передёргивание подряд не
 * успевает и оставляет порт погашенным. */
static uint32_t s_relaunch_at_ms;
#define RELAUNCH_COOLDOWN_MS 3000
/* Свой счёт кадров ввода: GIP-овский номер руля для этого не годится, см.
 * место отправки. */
static uint16_t s_input_seq;

/*
 * Мёртвая рука: снять силы, когда донгл пропал.
 *
 * Опасность настоящая и односторонняя. Консоль задаёт силу и ждёт, что
 * следующее сообщение её сменит; если линк оборвался посреди удара, руль
 * останется с приложенной силой навсегда — сменить её станет некому, а
 * заклиненный мотор греется.
 *
 * Снимается штатно и без выдумок: `H001861`, Set Equations States (`0x0C`),
 * таблицы 331–333. Все шестнадцать полубайтов состояний равны `0xF`
 * («Ignore — do not change state»), то есть сами эффекты не трогаются, а
 * последний байт — Loop State: `0x02` = Forces Off, «no forces should be
 * output to the motors». Возврат — тот же кадр с `0x00` = Normal, «the only
 * state where forces should be sent to the motors».
 *
 * Признак обрыва — потеря пира линком, а не тишина в силах: пауза в силах
 * это норма (меню, стоянка), и снимать по ней значило бы глушить обратную
 * связь до следующего Loop State от консоли, которого она не пришлёт.
 *
 * **Окно своё, а не унаследованное от пиринга — и выведено из замера.**
 *
 * Первая версия смотрела на потерю пира, и живой обрыв 03.08.2026 показал
 * цену: `peer silent for 10000 ms, released` — ровно столько руль простоял
 * бы с приложенной силой. К «немедленно» из И3 это отношения не имеет.
 *
 * По прикладному трафику окно короче полутора секунд не построить:
 * максимум нормального промежутка между кадрами от донгла составил 1257 мс
 * (меню, стоянка, тишина в силах). Поэтому донгл шлёт пульс раз в 100 мс,
 * а здесь стоит 300 мс — окно переживает две потерянные подряд посылки.
 */
#define DEADMAN_SILENCE_MS 300
#define FFB_LOOP_FORCES_OFF 0x02u
#define FFB_LOOP_NORMAL 0x00u

static bool s_ffb_seen;
static bool s_forces_off;
/*
 * Метка живости — **32 бита в миллисекундах**, а не 64 в микросекундах.
 *
 * Пишет её колбэк линка из задачи Wi-Fi, читает главный цикл из своей.
 * 64-битное слово на Xtensa читается и пишется двумя половинками, и они
 * разъезжаются: 03.08.2026 это дало `dongle silent 1271310319 ms` —
 * полтора месяца тишины на ровном месте, ложное снятие сил и тут же
 * возврат. Разность беззнаковых 32-битных переживает переполнение сама
 * (раз в 49 суток), а миллисекунды для окна в 300 мс — точность с запасом.
 */
static volatile uint32_t s_last_link_ms;
/* Наш вклад в задержку сил: от кадра из радио до сдачи рулю. */
static volatile uint32_t s_ffb_lat_sum;
static volatile uint32_t s_ffb_lat_n;
static volatile uint32_t s_ffb_lat_max;
static uint8_t s_loop_seq;

static void queue_bytes(const uint8_t *data, size_t len, const char *what);
static void try_send_next(void);

static void send_loop_state(uint8_t state, const char *what)
{
    uint8_t msg[13] = {
        0x0Cu, 0x00u, 0x00u, 0x09u,
        /* шестнадцать состояний эффектов, все «не менять» */
        0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu,
        0x00u /* Loop State */
    };

    s_loop_seq = (uint8_t)(s_loop_seq + 1u);
    if (s_loop_seq == 0u) {
        s_loop_seq = 1u;
    }
    msg[2] = s_loop_seq;
    msg[12] = state;
    queue_bytes(msg, sizeof(msg), what);
    try_send_next();
}
/* Что уходит рулю от консоли, по номерам сообщений: «START не доходит» и
 * «руль его игнорирует» — разные диагнозы. */
static volatile uint32_t s_to_wheel_msgs[64];

static void tunnel_push(const uint8_t *data, size_t len)
{
    uint8_t next = (uint8_t)((s_tunnel_head + 1) % TUNNEL_QUEUE_LEN);

    if (len > EP_BUF_MAX || next == s_tunnel_tail) {
        s_tunnel_dropped++;
        return;
    }
    memcpy(s_tunnel_queue[s_tunnel_head].data, data, len);
    s_tunnel_queue[s_tunnel_head].length = (uint8_t)len;
    s_tunnel_head = next;
}

/*
 * Отдаёт линку **по одному кадру за раз**, дожидаясь подтверждения.
 *
 * Порядок здесь важнее скорости. Сообщения знакомства — фрагменты одного
 * блоба, и собираются они по смещениям: пришедший раньше своего соседа
 * фрагмент это дыра, а дыра заставляет консоль просить блоб заново. Надёжная
 * дисциплина порядок не обещает — повторённый кадр приходит после тех, что
 * ушли следом за ним, а повторы были: три за полминуты.
 *
 * Цена — круг подтверждения на кадр, порядка трёх миллисекунд при замеренном
 * RTT 2.42 мс. Руль отдаёт фрагменты раз в 4 мс, так что труба успевает.
 */
/*
 * Аутентификация возится **одним выстрелом**, без надёжной дисциплины линка.
 *
 * Причина измерена: handshake — это сотни сообщений пачками, и надёжный
 * слой поверх него захлёбывался (316 повторов, два брошенных кадра за
 * минуту), а брошенный кадр рвёт обмен целиком. Двойная надёжность здесь
 * лишняя: у самого GIP есть своя дисциплина — ACME с подтверждением и
 * повтор неподтверждённого, — и обе стороны ею пользуются. Потерянное
 * сообщение переспросит консоль или руль, как они делают это и на проводе.
 *
 * Порядок сохраняется сам: отправка идёт сразу и в порядке поступления.
 */
static void tunnel_drain(void)
{
    while (s_tunnel_tail != s_tunnel_head) {
        /* Номер у каждого кадра свой: приёмник линка отсеивает дубликаты
         * по номеру, и с постоянным нулём всё, кроме первого кадра,
         * отбрасывалось как повтор. */
        if (g920_link_send(G920_FRAME_AUTH, s_tunnel_seq++, 0,
                           s_tunnel_queue[s_tunnel_tail].data,
                           s_tunnel_queue[s_tunnel_tail].length)
            != G920_LINK_OK) {
            return;
        }
        s_tunnel_tail = (uint8_t)((s_tunnel_tail + 1) % TUNNEL_QUEUE_LEN);
        s_tunnel_sent++;
    }
}

/*
 * Режим трубы.
 *
 * Пока личности нет, TX **сам** знакомится с рулём: просит метаданные,
 * запускает его и тем самым снимает личность. Это веха M1, и без неё донглу
 * нечем представляться.
 *
 * Как только личность снята, местное знакомство становится вредным. Руль,
 * запущенный нами, живёт в **нашей** сессии: его нумерация, его состояние
 * security — всё привязано к хосту, который его поднял. Консоль ведёт свой
 * разговор, и её security-сообщения попадают в чужую беседу — 02.08.2026
 * это видно прямо: руль ответил нулём на 34 доехавших сообщения, хотя
 * потерь на линке не было ни одной.
 *
 * Поэтому со второго запуска TX не говорит с рулём ни слова от себя: всё,
 * что приходит от консоли, уходит рулю вербатим, всё, что отвечает руль, —
 * консоли. Тогда сессия у них одна, и security оказывается там, где ей место
 * (И1: туннель не пересобирает чужое представление).
 */
static bool s_tunnel;

/*
 * Опыт: отвечает ли руль на подтверждение, если попросить его флагом ACME.
 *
 * Вопрос, который нельзя решить наблюдением за консолью: руль молчит на
 * аутентификацию потому, что не умеет её, или потому, что вообще не
 * подтверждает чужие сообщения? Спека требует подтверждать **любое**
 * сообщение с ACME, независимо от содержимого.
 *
 * Поэтому шлются два сообщения подряд, оформленные одинаково:
 *   1. запрос начальных отчётов (0x0A) — на него руль заведомо отвечает;
 *   2. Host Hello аутентификации (0x06) в раскладке из `xone`.
 * Оба с ACME. Разный ответ на них и будет ответом на вопрос.
 *
 * Ничего, что двигает мотор, здесь нет: 0x0A с типом 0x00 — читающий,
 * 0x06 — криптография.
 */
static bool s_probe_done;
static uint64_t s_probe_at_us;

/*
 * Включение руля так, как это делает консоль.
 *
 * Строка из GP2040-CE (`XBOXONE_POWER_ON`) и joypad-os; наша собственная
 * трасса подтверждает её живьём — консоль шлёт ровно `05 20 02 0f 06 62 45
 * … 1f`, 19 байт. Однобайтный `Set Device State: Start` из спеки руля руль
 * тоже принимает и даже начинает слать ввод, но аутентификацию после него
 * **не открывает**: joypad-os про это пишет, что источник, не прошедший
 * настоящее включение, auth игнорирует и больше не восстанавливается.
 */
static const uint8_t POWER_ON[] = { 0x05, 0x20, 0x02, 0x0F, 0x06, 0x62, 0x45,
                                    0xB8, 0x77, 0x26, 0x2C, 0x55, 0x53, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x1F };
static const uint8_t POWER_ON_SINGLE[] = { 0x05, 0x20, 0x03, 0x01, 0x00 };

static bool s_wheel_ready;

/*
 * Знакомство с рулём **не имеет права кончаться тупиком**, а имело их два.
 *
 * Оба видны прямо в `common/gip/gip_host.c` и оба выглядят одинаково
 * снаружи: руль на шине, опрос идёт, счётчики растут — а включения нет, то
 * есть нет калибровочного проворота и нет сил. Ровно эта жалоба и звучит:
 * «пока не подключишь мак, руль не калибруется и не работает».
 *
 *   1. **Arrival навсегда.** Пока не пришёл Hello, `g920_gip_host_tick`
 *      не делает ничего: у состояния Arrival нет ни таймаута, ни повтора.
 *      Hello руль шлёт один раз, и если он ушёл мимо нас — мы захватили
 *      интерфейс позже, чем руль представился, или сообщение потерялось —
 *      второй раз его никто не пришлёт. Ждать нечего, а мы ждём вечно.
 *
 *   2. **Metadata навсегда.** Запрос метаданных повторяется четыре раза
 *      (`G920_GIP_METADATA_RETRY_MAX`, из спеки), после чего тик молча
 *      возвращает ноль. Спека на этом месте велит хосту **пометить
 *      устройство на удаление**, то есть начать сначала; у нас же после
 *      четвёртой попытки не происходит больше ничего и никогда. Два
 *      секунды тишины на холодном старте — а руль после подачи питания
 *      готов не мгновенно — и мост стоит мёртвым до перезагрузки руками.
 *
 * Отсюда лестница. Сперва просим руль начать сначала — `Set Device State:
 * Reset` возвращает устройство в Arrival, откуда оно снова шлёт Hello, и
 * своё знакомство мы тоже начинаем с чистого листа (иначе счёт запросов
 * метаданных уже упёрся в предел). Если руль на просьбу не отзывается —
 * свежая энумерация: она заведомо возвращает его в Arrival, потому что это
 * сброс шины, а не сообщение, которое можно проигнорировать.
 *
 * Лестница **не кончается**, как и поиск устройства на шине: сдавшийся
 * мост оставляет человека с мёртвым рулём, а один круг стоит восьми секунд
 * и ничего не ломает. Печатается только начало — дальше редко.
 */
/*
 * Порог считается по чужим срокам, а не назначается на глаз: вмешиваться
 * можно только там, где `gip_host` уже истратил всё своё. Он отводит на
 * метаданные четыре запроса с шагом 500 мс (последний уходит на 1500-й
 * миллисекунде и ждёт ответа до 2000-й), а собранным метаданным — ещё
 * 400 мс на completion-фрагмент. Итого 2.4 с, и всё это время знакомство
 * **идёт**, а не стоит. 3 секунды — первый момент, когда молчание уже точно
 * означает тупик.
 */
#define HANDSHAKE_STALL_MS 3000
#define HANDSHAKE_NUDGES_MAX 3
/* Сколько кругов идём бодро, прежде чем сбавить до одного раза в пять
 * минут: каждый круг — это калибровочный проворот, то есть износ. */
#define HANDSHAKE_ROUNDS_BRISK 3
#define HANDSHAKE_STALL_SLOW_MS 300000

static g920_gip_host_state_t s_handshake_state;
static uint32_t s_handshake_at_ms;
static uint32_t s_handshake_nudges;
static uint32_t s_handshake_rounds;

/*
 * Отправка личности на RX — вторая половина M4.
 *
 * **Частями, а не одним кадром.** Надёжная дисциплина линка возит не больше
 * `G920_RELIABLE_PAYLOAD_MAX` = 512 байт (это её слот, а не ограничение
 * радио), а личность G920 весит 655 и вырастет ещё на HID report descriptor.
 * Первая версия слала одним куском и честно получала `QUEUE_FULL`.
 *
 * Заголовок части — четыре байта: смещение и полная длина. Не номер части,
 * а именно смещение: приёмнику тогда всё равно, каким размером резал
 * отправитель, и куски разного размера не ломают сборку.
 *
 * Дисциплина надёжная: личность — не то, что можно потерять молча, повторами
 * занимается линк. Следующая часть уходит только после того, как предыдущая
 * принята в очередь, — иначе мы просто переполним её сами.
 */
/* Тот же код, что шлёт донгл: см. `main_gip.c`. */
#define CONTROL_NEED_IDENTITY 0x01u
/* Те же коды, что у донгла: см. `main_gip.c`. */
#define CONTROL_RESET_WHEEL 0x02u
/*
 * Руль поднялся и стоит в Arrival — донгл может звать консоль.
 * Передатчик шлёт это по Announce руля, один раз на его сессию.
 */
#define CONTROL_WHEEL_READY 0x03u

#define IDENTITY_SEND_RETRY_MS 200
#define IDENTITY_LINK_CHUNK 508
#define IDENTITY_LINK_HEADER 4

static bool s_identity_sent;
static uint64_t s_identity_try_us;
static size_t s_identity_offset;
static uint32_t s_identity_stalls;

static void try_send_next(void);
static void queue_packet(const g920_gip_host_packet_t *packet);

static void link_frame(void *ctx, const g920_frame_t *frame,
                       const uint8_t *peer_mac, g920_rx_verdict_t verdict)
{
    (void)ctx;
    (void)peer_mac;
    (void)verdict;
    if (frame == NULL) {
        return;
    }
    /*
     * Отметка живости — **до всякого разбора**: мёртвой руке важно, что
     * донгл вообще говорит, а не что именно он сказал. Пульс попадает сюда
     * же и своей обработки не требует.
     */
    s_last_link_ms = (uint32_t)(g920_timestamp_us() / 1000u);
    /*
     * Пульс отработан и **не печатается**.
     *
     * Он приходит десять раз в секунду, а строка в UART на 115200 стоит
     * миллисекунды в том же потоке, через который идут повторы линка и силы.
     * Печать в горячем пути в этом проекте уже дважды управляла временем на
     * шине — сорвала знакомство с рулём в M1 и уронила главный цикл в
     * watchdog в M8. Заводить третий такой источник, да ещё периодический,
     * нельзя. Живость и без того видна: оба срабатывания мёртвой руки
     * печатаются.
     */
    if (frame->type == G920_FRAME_ALIVE) {
        return;
    }
    /*
     * Единственное, что TX слушает в этой вехе, — просьбу прислать личность.
     * Донгл переживает передатчик: его вынимают из консоли и втыкают снова,
     * а руль всё это время стоит на месте. Без просьбы личность ловилась бы
     * в единственное окно после включения обоих.
     */
    /*
     * Security от консоли: отдать рулю **вербатим**, без единого толкования.
     * Это и есть passthrough вехи M8 — ключ живёт в крипточипе руля, и
     * никакой донгл ответить за него не может.
     */
    /*
     * Силы от консоли — рулю, немедленно и вербатим.
     *
     * Свой тип кадра, свежая дисциплина: повторять силу нельзя, она к
     * моменту повтора уже неверна. Отдаётся тем же путём, что и всё
     * остальное, но **впереди** очереди знакомства не лезет: очередь
     * шестнадцатиместная и в работе почти пуста.
     *
     * ⚠ Мёртвой руки здесь нет: оборвётся линк с приложенной силой — руль
     * останется с ней. Это M10.
     */
    if (frame->type == G920_FRAME_FFB) {
        /*
         * Замер собственного вклада в задержку сил: от прихода кадра из
         * радио до сдачи пакета рулю.
         *
         * Нужен, чтобы про наш тракт можно было говорить числом, а не
         * мнением. Человек за рулём 03.08.2026: «обратная связь срабатывает
         * раньше события». Раньше консоли мы отдать не можем — тракт только
         * добавляет, — но сколько именно добавляет, до сих пор никто не
         * мерил.
         */
        uint32_t at_us = (uint32_t)g920_timestamp_us();

        s_ffb_seen = true;
        queue_bytes(frame->payload, frame->length, "ffb");
        try_send_next();
        {
            uint32_t took = (uint32_t)g920_timestamp_us() - at_us;

            s_ffb_lat_sum += took;
            s_ffb_lat_n++;
            if (took > s_ffb_lat_max) {
                s_ffb_lat_max = took;
            }
        }
        return;
    }

    if (frame->type == G920_FRAME_AUTH) {
        {
            queue_bytes(frame->payload, frame->length, "auth passthrough");
            try_send_next();
            {
                g920_gip_header_t hdr;

                if (g920_gip_header_parse(&hdr, frame->payload, frame->length)
                    == G920_GIP_OK) {
                    s_to_wheel_msgs[hdr.message_type & 0x3Fu]++;
                }
            }
            /*
             * Одна строка заголовка на сообщение: руль на security молчит, и
             * первое, что надо увидеть, — что именно мы ему отдаём. Сырьё
             * целиком лежит в трассе, здесь только шапка.
             */
            {
                char line[G920_HEXDUMP_LINE_MAX];

                if (g920_hexdump_line(line, sizeof(line), 0, frame->payload,
                                      (frame->length < 16) ? frame->length : 16)
                    > 0) {
                    G920_LOGI(M1, "auth to wheel: %s", line);
                }
            }
        }
        return;
    }

    if (frame->type == G920_FRAME_CONTROL && frame->length >= 1
        && frame->payload[0] == CONTROL_RESET_WHEEL) {
        /*
         * Единственная команда, которую TX сочиняет сам в режиме трубы, и
         * она из спеки дословно: Set Device State: Reset возвращает
         * устройство в Arrival, откуда оно снова шлёт Hello. Без неё вторая
         * консоль ждёт представления, которое руль уже сделал первой.
         */
        static const uint8_t reset[] = { 0x05, 0x20, 0x01, 0x01, 0x07 };

        queue_bytes(reset, sizeof(reset), "set device state: reset");
        try_send_next();
        /*
         * ⚠ И **переподнять руль по USB**, а не ограничиться командой GIP.
         *
         * Измерено 03.08.2026: команда доходит (RX просит дважды — TX
         * исполняет дважды), но руль из состояния «между Arrival и Idle» по
         * ней не выбирается: за двадцать минут от него пришло 451 сообщение
         * вместо тысяч, из них 132 отчёта ввода, и на security он не отвечал
         * ни разу. Каждый раз помогал только сброс передатчика, то есть
         * свежая энумерация.
         *
         * Спека даёт `Set Device State: Reset` как способ вернуть устройство
         * в Arrival, и мы его шлём — но конкретный руль на него не
         * отзывается. Поэтому следом идёт то, что заведомо работает:
         * передёргивание корневого порта. Обе меры вместе, а не вместо друг
         * друга: команда протокола дешевле и, если руль её однажды послушает,
         * переподнятие просто не понадобится.
         */
        s_relaunch_wheel = true;
        return;
    }

    if (frame->type == G920_FRAME_CONTROL && frame->length >= 1
        && frame->payload[0] == CONTROL_NEED_IDENTITY) {
        if (s_identity_saved) {
            s_identity_sent = false;
            s_identity_offset = 0;
            s_identity_try_us = 0;
            G920_LOGI(M1, "link: dongle asks for identity, sending again");
        } else {
            G920_LOGW(M1, "link: dongle asks for identity, but none captured");
        }
        return;
    }
    G920_LOGI(M1, "link: frame %s, %u bytes",
              g920_frame_type_name(frame->type), (unsigned)frame->length);
}

static void send_identity(void)
{
    uint64_t now = g920_timestamp_us();
    g920_link_status_t status;

    if (s_identity_sent || !s_identity_saved) {
        return;
    }
    if (s_identity_try_us != 0
        && (now - s_identity_try_us) < (uint64_t)IDENTITY_SEND_RETRY_MS * 1000u) {
        return;
    }
    s_identity_try_us = now;

    {
        size_t total = g920_identity_size(&s_identity);
        size_t chunk = total - s_identity_offset;
        uint8_t frame[IDENTITY_LINK_HEADER + IDENTITY_LINK_CHUNK];

        if (chunk > IDENTITY_LINK_CHUNK) {
            chunk = IDENTITY_LINK_CHUNK;
        }
        frame[0] = (uint8_t)(s_identity_offset & 0xFFu);
        frame[1] = (uint8_t)((s_identity_offset >> 8) & 0xFFu);
        frame[2] = (uint8_t)(total & 0xFFu);
        frame[3] = (uint8_t)((total >> 8) & 0xFFu);
        memcpy(frame + IDENTITY_LINK_HEADER, s_identity_buffer + s_identity_offset,
               chunk);

        status = g920_link_send_reliable(G920_FRAME_DESCRIPTOR, frame,
                                         (uint16_t)(IDENTITY_LINK_HEADER + chunk));
        if (status == G920_LINK_OK) {
            s_identity_offset += chunk;
            G920_LOGI(M1, "identity: sent %u of %u bytes",
                      (unsigned)s_identity_offset, (unsigned)total);
            if (s_identity_offset >= total) {
                s_identity_sent = true;
            }
        } else {
            /* Печатаем редко: очередь бывает занята повторами, и это
             * нормальная работа линка, а не отказ. */
            if ((s_identity_stalls++ % 25u) == 0u) {
                G920_LOGW(M1, "identity: link busy (%s), retrying",
                          (status == G920_LINK_NO_PEER) ? "no peer" : "queue");
            }
        }
    }
}

/*
 * Трасса обмена в PSRAM.
 *
 * Прямая печать в UART в горячем пути **управляла временем на шине**: на
 * 115200 один 64-байтный пакет хексдампом занимает ~45 мс, а протокол
 * отводит на подтверждение 100 мс. Живой руль показал обе стадии этой
 * болезни — сперва бросал знакомство на полпути, а когда приём ускорили,
 * переполнялась очередь ответов.
 *
 * Поэтому в горячем пути только memcpy в кольцо, а печать — после того,
 * как шина затихнет. Ровно для этого `common/trace` и писался в M2, там же
 * записано, что подключается он в M1.
 *
 * Политика KEEP_OLDEST: в полном дампе ценно начало, и потерянный хвост
 * честнее затёртого знакомства. Потери видны счётчиком refused.
 */
/*
 * Трасса трафика — **инструмент отладки, и на проде её нет вовсе**.
 *
 * Она пишет в PSRAM каждый пакет, пришедший с руля, то есть memcpy в
 * горячем пути двести с лишним раз в секунду, плюс четверть мегабайта
 * PSRAM под кольцо. Пока за ней кто-то смотрит, это честная цена; когда
 * смотреть некому — чистая трата, а горячий путь в этом проекте уже дважды
 * ломал обмен (сорванное знакомство в M1, watchdog в M8).
 *
 * `G920_TRACE_BYTES` не задан — кольцо не заводится, `trace_put`
 * компилируется в пустоту, `g920_trace_write` в образ не попадает.
 * Отдельный флаг от `G920_LOG_*` намеренно: лог и трасса трафика — разные
 * вещи, и выключать их поодиночке надо уметь.
 */
#ifndef G920_TRACE_BYTES
#define G920_TRACE_BYTES 0
#endif

#define TRACE_BYTES (G920_TRACE_BYTES)
/* Тишина, после которой трасса считается снятой и печатается. */
#define TRACE_QUIET_MS 500

static g920_trace_t s_trace;
static bool s_trace_ready;
static volatile uint64_t s_last_in_us;
static bool s_trace_dumped;

static void trace_put(g920_trace_kind_t kind, const uint8_t *data, size_t len)
{
    if (TRACE_BYTES == 0) {
        /* Условие константное: ветка целиком уходит из образа, но остаётся
         * проверяемой компилятором — как и у макросов лога. */
        (void)kind;
        (void)data;
        (void)len;
        return;
    }
    if (!s_trace_ready) {
        return;
    }
    (void)g920_trace_write(&s_trace, g920_timestamp_us(), kind, data,
                           (uint16_t)len);
}

/*
 * Печать снятой трассы: строка разбора и сырые байты на каждую запись.
 *
 * Здесь UART уже никому не мешает — обмен закончен, и время на шине больше
 * ни от чего не зависит.
 */
/*
 * Печать трассы **порциями**, а не одним заходом.
 *
 * Сплошная печать тысячи записей занимает секунды, и всё это время главный
 * цикл не крутится. Цена выяснилась на живом стенде: `g920_link_tick`
 * двигает повторы надёжной дисциплины **по одному кадру за вызов** (об этом
 * прямо предупреждает `queue.h`), и пока TX печатал, security-сообщение от
 * консоли шло к рулю **6.6 секунды** при бюджете протокола в 100 мс.
 * Диагностика, съедающая то, что диагностирует, — худший вид диагностики.
 *
 * Двадцать записей за проход: это единицы миллисекунд UART и не мешает ни
 * линку, ни шине.
 */
#define TRACE_DUMP_SLICE 20

static g920_trace_cursor_t s_dump_cursor;
static bool s_dumping;

static void dump_trace_slice(void)
{
    g920_trace_record_t record;
    char line[G920_TRACE_LINE_MAX];
    int printed = 0;

    while (printed < TRACE_DUMP_SLICE
           && g920_trace_next(&s_trace, &s_dump_cursor, &record)) {
        const char *dir = (record.kind == G920_TRACE_USB_IN) ? "in" : "out";

        if (g920_trace_format(line, sizeof(line), &record) > 0) {
            G920_LOGI(M1, "%s", line);
        }
        (void)log_message(dir, record.payload, record.length, NULL);
        printed++;
    }
    if (printed < TRACE_DUMP_SLICE) {
        s_dumping = false;
        G920_LOGI(M1, "--- trace end ---");
    }
}

static void dump_trace(void)
{
    G920_LOGI(M1, "--- trace: %u records, %u refused ---",
              (unsigned)g920_trace_count(&s_trace),
              (unsigned)g920_trace_refused(&s_trace));
    g920_trace_rewind(&s_trace, &s_dump_cursor);
    s_dumping = true;
}

/*
 * Отдать очередной пакет, если труба свободна.
 *
 * Зовётся из трёх мест: сразу после разбора принятого, из завершения
 * предыдущей отправки и из главного цикла. Первые два — ради скорости,
 * третий — чтобы очередь не залипла, если оба события уже прошли.
 *
 * Почему это важно именно здесь. У протокола на подтверждение 100 мс, а
 * отправитель повторяет сообщение каждые 60 мс, пока его не подтвердят.
 * Первая версия складывала ответ в очередь и отдавала его в главном цикле
 * **после** печати принятого пакета — а печать 64 байт хексдампом на
 * 115200 занимает десятки миллисекунд. Живой руль на это ответил тем, что
 * бросал знакомство на полпути и начинал заново. Лог не должен управлять
 * временем на шине.
 */
static void try_send_next(void)
{
    const out_packet_t *packet = NULL;

    /*
     * Труба берётся под замком: зовут отсюда две задачи на двух ядрах, и
     * без атомарного «свободна → занята» оба вызова проходили проверку и
     * сдавали один и тот же transfer дважды. Флаг занятости с этого места
     * принадлежит вызывающему: `host_send` его не трогает, а отпускают его
     * либо завершение отправки (`on_out_done`), либо отказ ниже.
     */
    portENTER_CRITICAL_SAFE(&s_out_lock);
    if (!s_out_busy && s_out_tail != s_out_head) {
        s_out_busy = true;
        packet = &s_out_queue[s_out_tail];
    }
    portEXIT_CRITICAL_SAFE(&s_out_lock);
    if (packet == NULL) {
        return;
    }
    if (host_send(packet->what, packet->data, packet->length)) {
        portENTER_CRITICAL_SAFE(&s_out_lock);
        s_out_tail = (uint8_t)((s_out_tail + 1) % OUT_QUEUE_LEN);
        portEXIT_CRITICAL_SAFE(&s_out_lock);
    } else {
        portENTER_CRITICAL_SAFE(&s_out_lock);
        s_out_busy = false;
        portEXIT_CRITICAL_SAFE(&s_out_lock);
    }
}

static void pump_sequencer(const uint8_t *data, size_t len)
{
    g920_gip_host_packet_t out[G920_GIP_HOST_OUT_MAX];
    uint64_t now_ms = g920_timestamp_us() / 1000u;
    int count = g920_gip_host_on_packet(&s_host, data, len, now_ms, out,
                                        G920_GIP_HOST_OUT_MAX);

    for (int i = 0; i < count; i++) {
        queue_packet(&out[i]);
    }
    try_send_next();
}

static void on_in_done(usb_transfer_t *transfer)
{
    g920_gip_header_t header;
    uint8_t copy[EP_BUF_MAX];
    size_t len = 0;
    bool received = false;
    bool first = false;

    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        if (s_first_in_us == 0) {
            s_first_in_us = g920_timestamp_us();
            first = true;
        }
        s_in_msgs++;
        /*
         * Порядок здесь — не стиль, а требование протокола, и он выведен из
         * двух живых прогонов.
         *
         * Печать одного 64-байтного пакета хексдампом на 115200 занимает
         * около 45 мс. Пока первая версия печатала, ответ ждал в очереди —
         * руль бросал знакомство на полпути. Вторая версия отвечала до
         * печати, и стало лучше, но фрагменты всё равно приходили раз в
         * 68 мс: приём **пересдавался** после печати, то есть руль ждал не
         * ответа, а нашего следующего опроса.
         *
         * Отсюда порядок: снять копию, сразу пересдать приём, ответить, и
         * только потом печатать. Копия нужна именно из-за пересдачи — буфер
         * передачи после неё принадлежит стеку, и печатать из него значит
         * печатать то, что уже переписали.
         */
        len = (size_t)transfer->actual_num_bytes;
        if (len > sizeof(copy)) {
            len = sizeof(copy);
        }
        memcpy(copy, transfer->data_buffer, len);
        received = true;
    } else if (transfer->status == USB_TRANSFER_STATUS_NO_DEVICE
               || transfer->status == USB_TRANSFER_STATUS_CANCELED) {
        /* Устройство ушло или поток гасят — пересдавать некуда. */
        s_in_running = false;
        return;
    } else {
        s_in_errors++;
        G920_LOGW(M1, "in transfer status %d", (int)transfer->status);
        if (transfer->status == USB_TRANSFER_STATUS_STALL) {
            /*
             * Halt сам не снимается. Не снять — эндпоинт замолчит навсегда,
             * и в логе это будет выглядеть как «руль перестал отвечать».
             */
            (void)usb_host_endpoint_clear(transfer->device_handle, s_ep_in);
        }
    }

    if (usb_host_transfer_submit(transfer) != ESP_OK) {
        s_in_running = false;
        G920_LOGE(M1, "in resubmit failed, stream stopped");
    }

    if (!received) {
        return;
    }

    /*
     * Сперва записать принятое, потом отвечать — иначе в трассе ответ стоит
     * раньше вызвавшего его пакета, а трасса и есть предмет вехи. Запись —
     * это memcpy в PSRAM, единицы микросекунд: на время ответа она не
     * влияет, в отличие от печати.
     */
    trace_put(G920_TRACE_USB_IN, copy, len);

    /*
     * Hello перехватывается **здесь и только здесь**: устройство шлёт его
     * в состоянии Arrival и больше никогда. Пропустив, второй раз его не
     * спросишь — придётся переподключать руль.
     */
    if (s_hello_len == 0 && len >= G920_GIP_HEADER_MIN) {
        g920_gip_header_t hello;

        if (g920_gip_header_parse(&hello, copy, len) == G920_GIP_OK
            && g920_gip_message_number(&hello) == G920_GIP_MSG_HELLO) {
            size_t payload = len - hello.header_length;

            if (payload > sizeof(s_hello)) {
                payload = sizeof(s_hello);
            }
            memcpy(s_hello, copy + hello.header_length, payload);
            s_hello_len = (uint16_t)payload;
        }
    }

    {
        g920_gip_header_t msg;

        if (g920_gip_header_parse(&msg, copy, len) == G920_GIP_OK) {
            uint8_t number = g920_gip_message_number(&msg);

            s_wheel_msgs[number & 0x1Fu]++;

            /*
             * Руль представился (`0x02` Announce) — значит он поднялся и
             * стоит в Arrival, то есть готов начать знакомство с нуля.
             * Сообщаем это донглу **явно**.
             *
             * Явно, а не признаком, и это исправление собственной ошибки.
             * Донгл ждал возвращения руля по «пришёл любой кадр из радио»,
             * но кадры всё это время шли от **старого**, ещё не
             * перезапущенного руля, и ожидание снималось мгновенно. Консоль
             * начинала security с рулём из прошлой сессии — отсюда «кнопки
             * не работают, кроме Xbox» сразу после включения бокса.
             *
             * Announce у руля один на сессию, поэтому и сообщение уходит
             * один раз — лишнего трафика это не создаёт.
             */
            if (number == 0x02u) {
                const uint8_t ready = CONTROL_WHEEL_READY;

                G920_LOGI(M1, "wheel announced itself — telling the dongle "
                              "it may call the console");
                (void)g920_link_send_reliable(G920_FRAME_CONTROL, &ready,
                                              sizeof(ready));
            }

            /*
             * Отчёты ввода уходят консоли всегда, а не только «после
             * аутентификации».
             *
             * Так требует хост: пока идёт handshake, устройство обязано
             * продолжать слать ввод (в joypad-os для этого даже
             * синтезируются холостые отчёты каждые 25 мс). Молчащее
             * устройство консоль считает умершим и обмен бросает — ровно
             * это мы и наблюдали, останавливаясь на полусотне сообщений.
             *
             * Свежей дисциплиной: поток непрерывен, и опоздавший отчёт хуже
             * пропущенного — следующий придёт через 4 мс.
             */
            if (msg.message_type == 0x20u) {
                /* Отметка живости руля — до всякой пересылки: она нужна
                 * даже тогда, когда отдать отчёт некуда. */
                s_last_input_ms = (uint32_t)(g920_timestamp_us() / 1000u);
                /*
                 * Номер кадра — **свой**, а не GIP-овский номер руля.
                 *
                 * Тот считается по-своему: у каждого типа сообщения GIP свой
                 * пул, счёт восьмибитный и заворачивается каждые 256
                 * отчётов, а на 250 Гц это раз в секунду. Приёмник линка
                 * отсеивает дубликаты по номеру кадра — и всё, что после
                 * заворота выглядело «позади», он честно выбрасывал. Отсюда
                 * `back 42` на RX против 230 отчётов, отданных здесь.
                 *
                 * Та же болезнь, что и у трубы: два разных счёта на один тип
                 * кадра. Лечится одинаково — счёт один и монотонный.
                 */
                (void)g920_link_send(G920_FRAME_INPUT, s_input_seq++, 0, copy,
                                     (uint16_t)len);
            }

            if (number == 0x06u || msg.message_type == 0x1Eu) {
                /* Ответ руля на аутентификацию — единственное, что уходит
                 * консоли из его слов: остальным разговором она ведёт нас
                 * саму, а не руль. */
                tunnel_push(copy, len);
            }

            /*
             * Статическая конфигурация (`0x21`) и Статус (`0x03`) — тоже
             * консоли.
             *
             * Это ответы на то, что она спрашивает **сама**: `0x21` — на
             * Initial Reports Request, `0x03` — на переход в Active. Ни то
             * ни другое донгл сочинить не может: в `0x21` лежат разрядность
             * осей, пределы угла (270°…900°) и маска FFB конкретного руля, а
             * в `0x03` — его питание.
             *
             * Наблюдалось 03.08.2026: security проходит целиком, после чего
             * консоль полсекунда за полсекундой шлёт Set Device State и
             * Initial Reports Request и не получает ничего — руль отвечал,
             * ответ оставался на этой плате. `03=0` в счётчиках всё время
             * означало не «руль молчит», а «мы не спрашивали и не возили».
             */
            if (msg.message_type == 0x21u || msg.message_type == 0x03u) {
                tunnel_push(copy, len);
            }

            /*
             * Кнопка Xbox (`0x07`, Guide Button Status) — консоли, всегда.
             *
             * Это единственное сообщение, которое обязано доходить до хоста
             * **когда хоста, считай, нет**: им геймпад будит выключённую
             * консоль. У нас оно не возилось вовсе — руль его шлёт (в
             * счётчиках `07=4`), а список пересылаемого его не содержал, и
             * бокс с руля не включался.
             *
             * По спеке (`H001419`, таблица сообщений) направление Upstream,
             * флаги `0x20`, нагрузка 2 байта. Толковать нечего: уходит
             * вербатим, как и всё остальное от руля.
             */
            if (number == 0x07u) {
                tunnel_push(copy, len);
            }

            /*
             * **Подтверждение руля на security тоже уходит консоли.**
             *
             * Без него обмен не идёт вовсе, и это видно в трассе от
             * 03.08.2026: консоль шлёт каждое `0x06` с флагом ACME, ждёт
             * ACK и, не получив, повторяет его четыре раза с шагом 256 мс,
             * после чего начинает знакомство с начала — за двадцать секунд
             * пять полных перезапусков.
             *
             * Подтверждает руль: сообщение доехало до него вербатим, вместе
             * с Sequence ID консоли, поэтому его ACK ссылается ровно на то
             * сообщение, которого консоль ждёт. Сочинять ACK самим значило
             * бы отвечать за руль в разговоре, который мы не ведём (И1).
             *
             * Отбор по RefMessageType — зеркало того, что донгл уже делает
             * во встречную сторону (`main_gip.c`): подтверждения на **наши**
             * сообщения к рулю (метаданные, Set Device State) остаются нам.
             */
            if (number == G920_GIP_MSG_PROTOCOL_CONTROL
                && len > msg.header_length) {
                g920_gip_control_t control;

                if (g920_gip_control_parse(&control, copy + msg.header_length,
                                           len - msg.header_length)
                        == G920_GIP_OK
                    && control.ref_message_type == 0x06u) {
                    tunnel_push(copy, len);
                }
            }

            if (s_tunnel) {
                /*
                 * Всё, что говорит руль, уходит консоли как есть.
                 *
                 * Отчёты ввода (0x20) — свежей дисциплиной: их поток
                 * непрерывен, и опоздавший отчёт хуже пропущенного, руль
                 * всё равно шлёт следующий через 4 мс. Остальное — надёжной:
                 * знакомство и security потерять нельзя.
                 */
                if (number == 0x00u
                    && g920_gip_data_class(&msg) == G920_GIP_CLASS_LOW_LATENCY) {
                    (void)g920_link_send(G920_FRAME_INPUT, msg.sequence, 0,
                                         copy, (uint16_t)len);
                } else {
                    tunnel_push(copy, len);
                }
            }
        }
    }

    /* В режиме трубы своего разговора с рулём нет: отвечает консоль. */
    if (!s_tunnel) {
        pump_sequencer(copy, len);
    }
    s_last_in_us = g920_timestamp_us();

    if (first) {
        G920_LOGI(M1, "wheel speaks: first message %u ms after claim",
                  (unsigned)((s_first_in_us - s_claim_us) / 1000u));
    }
    (void)header;
}

static void try_send_next(void);

static void on_out_done(usb_transfer_t *transfer)
{
    if (transfer->status != USB_TRANSFER_STATUS_COMPLETED) {
        G920_LOGW(M1, "out transfer status %d", (int)transfer->status);
    }
    s_out_busy = false;
    /* Труба освободилась — сразу отдать следующее, не дожидаясь цикла: у
     * протокола на подтверждение сто миллисекунд, а тик цикла плюс печать
     * съедают их незаметно. */
    try_send_next();
}

/*
 * Отправка одного сообщения в interrupt OUT.
 *
 * Очереди нет намеренно: в M1 хост говорит редко и по одному сообщению, а
 * очередь — это про порядок и повторы, то есть про M8.
 *
 * ⚠ Через эту функцию не проходит ничего, что двигает мотор. Единственные
 * её пользователи в этой вехе — системные сообщения GIP.
 */
static bool host_send(const char *what, const uint8_t *data, size_t len)
{
    /*
     * ⚠ Флагом занятости владеет вызывающий (`try_send_next` берёт его под
     * замком до вызова). Здесь его не проверять и не ставить: вторая точка
     * принятия решения — это и была гонка двух задач на одном transfer.
     */
    if (s_out == NULL || s_device == NULL || len == 0 || len > EP_BUF_MAX) {
        return false;
    }

    memcpy(s_out->data_buffer, data, len);
    s_out->num_bytes = (int)len;
    s_out->device_handle = s_device;
    s_out->bEndpointAddress = s_ep_out;
    s_out->callback = on_out_done;
    s_out->context = NULL;

    if (usb_host_transfer_submit(s_out) != ESP_OK) {
        G920_LOGE(M1, "out submit failed (%s)", what);
        return false;
    }
    trace_put(G920_TRACE_USB_OUT, data, len);
    /* Одна короткая строка: по ней видно ход обмена вживую, а байты уже
     * лежат в трассе и будут напечатаны, когда шина затихнет. */
    G920_LOGI(M1, "out: %s", what);
    return true;
}

/*
 * Ищет в конфигурации интерфейс GIP и его interrupt-эндпоинты.
 *
 * Ищется по классу, а не по номеру интерфейса: у спеки номер #0, но у
 * конкретного руля может быть иначе, и подставить номер вместо признака
 * значило бы поверить документу больше, чем железу.
 */
static bool find_gip_interface(const usb_config_desc_t *config,
                               const usb_intf_desc_t **out_intf,
                               const usb_ep_desc_t **out_ep_in,
                               const usb_ep_desc_t **out_ep_out)
{
    for (uint8_t number = 0; number < config->bNumInterfaces; number++) {
        int offset = 0;
        const usb_intf_desc_t *intf =
            usb_parse_interface_descriptor(config, number, 0, &offset);
        const usb_ep_desc_t *ep_in = NULL;
        const usb_ep_desc_t *ep_out = NULL;

        if (intf == NULL) {
            continue;
        }
        G920_LOGI(M1, "interface %u: class %02x/%02x/%02x, %u endpoints",
                  (unsigned)intf->bInterfaceNumber,
                  (unsigned)intf->bInterfaceClass,
                  (unsigned)intf->bInterfaceSubClass,
                  (unsigned)intf->bInterfaceProtocol,
                  (unsigned)intf->bNumEndpoints);

        if (intf->bInterfaceClass != GIP_IF_CLASS
            || intf->bInterfaceSubClass != GIP_IF_SUBCLASS
            || intf->bInterfaceProtocol != GIP_IF_PROTOCOL) {
            continue;
        }

        for (uint8_t i = 0; i < intf->bNumEndpoints; i++) {
            int ep_offset = offset;
            const usb_ep_desc_t *ep = usb_parse_endpoint_descriptor_by_index(
                intf, i, config->wTotalLength, &ep_offset);

            if (ep == NULL) {
                continue;
            }
            G920_LOGI(M1, "  endpoint %02x: type %u, mps %u, bInterval %u",
                      (unsigned)ep->bEndpointAddress,
                      (unsigned)USB_EP_DESC_GET_XFERTYPE(ep),
                      (unsigned)USB_EP_DESC_GET_MPS(ep),
                      (unsigned)ep->bInterval);

            if (USB_EP_DESC_GET_XFERTYPE(ep) != USB_TRANSFER_TYPE_INTR) {
                continue;
            }
            if (USB_EP_DESC_GET_EP_DIR(ep)) {
                ep_in = (ep_in == NULL) ? ep : ep_in;
            } else {
                ep_out = (ep_out == NULL) ? ep : ep_out;
            }
        }

        if (ep_in == NULL || ep_out == NULL) {
            G920_LOGE(M1,
                      "interface %u looks like GIP but lacks an interrupt "
                      "%s endpoint",
                      (unsigned)intf->bInterfaceNumber,
                      (ep_in == NULL) ? "IN" : "OUT");
            continue;
        }

        *out_intf = intf;
        *out_ep_in = ep_in;
        *out_ep_out = ep_out;
        return true;
    }
    return false;
}

/* Гасит поток и отдаёт всё, что было взято у стека. */
static void release_interface(void)
{
    if (s_claimed && s_device != NULL) {
        /* Флаг снимается до halt: иначе колбэк отменённой передачи успеет
         * пересдать её обратно. */
        s_in_running = false;
        (void)usb_host_endpoint_halt(s_device, s_ep_in);
        (void)usb_host_endpoint_flush(s_device, s_ep_in);
        (void)usb_host_endpoint_halt(s_device, s_ep_out);
        (void)usb_host_endpoint_flush(s_device, s_ep_out);

        /* Дать колбэкам отменённых передач отработать: они приходят из той
         * же очереди, что и всё остальное. */
        for (int i = 0; i < 5; i++) {
            usb_host_client_handle_events(s_client, pdMS_TO_TICKS(10));
        }
        (void)usb_host_interface_release(s_client, s_device, s_intf_num);
    }
    s_claimed = false;
    s_in_running = false;
    s_out_busy = false;

    if (s_in != NULL) {
        (void)usb_host_transfer_free(s_in);
        s_in = NULL;
    }
    if (s_out != NULL) {
        (void)usb_host_transfer_free(s_out);
        s_out = NULL;
    }
}

/*
 * Захват интерфейса и запуск непрерывного чтения.
 *
 * Пересдача идёт из колбэка — так поток не зависит от того, чем занят
 * главный цикл, и между двумя опросами нет паузы в размере тика. Одной
 * передачи хватает: интервал опроса задаёт эндпоинт (bInterval), а не мы.
 */
static bool claim_and_listen(void)
{
    const usb_config_desc_t *config = NULL;
    const usb_intf_desc_t *intf = NULL;
    const usb_ep_desc_t *ep_in = NULL;
    const usb_ep_desc_t *ep_out = NULL;
    uint16_t mps;

    if (usb_host_get_active_config_descriptor(s_device, &config) != ESP_OK
        || config == NULL) {
        G920_LOGE(M1, "no active config, cannot claim");
        return false;
    }
    if (!find_gip_interface(config, &intf, &ep_in, &ep_out)) {
        G920_LOGE(M1, "no %02x/%02x/%02x interface — this is not a GIP device",
                  GIP_IF_CLASS, GIP_IF_SUBCLASS, GIP_IF_PROTOCOL);
        return false;
    }

    mps = USB_EP_DESC_GET_MPS(ep_in);
    if (mps == 0 || mps > EP_BUF_MAX) {
        G920_LOGE(M1, "IN endpoint mps %u out of range", (unsigned)mps);
        return false;
    }

    s_intf_num = intf->bInterfaceNumber;
    s_ep_in = ep_in->bEndpointAddress;
    s_ep_out = ep_out->bEndpointAddress;

    if (usb_host_interface_claim(s_client, s_device, s_intf_num, 0) != ESP_OK) {
        G920_LOGE(M1, "interface %u claim failed", (unsigned)s_intf_num);
        return false;
    }
    s_claimed = true;

    if (usb_host_transfer_alloc(EP_BUF_MAX, 0, &s_in) != ESP_OK
        || usb_host_transfer_alloc(EP_BUF_MAX, 0, &s_out) != ESP_OK) {
        G920_LOGE(M1, "transfer alloc failed");
        release_interface();
        return false;
    }

    s_in->device_handle = s_device;
    s_in->bEndpointAddress = s_ep_in;
    /* IN-передача обязана быть кратна MPS — иначе стек её не примет. */
    s_in->num_bytes = mps;
    s_in->callback = on_in_done;
    s_in->context = NULL;

    s_in_msgs = 0;
    s_in_errors = 0;
    s_first_in_us = 0;
    s_claim_us = g920_timestamp_us();

    /* Обмен начинается с чистого листа на каждое подключение: устройство
     * после переподключения считает себя новым, и помнить о нём старое
     * значило бы отвечать на позапрошлый разговор. */
    g920_gip_host_init(&s_host, s_metadata, sizeof(s_metadata));
    s_metadata_logged = false;
    s_wheel_ready = false;
    s_out_head = 0;
    s_out_tail = 0;
    /* Часы застревания идут от захвата: до него ждать было нечего. */
    s_handshake_state = s_host.state;
    s_handshake_at_ms = (uint32_t)(g920_timestamp_us() / 1000u);
    s_handshake_nudges = 0;

    if (usb_host_transfer_submit(s_in) != ESP_OK) {
        G920_LOGE(M1, "in submit failed");
        release_interface();
        return false;
    }
    s_in_running = true;

    G920_LOGI(M1,
              "claimed interface %u, listening on ep %02x (mps %u, bInterval "
              "%u), ep %02x ready for out",
              (unsigned)s_intf_num, (unsigned)s_ep_in, (unsigned)mps,
              (unsigned)ep_in->bInterval, (unsigned)s_ep_out);
    G920_LOGI(M1, "wheel should announce itself every 500 ms on its own");
    return true;
}

/*
 * Переподключение устройства без человека: снять и подать питание корневого
 * порта.
 *
 * Зачем. У стека ESP-IDF известная особенность: устройство, которое уже
 * висело на шине **до** установки хоста, может не быть замечено, и в логе
 * это выглядит как `Root port reset failed`; лечится вытащить-воткнуть
 * (espressif/esp-idf#10086). Наша прошивка попадает в этот случай по
 * построению — она нарочно ждёт подтяжку и поднимает стек, когда устройство
 * уже на месте. Просить человека передёрнуть кабель значит требовать
 * присутствия у стола ради дефекта, который снимается программно: снятие
 * питания с корневого порта отключает всё, что ниже, а возврат заставляет
 * стек пройти обнаружение заново.
 *
 * Попыток немного и они конечны: если три переподключения не помогли, дело
 * не в этой особенности, и повторять её бесконечно значит прятать другую
 * причину за шумом.
 */
/*
 * Сколько первых попыток печатать подробно. **Не предел попыток** — их
 * больше нет: руль может появиться позже нас, и мост обязан ждать его
 * столько, сколько нужно.
 */
#define REPOWER_ATTEMPTS_MAX 3

static uint32_t s_repower_attempts;

static void kick_root_port(void)
{
    esp_err_t off;
    esp_err_t on;

    s_repower_attempts++;
    /* Печатаем первые попытки и дальше редко: попытки теперь бесконечны,
     * и строка на каждую сделала бы журнал нечитаемым. */
    if (s_repower_attempts <= REPOWER_ATTEMPTS_MAX
        || (s_repower_attempts % 20u) == 0u) {
        G920_LOGW(M1, "no device on the bus — re-powering the root port "
                      "(attempt %u)",
                  (unsigned)s_repower_attempts);
    }

    off = usb_host_lib_set_root_port_power(false);
#ifdef G920_VBUS_GPIO
    /*
     * Питание розетки гасится вместе с портом — но не на каждой попытке.
     *
     * Без цикла VBUS руль остаётся под своими 5 В, отключения не видит и
     * из залипания не выходит — измерено (9 сообщений против 241, разбор
     * у replug_wheel). Но и на каждой попытке нельзя: после снятия VBUS
     * G920 поднимает свой USB ~6.2 с (лог 22.08.2026), а попытки идут
     * каждые ~5 с — цикл в каждой не давал бы рулю добежать до подтяжки
     * никогда. Каждая третья: две попытки руль спокойно доезжает, третья
     * выбивает залипание, если доехать было некому.
     */
    if ((s_repower_attempts % 3u) == 0u) {
        vbus_set(false);
        vTaskDelay(pdMS_TO_TICKS(VBUS_OFF_MS));
        vbus_set(true);
    } else {
        vTaskDelay(pdMS_TO_TICKS(300));
    }
#else
    vTaskDelay(pdMS_TO_TICKS(300));
#endif
    on = usb_host_lib_set_root_port_power(true);

    /*
     * Включение проверяется и повторяется. Погасший и не вернувшийся порт —
     * это обесточенный руль, то есть отказ всего моста; молча принять
     * `ESP_ERR_INVALID_STATE` здесь нельзя.
     */
    for (int retry = 0; on != ESP_OK && retry < 5; retry++) {
        vTaskDelay(pdMS_TO_TICKS(200));
        on = usb_host_lib_set_root_port_power(true);
        G920_LOGW(M1, "root port power on retry %d: %s", retry + 1,
                  esp_err_to_name(on));
    }
    if (on != ESP_OK) {
        G920_LOGE(M1, "root port stayed off (%s) — the wheel has no bus",
                  esp_err_to_name(on));
    }
    G920_LOGI(M1, "root port power: off %s, on %s", esp_err_to_name(off),
              esp_err_to_name(on));
}

/*
 * Переподключение руля — «вынуть и воткнуть» без человека.
 *
 * Две ступени, и разница между ними принципиальная.
 *
 * **Без ключа на VBUS** всё, что мы можем, — погасить корневой порт. Руль
 * при этом остаётся под своими 5 В и **отключения не видит**: пропадает
 * опрос, через 3 мс тишины он уходит в suspend, а на возврате питания порта
 * получает сброс шины. Адрес и конфигурацию он теряет, то есть формально
 * начинает сначала, — но 03.08.2026 измерено, что живой G920 после такого
 * не оживает (9 сообщений против 241 после полного сброса платы).
 *
 * **С ключом на VBUS** это настоящее переподключение: 5 В с розетки
 * снимаются, руль видит уход хоста, гасит подтяжку 1.5 кОм и складывает
 * свою USB-часть целиком. Возврат питания для него неотличим от того, что
 * человек воткнул кабель. Именно этого руль и слушается — и именно поэтому
 * VBUS теперь наш: он приходит с развязанного модуля, а не из руля.
 *
 * Ключ подключается флагом сборки `-DG920_VBUS_GPIO=<пин>`; не задан —
 * работает первая ступень, и прошивка честно говорит об этом в лог.
 *
 * `VBUS_OFF_MS` объявлен выше, у ожидания подтяжки: пауза одна на всех.
 */
#ifdef G920_VBUS_GPIO
static void vbus_set(bool on)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << (G920_VBUS_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    (void)gpio_config(&cfg);
    /* Единица — питание есть: ключ включается уровнем, а не его
     * отсутствием, чтобы обесточенный пин при сбросе платы не подавал
     * VBUS сам собой. */
    (void)gpio_set_level((gpio_num_t)(G920_VBUS_GPIO), on ? 1 : 0);
}
#endif

static bool wheel_is_alive(uint32_t now_ms)
{
    if (s_host.state == G920_GIP_HOST_ACTIVE) {
        return true;
    }
    return s_last_input_ms != 0
           && (uint32_t)(now_ms - s_last_input_ms) < WHEEL_ALIVE_MS;
}

static void replug_wheel(const char *why)
{
    /*
     * Порядок обязателен и тот же, что при переподнятии по просьбе донгла:
     * сперва отпустить интерфейс и закрыть устройство, потом трогать
     * питание. Погасить под открытым устройством значит оставить стеку
     * ссылки на то, чего уже нет.
     */
    release_interface();
    if (s_device != NULL) {
        (void)usb_host_device_close(s_client, s_device);
        s_device = NULL;
    }
#ifdef G920_VBUS_GPIO
    G920_LOGW(M1, "%s — unplugging the wheel: vbus off for %u ms", why,
              (unsigned)VBUS_OFF_MS);
    vbus_set(false);
    vTaskDelay(pdMS_TO_TICKS(VBUS_OFF_MS));
    vbus_set(true);
    /*
     * Дальше стек справится сам: с точки зрения шины это обычное горячее
     * подключение, а его ESP-IDF обрабатывает штатно. Передёргивать порт
     * следом не нужно и вредно — это второй сброс поверх первого.
     */
#else
    G920_LOGW(M1, "%s — re-powering the root port (no vbus switch built: "
                  "the wheel keeps its 5V and may ignore this)",
              why);
    kick_root_port();
#endif
}

/*
 * Последнее средство: перезагрузить **себя**.
 *
 * Это не выдумка от отчаяния, а то самое, что человек делает руками, и
 * измеренное лучшее из доступного.
 *
 * Измерено 03.08.2026 и записано тут же, в разборе переподнятия руля:
 * передёргивание корневого порта шину поднимает, но руль после него не
 * оживает — 9 сообщений и тишина против 241 за 14 секунд после полного
 * сброса. При сбросе заново поднимается **весь стек хоста и сам PHY**, а не
 * только питание порта, и руль видит настоящий сброс шины.
 *
 * А снаружи это выглядит так: пока мак не воткнут в передатчик, руль не
 * калибруется; воткнул — заработало. Мак ничего не чинит. Он дёргает схему
 * автосброса платы, то есть **перезагружает передатчик** — ровно то, что
 * делает эта функция. Мост, который умеет это сам, не требует человека у
 * стола и не требует мака вовсе.
 *
 * Ограничитель обязателен: перезагрузка, не решившая задачу, повторённая
 * бесконечно, — это петля, в которой мост не живёт и минуты. Счётчик лежит
 * в RTC-памяти: она переживает `esp_restart` и не переживает снятие питания,
 * то есть считает ровно «за это включение». На холодном старте она не
 * инициализирована ничем, поэтому сбрасывается по причине сброса.
 *
 * Исчерпав лимит, мост **не сдаётся**, а продолжает передёргивать порт: три
 * перезагрузки не помогли — значит дело не в них, и петля из них ничего не
 * добавит.
 */
#define SELF_RESTART_MAX 3

static RTC_NOINIT_ATTR uint32_t s_self_restarts;
static uint32_t s_self_restart_notes;

static void restart_self(const char *why)
{
    if (s_self_restarts >= SELF_RESTART_MAX) {
        if ((s_self_restart_notes++ % 20u) == 0u) {
            G920_LOGE(M1,
                      "%s — %u self-restarts since power-on did not help, "
                      "staying up and working the bus instead",
                      why, (unsigned)s_self_restarts);
        }
        return;
    }
    s_self_restarts++;
    G920_LOGE(M1, "%s — restarting the transmitter (%u of %u since power-on)",
              why, (unsigned)s_self_restarts, (unsigned)SELF_RESTART_MAX);
    vTaskDelay(pdMS_TO_TICKS(50)); /* дать строке уйти в UART */
    esp_restart();
}

/*
 * Личность в NVS — **частями, а не одним блобом**.
 *
 * Так предписано `g920/store.h`: потолок записи 512 байт, и он не случаен —
 * запись целиком собирается в буфере на стеке задачи. Личность G920 весит
 * 655 байт и растёт: в режиме `c262` к ней добавится HID report descriptor.
 *
 * Куски нарезаются по границе буфера, а не по секциям: контейнер — поток
 * байтов, и резать его по смыслу значило бы разбирать то, что положено
 * хранить непрозрачным (И1). Длина целого лежит отдельной записью, по ней
 * читатель знает, сколько кусков собирать.
 */
#define IDENTITY_CHUNK 512

static g920_store_status_t store_identity(const uint8_t *data, size_t len)
{
    uint8_t total[4];
    size_t offset = 0;
    unsigned index = 0;

    total[0] = (uint8_t)(len & 0xFFu);
    total[1] = (uint8_t)((len >> 8) & 0xFFu);
    total[2] = (uint8_t)((len >> 16) & 0xFFu);
    total[3] = (uint8_t)((len >> 24) & 0xFFu);

    while (offset < len) {
        size_t chunk = len - offset;
        char key[G920_STORE_KEY_MAX + 1];
        g920_store_status_t status;

        if (chunk > IDENTITY_CHUNK) {
            chunk = IDENTITY_CHUNK;
        }
        (void)snprintf(key, sizeof(key), "id%u", index);
        status = g920_store_write(key, G920_STORE_KIND_IDENTITY, 1,
                                  data + offset, chunk);
        if (status != G920_STORE_OK) {
            return status;
        }
        offset += chunk;
        index++;
    }

    /* Длина пишется **последней**: до неё запись считается незавершённой, и
     * оборванное посреди кусков сохранение не выглядит как готовая
     * личность. */
    return g920_store_write("idlen", G920_STORE_KIND_IDENTITY, 1, total,
                            sizeof(total));
}

/*
 * Сборка личности руля и сохранение её в NVS.
 *
 * Зовётся один раз, когда собраны метаданные: до этого личность неполна, а
 * после — уже не меняется, устройство статично, пока подключено.
 *
 * Секции складываются **как есть**, без разбора и без перекладывания в свои
 * структуры (И1). Единственное, что читается по смыслу, — VID/PID/revision
 * для отпечатка, и то по стандартным смещениям USB.
 */
static void capture_identity(void)
{
    const usb_device_desc_t *device = NULL;
    const usb_config_desc_t *config = NULL;
    usb_device_info_t info;
    g920_identity_fingerprint_t fingerprint;
    const uint8_t *blob;
    size_t blob_len = 0;
    g920_store_status_t stored;

    if (g920_identity_init(&s_identity, s_identity_buffer,
                           sizeof(s_identity_buffer))
        != G920_IDENTITY_OK) {
        G920_LOGE(M1, "identity: init failed");
        return;
    }

    if (usb_host_get_device_descriptor(s_device, &device) != ESP_OK
        || device == NULL) {
        G920_LOGE(M1, "identity: no device descriptor");
        return;
    }
    if (g920_identity_fingerprint_from_device_descriptor(
            &fingerprint, (const uint8_t *)device, device->bLength)) {
        g920_identity_set_fingerprint(&s_identity, fingerprint);
    }
    (void)g920_identity_add(&s_identity, G920_ID_DEVICE_DESC, 0, device,
                            device->bLength);

    if (usb_host_get_active_config_descriptor(s_device, &config) == ESP_OK
        && config != NULL) {
        /* Целиком, вместе с интерфейсами и эндпоинтами: конфигурация — это
         * цепочка, и обрывать её по bLength первого дескриптора значило бы
         * выбросить как раз то, что делает руль рулём. */
        (void)g920_identity_add(&s_identity, G920_ID_CONFIG_DESC, 0, config,
                                config->wTotalLength);
    }

    /*
     * Строки берутся у стека: он прочитал их при энумерации, и повторный
     * control-запрос дал бы те же байты, только с риском разойтись с тем,
     * что видел хост.
     */
    if (usb_host_device_info(s_device, &info) == ESP_OK) {
        const usb_str_desc_t *strings[3] = { info.str_desc_manufacturer,
                                             info.str_desc_product,
                                             info.str_desc_serial_num };
        const uint8_t index[3] = { device->iManufacturer, device->iProduct,
                                   device->iSerialNumber };

        for (int i = 0; i < 3; i++) {
            if (strings[i] != NULL && strings[i]->bLength > 0) {
                (void)g920_identity_add(&s_identity, G920_ID_STRING_DESC,
                                        index[i], strings[i],
                                        strings[i]->bLength);
            }
        }
    }

    if (s_hello_len > 0) {
        (void)g920_identity_add(&s_identity, G920_ID_GIP_HELLO, 0, s_hello,
                                s_hello_len);
    } else {
        G920_LOGW(M1, "identity: hello was not captured");
    }

    blob = g920_gip_host_metadata(&s_host, &blob_len);
    if (blob != NULL && blob_len > 0) {
        (void)g920_identity_add(&s_identity, G920_ID_GIP_METADATA, 0, blob,
                                (uint16_t)blob_len);
    }

    G920_LOGI(M1, "identity: %04x:%04x rev %04x, %u sections, %u bytes",
              (unsigned)s_identity.fingerprint.vendor_id,
              (unsigned)s_identity.fingerprint.product_id,
              (unsigned)s_identity.fingerprint.device_release,
              (unsigned)g920_identity_section_count(&s_identity),
              (unsigned)g920_identity_size(&s_identity));

    stored = store_identity(s_identity_buffer, g920_identity_size(&s_identity));
    if (stored == G920_STORE_OK) {
        G920_LOGI(M1, "identity: saved to nvs");
    } else {
        G920_LOGE(M1, "identity: store failed (%s)",
                  g920_store_status_name(stored));
    }
    s_identity_saved = true;
}

/*
 * ⚠ `identity_in_nvs()` удалена 03.08.2026 по вердикту судьи.
 *
 * Она решала, «знакомиться с рулём самим или сразу становиться трубой», —
 * то есть обслуживала сквозной туннель, который проверен и отвергнут
 * опытом (см. `s_tunnel` ниже). Вызывающих не осталось, компилятор говорил
 * об этом предупреждением на каждой сборке, а комментарий над ней
 * по-прежнему описывал развилку, которой в прошивке нет.
 *
 * Мёртвая функция с живым комментарием — это ровно то, за что M3 получала
 * BLOCK: документ уверяет в одном, исполняется другое. Понадобится снова —
 * восстанавливается из истории вместе с той развилкой, ради которой писалась.
 */

/*
 * Два пробных сообщения с ACME: безобидное и auth. Оба просят подтверждения.
 */
static void probe_ack(void)
{
    /* 0x0A, тип запроса 0x00 (начальные отчёты), с ACME. */
    static const uint8_t benign[] = { 0x0A, 0x10, 0x11, 0x03, 0x00, 0x00, 0x00 };
    /*
     * Host Hello версии 1 в раскладке `xone`: context 0, options
     * ACKNOWLEDGE|FROM_HOST, error 0, command 0x01, длина данных big-endian,
     * дальше случайные байты. Содержимое руль вправе отвергнуть — нас
     * интересует только, подтвердит ли он приём.
     */
    static const uint8_t hello[] = {
        0x06, 0x30, 0x11, 0x1A, /* заголовок GIP: система + ACME, 26 байт */
        0x00, 0x41, 0x00, 0x01, 0x00, 0x16, /* заголовок handshake */
        0x01, 0x00, 0x00, 0x16, /* заголовок данных: команда, версия, длина */
        0xA5, 0x5A, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
        0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
        0x0F, 0x10, 0x11, 0x12
    };
    queue_bytes(benign, sizeof(benign), "probe: wheel request with ACME");
    queue_bytes(hello, sizeof(hello), "probe: auth host hello with ACME");
    try_send_next();
    G920_LOGW(M1, "probe: sent 0x0a and 0x06, both asking for an ack");
}

/* --- события клиента ------------------------------------------------------ */

/*
 * Колбэк зовётся из usb_host_client_handle_events, то есть из нашей же
 * задачи. Открывать устройство прямо здесь стек не запрещает, но делать
 * длинную работу в колбэке — привычка, от которой потом трудно избавиться:
 * откладываем всё в цикл.
 */
static void on_client_event(const usb_host_client_event_msg_t *msg, void *arg)
{
    (void)arg;

    if (msg == NULL) {
        return;
    }
    switch (msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        s_pending_addr = msg->new_dev.address;
        s_pending_new = true;
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        s_pending_gone = true;
        break;
    default:
        break;
    }
}

/* --- задача обслуживания библиотеки --------------------------------------- */

/*
 * У стека две очереди событий: своя и клиентская, и крутить их надо порознь.
 * Библиотечную — в отдельной задаче, потому что она блокирующая и должна
 * идти независимо от того, чем занят прикладной цикл.
 */
static void usb_lib_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t flags = 0;

        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            G920_LOGW(M1, "no clients left");
        }
        if (flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            G920_LOGW(M1, "all devices freed");
        }
    }
}

void app_main(void)
{
    char version[G920_VERSION_STR_MAX];
    g920_store_status_t status;
    bool led;
    bool on = false;
    uint32_t since_blink_ms = 0;
    uint32_t since_note_ms = 0;

    usb_host_config_t host_config = {
        /* PHY поднимает сама библиотека: внешнего у нас нет. */
        .skip_phy_setup = false,
        .root_port_unpowered = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
        .enum_filter_cb = NULL,
    };
    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = CLIENT_EVENT_QUEUE,
        .async = {
            .client_event_callback = on_client_event,
            .callback_arg = NULL,
        },
    };

    if (g920_version_format(version, sizeof(version), g920_firmware_version())
        < 0) {
        version[0] = '?';
        version[1] = '\0';
    }
    G920_LOGI(TAG, "fw %s, role TX, mode usb-host (m1 probe)", version);

    /*
     * Счётчик собственных перезагрузок живёт в RTC-памяти и ничем не
     * инициализируется — значит на холодном старте в нём мусор. Обнуляем по
     * причине сброса: всё, кроме программного, — это новое включение.
     */
    if (esp_reset_reason() != ESP_RST_SW) {
        s_self_restarts = 0;
    } else if (s_self_restarts > 0) {
        G920_LOGW(TAG, "this is self-restart %u of %u since power-on",
                  (unsigned)s_self_restarts, (unsigned)SELF_RESTART_MAX);
    }

#if G920_LOG_BUILD_LEVEL < G920_LOG_LEVEL_ERROR
    /*
     * Свой лог выключен сборкой — значит и чужой не нужен.
     *
     * Макросы `G920_LOG*` убирают **наши** вызовы, а `boot:`, `wifi:`,
     * `usb_host:` и прочее печатает сам ESP-IDF, мимо них, через свой
     * уровень. На проде это десятки строк в UART, которого никто не
     * слушает, и они идут через тот же горячий поток.
     *
     * Гасится здесь, а не в `sdkconfig`: уровень ESP-IDF задаётся на весь
     * проект, а профилей у проекта восемь. Накладку `SDKCONFIG_DEFAULTS`
     * через `board_build.cmake_extra_args` я проверил — PlatformIO её не
     * применяет, сгенерированный `sdkconfig` оставался на уровне INFO.
     *
     * ⚠ Что этим **не** гасится: баннер загрузчика и строки `boot:` /
     * `esp_image:` / `cpu_start:` — они печатаются до `app_main`, то есть
     * во время запуска, а не после него. Чтобы убрать и их, нужен
     * `CONFIG_BOOTLOADER_LOG_LEVEL_NONE` в `sdkconfig.defaults`, и это
     * решение на весь проект, а не на профиль.
     */
    esp_log_level_set("*", ESP_LOG_NONE);
#endif

#ifdef G920_LOG_PSRAM_BYTES
    /*
     * Первой строкой в UART уходит версия — по ней человек с переходником
     * убеждается, что плата жива и какая на ней прошивка. Дальше лог
     * переезжает в кольцо, и порт замолкает.
     */
    (void)g920_trace_log_to_psram(G920_LOG_PSRAM_BYTES);
#endif

    status = g920_store_init();
    if (status != G920_STORE_OK) {
        G920_LOGE(TAG, "store init: %s", g920_store_status_name(status));
    }

    led = g920_board_led_init();
    G920_LOGI(TAG, "led %s %s", g920_board_led_kind(), led ? "ok" : "absent");

    /*
     * Отказ PSRAM громкий и не молчаливый переход на внутреннюю память:
     * внутренней хватило бы на пару сотен пакетов, а выглядело бы это как
     * потери в обмене — то есть как дефект руля или радио.
     */
    s_trace_ready =
        TRACE_BYTES != 0
        && g920_trace_init_psram(&s_trace, TRACE_BYTES, G920_TRACE_KEEP_OLDEST);
    if (s_trace_ready) {
        G920_LOGI(TAG, "trace: %u KB in psram", (unsigned)(TRACE_BYTES / 1024));
    } else {
        G920_LOGE(TAG, "trace: psram unavailable, traffic will not be recorded");
    }

    /*
     * Руль ведёт **сам TX**, а по радио идёт только аутентификация.
     *
     * Так устроены обе работающие реализации — GP2040-CE и joypad-os, — и
     * причина в их коде названа прямо: источник аутентификации обязан
     * пройти настоящий handshake announce → дескриптор → **power-on**, и
     * только тогда он отвечает на 0x06. Сквозной туннель этого не даёт:
     * консоль ведёт руль своими сообщениями, но её собственная сессия при
     * этом не доходит даже до Host Hello.
     */
    s_tunnel = false;
    G920_LOGI(TAG, "mode: local (drive the wheel, relay auth only)");

    if (g920_link_init(link_frame, NULL) != G920_LINK_OK) {
        G920_LOGE(TAG, "link init failed — identity will stay on this board");
    } else {
        G920_LOGI(TAG, "link up, looking for the dongle");
    }


    /*
     * ⚠ Пины GPIO19/20 до стека НЕ трогать. Совсем.
     *
     * Предстековая диагностика — wire check, пробы, ожидание подтяжки —
     * крутила эти пины как GPIO, и после неё пады доставались USB-PHY в
     * состоянии, в котором стек **не видел статически подключённое
     * устройство**: подтяжка руля на месте, а `devices 0` навсегда
     * (лог 22.08.2026). Горячее подключение при этом ловилось — потому
     * стенд и «работал от ноута» (руль приходил фронтом при живом стеке)
     * и «от касания корпуса» (глитч рождал фронт). Стоило это открытие
     * недель охоты за землёй, шумом и питанием: G920 держит подтяжку от
     * своего 24 В и на VBUS-циклы не реагирует, так что единственный
     * рабочий путь — стек поднимается на нетронутых падах и находит руль
     * по уровню, как любой хост находит воткнутое до загрузки устройство.
     *
     * Диагностика осталась под `-DG920_PRESTACK_PROBES` — для стола, где
     * человек с тестером ищет обрыв провода. Это отладочный профиль; в
     * нём руль обязан прийти фронтом (при ключе VBUS сливается и
     * подаётся после стека), и лечится это как раньше — касанием.
     */
#ifdef G920_PRESTACK_PROBES
#ifdef G920_VBUS_GPIO
    vbus_set(false);
    G920_LOGI(M1, "vbus held off for the prestack probes");
#endif
    check_wires();
    probe_lines();
#ifdef G920_VBUS_GPIO
    drain_vbus();
#else
    wait_for_pullup();
#endif
#else
#ifdef G920_VBUS_GPIO
    vbus_set(true);
    G920_LOGI(M1, "vbus switch on gpio%d: on", (int)(G920_VBUS_GPIO));
#endif
#endif

    if (usb_host_install(&host_config) != ESP_OK) {
        G920_LOGE(TAG, "usb host install failed");
        g920_board_led_set(G920_IND_FAULT);
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    if (xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 5, NULL) != pdPASS) {
        G920_LOGE(TAG, "usb lib task failed");
        g920_board_led_set(G920_IND_FAULT);
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    if (usb_host_client_register(&client_config, &s_client) != ESP_OK) {
        G920_LOGE(TAG, "usb client register failed");
        g920_board_led_set(G920_IND_FAULT);
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    G920_LOGI(M1, "host up, waiting for a device on GPIO19/20");
    G920_LOGI(M1, "vbus must come from an external 5V source, not the board");

#ifdef G920_VBUS_GPIO
    /*
     * В рабочем пути VBUS уже подан, и это но-оп. В отладочном
     * (`G920_PRESTACK_PROBES`) — первое включение после проб: стек уже
     * слушает, и появление руля станет фронтом подключения.
     */
    vbus_set(true);
#endif

    for (;;) {
        usb_host_client_handle_events(s_client, pdMS_TO_TICKS(TICK_MS));

        if (s_pending_new) {
            uint8_t addr = s_pending_addr;

            s_pending_new = false;
            G920_LOGI(M1, "device connected, addr %u", (unsigned)addr);
            if (usb_host_device_open(s_client, addr, &s_device) == ESP_OK) {
                report_device(s_device);
                if (!claim_and_listen()) {
                    G920_LOGE(M1, "listening did not start");
                }
            } else {
                G920_LOGE(M1, "device open failed");
            }
        }
        if (s_relaunch_wheel) {
            uint32_t now_ms = (uint32_t)(g920_timestamp_us() / 1000u);

            s_relaunch_wheel = false;
            /*
             * Не чаще раза в `RELAUNCH_COOLDOWN_MS`, и это не вежливость.
             *
             * Донгл просит вернуть руль дважды подряд — консоль настраивает
             * устройство два раза за подключение. Второе передёргивание
             * приходило, пока порт ещё поднимался, и **включить его обратно
             * не удавалось**: в логе `off ESP_OK, on ESP_ERR_INVALID_STATE`.
             * Это худший из возможных исходов — руль остаётся обесточенным,
             * и снаружи это выглядит как «мост сдох».
             */
            /*
             * Своя загрузка — тоже повод помолчать. Донгл просит вернуть
             * руль дважды, и вторая просьба приходит уже в свежезагруженный
             * передатчик. Без этого условия она вызвала бы вторую
             * перезагрузку, а та — третью: петля.
             */
            if (now_ms < RELAUNCH_COOLDOWN_MS) {
                G920_LOGI(M1, "relaunch asked %u ms after boot — ignoring",
                          (unsigned)now_ms);
                goto relaunch_done;
            }
            /*
             * Руль ещё **ни разу не проходил security** в этой своей сессии
             * — значит он уже там, куда его просят вернуть, и трогать его
             * незачем.
             *
             * Смысл переподнятия ровно один: вытащить руль из **чужой,
             * состоявшейся** сессии, где он security прошёл и заново её не
             * начнёт. Девственный руль этой болезнью не болен.
             *
             * Цена лишнего переподнятия видна человеку напрямую: руль делает
             * второй калибровочный проворот при каждом включении. Ровно на
             * это и была жалоба 03.08.2026 — «после включения руль
             * калибруется 2 раза».
             */
            if (s_wheel_msgs[0x06] == 0u) {
                G920_LOGI(M1, "relaunch asked, but the wheel has not done "
                              "security yet — it is already fresh, skipping");
                goto relaunch_done;
            }
            if (s_relaunch_at_ms != 0
                && (uint32_t)(now_ms - s_relaunch_at_ms) < RELAUNCH_COOLDOWN_MS) {
                G920_LOGI(M1, "relaunch asked again %u ms after the last one "
                              "— ignoring, the port is still coming up",
                          (unsigned)(now_ms - s_relaunch_at_ms));
                goto relaunch_done;
            }
            s_relaunch_at_ms = now_ms;
            G920_LOGW(M1, "console started over — relaunching the wheel on usb");
            /*
             * Порядок обязателен: сначала отпустить интерфейс и закрыть
             * устройство, потом гасить порт. Погасить под открытым
             * устройством значит оставить стеку ссылки на то, чего уже нет.
             */
            release_interface();
            if (s_device != NULL) {
                (void)usb_host_device_close(s_client, s_device);
                s_device = NULL;
            }
            /*
             * **Перезагрузка передатчика целиком, а не передёргивание
             * порта.** Выбрано замером, дважды, и второй замер снял
             * возражение, из-за которого этот способ днём откатывался.
             *
             * Замер первый: передёргивание порта шину поднимает, но руль
             * после него не оживает — 9 сообщений и тишина против 241 за
             * 14 секунд после полного сброса. При сбросе заново поднимается
             * весь стек хоста, а не только питание порта.
             *
             * Замер второй, из-за которого способ и вернулся. Днём сброс
             * ломал security, и я счёл виновным сам сброс. Виновата была
             * не перезагрузка, а её последствие: TX после загрузки шлёт
             * личность руля заново, а донгл на **любую** присланную личность
             * переподключался к консоли и убивал её сессию. Это исправлено
             * (донгл сравнивает байты и на ту же личность шину не трогает),
             * и проверено на живом залипшем стенде: сброс передатчика поднял
             * руль — `delivered 267 -> 508`, 223 отчёта доехали до консоли
             * без потерь, — а сессия консоли **не шелохнулась**: `events` и
             * `announce` остались прежними.
             *
             * Что переживает перезагрузку: личность руля и адрес пира лежат
             * в NVS, знакомство не начинается с нуля. Что стоит: около
             * секунды на загрузку плюс энумерация и калибровочный проворот
             * руля — тот же, что при любом включении.
             *
             * Возврата из этого вызова нет.
             */
            G920_LOGW(M1, "restarting to relaunch the wheel — port re-power "
                          "alone leaves it silent");
            vTaskDelay(pdMS_TO_TICKS(50)); /* дать строке уйти в UART */
            esp_restart();
        relaunch_done:;
        }

        if (s_pending_gone) {
            s_pending_gone = false;
            G920_LOGW(M1, "device gone after %u messages in, %u errors",
                      (unsigned)s_in_msgs, (unsigned)s_in_errors);
            release_interface();
            if (s_device != NULL) {
                (void)usb_host_device_close(s_client, s_device);
                s_device = NULL;
            }
        }

        /*
         * Обмен: сначала отдать накопленное, потом дать последовательности
         * посмотреть на часы. Порядок важен — повтор запроса метаданных
         * имеет смысл только после того, как ушёл предыдущий.
         */
        /*
         * Такт линка **несколько раз за проход**: он отдаёт по кадру за
         * вызов, а консоль шлёт security пачками по четыре-пять. При одном
         * вызове на проход очередь разбирается медленнее, чем наполняется,
         * и повторы копятся — 67 повторов и один брошенный кадр за минуту
         * на живом стенде, а брошенный кадр рвёт security-обмен целиком.
         */
        for (int t = 0; t < 6; t++) {
            g920_link_tick(g920_timestamp_us());
        }
        tunnel_drain();

        /*
         * Донгл появился — а руль уже в чужой, состоявшейся сессии.
         * Перезагружаемся, чтобы он поднялся заново вместе со связкой.
         *
         * Повод такой же, как у донгла с обратной стороны: сессия, в
         * которой руль прошёл security с прежним собеседником, для нового
         * бесполезна, и сам он из неё не выйдет. Раньше это выправлялось
         * минутами — после снятия питания с руля консоль успевала прислать
         * **500** запросов начальных отчётов, прежде чем что-то сдвигалось.
         *
         * Условие «руль уже проходил security» обязательно и исключает
         * петлю: после перезагрузки руль девственный (`06 == 0`), и на
         * следующее появление пира мы не среагируем. Оно же не даёт
         * перезагружаться при обычном первом знакомстве.
         *
         * Время загрузки тоже проверяется: пир появляется сразу после
         * старта, и без этого мы бы перезагружались по кругу.
         */
        {
            const bool peer = g920_link_has_peer();
            const uint32_t now_ms = (uint32_t)(g920_timestamp_us() / 1000u);

            if (peer && !s_peer_seen && now_ms >= RELAUNCH_COOLDOWN_MS
                && s_wheel_msgs[0x06] != 0u) {
                G920_LOGW(M1, "dongle showed up while the wheel is in a stale "
                              "session — restarting to bring both up together");
                vTaskDelay(pdMS_TO_TICKS(50));
                esp_restart();
            }

            /*
             * Связка поднялась — переподключить руль **один раз**.
             *
             * Зачем именно в этот момент. Руль энумерируется первым, пока
             * стенд только просыпается: 24 В внутри руля ещё гуляют, радио
             * поднимается, преобразователь выходит на режим. Знакомство,
             * состоявшееся в этих условиях, живым не бывает — и это ровно
             * то, что человек чинил рукой, касаясь земли или втыкая мак.
             * Связь с донглом — последнее, что встаёт в этой цепочке, и
             * значит лучший из доступных признаков «стенд готов, можно
             * знакомиться начисто».
             *
             * Один раз за загрузку: переподключение стоит калибровочного
             * проворота, и повторять его на каждое появление пира значило
             * бы крутить руль без нужды. Жалоба «после включения руль
             * калибруется 2 раза» родилась ровно из такой щедрости.
             *
             * Работающий руль не трогаем: если знакомство дошло до active,
             * чинить нечего, а проворот человек увидит.
             */
            if (peer && !s_link_replug_done && now_ms >= RELAUNCH_COOLDOWN_MS) {
                s_link_replug_done = true;
                if (s_device == NULL) {
                    /*
                     * Руля на шине ещё нет — и переподключать нечего, и
                     * нельзя. 22.08.2026 лог поймал этот replug с поличным:
                     * он срабатывал через 30 мс после того, как руль
                     * наконец выставил подтяжку (G920 нужно ~6 с после
                     * VBUS), и снимал питание прямо под энумерацией — руль
                     * не вставал вовсе. Знакомство, которое начнётся позже
                     * связки, идёт в уже стабильном стенде: чинить его не
                     * от чего, replug лечит только знакомство, начатое до.
                     */
                    G920_LOGI(M1, "link with the dongle is up before the "
                                  "wheel — letting it arrive in peace");
                } else if (wheel_is_alive(now_ms)) {
                    G920_LOGI(M1, "link with the dongle is up and the wheel is "
                                  "already alive — leaving it alone");
                } else {
                    replug_wheel("link with the dongle is up");
                }
            }
            s_peer_seen = peer;
        }

        /*
         * Мёртвая рука. Проверяется каждым тактом и только по факту смены
         * состояния: слать Forces Off повторно бессмысленно, а слать Normal
         * без нужды — значит вернуть силы там, где их никто не просил.
         *
         * Условие «силы вообще были» обязательно: без него донгл, ещё ни
         * разу не подключавшийся, получал бы Forces Off при первом же
         * тайм-ауте пиринга, и первая же сила пришла бы в заглушенную петлю.
         */
        if (s_claimed && s_ffb_seen && s_last_link_ms != 0) {
            uint32_t now_ms = (uint32_t)(g920_timestamp_us() / 1000u);
            uint32_t quiet_ms = now_ms - s_last_link_ms;
            bool quiet = quiet_ms > DEADMAN_SILENCE_MS;

            if (quiet && !s_forces_off) {
                s_forces_off = true;
                send_loop_state(FFB_LOOP_FORCES_OFF, "dead man: forces off");
                G920_LOGW(M1, "dongle silent %u ms — forces off",
                          (unsigned)quiet_ms);
            } else if (!quiet && s_forces_off) {
                s_forces_off = false;
                send_loop_state(FFB_LOOP_NORMAL, "dead man: forces back");
                G920_LOGW(M1, "dongle speaks again — forces back to normal");
            }
        }

        /*
         * ⚠ Опыт с подтверждением **выключен** 03.08.2026.
         *
         * `probe_ack()` шлёт рулю сочинённое сообщение `0x06` — то есть
         * лезет ровно в тот канал, который мы чиним, и притом с выдуманным
         * Sequence ID (0x11). У security по спеке (§ «GIP Sequence ID»,
         * H001419) **свой пул номеров**, отдельный от глобального: чужая
         * запись в него сбивает счёт обеим настоящим сторонам разговора.
         * Свой вопрос опыт уже закрыл — руль на ACME отвечает, это видно в
         * живых трассах, — а вреда от него теперь больше, чем пользы.
         *
         * Не удалено, а обесточено: понадобится — включается одной строкой.
         */
        (void)probe_ack;
        (void)s_probe_at_us;
        s_probe_done = true;

        send_identity();
        if (s_dumping) {
            dump_trace_slice();
        }

        if (s_claimed) {
            try_send_next();

            {
                g920_gip_host_packet_t tick_out[G920_GIP_HOST_OUT_MAX];
                int count = g920_gip_host_tick(
                    &s_host, g920_timestamp_us() / 1000u, tick_out,
                    G920_GIP_HOST_OUT_MAX);

                for (int i = 0; i < count; i++) {
                    queue_packet(&tick_out[i]);
                }
            }

            /*
             * Печатается **один раз за подключение**, в первую же паузу.
             *
             * Раньше дамп перевзводился на каждом входящем — и пока руль
             * шлёт ввод, TX печатал непрерывно, растягивая проход цикла с
             * 2 мс до трёх десятков. Через этот же цикл идут повторы линка,
             * и security-обмен от этого шёл к рулю секундами. Трасса
             * знакомства снимается один раз, а дальше она только мешает.
             */
            /*
             * Дамп откладывается на 25 секунд после первого сообщения руля.
             *
             * Печать по первой тишине снимала знакомство, но обмен
             * аутентификации начинается позже — и в трассе его не было
             * никогда. Считать по счётчикам, что там происходит, я уже
             * пробовал: они показывают итог, а нужен порядок.
             */
            /*
             * ⚠ Автоматический дамп **выключен** 03.08.2026: он не
             * наблюдает обмен, а ломает его.
             *
             * Замерено в трассе `m8-tx-auth-loop`: на 25-й секунде дамп
             * начинает печатать, главный цикл перестаёт крутиться, и
             * **срабатывает task watchdog** (12 срабатываний за 25 секунд
             * захвата). Через этот же цикл идут `g920_link_tick` и
             * `tunnel_drain`, то есть на время печати труба стоит целиком —
             * а консоль отводит на ответ 100 мс.
             *
             * Это ровно та болезнь, которую в M1 уже лечили («печать в
             * горячем пути управляла временем на шине»), вернувшаяся через
             * другую дверь: там убрали печать **каждого** пакета, а здесь
             * осталась разовая печать **всей** трассы.
             *
             * Функция оставлена на месте: трасса пишется по-прежнему, и
             * снять её будет чем, когда для этого появится тихая минута.
             * Сам собой посреди security дамп больше не включается.
             */
            (void)s_trace_dumped;
            (void)dump_trace;

            if (!s_tunnel && s_host.metadata_ready && !s_identity_saved) {
                capture_identity();
            }

            /* Руль объявился заново — значит его сессия началась сначала,
             * и включать его надо тоже заново. */
            if (s_host.state == G920_GIP_HOST_ARRIVAL) {
                s_wheel_ready = false;
            }

            /*
             * Знакомство встало — вывести его из тупика (см. лестницу над
             * `HANDSHAKE_STALL_MS`).
             *
             * Часы идут от **смены состояния**, а не от захвата: пока
             * знакомство движется, застревания нет, а стоит оно ровно там,
             * где кончились собственные средства `gip_host` — в Arrival без
             * Hello и в Metadata после четвёртого запроса.
             */
            {
                uint32_t now_ms = (uint32_t)(g920_timestamp_us() / 1000u);

                if (s_host.state != s_handshake_state) {
                    s_handshake_state = s_host.state;
                    s_handshake_at_ms = now_ms;
                    s_handshake_nudges = 0;
                    s_handshake_rounds = 0;
                }
                /*
                 * ⚠ Живой руль не чинят. Отчёты ввода идут — значит он
                 * включён и работает, кто бы его ни включил; наше знакомство
                 * при этом может стоять в `arrival` навсегда, и это не
                 * неисправность, а разделение труда с консолью.
                 *
                 * Часы застревания при этом **взводятся заново**: если поток
                 * ввода когда-нибудь оборвётся, отсчёт пойдёт от обрыва, а
                 * не от давно прошедшей смены состояния.
                 */
                if (wheel_is_alive(now_ms)) {
                    s_handshake_at_ms = now_ms;
                    s_handshake_nudges = 0;
                }
                /*
                 * Отчаявшись — сбавить шаг, а не долбить.
                 *
                 * Три круга не помогли — значит дело не в знакомстве, а
                 * каждый круг стоит рулю калибровочного проворота, то есть
                 * механики. Дальше пробуем раз в пять минут: мост не
                 * сдаётся, но и не изнашивает то, что чинит.
                 */
                else if (s_host.state != G920_GIP_HOST_ACTIVE
                    && (uint32_t)(now_ms - s_handshake_at_ms)
                           >= ((s_handshake_rounds < HANDSHAKE_ROUNDS_BRISK)
                                   ? HANDSHAKE_STALL_MS
                                   : HANDSHAKE_STALL_SLOW_MS)) {
                    /* Печатаем первый круг подробно, дальше редко: круги
                     * бесконечны, и строка на каждый сделала бы журнал
                     * нечитаемым ровно тогда, когда по нему разбирают,
                     * почему руль не включился. */
                    const bool loud = (s_handshake_rounds == 0);

                    s_handshake_at_ms = now_ms;
                    s_handshake_nudges++;
                    if (s_handshake_nudges <= HANDSHAKE_NUDGES_MAX) {
                        /* Тот же `Set Device State: Reset` из спеки, что
                         * шлётся по просьбе донгла: устройство возвращается
                         * в Arrival и представляется заново. */
                        static const uint8_t reset[] = { 0x05, 0x20, 0x01,
                                                         0x01, 0x07 };

                        if (loud) {
                            G920_LOGW(M1,
                                      "handshake stuck in %s for %u ms "
                                      "(hellos %u, metadata asked %u) — asking "
                                      "the wheel to start over (%u of %u)",
                                      g920_gip_host_state_name(s_host.state),
                                      (unsigned)HANDSHAKE_STALL_MS,
                                      (unsigned)s_host.hellos,
                                      (unsigned)s_host.metadata_requests,
                                      (unsigned)s_handshake_nudges,
                                      (unsigned)HANDSHAKE_NUDGES_MAX);
                        }
                        /*
                         * И **своё** знакомство с чистого листа: счёт
                         * запросов метаданных уже упёрся в предел спеки, и
                         * без сброса тик не пошлёт ни одного нового запроса,
                         * даже когда руль снова представится.
                         */
                        g920_gip_host_init(&s_host, s_metadata,
                                           sizeof(s_metadata));
                        s_handshake_state = s_host.state;
                        s_metadata_logged = false;
                        s_wheel_ready = false;
                        queue_bytes(reset, sizeof(reset),
                                    "set device state: reset (handshake stuck)");
                        try_send_next();
                    } else {
                        /*
                         * Просьбу руль не услышал — остаётся сброс шины.
                         * Он возвращает устройство в Arrival заведомо:
                         * энумерацию проигнорировать нельзя, в отличие от
                         * сообщения.
                         *
                         * Порядок обязателен и тот же, что при переподнятии
                         * по просьбе донгла: сперва отпустить интерфейс и
                         * закрыть устройство, потом трогать питание порта.
                         */
                        s_handshake_nudges = 0;
                        s_handshake_rounds++;
                        G920_LOGW(M1,
                                  "wheel ignored %u nudges in %s — "
                                  "re-enumerating it (round %u)",
                                  (unsigned)HANDSHAKE_NUDGES_MAX,
                                  g920_gip_host_state_name(s_host.state),
                                  (unsigned)s_handshake_rounds);
                        release_interface();
                        if (s_device != NULL) {
                            (void)usb_host_device_close(s_client, s_device);
                            s_device = NULL;
                        }
                        /*
                         * Второй круг — значит и энумерация не помогла.
                         * Дальше только полный сброс: он один поднимает
                         * стек и PHY заново (замер 03.08.2026).
                         */
                        if (s_handshake_rounds >= 2) {
                            restart_self("wheel never reached active");
                        }
                        kick_root_port();
                    }
                }
            }

            /* Метаданные собраны — включаем руль по-настоящему. */
            if (s_host.metadata_ready && !s_wheel_ready) {
                s_wheel_ready = true;
                queue_bytes(POWER_ON, sizeof(POWER_ON),
                            "power on (console string)");
                queue_bytes(POWER_ON_SINGLE, sizeof(POWER_ON_SINGLE),
                            "power on (single)");
                try_send_next();
                G920_LOGW(M1, "wheel powered on the way a console does");
            }

            if (s_host.metadata_ready && !s_metadata_logged) {
                size_t length = 0;
                const uint8_t *blob = g920_gip_host_metadata(&s_host, &length);

                s_metadata_logged = true;
                G920_LOGI(M1, "metadata complete: %u bytes", (unsigned)length);
                if (blob != NULL) {
                    /* Сырьём и целиком: в M4 эти байты станут личностью
                     * руля, и сверяться они будут побайтово (И1). */
                    dump("metadata", blob, length);
                }
            }
        }

        since_blink_ms += TICK_MS;
        if (since_blink_ms >= 200) {
            since_blink_ms = 0;
            on = !on;
            /* Зелёный — устройство открыто, жёлтый — ждём. */
            g920_board_led_set(
                (s_device != NULL) ? (on ? G920_IND_OK : G920_IND_OFF)
                                   : (on ? G920_IND_DETECT : G920_IND_OFF));
        }

        since_note_ms += TICK_MS;
        if (since_note_ms >= 5000) {
            since_note_ms = 0;
            if (s_device == NULL) {
                usb_host_lib_info_t info;

                if (usb_host_lib_info(&info) == ESP_OK) {
                    G920_LOGI(M1, "waiting: devices %d, clients %d",
                              info.num_devices, info.num_clients);
                }
                /*
                 * Устройство на шине есть, а события о нём не было.
                 *
                 * Так у ESP-IDF выглядит подключение, случившееся не вовремя
                 * (esp-idf#10086): стек энумерировал руль, а клиенту не
                 * сказал. Раз стек сам называет число устройств, спросим у
                 * него адрес и откроем — это честнее, чем ждать события,
                 * которого уже не будет. Проверено 02.08.2026: после
                 * снятия и возврата питания руля лог показывал
                 * `devices 1, clients 1` и стоял так бесконечно.
                 */
                if (info.num_devices > 0) {
                    uint8_t addrs[4];
                    int found = 0;
                    esp_err_t listed = usb_host_device_addr_list_fill(
                        sizeof(addrs), addrs, &found);

                    /*
                     * Устройство числится на шине, но адреса у него нет —
                     * значит энумерация встала на полпути
                     * (`CHECK_SHORT_DEV_DESC FAILED` в логе стека). Само оно
                     * из этого состояния не выйдет: помогает только снятие
                     * питания с корневого порта, то есть то же, что вынуть и
                     * воткнуть кабель.
                     */
                    if (listed != ESP_OK || found == 0) {
                        /* Тоже без предела: застрявшая энумерация сама не
                         * рассосётся, а сдавшийся мост оставляет руль
                         * обесточенным до перезагрузки руками. */
                        kick_root_port();
                    } else if (found > 0) {
                        G920_LOGW(M1, "device %u is on the bus but no event "
                                      "came — opening it directly",
                                  (unsigned)addrs[0]);
                        s_pending_addr = addrs[0];
                        s_pending_new = true;
                    }
                } else {
                    /*
                     * ⚠ Пробуем **бесконечно**, а не три раза.
                     *
                     * Здесь стоял предел в три попытки, после которого
                     * передатчик писал «не помогло» и переставал пытаться
                     * навсегда. Это прямой дефект: руль может появиться
                     * позже нас. При холодном включении он готовится дольше,
                     * чем грузится плата, и три попытки успевают истратиться
                     * впустую — дальше мост стоит с обесточенным портом и
                     * ждёт события, которого уже некому породить.
                     *
                     * Так это и проявлялось у человека 03.08.2026: руль
                     * поднимался только если плата стартовала **с
                     * подключённым переходником** — тот задерживает загрузку
                     * схемой автосброса, и руль успевал. Без переходника
                     * `waiting: devices 0` держалось бесконечно.
                     *
                     * Мост, сдавшийся навсегда, хуже моста, который
                     * продолжает пробовать: попытка стоит передёргивания
                     * порта раз в несколько секунд и ничего не ломает, а
                     * отказ стоит неработающего руля до перезагрузки руками.
                     *
                     * Печатаем редко — иначе журнал становится нечитаемым
                     * ровно тогда, когда по нему разбирают «почему руля нет».
                     */
                    kick_root_port();
                }
                /*
                 * Порт передёрнут трижды, а руля на шине так и нет — этим
                 * средством больше ничего не добьёшься. Перезагружаемся:
                 * поднимется весь стек хоста и сам PHY, а не только питание
                 * порта. Ограничитель и обоснование — у `restart_self`.
                 */
                /*
                 * Порог перезагрузки — 7, а не 3, и это не щедрость.
                 * Каждая третья попытка передёргивает VBUS (см.
                 * kick_root_port), и рулю после этого нужно время дойти до
                 * подтяжки и энумерации. Лог 22.08.2026 поймал рестарт,
                 * прилетевший через миг после цикла VBUS третьей попытки —
                 * руль уже возвращался, а мост его не дождался. Семь
                 * попыток — это два полных круга с циклами VBUS (3-я и
                 * 6-я) и время на детект после каждого.
                 */
                if (s_repower_attempts >= 7u) {
                    restart_self("no wheel on the bus after re-powering the "
                                 "port");
                }
            } else if (!s_claimed) {
                /*
                 * Устройство открыто, а интерфейс не взят — и это тоже было
                 * тупиком навсегда.
                 *
                 * Захват пробуется один раз, при разборе события; отказ
                 * печатался строкой «listening did not start», и на этом всё
                 * кончалось: события больше не будет, а без интерфейса нет
                 * ни опроса, ни отправки. Снаружи это ровно то же, что и
                 * застрявшее знакомство, — руль на шине, и он мёртв.
                 */
                G920_LOGW(M1, "device is open but the interface is not "
                              "claimed — trying again");
                if (!claim_and_listen()) {
                    release_interface();
                    (void)usb_host_device_close(s_client, s_device);
                    s_device = NULL;
                    kick_root_port();
                }
            } else if (s_claimed && s_in_msgs == 0) {
                /*
                 * Пять секунд — это десять пропущенных Hello. Молчание при
                 * живом опросе значит другое, чем молчание без него, и
                 * назвать его вслух дешевле, чем гадать по логу потом.
                 */
                G920_LOGW(M1,
                          "polling for %u s, wheel said nothing (in running: "
                          "%s, errors %u)",
                          (unsigned)(g920_timestamp_us() - s_claim_us)
                              / 1000000u,
                          s_in_running ? "yes" : "no", (unsigned)s_in_errors);
            } else if (s_claimed) {
                G920_LOGI(M1,
                          "tunnel: sent %u, dropped %u | link: retries %u, "
                          "gave up %u, pending %u, send failed %u",
                          (unsigned)s_tunnel_sent, (unsigned)s_tunnel_dropped,
                          (unsigned)g920_link_reliable_retries(),
                          (unsigned)g920_link_reliable_gave_up(),
                          (unsigned)g920_link_reliable_pending(),
                          (unsigned)g920_link_reliable_send_failed());
                if (s_ffb_lat_n > 0) {
                    G920_LOGI(M1, "ffb latency: avg %u us, max %u us, n %u",
                              (unsigned)(s_ffb_lat_sum / s_ffb_lat_n),
                              (unsigned)s_ffb_lat_max,
                              (unsigned)s_ffb_lat_n);
                }
                {
                    char list[128];
                    int off = 0;

                    /* Все ненулевые номера: считать только ожидаемые — значит
                     * не увидеть неожиданное. Ответ на аутентификацию,
                     * например, бывает и сообщением 0x1E. */
                    for (int i = 0; i < 64; i++) {
                        if (s_wheel_msgs[i] != 0 && off < (int)sizeof(list) - 16) {
                            off += snprintf(list + off, sizeof(list) - off,
                                            "%02x=%u ", i,
                                            (unsigned)s_wheel_msgs[i]);
                        }
                    }
                    list[(off > 0) ? off : 0] = '\0';
                    G920_LOGI(M1, "wheel says: %s", list);
                }
                {
                    char list[128];
                    int off = 0;

                    for (int i = 0; i < 64; i++) {
                        if (s_to_wheel_msgs[i] != 0
                            && off < (int)sizeof(list) - 16) {
                            off += snprintf(list + off, sizeof(list) - off,
                                            "%02x=%u ", i,
                                            (unsigned)s_to_wheel_msgs[i]);
                        }
                    }
                    list[(off > 0) ? off : 0] = '\0';
                    G920_LOGI(M1, "to wheel: %s", list);
                }
                /*
                 * Состояние знакомства в отчёте — потому что без него
                 * «руль не включился» и «руль не ответил» выглядят в логе
                 * одинаково, а чинятся в разных местах.
                 */
                G920_LOGI(M1,
                          "handshake: %s, hellos %u, metadata asked %u, "
                          "ready %s, powered on %s",
                          g920_gip_host_state_name(s_host.state),
                          (unsigned)s_host.hellos,
                          (unsigned)s_host.metadata_requests,
                          s_host.metadata_ready ? "yes" : "no",
                          s_wheel_ready ? "yes" : "no");
                G920_LOGI(M1,
                          "in: %u msgs, %u errors | from wheel: 02=%u 03=%u "
                          "04=%u 06=%u 20=%u 21=%u",
                          (unsigned)s_in_msgs, (unsigned)s_in_errors,
                          (unsigned)s_wheel_msgs[0x02], (unsigned)s_wheel_msgs[0x03],
                          (unsigned)s_wheel_msgs[0x04], (unsigned)s_wheel_msgs[0x06],
                          (unsigned)s_wheel_msgs[0x00], (unsigned)s_wheel_msgs[0x01]);
            }
        }
    }
}

#endif /* G920_MODE_HOST */
