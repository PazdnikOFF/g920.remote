/*
 * ⚠ Прошивки RX разведены **флагом сборки**, а не `build_src_filter`: при
 * `framework = espidf` PlatformIO собирает `src/` через CMake, и фильтр
 * scons там не действует вовсе — проверено, оба `main` попадали в сборку и
 * не линковались. Поэтому файл, не относящийся к профилю, компилируется
 * пустым. Это не изящно, зато честно: невыполняемая настройка выглядела бы
 * работающей.
 */
#include "gip_device.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_private/usb_phy.h"

#include "device/usbd_pvt.h"
#include "tusb.h"

#include "g920/gip_stub.h"
#include "g920/identity.h"
#include "g920/hexdump.h"
#include "g920/log.h"
#include "g920/timestamp.h"

#define TAG "usbdev"

#ifdef G920_MODE_GIP

/*
 * Заглушечные идентификаторы. **Не логитековские и не должны ими быть**
 * (И2): пока личность не приехала от TX, устройство обязано выглядеть
 * посторонним. Настоящие VID/PID появятся в M4 и придут параметром.
 */
#define STUB_VID 0x1209u /* pid.codes, диапазон для своих поделок */
#define STUB_PID 0x0001u
#define STUB_RELEASE 0x0100u

static g920_hostlog_t *s_log;
static SemaphoreHandle_t s_log_lock;
static usb_phy_handle_t s_phy;

/* Дескрипторы собираются один раз при инициализации — из заглушки. */
/*
 * Личность руля, если она уже приехала с TX.
 *
 * Указатель, а не копия: контейнер лежит в буфере вызывающего и живёт всю
 * работу прошивки. Копировать значило бы держать вторую истину о том, чем
 * донгл представляется, — а расхождение между заявленной личностью и той,
 * за которую ручается security-обмен, это как раз тот баг, который потом
 * не воспроизводится.
 *
 * Пока личности нет, отдаётся заглушка M2. Это не «режим по умолчанию», а
 * честное «донгл ещё не знаком с рулём»: у заглушки чужие VID/PID и пустой
 * Announce, и ни один хост не примет её за игровое устройство.
 */
static const g920_identity_t *s_identity;

/*
 * Приём сообщений от хоста. Колбэк зовётся из задачи USB — значит копия и
 * флаг, а вся работа в главном цикле. Раньше принятое только считалось в
 * журнале: заглушке отвечать было нечем, а теперь есть чем.
 */
static g920_gip_device_rx_cb_t s_on_message;
static void *s_on_message_ctx;

void g920_gip_device_set_rx(g920_gip_device_rx_cb_t cb, void *ctx)
{
    s_on_message = cb;
    s_on_message_ctx = ctx;
}

static const uint8_t *identity_section(g920_identity_section_t type,
                                       uint8_t index)
{
    uint16_t length = 0;

    if (s_identity == NULL) {
        return NULL;
    }
    return g920_identity_find(s_identity, type, index, &length);
}

static uint8_t s_device_desc[G920_STUB_DEVICE_DESC_SIZE];
static uint8_t s_config_desc[G920_STUB_CONFIG_DESC_SIZE];
static uint8_t s_msos_string[G920_MSOS_STRING_SIZE];
static uint8_t s_msos_compat[G920_MSOS_COMPAT_ID_SIZE];

/* Состояние нашего класса. */
static struct {
    uint8_t itf;
    uint8_t ep_in;
    uint8_t ep_out;
    bool open;
    volatile bool in_busy;
    volatile bool configured;
    /* Шина усыплена хостом: молчим до resume. */
    volatile bool suspended;
    volatile bool in_polled_seen;
} s_gip;

/*
 * Сколько своих посылок класть в журнал. Дальше только счётчик: журнал
 * описывает хост, и забивать его собой значит потерять его смысл.
 */
#define GIP_OUT_LOGGED_MAX 3

static uint32_t s_sent_total;

/* Буферы DMA: внутренняя память, выравнивание по 4 — требование dwc2. */
CFG_TUSB_MEM_ALIGN static uint8_t s_out_buf[G920_GIP_USB_EP_SIZE];
CFG_TUSB_MEM_ALIGN static uint8_t s_in_buf[G920_GIP_USB_EP_SIZE];

/* --- журнал --------------------------------------------------------------- */

void g920_gip_device_lock(void)
{
    if (s_log_lock != NULL) {
        xSemaphoreTake(s_log_lock, portMAX_DELAY);
    }
}

void g920_gip_device_unlock(void)
{
    if (s_log_lock != NULL) {
        xSemaphoreGive(s_log_lock);
    }
}

/*
 * Событие в журнал. Зовётся из задачи tusb, поэтому под замком: приложение
 * читает журнал из своей задачи, а частичная запись выглядела бы как
 * событие, которого не было.
 */
static void note(g920_host_event_type_t type)
{
    if (s_log == NULL) {
        return;
    }
    g920_gip_device_lock();
    (void)g920_hostlog_add(s_log, g920_timestamp_us(), type);
    g920_gip_device_unlock();
}

static void note_setup(const tusb_control_request_t *r,
                       g920_host_answer_t answer)
{
    if (s_log == NULL || r == NULL) {
        return;
    }
    g920_gip_device_lock();
    (void)g920_hostlog_add_setup(s_log, g920_timestamp_us(),
                                 r->bmRequestType, r->bRequest, r->wValue,
                                 r->wIndex, r->wLength, answer);
    g920_gip_device_unlock();
}

/* --- дескрипторы ---------------------------------------------------------- */

/*
 * Все три колбэка — наши, и это главная причина, по которой взят голый
 * TinyUSB. Обёртка реализует их сама, а нам нужно и видеть каждый запрос,
 * и отвечать на индекс 0xEE, которого в её массиве строк быть не может.
 */

/*
 * Запрос дескриптора — в журнал.
 *
 * Это выяснилось на живом маке и стоило вехе половины смысла: стандартные
 * control-запросы обрабатывает **ядро** TinyUSB и зовёт эти колбэки
 * напрямую. Ни до класса, ни до vendor-хука они не доходят, поэтому первая
 * версия писала ноль GET_DESCRIPTOR при полностью прошедшей энумерации —
 * устройство на шине было видно, а трассы, ради которой веха и делается,
 * не было вовсе.
 *
 * Собираем wValue обратно (тип в старшем байте, индекс в младшем): в этом
 * виде его ждёт `g920_hostlog_saw_descriptor`, по нему же считается
 * «спрашивал ли хост Device Qualifier».
 */
static void note_descriptor(uint8_t type, uint8_t index,
                            g920_host_answer_t answer)
{
    if (s_log == NULL) {
        return;
    }
    g920_gip_device_lock();
    (void)g920_hostlog_add_setup(s_log, g920_timestamp_us(), 0x80u,
                                 G920_USB_REQ_GET_DESCRIPTOR,
                                 (uint16_t)((uint16_t)type << 8 | index), 0, 0,
                                 answer);
    g920_gip_device_unlock();
}

uint8_t const *tud_descriptor_device_cb(void)
{
    const uint8_t *real = identity_section(G920_ID_DEVICE_DESC, 0);

    note_descriptor(TUSB_DESC_DEVICE, 0, G920_HOST_ANSWER_DATA);
    return (real != NULL) ? real : s_device_desc;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    const uint8_t *real = identity_section(G920_ID_CONFIG_DESC, 0);

    note_descriptor(TUSB_DESC_CONFIGURATION, index, G920_HOST_ANSWER_DATA);
    return (real != NULL) ? real : s_config_desc;
}

/*
 * Device Qualifier. Full-speed устройству он не положен, и правильный
 * ответ — STALL. Колбэк слабый и по умолчанию молчит, но тогда запрос не
 * попал бы в журнал — а спросил его хост или нет, это **признак
 * платформы**: Windows и Xbox спрашивают, macOS нет.
 *
 * Решение **сверяется** с `g920_usb_should_stall_get_descriptor` — и это не
 * украшение.
 * Раньше здесь стоял безусловный `NULL`, а комментарий уверял, что
 * «политика в предикате, здесь исполнение». Это была неправда: предикат не
 * звался ниоткуда, знание «тип 0x06 → STALL» лежало в двух несвязанных
 * местах, и поменять его можно было так, что все тесты остались бы
 * зелёными, а поведение — прежним. Ровно этим соседняя веха M3
 * блокировалась девять раз подряд.
 */
uint8_t const *tud_descriptor_device_qualifier_cb(void)
{
    uint16_t w_value = (uint16_t)G920_USB_DESC_DEVICE_QUALIFIER << 8;

    if (!g920_usb_should_stall_get_descriptor(w_value)) {
        /*
         * Предикат — источник правды, а не подсказка. Сказал «отвечать» —
         * а отвечать нечем: дескриптора Device Qualifier у заглушки нет и
         * по спеке не должно быть. Значит разошлись политика и
         * возможности, и молчать об этом нельзя.
         */
        G920_LOGE(TAG, "device qualifier: policy says answer, nothing to send");
    }
    note_descriptor(G920_USB_DESC_DEVICE_QUALIFIER, 0,
                    G920_HOST_ANSWER_STALL);
    return NULL;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;

    /*
     * MS OS Descriptor. Заглушка **не объявляет ни одной обычной строки**
     * намеренно (И2: строки — часть личности), поэтому любой другой индекс
     * получает STALL, и это осмысленный ответ, а не дыра.
     */
    /*
     * Строки настоящего руля, если личность приехала. Отдаются **как
     * сняты** — уже в UTF-16 вместе с заголовком, ровно теми байтами,
     * которые видел USB-хост на TX (И1).
     */
    if (index != G920_MSOS_STRING_INDEX) {
        const uint8_t *real = identity_section(G920_ID_STRING_DESC, index);

        if (real != NULL) {
            note_descriptor(TUSB_DESC_STRING, index, G920_HOST_ANSWER_DATA);
            return (uint16_t const *)(const void *)real;
        }
    }

    if (index == G920_MSOS_STRING_INDEX) {
        note_descriptor(TUSB_DESC_STRING, index, G920_HOST_ANSWER_DATA);
        /* Строка уже собрана в UTF-16 вместе с заголовком — отдаём как
         * есть. Выравнивание гарантировано типом. */
        return (uint16_t const *)(const void *)s_msos_string;
    }
    note_descriptor(TUSB_DESC_STRING, index, G920_HOST_ANSWER_STALL);
    return NULL;
}

/*
 * Vendor-запрос MS OS. Приходит на устройство, а не на интерфейс, поэтому
 * до класса не доходит и обрабатывается здесь.
 *
 * Extended Compatible ID (wIndex 0x0004) отвечается, Extended Properties
 * (0x0005) получает STALL — политика живёт в `g920/gip_stub.h` и покрыта
 * тестами там же. Здесь только исполнение.
 */
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request)
{
    if (stage != CONTROL_STAGE_SETUP) {
        return true;
    }
    if (request->bRequest != G920_MSOS_VENDOR_CODE) {
        note_setup(request, G920_HOST_ANSWER_STALL);
        return false;
    }
    if (g920_usb_should_stall_ms_os(request->wIndex)) {
        note_setup(request, G920_HOST_ANSWER_STALL);
        return false;
    }
    note_setup(request, G920_HOST_ANSWER_DATA);
    return tud_control_xfer(rhport, request, s_msos_compat,
                            (uint16_t)sizeof(s_msos_compat));
}

/* --- состояние шины ------------------------------------------------------- */

void tud_mount_cb(void)
{
    s_gip.configured = true;
    note(G920_HOST_EV_SET_CONFIG);
}

void tud_umount_cb(void)
{
    s_gip.configured = false;
    note(G920_HOST_EV_BUS_RESET);
}

/*
 * Suspend — это **приказ замолчать**, а не просто событие для журнала.
 *
 * Спека требует прямо (`H001419`, § «Enumeration»): «USB Reset and USB
 * Suspend must be handled by all GIP states». До 03.08.2026 мы их только
 * записывали: донгл продолжал слать Announce раз в 500 мс независимо от
 * состояния шины — 24746 штук за один сеанс.
 *
 * Цена оказалась видна не в трассе, а в комнате: **бокс переставал
 * выключаться и включался сам**. Уснувшая шина видит устройство, которое
 * продолжает в неё говорить, и просыпается. На девките это ещё заметнее —
 * донгл питается от отладочного порта, а не от шины, и переживает уход
 * консоли целиком.
 *
 * Удалённого пробуждения мы не заявляли (`bmAttributes` = 0x80), то есть
 * права будить хост у нас нет вовсе. Молчим до `resume`.
 */
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    s_gip.suspended = true;
    note(G920_HOST_EV_SUSPEND);
}

void tud_resume_cb(void)
{
    s_gip.suspended = false;
    note(G920_HOST_EV_RESUME);
}

bool g920_gip_device_suspended(void)
{
    return s_gip.suspended;
}

/* --- свой класс ----------------------------------------------------------- */

static void gip_init(void)
{
    memset(&s_gip, 0, sizeof(s_gip));
}

static bool gip_deinit(void)
{
    return true;
}

static void gip_reset(uint8_t rhport)
{
    (void)rhport;
    s_gip.open = false;
    s_gip.in_busy = false;
    s_gip.configured = false;
}

static uint16_t gip_open(uint8_t rhport, tusb_desc_interface_t const *itf,
                         uint16_t max_len)
{
    uint8_t const *p;
    uint16_t claimed;

    /* Наш интерфейс — только тот, что объявляет класс GIP. Чужие не
     * трогаем: заявить чужой интерфейс значит молча сломать его. */
    if (itf->bInterfaceClass != G920_GIP_USB_CLASS
        || itf->bInterfaceSubClass != G920_GIP_USB_SUBCLASS
        || itf->bInterfaceProtocol != G920_GIP_USB_PROTOCOL) {
        G920_LOGW(TAG, "open: not ours, class %02x/%02x/%02x",
                  itf->bInterfaceClass, itf->bInterfaceSubClass,
                  itf->bInterfaceProtocol);
        return 0;
    }

    claimed = (uint16_t)sizeof(tusb_desc_interface_t);
    p = (uint8_t const *)itf + sizeof(tusb_desc_interface_t);
    s_gip.itf = itf->bInterfaceNumber;

    for (uint8_t i = 0; i < itf->bNumEndpoints; i++) {
        tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *)p;

        if (claimed + ep->bLength > max_len
            || ep->bDescriptorType != TUSB_DESC_ENDPOINT) {
            /* Отказ здесь роняет SET_CONFIGURATION целиком, а наружу это
             * выходит как «устройство видно, но не настроено» — молчать
             * про такое нельзя, диагноз иначе не поставить. */
            G920_LOGE(TAG,
                      "open: bad endpoint %u, type %02x, claimed %u of %u",
                      (unsigned)i, ep->bDescriptorType, (unsigned)claimed,
                      (unsigned)max_len);
            return 0;
        }
        if (!usbd_edpt_open(rhport, ep)) {
            G920_LOGE(TAG, "open: edpt_open failed for %02x",
                      ep->bEndpointAddress);
            return 0;
        }
        if (tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_IN) {
            s_gip.ep_in = ep->bEndpointAddress;
        } else {
            s_gip.ep_out = ep->bEndpointAddress;
        }
        claimed = (uint16_t)(claimed + ep->bLength);
        p += ep->bLength;
    }

    s_gip.open = true;
    G920_LOGI(TAG, "open: itf %u, in %02x, out %02x, claimed %u",
              (unsigned)s_gip.itf, s_gip.ep_in, s_gip.ep_out,
              (unsigned)claimed);
    /* OUT взводим сразу: пока он не взведён, хост, начавший слать, получит
     * NAK, и в трассе это будет выглядеть как молчание устройства. */
    if (s_gip.ep_out != 0) {
        (void)usbd_edpt_xfer(rhport, s_gip.ep_out, s_out_buf,
                             sizeof(s_out_buf));
    }
    return claimed;
}

/*
 * Control-запросы, адресованные нашему интерфейсу. Своих у GIP нет — всё
 * идёт по interrupt-эндпоинтам, — но записать их надо: хост, спросивший
 * что-то у интерфейса, этим себя и выдаёт.
 */
static bool gip_control_xfer(uint8_t rhport, uint8_t stage,
                             tusb_control_request_t const *request)
{
    (void)rhport;
    if (stage != CONTROL_STAGE_SETUP) {
        return true;
    }
    note_setup(request, G920_HOST_ANSWER_STALL);
    return false;
}

static bool gip_xfer(uint8_t rhport, uint8_t ep_addr, xfer_result_t result,
                     uint32_t xferred)
{
    uint64_t now = g920_timestamp_us();

    if (ep_addr == s_gip.ep_out) {
        if (result == XFER_RESULT_SUCCESS && xferred > 0 && s_log != NULL) {
            /*
             * Нагрузку **не разбираем** (И1): в журнал идут только тип
             * сообщения из первого байта заголовка и длина. Что внутри —
             * дело туннеля, а не наблюдателя.
             */
            g920_gip_device_lock();
            (void)g920_hostlog_add_gip(s_log, now, true, s_out_buf[0],
                                       (uint16_t)xferred);
            g920_gip_device_unlock();
        }
        if (result == XFER_RESULT_SUCCESS && xferred > 0
            && s_on_message != NULL) {
            s_on_message(s_on_message_ctx, s_out_buf, (uint16_t)xferred);
        }
        (void)usbd_edpt_xfer(rhport, s_gip.ep_out, s_out_buf,
                             sizeof(s_out_buf));
        return true;
    }

    if (ep_addr == s_gip.ep_in) {
        s_gip.in_busy = false;
        /*
         * Первая успешная передача в IN означает, что хост его **опросил**:
         * до этого transfer висит взведённым и не завершается. Это и есть
         * событие, от которого считается T2.
         */
        if (!s_gip.in_polled_seen) {
            s_gip.in_polled_seen = true;
            g920_gip_device_lock();
            (void)g920_hostlog_add(s_log, now, G920_HOST_EV_IN_POLLED);
            g920_gip_device_unlock();
        }
    }
    return true;
}

static usbd_class_driver_t const s_driver = {
    .name = "GIP",
    .init = gip_init,
    .deinit = gip_deinit,
    .reset = gip_reset,
    .open = gip_open,
    .control_xfer_cb = gip_control_xfer,
    .xfer_cb = gip_xfer,
    .xfer_isr = NULL,
    .sof = NULL,
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count)
{
    *driver_count = 1;
    return &s_driver;
}

/* --- наружу --------------------------------------------------------------- */

/*
 * Установить личность. Если хост уже настроил заглушку, устройство
 * **переподключается**: дескрипторы читаются один раз при энумерации, и
 * подменить их на лету нельзя — хост о подмене не узнает.
 */
void g920_gip_device_set_identity(const g920_identity_t *identity)
{
    bool was_mounted = tud_mounted();

    s_identity = identity;
    if (was_mounted) {
        G920_LOGI(TAG, "identity installed, re-enumerating");
        tud_disconnect();
        vTaskDelay(pdMS_TO_TICKS(150));
        tud_connect();
    } else {
        G920_LOGI(TAG, "identity installed before enumeration");
    }
}

uint32_t g920_gip_device_sent(void)
{
    return s_sent_total;
}

bool g920_gip_device_configured(void)
{
    return s_gip.configured && s_gip.open;
}

bool g920_gip_device_send(const uint8_t *data, uint16_t length)
{
    /*
     * Отказ на усыплённой шине — не оптимизация, а обязанность. Проверка
     * стоит здесь, а не у каждого вызывающего: замолчать должны **все**
     * источники разом, иначе достаточно забыть один.
     */
    if (s_gip.suspended) {
        return false;
    }

    if (!g920_gip_device_configured() || data == NULL || length == 0
        || length > sizeof(s_in_buf)) {
        return false;
    }
    if (s_gip.in_busy || !usbd_edpt_claim(0, s_gip.ep_in)) {
        return false;
    }
    memcpy(s_in_buf, data, length);
    s_gip.in_busy = true;
    if (!usbd_edpt_xfer(0, s_gip.ep_in, s_in_buf, length)) {
        s_gip.in_busy = false;
        (void)usbd_edpt_release(0, s_gip.ep_in);
        return false;
    }
    /*
     * В журнал идут только первые несколько своих посылок.
     *
     * Журнал заведён под поведение **хоста**, а Hello раз в полсекунды за
     * несколько минут вытесняет из него всё остальное: на живом прогоне
     * счётчик потерь пошёл расти, и терялись при этом события хоста —
     * ровно то, ради чего журнал существует. Первых нескольких довольно,
     * чтобы увидеть, что мы вообще начали слать; остальные считает
     * отдельный счётчик, который ничего не вытесняет.
     */
    s_sent_total++;
    if (s_log != NULL && s_sent_total <= GIP_OUT_LOGGED_MAX) {
        g920_gip_device_lock();
        (void)g920_hostlog_add_gip(s_log, g920_timestamp_us(), false, data[0],
                                   length);
        g920_gip_device_unlock();
    }
    return true;
}

/* Дамп буфера в лог построчно: готовой функции «весь буфер» нет, есть
 * форматирование одной строки — она и нужна, чтобы строки шли через тот же
 * приёмник лога, что и всё остальное. */
static void dump(const char *what, const uint8_t *data, size_t len)
{
    char line[G920_HEXDUMP_LINE_MAX];
    size_t lines = g920_hexdump_line_count(len);

    G920_LOGI(TAG, "%s, %u bytes:", what, (unsigned)len);
    for (size_t i = 0; i < lines; i++) {
        size_t offset = i * G920_HEXDUMP_BYTES_PER_LINE;
        size_t chunk = len - offset;

        if (chunk > G920_HEXDUMP_BYTES_PER_LINE) {
            chunk = G920_HEXDUMP_BYTES_PER_LINE;
        }
        if (g920_hexdump_line(line, sizeof(line), offset, data + offset, chunk)
            > 0) {
            G920_LOGI(TAG, "%s", line);
        }
    }
}

static void tusb_task(void *arg)
{
    (void)arg;
    for (;;) {
        tud_task();
    }
}

esp_err_t g920_gip_device_init(g920_hostlog_t *log)
{
    g920_stub_ids_t ids = { STUB_VID, STUB_PID, STUB_RELEASE };
    usb_phy_config_t phy = {
        .controller = USB_PHY_CTRL_OTG,
        .target = USB_PHY_TARGET_INT,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .otg_speed = USB_PHY_SPEED_UNDEFINED,
        .ext_io_conf = NULL,
        .otg_io_conf = NULL,
    };

    s_log = log;
    s_log_lock = xSemaphoreCreateMutex();
    if (s_log_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /*
     * Дескрипторы — из заглушки, ни одного байта здесь. Это и есть И2 в
     * исполнении: единственное место, где живут байты личности, — общий
     * код, и там же тест ищет в готовых байтах `046d`/`c261`/`c262`.
     */
    if (g920_stub_device_descriptor(s_device_desc, sizeof(s_device_desc), ids)
            < 0
        || g920_stub_config_descriptor(s_config_desc, sizeof(s_config_desc))
               < 0
        || g920_msos_string_descriptor(s_msos_string, sizeof(s_msos_string),
                                       G920_MSOS_VENDOR_CODE)
               < 0
        || g920_msos_compatible_id(s_msos_compat, sizeof(s_msos_compat), 0)
               < 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    /*
     * Точка отсчёта журнала — **появление питания**, а не bus reset (И4).
     * Мы питаемся от той же шины, поэтому наша загрузка и есть VBUS: до
     * неё хост нам ничего сказать не мог.
     */
    if (s_log != NULL) {
        g920_hostlog_mark_origin(s_log, g920_timestamp_us());
        (void)g920_hostlog_add(s_log, g920_timestamp_us(), G920_HOST_EV_VBUS);
    }

    if (usb_new_phy(&phy, &s_phy) != ESP_OK) {
        return ESP_FAIL;
    }
    if (!tud_init(0)) {
        return ESP_FAIL;
    }
    if (xTaskCreate(tusb_task, "tusb", 4096, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    /*
     * Дескрипторы в лог целиком. Это не отладочный мусор: в M2 предмет
     * проверки — что именно хост увидел, и сверять его реакцию можно
     * только с байтами, которые ему отдали.
     */
    dump("device desc", s_device_desc, sizeof(s_device_desc));
    dump("config desc", s_config_desc, sizeof(s_config_desc));

    G920_LOGI(TAG, "gip stub up: vid %04x pid %04x, class %02x/%02x/%02x",
              (unsigned)STUB_VID, (unsigned)STUB_PID,
              (unsigned)G920_GIP_USB_CLASS, (unsigned)G920_GIP_USB_SUBCLASS,
              (unsigned)G920_GIP_USB_PROTOCOL);
    return ESP_OK;
}

#endif /* G920_MODE_GIP */
