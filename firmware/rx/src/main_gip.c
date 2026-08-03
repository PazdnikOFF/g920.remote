/*
 * RX в режиме GIP-устройства — прошивка вехи M2.
 *
 * Задача одна: воткнуться в хост, заставить его начать энумерацию и
 * записать, что он делает. Из этих трасс в M6 выводятся пороги детектора
 * платформы, поэтому здесь важна не работа, а **наблюдение**.
 *
 * Радиолинка здесь нет намеренно. Он к опознанию хоста не относится, а
 * ESP-NOW рядом с USB — это лишний источник задержек и лишние строки в
 * логе ровно там, где меряются интервалы между control-запросами.
 * Замерочная прошивка линка живёт в `main.c` и собирается профилями
 * `rx-devkit` / `rx-dongle`; эта — профилем `rx-gip`.
 *
 * ⚠ **Устройство — заглушка (И2).** Ни VID/PID руля, ни его строк, ни
 * блоба метаданных здесь нет и быть не может: личность приезжает от TX в
 * M4. Пока её нет, задача заглушки — не притвориться рулём, а показать,
 * кто на том конце провода.
 */

/*
 * ⚠ Развод прошивок — флагом сборки, а не `build_src_filter`: для
 * `framework = espidf` PlatformIO собирает `src/` через CMake, и фильтр
 * scons там не работает вовсе (проверено — файлы всё равно попадают в
 * сборку, и два `app_main` не линкуются). Поэтому лишний файл компилируется
 * пустым.
 */
#ifdef G920_MODE_GIP

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gip_device.h"

#include "g920/board.h"
#include "g920/gip.h"
#include "g920/hexdump.h"
#include "g920/gip_control.h"
#include "g920/gip_stub.h"
#include "g920/hostlog.h"
#include "esp_log.h"

#include "g920/log.h"
#include <stdio.h>

#include "esp_timer.h"

#include "g920/identity.h"
#include "g920/link.h"
#include "g920/store.h"
#include "g920/timestamp.h"
#include "g920/trace.h"
#include "g920/version.h"

static const char *TAG = "boot";
static const char *M2 = "m2";

/*
 * Такт в миллисекунду, а не в десять: в T2 иначе попадает задержка моего
 * же цикла. С тактом 10 мс из измеренных тогда 17.6 мс до десяти
 * описывали бы наблюдателя, а не хост; с миллисекундным тактом осталось
 * 16.0 мс, и своего в них не больше миллисекунды.
 */
/*
 * Такт цикла: 2 мс, а не 10. Им двигается `g920_link_tick`, отдающий по
 * кадру за вызов, а через линк идёт security — разговор с чужим таймаутом
 * в 100 мс. При такте 10 мс повторы не укладываются в него никогда.
 */
#define TICK_MS 2
#define HELLO_EVERY_MS 500
#define REPORT_EVERY_MS 5000
/* Сколько событий журнала печатать за один проход цикла. */
#define HOSTLOG_SLICE 8
#define BLINK_EVERY_MS 200

/*
 * Ёмкость журнала. Энумерация Windows — это десятки control-запросов;
 * 256 записей по 16 байт это 4 КБ, и заведомо больше любой из шести трасс
 * M2. Переполнение теряет **хвост**, а не голову: в фингерпринте ценно
 * начало (см. hostlog.h).
 */
#define HOSTLOG_CAPACITY 256

static g920_host_event_t hostlog_storage[HOSTLOG_CAPACITY];
static g920_hostlog_t hostlog;

/*
 * Hello — Announce (`0x02`), системное сообщение GIP.
 *
 * **Нагрузки у него здесь нет, и это не упрощение.** Настоящий Announce
 * несёт блоб метаданных устройства — часть личности, которой у заглушки
 * нет и не должно быть до M4. Отправлять сюда выдуманные метаданные
 * значило бы сочинить личность, то есть нарушить И2 ровно тем способом,
 * от которого он и заведён.
 *
 * Смысл пустого Hello — проверяемый: увидеть, **опрашивает ли хост IN и
 * отвечает ли** хоть чем-нибудь. Ответ (или его отсутствие) сам по себе
 * различает платформы, а полноценный Announce станет возможен, когда M1
 * привезёт настоящий блоб.
 */
#define IDENTITY_BYTES 3072
#define IDENTITY_CHUNK 512

/* Личность руля: буфер, разбор и признак наличия — нужны и отправке
 * Announce, и приёму по радио, поэтому объявлены до обоих. */
static uint8_t identity_buffer[IDENTITY_BYTES];
static g920_identity_t identity;
static volatile bool identity_pending;
static volatile uint16_t identity_pending_len;
static bool have_identity;
/*
 * Копия **уже установленной** личности — отдельно от `identity_buffer`,
 * потому что приходящая по радио пишется в тот же буфер и затирает старую
 * до того, как её станет с чем сравнивать.
 *
 * Нужна ради одного решения: переподключаться или нет. Установка личности
 * влечёт переподключение на шине — дескрипторы читаются один раз при
 * энумерации, подменить их на лету нельзя. Но передатчик присылает личность
 * заново при **каждой своей загрузке**, и она при этом та же самая, уже
 * лежащая в NVS. Переподключение на ту же личность бесплатным не бывает:
 * замерено 03.08.2026 — сброс передатчика под живой сессией убивал её
 * насмерть, консоль настраивала нас заново и больше не говорила ни слова,
 * `sent` замирало, отдача переставала проходить совсем, и само это не
 * проходило за 45 секунд наблюдения. Тот же механизм утром сам включал
 * выключённый бокс: переподключение — это для консоли сигнал «воткнули
 * геймпад».
 */
static uint8_t installed_identity[IDENTITY_BYTES];
static uint16_t installed_len;

/*
 * Ответ на запрос метаданных — зеркало того, что делает руль.
 *
 * Хост, приняв Announce, просит метаданные (сообщение 0x04) и повторяет
 * запрос каждые 500 мс до четырёх раз, после чего помечает устройство на
 * удаление. Живой Xbox 02.08.2026 просил их именно так — по трассе видно
 * тип 04 раз в полсекунды вперемешку с Set Device State.
 *
 * Блоб уезжает фрагментами по 58 байт: столько же брал настоящий руль,
 * и вместе с шестибайтовым заголовком это ровно один пакет в 64 байта.
 * Каждый фрагмент просит подтверждения (ACME) — так делал руль, и это
 * проще, чем гадать, какие фрагменты хост подтвердит. Следующий уходит,
 * только когда пришло Protocol Control на предыдущий.
 *
 * Транзакция закрывается нулевым completion-фрагментом: без него хост
 * считает передачу незаконченной. Это ровно то, на чём споткнулся TX с
 * другой стороны — там ранний Start отвергал руль.
 */
#define METADATA_FRAGMENT 58

static uint16_t metadata_offset;
static uint16_t metadata_total;
static uint8_t metadata_sequence;
static bool metadata_sending;
static bool metadata_completion_due;
static volatile bool metadata_send_now;

/* Счётчики passthrough: сколько security-сообщений ушло к рулю и сколько
 * не влезло в линк. Разница между ними и есть мера того, поспевает ли
 * радио за консолью. */
static volatile uint32_t auth_sent;
static volatile uint32_t auth_dropped;
static volatile uint32_t auth_back;
/*
 * Отдельно от `auth_back` — **судьба отчётов ввода**, и именно она была
 * слепым пятном: отдача консоли делается через `(void)g920_gip_device_send`,
 * то есть отказ эндпоинта никуда не попадал. «Кадр доехал по радио» и «кадр
 * дошёл до консоли» — разные события, и меру расхождения нужно видеть.
 */
static volatile uint32_t input_fwd;
static volatile uint32_t input_lost;
/*
 * То же самое для **ответов руля на security**, и это счёт совсем другого
 * веса: ввод — поток, потерянный кадр в нём сменяется следующим через
 * миллисекунды. Ответ на security незаменим: потеряли — обмен не сойдётся,
 * и консоль объявит отказ. Руль отвечает пачками по десятку 64-байтных
 * сообщений подряд, эндпоинт столько подряд не берёт.
 */
static volatile uint32_t auth_host_ok;
static volatile uint32_t auth_host_lost;
/* Повтор последнего состояния раз в 25 мс: сколько его ушло и сколько
 * отказано. Он делит эндпоинт со свежим вводом, и если свежий теряется,
 * первый подозреваемый — этот поток. */
static volatile uint32_t idle_fwd;
static volatile uint32_t idle_lost;
/* Признак живого обмена: сбрасывает таймер перезапуска сессии. */
static volatile bool auth_alive;
/* Консоль объявила security принятой: тишина на 0x06 после этого — норма. */
static volatile bool auth_done;
/*
 * Хост нам хоть раз ответил — значит Announce пора прекратить.
 *
 * Спека (`H001419`): «Anytime a GIP device is in the "Arrival" state it
 * should only send GIP "Hellos" at 500 mS intervals **until the host
 * responds**». Мы объявлялись вечно — 10500 Announce за сеанс, — и это не
 * только нарушение буквы. Консоль в дежурном режиме шину **не усыпляет**
 * (за всю перезагрузку `SUSPEND` не пришёл ни разу), её стек USB жив, и
 * устройство, которое раз в полсекунды объявляет себя заново, она читает
 * как подключаемый геймпад — то есть как повод проснуться. Отсюда «бокс
 * выключается и сам включается обратно».
 *
 * Сбрасывается вместе с состоянием настройки: хост отвалился — мы снова в
 * Arrival и снова обязаны представляться.
 */
static volatile bool host_answered;
/*
 * Announce считается **отдельно**.
 *
 * До 03.08.2026 в отчёте стояло `hello sent %u`, а подставлялся
 * `g920_gip_device_sent()` — счётчик **всех** отправленных хосту сообщений,
 * включая отчёты ввода и ретрансляцию ответов руля. Подпись врала, и я на
 * ней построил диагноз: принял «10500 отправок» за «10500 объявлений».
 * Считать надо то, что подписано.
 */
static volatile uint32_t hello_count;
/*
 * Сколько прошло с последнего слова хоста. Отдельно от `auth_alive`:
 * тот про security, а этот про то, жив ли собеседник вообще.
 */
static volatile uint32_t since_host_ms;
/*
 * Хост молчит дольше этого — считаем, что его нет, и **не трогаем шину**.
 *
 * Полсекунды с запасом: работающая консоль шлёт `Set Device State` и запрос
 * начальных отчётов раз в 500 мс без остановки, и это самый редкий её
 * периодический сигнал из наблюдавшихся.
 */
#define HOST_GONE_MS 1500
/* Свой счёт кадров для сил: тип кадра свой, значит и номера свои. */
static uint16_t ffb_seq;
/*
 * Пульс: «донгл жив», раз в 100 мс. По нему TX держит мёртвую руку.
 *
 * Свой тип кадра (`G920_FRAME_ALIVE`) и свой счёт — иначе никак. Судить о
 * живости по прикладному трафику нельзя: замер 03.08.2026 по четырём
 * прогонам дал медиану промежутка 4–132 мс, но **максимум нормального
 * промежутка 1257 мс** (меню, стоянка, тишина в силах), то есть окно
 * пришлось бы делать больше полутора секунд.
 *
 * ⚠ Первая попытка слала пульс типом `CONTROL` и стоила живого управления:
 * `CONTROL` — надёжная дисциплина, у неё **один** трекер номеров на все
 * типы, пульс обгонял security, и та объявлялась повтором. До руля дошло
 * 1 сообщение из 38. Закреплено тестами
 * `test_own_count_on_reliable_type_eats_auth` и
 * `test_alive_stream_does_not_touch_auth`.
 */
#define HEARTBEAT_MS 100
static uint16_t alive_seq;
static uint32_t since_heartbeat_ms;
static volatile uint32_t ffb_sent;
static volatile uint32_t ffb_refused;
/* Статус: разрешён только после START, дальше по расписанию спеки. */
static volatile bool start_seen;
static uint32_t status_due_ms;
static uint32_t status_since_start_ms;
static uint32_t status_count;
static uint8_t status_sequence;
/*
 * Очередь ответов руля консоли на то время, пока эндпоинт занят.
 *
 * Заведена по замеру, а не на всякий случай: без неё ответ на security
 * при занятом эндпоинте **выбрасывался молча** — отдача делается через
 * `g920_gip_device_send`, а её отказ игнорировался. На живом стенде из 88
 * ответов руля терялось 4, и этого хватало, чтобы обмен не сошёлся:
 * консоль объявляла `06 20 .. 02 01 02` вместо `01 00`. Отсюда и вся
 * пестрота «то поднимается, то нет» — руль отвечает пачками по десятку
 * 64-байтных сообщений подряд, а эндпоинт столько подряд не берёт.
 *
 * Почему очередь, а не повтор последнего: у security **важен порядок**.
 * Поэтому пока в очереди что-то есть, новые ответы тоже идут в неё, а не
 * вперёд неё — иначе крипточип получит свои сообщения вперемешку.
 *
 * Ввод сюда **не кладётся** намеренно: он поток, и потерянный кадр в нём
 * через миллисекунды сменяется свежим. Держать очередь для потока значит
 * отдавать консоли устаревшее состояние — ровно то, чего мы избегаем.
 *
 * Один производитель (задача линка) и один потребитель (главный цикл),
 * поэтому хватает двух индексов без блокировки.
 */
/*
 * Длина выведена из замера, а не выбрана круглой: в трёх прогонах подъёма
 * пик очереди был 1, 0 и **15**. Пятнадцать из двадцати четырёх — запас
 * слишком тонкий для очереди, переполнение которой означает потерю
 * незаменимого ответа и отказ security. Тридцать два дают вдвое, и стоят
 * 512 байт при 327 килобайтах RAM — цена, о которой нечего думать.
 */
#define AUTH_QUEUE_LEN 32
static struct {
    uint8_t len;
    uint8_t data[64];
} auth_queue[AUTH_QUEUE_LEN];
static volatile uint32_t auth_q_head; /* пишет линк */
static volatile uint32_t auth_q_tail; /* читает главный цикл */
/* Очередь переполнилась — это уже не заминка, а потеря ответа. */
static volatile uint32_t auth_q_overflow;
static volatile uint32_t auth_q_max;

/* Последний отчёт ввода от руля и время с его отправки консоли. */
static uint8_t last_input[64];
static volatile uint8_t last_input_len;
static volatile uint32_t since_input_ms;

/* Как часто повторять последний отчёт, если руль молчит. Столько же
 * отводит joypad-os своим холостым отчётам во время аутентификации. */
#define IDLE_REPORT_MS 25

/*
 * Труба консоль → руль: буфер и строгий порядок.
 *
 * Причина та же, что и на TX: фрагменты собираются по смещениям, а повтор
 * надёжной дисциплины ломает порядок. И заодно уходит отправка из колбэка
 * USB — из него нельзя занимать время.
 */
#define TUNNEL_QUEUE_LEN 128
#define TUNNEL_MSG_MAX 64

static struct {
    uint8_t data[TUNNEL_MSG_MAX];
    uint8_t length;
} tunnel_queue[TUNNEL_QUEUE_LEN];
static volatile uint8_t tunnel_head;
static volatile uint8_t tunnel_tail;

/*
 * Просьба к TX прислать личность.
 *
 * Нужна потому, что донгл переживает передатчик: воткнули в консоль,
 * вынули, воткнули снова — а руль всё это время стоял и никуда не девался.
 * Без просьбы личность пришлось бы ловить в единственное окно сразу после
 * включения обоих, и любой сброс донгла терял её насовсем.
 *
 * Один байт кода в CONTROL: тип кадра уже отделяет служебное от прикладного,
 * а сочинять внутри него ещё один протокол незачем.
 */
#define CONTROL_NEED_IDENTITY 0x01u
/*
 * «Консоль подключилась заново — верни руль в начало разговора».
 *
 * Нужно потому, что руль объявляется только один раз за сессию. Первая
 * консоль довела его до Idle, донгл вынули и воткнули снова — и новая
 * консоль ждёт Hello, которого не будет никогда: устройство считает себя
 * уже представленным. Спека (H001419, § 2.1) даёт ровно один способ вернуть
 * его в Arrival — **Set Device State: Reset**, после которого «device should
 * send GIP Hellos at 500 mS intervals until the host responds, as it does if
 * it were connecting for the first time».
 *
 * ⚠ Сброс запускает руль заново, то есть он сделает калибровочный проворот.
 * Это его штатный запуск, но моторы при этом двигаются.
 */
#define CONTROL_RESET_WHEEL 0x02u

/*
 * Служебный кадр — **надёжной дисциплиной**, а не своим номером.
 *
 * До 03.08.2026 они слались `g920_link_send(..., 0, 0, ...)`, то есть с
 * постоянным номером 0. У надёжной дисциплины один трекер на все её типы, и
 * в живой сессии её номера уходят далеко вперёд — кадр с номером 0
 * объявляется `DUPLICATE` и наверх не отдаётся. Проверено на модели:
 * после 200 кадров AUTH подряд `CONTROL` с номером 0 не проходит ни разу.
 *
 * Цена была видна на стенде и выглядела как чужая беда: просьба вернуть
 * руль в начало разговора **не выполнялась ни разу** (`set device state:
 * reset` в логе TX — ноль), и после перезагрузки консоли руль оставался в
 * старой сессии, где security заново не проходит.
 *
 * Номер выдаёт сам линк, из общего счёта надёжной дисциплины. Своих счётов
 * на общем трекере в этом проекте больше не заводим — это уже третий раз.
 */
static void send_control(uint8_t code)
{
    (void)g920_link_send_reliable(G920_FRAME_CONTROL, &code, sizeof(code));
}

static void tunnel_push(const uint8_t *data, uint16_t length)
{
    uint8_t next = (uint8_t)((tunnel_head + 1) % TUNNEL_QUEUE_LEN);

    if (length > TUNNEL_MSG_MAX || next == tunnel_tail) {
        auth_dropped++;
        return;
    }
    memcpy(tunnel_queue[tunnel_head].data, data, length);
    tunnel_queue[tunnel_head].length = (uint8_t)length;
    tunnel_head = next;
}

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
    while (tunnel_tail != tunnel_head) {
        /* Номер у каждого кадра свой: приёмник линка отсеивает дубликаты
         * по номеру, и с постоянным нулём всё, кроме первого кадра,
         * отбрасывалось как повтор. */
        /*
         * Одна дисциплина на один тип кадра — иначе номера дерутся.
         *
         * Было так: security уходил одним выстрелом со **своим** счётчиком
         * `tunnel_seq`, а подтверждения того же типа `AUTH` — надёжной
         * дисциплиной с её собственным счётчиком. На приёмнике TX номер у
         * типа кадра один, и кадр с номером «позади» отбрасывается как
         * повтор. Измерено 03.08.2026 при `dropped 0` и нуле отказов
         * отправки: из 49 сообщений консоли до руля доехал 21, а
         * фрагментированные (`06 f0`, 64 байта) и короткие Security Control
         * (`06 20`, 6 байт) — **ни одного из шести**. Выглядело это как
         * потеря в эфире, а эфир был ни при чём.
         *
         * Прежний довод против надёжной дисциплины («захлёбывается») снят
         * замером: он снимался до того, как подтверждения руля пошли
         * консоли, то есть на обмене, который и без того шёл вхолостую.
         */
        g920_link_status_t st = g920_link_send_reliable(
            G920_FRAME_AUTH, tunnel_queue[tunnel_tail].data,
            tunnel_queue[tunnel_tail].length);

        if (st != G920_LINK_OK) {
            /*
             * Отказ отправки печатается с длиной кадра.
             *
             * Повод конкретный: в трассе от 03.08.2026 фрагментированные
             * сообщения консоли (`06 f0 …`, ровно 64 байта) уходили из
             * очереди все восемь раз и не доехали до TX ни разу, при
             * `dropped 0`. Молчаливый отказ ровно на одной длине выглядит
             * как потеря в эфире, а лечится совсем не тем.
             */
            G920_LOGW(M2, "auth: link refused %u bytes (status %d)",
                      (unsigned)tunnel_queue[tunnel_tail].length, (int)st);
            return;
        }
        tunnel_tail = (uint8_t)((tunnel_tail + 1) % TUNNEL_QUEUE_LEN);
        auth_sent++;
    }
}

static bool send_metadata_fragment(void)
{
    const uint8_t *blob;
    uint16_t total = 0;
    uint8_t buf[G920_GIP_HEADER_MAX + METADATA_FRAGMENT];
    g920_gip_header_t header;
    uint16_t chunk;
    int built;

    blob = g920_identity_find(&identity, G920_ID_GIP_METADATA, 0, &total);
    if (blob == NULL || total == 0) {
        return false;
    }
    metadata_total = total;

    memset(&header, 0, sizeof(header));
    header.message_type = 0x04u;
    header.sequence = metadata_sequence;
    header.length_bytes = 1;
    header.tlo_bytes = 2; /* так же, как это делает руль: поле чётной ширины */

    if (metadata_completion_due) {
        /* Нулевой completion: длина ноль, смещение — полная длина. */
        header.flags = G920_GIP_FLAG_FRAGMENT | G920_GIP_FLAG_SYSTEM
                       | G920_GIP_FLAG_ACME;
        header.payload_length = 0;
        header.tlo = total;
        chunk = 0;
    } else {
        chunk = (uint16_t)(total - metadata_offset);
        if (chunk > METADATA_FRAGMENT) {
            chunk = METADATA_FRAGMENT;
        }
        header.flags = G920_GIP_FLAG_FRAGMENT | G920_GIP_FLAG_SYSTEM
                       | G920_GIP_FLAG_ACME;
        if (metadata_offset == 0) {
            header.flags |= G920_GIP_FLAG_INIT_FRAG;
            header.tlo = total; /* у первого фрагмента TLO — полная длина */
        } else {
            header.tlo = metadata_offset; /* у прочих — смещение */
        }
        header.payload_length = chunk;
    }

    built = g920_gip_header_build(buf, sizeof(buf), &header);
    if (built <= 0) {
        return false;
    }
    if (chunk > 0) {
        memcpy(buf + built, blob + metadata_offset, chunk);
    }
    if (!g920_gip_device_send(buf, (uint16_t)(built + chunk))) {
        return false;
    }

    if (metadata_completion_due) {
        metadata_completion_due = false;
        metadata_sending = false;
        G920_LOGI(TAG, "metadata: sent %u bytes to host", (unsigned)total);
    } else {
        metadata_offset = (uint16_t)(metadata_offset + chunk);
        if (metadata_offset >= total) {
            metadata_completion_due = true;
        }
    }
    return true;
}

/*
 * Сообщение от хоста. Зовётся из задачи USB, поэтому здесь только разбор
 * заголовка и флаг: отправка — дело главного цикла.
 */
static void on_host_message(void *ctx, const uint8_t *data, uint16_t length)
{
    g920_gip_header_t header;

    (void)ctx;
    /*
     * Хост заговорил — считается **любое** его сообщение, ещё до разбора:
     * ответом по спеке может быть и запрос метаданных, и Set Device State,
     * и Off с Reset. Что именно он сказал, для прекращения Announce неважно.
     */
    host_answered = true;
    since_host_ms = 0;
    if (!have_identity
        || g920_gip_header_parse(&header, data, length) != G920_GIP_OK) {
        return;
    }

    /*
     * Режим трубы: пока пир на связи, **всё** сказанное хостом уходит рулю
     * вербатим, и отвечает ему руль, а не мы.
     *
     * Так пришлось сделать по результату живого опыта: с локальным ответом
     * на знакомство консоль и руль оказывались в **разных сессиях**, и
     * security повисала — руль ответил нулём на 34 доехавших сообщения при
     * нулевых потерях на линке. Сессия у разговора одна, и разорвать её
     * пополам нельзя.
     *
     * Локальные ответы остаются на случай, когда руля нет: тогда донгл
     * честно ведёт себя как заглушка M2, а не притворяется рулём.
     */
    /*
     * Донгл снова отвечает консоли **сам** — знакомство, метаданные,
     * подтверждения, — а по радио уходит только аутентификация.
     *
     * Сквозной туннель проверен и отвергнут опытом: при нём консоль не
     * доходила даже до Host Hello. В обеих работающих реализациях
     * (GP2040-CE, joypad-os) устроено именно так: каждая сторона ведёт свой
     * разговор, между ними ходит только `0x06`.
     */
    if (g920_link_has_peer() && g920_gip_message_number(&header) == 0x06u) {
        /* Обмен идёт — значит перезапускать сессию нельзя: он занимает
         * секунды, и рубить его на середине своим же таймером значит не
         * дать ему завершиться никогда. */
        auth_alive = true;
        /*
         * Security Control (`06 20 xx 02`, таблица 4-4 в `H001419`:
         * «Indicates security state») — консоль объявляет им исход обмена.
         *
         * Значение снято живьём 03.08.2026: пока handshake обрывался, в
         * конце круга приходило `01 02` и консоль тут же начинала знакомство
         * заново; когда обмен впервые прошёл целиком — пришло **`01 00`**, и
         * заново она не начала ни разу. Отсюда `01 00` = «security принята».
         *
         * Дальше молчание на `0x06` — норма, и таймер перезапуска сессии
         * обязан замолчать вместе с ним: иначе он рвёт **удавшийся** обмен
         * (наблюдалось, `restarting the session` на 36-й секунде рабочей
         * сессии).
         */
        if (length >= 6 && header.flags == 0x20u && data[4] == 0x01u
            && data[5] == 0x00u) {
            if (!auth_done) {
                auth_done = true;
                G920_LOGW(M2, "security accepted by the host (control 01 00)");
            }
        }
        tunnel_push(data, length);
        /* Диалог security печатается шапками в обе стороны: где он встаёт,
         * по счётчикам не видно, а по восьми первым байтам видно сразу. */
        if (g920_gip_message_number(&header) == 0x06u) {
            char line[G920_HEXDUMP_LINE_MAX];

            if (g920_hexdump_line(line, sizeof(line), 0, data,
                                  (length < 8) ? length : 8)
                > 0) {
                /* Длина — в самой строке: без неё не отличить «сообщение
                 * не уехало» от «уехало и потерялось», а лечится это
                 * по-разному. */
                G920_LOGI(M2, "host->wheel [%u] %s", (unsigned)length, line);
            }
        }
        return;
    }

    /*
     * Security (0x06) — **не наше дело**, и в этом весь замысел.
     *
     * Ответить на запрос консоли может только сам руль: ключ лежит в его
     * крипточипе и наружу не выходит. Поэтому сообщение уезжает по радио на
     * TX **вербатим**, слово в слово, а ответ руля так же вербатим уходит
     * хосту. Донгл здесь не участник разговора, а провод (И1).
     *
     * Подтверждения на security-сообщения пересылаются вместе с ними: они
     * часть той же надёжной передачи. Различаются по RefMessageType —
     * подтверждения на наши метаданные остаются нам.
     */
    if (g920_gip_message_number(&header) == 0x06u) {
        if (g920_link_send_reliable(G920_FRAME_AUTH, data, length)
            == G920_LINK_OK) {
            auth_sent++;
        } else {
            auth_dropped++;
        }
        return;
    }
    if (g920_gip_message_number(&header) == 0x01u) {
        g920_gip_control_t control;

        if (g920_gip_control_parse(&control, data + header.header_length,
                                   length - header.header_length)
                == G920_GIP_OK
            && control.ref_message_type == 0x06u) {
            (void)g920_link_send_reliable(G920_FRAME_AUTH, data, length);
            return;
        }
    }

    /*
     * Initial Reports Request (`0x0A`) — рулю, вербатим.
     *
     * После принятой security консоль шлёт его вместе с Set Device State
     * каждые 500 мс и ждёт ответа (`H001861`, таблица 3-12). Ответить на
     * него может только руль: в ответе его статическая конфигурация —
     * разрядность осей, пределы угла, маска FFB. Донгл этих чисел не знает
     * и сочинять их не станет (И2).
     *
     * Наблюдалось 03.08.2026: `GIP_IN type 0a len 7` каждые полсекунды без
     * конца — консоль спрашивает, а вопрос до руля не доходил вовсе.
     */
    if (g920_link_has_peer() && g920_gip_message_number(&header) == 0x0Au) {
        tunnel_push(data, length);
        return;
    }

    /*
     * Set Device State (`0x05`) — рулю, вербатим.
     *
     * Спека здесь однозначна (`H001419`, § «Notes on Gamepad Input Reports
     * during Startup»): «первый отчёт ввода после Set Device State: Start
     * **обязателен** и обязан отражать текущее состояние устройства», а
     * статус до START слать вообще запрещено. Значит START — это команда
     * рулю, а не нам, и отдавать её должен тот, кто её получил.
     *
     * Наблюдалось: консоль шлёт `05` каждые 500 мс без конца — ровно так и
     * ведёт себя хост, не дождавшийся перехода устройства в Active.
     */
    if (g920_link_has_peer() && g920_gip_message_number(&header) == 0x05u) {
        /*
         * START — это ещё и разрешение слать статус: до него спека его
         * запрещает прямо («status should not be sent before getting Set
         * Device State: START»). Подтип берём из нагрузки: `0x00` —
         * питание включить, он же START.
         */
        if (length > header.header_length && data[header.header_length] == 0x00u
            && !start_seen) {
            start_seen = true;
            status_due_ms = 0; /* «asap after receiving START» */
        }
        tunnel_push(data, length);
        return;
    }

    /*
     * Всё остальное от консоли — тоже рулю, вербатим.
     *
     * Сюда попадает обратная связь: `0x0B`/`0x0C`/`0x0D` (силы, вибрация,
     * уравнения по `H001443`) и всё, чего мы ещё не знаем по имени. Раньше
     * этот конец был глухой: неизвестное сообщение молча выбрасывалось, и
     * FFB не доходил до руля вовсе — «вибрации при врезании нет» ровно
     * отсюда.
     *
     * Перечислять типы поимённо здесь нельзя: сочинять, какие сообщения
     * консоли «правильные», значит толковать чужой разговор (И1). Своими
     * остаются ровно два: запрос метаданных (`0x04` — отвечаем мы, руль
     * консоли не представляется) и подтверждения на наши же метаданные.
     *
     * ⚠ Через этот проход к рулю впервые идут **силы**. Мёртвой руки (снять
     * силы при потере линка) пока нет — она в M10.
     */
    if (g920_link_has_peer() && g920_gip_message_number(&header) != 0x04u
        && g920_gip_message_number(&header) != 0x01u) {
        uint8_t number = g920_gip_message_number(&header);

        /*
         * Силы (`0x0B`), вибрация (`0x0C`) и уравнения (`0x0D`) идут
         * **свежей** дисциплиной, а не надёжной.
         *
         * Причина названа человеком за рулём: «что-то есть, но очень
         * большая задержка». Надёжная очередь одна на все типы, и силы
         * стояли в ней за повторами security и служебного трафика —
         * `retries 473, pending 1` ещё до того, как FFB туда добавился.
         * Это блокировка головы очереди, и лечится она не размером буфера.
         *
         * Свежая дисциплина здесь верна по существу, а не для скорости:
         * опоздавшая сила **хуже** пропущенной. Консоль пересчитывает и
         * шлёт следующую через доли секунды, а отработанная с запозданием
         * — это рывок в руках. Ровно тот же довод, по которому свежей
         * дисциплиной идёт поток ввода.
         *
         * Тип кадра свой (`G920_FRAME_FFB`), то есть и счёт номеров свой:
         * два счёта на один тип — это дефект, который в этой вехе уже
         * стоил трёх заходов.
         */
        if (number == 0x0Bu || number == 0x0Cu || number == 0x0Du) {
            /*
             * Исход отправки считается, а не выбрасывается.
             *
             * Свежая дисциплина шлёт в эфир немедленно и не повторяет — то
             * есть отказ здесь это **потерянная сила**, и заметить его
             * можно только счётчиком. Замер 03.08.2026 дал 87 сил в секунду
             * на выходе TX, а сколько их отдала консоль, было неизвестно:
             * разница между «доехало всё» и «доехали три четверти» — это
             * разница между «искать дальше» и «чинить очередь отправки».
             */
            if (g920_link_send(G920_FRAME_FFB, ffb_seq++, 0, data, length)
                == G920_LINK_OK) {
                ffb_sent++;
            } else {
                ffb_refused++;
            }
            return;
        }
        tunnel_push(data, length);
        if (length >= 1) {
            char line[G920_HEXDUMP_LINE_MAX];

            if (g920_hexdump_line(line, sizeof(line), 0, data,
                                  (length < 8) ? length : 8)
                > 0) {
                G920_LOGI(M2, "host->wheel [%u] %s", (unsigned)length, line);
            }
        }
        return;
    }

    switch (g920_gip_message_number(&header)) {
    case 0x04u: /* запрос метаданных: начинаем сначала */
        /*
         * Запрос метаданных после **уже состоявшейся** security означает
         * ровно одно: хост начал знакомство сначала. Это и есть надёжный
         * признак перезагрузки консоли — надёжнее, чем `SET_CONFIGURATION`,
         * которого может не быть вовсе: на девките донгл питается от
         * отладочного порта, а не от шины, и ухода консоли не замечает
         * (замерено 03.08.2026 — `T1` не изменился за всю перезагрузку).
         *
         * Отсюда две вещи. Первая: снять защёлку, иначе таймер перезапуска
         * сессии останется выключенным навсегда — он для этого случая и
         * заведён, а я его сам же и обезвредил. Вторая: попросить TX
         * вернуть руль в Arrival, иначе руль останется в старой сессии, где
         * security второй раз не проходит, и связка не поднимется никогда.
         */
        if (auth_done) {
            const uint8_t code = CONTROL_RESET_WHEEL;

            auth_done = false;
            start_seen = false;
            status_since_start_ms = 0;
            status_count = 0;
            /* Таймер перезапуска — величина главного цикла, отсюда до неё
             * не дотянуться; его отсчёт начнётся заново сам, как только
             * пойдёт security (`auth_alive`). */
            G920_LOGW(TAG, "host asks metadata again — console restarted, "
                           "taking the wheel back to arrival");
            send_control(code);
        }
        metadata_offset = 0;
        metadata_completion_due = false;
        metadata_sending = true;
        metadata_sequence++;
        if (metadata_sequence == 0) {
            metadata_sequence = 1;
        }
        metadata_send_now = true;
        break;
    case 0x01u: /* подтверждение: можно слать следующий фрагмент */
        if (metadata_sending) {
            metadata_send_now = true;
        }
        break;
    default:
        break;
    }
}

/*
 * Status Device (`0x03`) — единственное, что донгл сочиняет сам, и это не
 * вольность, а требование спеки к устройству.
 *
 * `H001419`, § «Status Device Command»: устройство **обязано** прислать
 * статус сразу после Set Device State: START, затем раз в секунду первые
 * десять секунд и раз в двадцать секунд дальше. Руль его не шлёт вообще —
 * `03=0` во всех прогонах, — и консоль поэтому повторяет START и запрос
 * начальных отчётов каждые полсекунды без конца. Ровно эта обязанность в
 * роадмапе записана за донглом («Status (0x03) локально», M7).
 *
 * Байты не выбираются: для проводного устройства без батареи таблицы 418 и
 * 420 оставляют по одному законному значению на поле.
 *   Status = `0x80`: питание Full Power (10), заряд Not charging (00),
 *   батарея Battery Absent (00) — «USB-only devices must show 00», уровень
 *   Critically Low (00) — «USB bus powered only devices should always
 *   report Critically Low».
 *   Extended Status = `0x00`: событий нет, а бит Device Active спека
 *   предписывает **держать нулём** устройствам без ИК-светодиода.
 *   Два байта следом — Reserved, «for future use», то есть нули.
 */
#define STATUS_PAYLOAD_STATUS 0x80u
#define STATUS_PAYLOAD_EXTENDED 0x00u

static bool send_status(void)
{
    uint8_t buf[G920_GIP_HEADER_MAX + 4];
    g920_gip_header_t header;
    int built;
    static const uint8_t payload[4] = { STATUS_PAYLOAD_STATUS,
                                        STATUS_PAYLOAD_EXTENDED, 0x00u,
                                        0x00u };

    memset(&header, 0, sizeof(header));
    header.message_type = 0x03u;
    header.flags = G920_GIP_FLAG_SYSTEM;
    /* «0x00 is reserved» — счётчик через ноль перешагивает. */
    status_sequence = (uint8_t)(status_sequence + 1u);
    if (status_sequence == 0u) {
        status_sequence = 1u;
    }
    header.sequence = status_sequence;
    header.payload_length = sizeof(payload);
    header.length_bytes = 1;

    built = g920_gip_header_build(buf, sizeof(buf), &header);
    if (built <= 0) {
        return false;
    }
    memcpy(buf + built, payload, sizeof(payload));
    return g920_gip_device_send(buf, (uint16_t)(built + sizeof(payload)));
}

static bool send_hello(uint8_t *sequence)
{
    uint8_t buf[G920_GIP_HEADER_MAX + 64];
    g920_gip_header_t header;
    const uint8_t *payload = NULL;
    uint16_t payload_len = 0;
    int built;

    /*
     * Нагрузка Announce — снятая с руля, а не сочинённая.
     *
     * До M4 здесь был пустой Announce, и это было правильно: настоящий
     * несёт Device ID, VID/PID и версии протоколов, то есть часть личности,
     * которой у заглушки не было. Придумать её значило бы нарушить И2.
     * Теперь личность приехала — и Announce становится настоящим.
     */
    if (have_identity) {
        payload = g920_identity_find(&identity, G920_ID_GIP_HELLO, 0,
                                     &payload_len);
        if (payload != NULL && payload_len > 64) {
            payload = NULL;
            payload_len = 0;
        }
    }

    memset(&header, 0, sizeof(header));
    header.message_type = 0x02u; /* Announce */
    header.flags = G920_GIP_FLAG_SYSTEM;
    header.sequence = *sequence;
    header.payload_length = payload_len;
    header.length_bytes = 1;

    built = g920_gip_header_build(buf, sizeof(buf), &header);
    if (built <= 0) {
        return false;
    }
    if (payload != NULL && payload_len > 0) {
        memcpy(buf + built, payload, payload_len);
        built += (int)payload_len;
    }
    if (!g920_gip_device_send(buf, (uint16_t)built)) {
        return false;
    }
    /*
     * Номер двигается **только после удачной отправки**.
     *
     * Первая версия увеличивала его на каждой попытке, и на Linux это
     * вылезло сразу: ядро настраивает устройство, но IN не опрашивает, наш
     * Hello висит взведённым, следующие попытки отказывают — а счётчик
     * бежит. Хост потом получил `00`, а следом `fa fb fc fd fe ff 00`, то
     * есть дыру в сотню номеров и обёртку через ноль. В M2 это никого не
     * смущает, но в M7 номер сообщения — часть протокола, и приёмная
     * сторона по нему решает, потерялось ли что-то.
     */
    (*sequence)++;
    return true;
}

static void report(void)
{
    uint32_t t1 = 0;
    uint32_t t2 = 0;

    g920_gip_device_lock();
    /*
     * `suspended` печатается рядом с `configured` не для полноты: он стоит
     * первым барьером в отправке, и пока его не было видно, намертво
     * замолчавший донгл выглядел как загадка — «настроен, но ничего не
     * уходит». Состояние, которое решает, поедет ли хоть один кадр, должно
     * быть в отчёте.
     */
    G920_LOGI(M2, "events %u, dropped %u, announce %u, sent %u, %s%s",
              (unsigned)g920_hostlog_count(&hostlog),
              (unsigned)g920_hostlog_dropped(&hostlog),
              (unsigned)hello_count, (unsigned)g920_gip_device_sent(),
              g920_gip_device_configured() ? "configured" : "not configured",
              g920_gip_device_suspended() ? ", SUSPENDED" : "");

    /*
     * T1 и T2 — не назначенные пороги, а **посчитанные из журнала**
     * величины: этого требует критерий судьи по M2. T1 идёт от VBUS (И4),
     * T2 — от SET_CONFIGURATION до первого опроса IN.
     */
    if (g920_hostlog_t1_us(&hostlog, &t1)) {
        G920_LOGI(M2, "T1 %u us", (unsigned)t1);
    }
    if (g920_hostlog_t2_us(&hostlog, &t2)) {
        G920_LOGI(M2, "T2 %u us", (unsigned)t2);
    }
    /*
     * «Прислал ли хост хоть что-нибудь по GIP» — самостоятельный признак,
     * а не мелочь: за все снятые эталоны не прислал никто, и именно из
     * этого следует, что дерево решений M6 не достроить до M4.
     */
    G920_LOGI(M2, "host GIP in: %s, IN polled: %s",
              g920_hostlog_saw_event(&hostlog, G920_HOST_EV_GIP_IN) ? "yes"
                                                                   : "no",
              g920_hostlog_saw_event(&hostlog, G920_HOST_EV_IN_POLLED)
                  ? "yes"
                  : "no");
    G920_LOGI(M2, "GET_DESCRIPTOR %u, stalls %u, device qualifier %s",
              (unsigned)g920_hostlog_count_request(
                  &hostlog, G920_USB_REQ_GET_DESCRIPTOR),
              (unsigned)g920_hostlog_count_stalls(&hostlog),
              g920_hostlog_saw_descriptor(&hostlog,
                                          G920_USB_DESC_DEVICE_QUALIFIER)
                  ? "asked"
                  : "not asked");
    g920_gip_device_unlock();
}

/*
 * Приём личности руля по радио — вторая половина M4.
 *
 * Донгл не имеет права знать G920 заранее (И2): всё, чем он представляется
 * хосту, обязано прийти с TX, снятое с живого устройства. Здесь личность
 * принимается, проверяется на целость и кладётся в NVS частями по 512 байт
 * — тем же способом, каким её пишет TX, и по той же причине: потолок записи
 * в `store.h` упирается в стек задачи.
 *
 * Длина пишется последней: оборванный приём не должен выглядеть готовой
 * личностью.
 */
/* Заголовок части, приходящей по радио: смещение и полная длина. */
#define IDENTITY_LINK_HEADER 4



/*
 * Через сколько начинать сессию заново, если аутентификация не прошла.
 *
 * Консоль, не получив ответа на Host Hello, сдаётся и больше к нему не
 * возвращается: дальше она шлёт только короткие control-сообщения. Ждать в
 * этом состоянии бессмысленно — нужна свежая энумерация, после которой и
 * консоль, и руль начинают разговор с чистого листа. Без этого каждая
 * попытка требует человека с донглом в руках.
 */
#define AUTH_RETRY_MS 30000
#define IDENTITY_ASK_EVERY_MS 2000


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
    return g920_store_write("idlen", G920_STORE_KIND_IDENTITY, 1, total,
                            sizeof(total));
}

/*
 * Колбэк линка зовётся из задачи Wi-Fi: тяжёлое — разбор и запись в NVS —
 * откладывается в главный цикл, здесь только копия и флаг.
 */
static void on_link_frame(void *ctx, const g920_frame_t *frame,
                          const uint8_t *peer_mac, g920_rx_verdict_t verdict)
{
    (void)ctx;
    (void)peer_mac;
    (void)verdict;

    if (frame == NULL) {
        return;
    }
    /* Ответ руля на security: отдаём хосту как есть, ничего не толкуя. */
    if (frame->type == G920_FRAME_AUTH || frame->type == G920_FRAME_INPUT) {
        auth_back++;
        auth_alive = true;
        /*
         * ⚠ Здесь **не место** будить консоль кнопкой Xbox, и это измерено.
         *
         * 03.08.2026 тут стояло: нажата кнопка `07 …01`, а хоста нет —
         * значит переподключиться на шине и тем разбудить. Условие «хоста
         * нет» проверялось через `configured`, и в этом была ошибка:
         * пока консоль **включается**, устройство ещё не настроено, то есть
         * для этой проверки хоста «нет». Человек держит кнопку, консоль в
         * этот момент энумерирует донгл — и мы рвём ей шину раз в четыре
         * секунды. Итог на живом стенде: консоль дошла до `configured`, но
         * вход не опрашивала **ни разу** (`IN polled: no`), announce ушло
         * больше тысячи, ответа не пришло ни одного. Руль стоял.
         *
         * Пробуждение само по себе не отменено, но ему нужен признак «хост
         * спит», который нельзя спутать с «хост как раз просыпается».
         * `configured` таким признаком не является.
         */
        /*
         * **Сначала отдать консоли, потом запоминать.**
         *
         * Порядок здесь не косметика. Отдача — единственное, что видит
         * консоль, и всё, что стоит перед ней, добавляется к задержке
         * ввода. Копия в 64 байта стоит десятки наносекунд, но занимать
         * ими критический путь незачем: запоминание нужно только для
         * повтора через 25 мс, и оно спокойно ждёт своей очереди.
         */
        if (frame->type == G920_FRAME_INPUT) {
            /* Ввод — поток: не влез, значит его сменит следующий отчёт. */
            if (g920_gip_device_send(frame->payload, frame->length)) {
                input_fwd++;
            } else {
                input_lost++;
            }
        } else {
            /*
             * Ответ руля. Пока очередь пуста — прямая отдача, то есть без
             * лишнего такта задержки. Как только она непуста, вперёд неё
             * лезть нельзя: порядок для security обязателен.
             */
            const uint32_t head = auth_q_head;
            const uint32_t tail = auth_q_tail;
            const bool queued = head != tail;

            if (!queued
                && g920_gip_device_send(frame->payload, frame->length)) {
                auth_host_ok++;
            } else if (frame->length <= sizeof(auth_queue[0].data)
                       && (uint32_t)(head - tail) < AUTH_QUEUE_LEN) {
                const uint32_t slot = head % AUTH_QUEUE_LEN;

                memcpy(auth_queue[slot].data, frame->payload, frame->length);
                auth_queue[slot].len = (uint8_t)frame->length;
                auth_q_head = head + 1;
                if ((uint32_t)(head + 1 - tail) > auth_q_max) {
                    auth_q_max = (uint32_t)(head + 1 - tail);
                }
            } else {
                /* Очередь полна либо сообщение длиннее ожидаемого — это
                 * настоящая потеря, и её видно отдельно от заминки. */
                auth_host_lost++;
                auth_q_overflow++;
            }
        }
        /*
         * Последний отчёт запоминается — до вердикта судьи не запоминался
         * нигде: `last_input` и `last_input_len` были объявлены и не
         * присваивались, поэтому повтор раз в 25 мс не исполнился ни разу.
         *
         * Повтор закрывает ровно одну дыру — заминку в эфире, когда консоль
         * иначе получает **ничего** вместо последнего известного состояния.
         * Свежий отчёт всегда идёт вперёд него и никогда им не задерживается.
         *
         * ⚠ Строгую форму И6 («отдавать только по своему таймеру») это не
         * закрывает намеренно: она переписала бы работающий путь ввода.
         * Вердикт по И6 остаётся в силе.
         */
        if (frame->type == G920_FRAME_INPUT && frame->length > 0
            && frame->length <= sizeof(last_input)) {
            memcpy(last_input, frame->payload, frame->length);
            last_input_len = (uint8_t)frame->length;
            since_input_ms = 0; /* свежий отчёт отодвигает повтор */
        }
        /*
         * Печатается **всё, кроме отчётов ввода** (`0x20`): их поток
         * непрерывен и забил бы журнал, а вопрос сейчас в другом — доезжают
         * ли до консоли ответы руля на её собственные запросы (`0x21` на
         * Initial Reports Request, `0x03` на Set Device State: Start).
         * Раньше печатался только `0x06`, и всё остальное было невидимо.
         */
        if (frame->length >= 1 && frame->payload[0] != 0x20u) {
            char line[G920_HEXDUMP_LINE_MAX];

            if (g920_hexdump_line(line, sizeof(line), 0, frame->payload,
                                  (frame->length < 8) ? frame->length : 8)
                > 0) {
                G920_LOGI(M2, "wheel->host [%u] %s", (unsigned)frame->length,
                          line);
            }
        }
        return;
    }
    if (frame->type != G920_FRAME_DESCRIPTOR) {
        return;
    }
    /*
     * Личность приходит частями: четыре байта заголовка — смещение и полная
     * длина, дальше кусок. Смещение, а не номер части: приёмнику тогда всё
     * равно, каким размером резал отправитель.
     */
    if (frame->length <= IDENTITY_LINK_HEADER) {
        return;
    }
    {
        uint16_t offset = (uint16_t)(frame->payload[0]
                                     | ((uint16_t)frame->payload[1] << 8));
        uint16_t total = (uint16_t)(frame->payload[2]
                                    | ((uint16_t)frame->payload[3] << 8));
        uint16_t chunk = (uint16_t)(frame->length - IDENTITY_LINK_HEADER);

        if (total == 0 || total > sizeof(identity_buffer)
            || (uint32_t)offset + chunk > total) {
            return;
        }
        memcpy(identity_buffer + offset, frame->payload + IDENTITY_LINK_HEADER,
               chunk);
        if ((uint32_t)offset + chunk >= total) {
            identity_pending_len = total;
            identity_pending = true;
        }
    }
}

/*
 * Достаёт личность из NVS. Возвращает false, если её там нет — это не
 * ошибка, а «донгл ещё не знакомили с рулём».
 */
static bool load_identity(void)
{
    uint8_t total[4];
    size_t got = 0;
    size_t length;
    size_t offset = 0;
    unsigned index = 0;

    if (g920_store_read("idlen", G920_STORE_KIND_IDENTITY, 1, total,
                        sizeof(total), &got, NULL)
            != G920_STORE_OK
        || got != sizeof(total)) {
        return false;
    }
    length = (size_t)total[0] | ((size_t)total[1] << 8)
             | ((size_t)total[2] << 16) | ((size_t)total[3] << 24);
    if (length == 0 || length > sizeof(identity_buffer)) {
        return false;
    }

    while (offset < length) {
        char key[G920_STORE_KEY_MAX + 1];
        size_t chunk = 0;

        (void)snprintf(key, sizeof(key), "id%u", index);
        if (g920_store_read(key, G920_STORE_KIND_IDENTITY, 1,
                            identity_buffer + offset, length - offset, &chunk,
                            NULL)
            != G920_STORE_OK) {
            return false;
        }
        offset += chunk;
        index++;
    }

    if (g920_identity_parse(&identity, identity_buffer, length)
        != G920_IDENTITY_OK) {
        return false;
    }
    G920_LOGI(TAG, "identity from nvs: %04x:%04x rev %04x, %u sections, %u bytes",
              (unsigned)identity.fingerprint.vendor_id,
              (unsigned)identity.fingerprint.product_id,
              (unsigned)identity.fingerprint.device_release,
              (unsigned)g920_identity_section_count(&identity),
              (unsigned)g920_identity_size(&identity));
    return true;
}

/*
 * Запомнить, какими байтами устройство представлено консоли сейчас.
 * Зовётся рядом с каждой установкой личности — иначе следующая такая же
 * личность не будет опознана как такая же и зря дёрнет шину.
 */
static void remember_installed(uint16_t length)
{
    if (length > sizeof(installed_identity)) {
        installed_len = 0; /* не поместилось — считаем, что не знаем */
        return;
    }
    memcpy(installed_identity, identity_buffer, length);
    installed_len = length;
}

static void take_identity(void)
{
    g920_identity_status_t parsed;
    g920_store_status_t stored;
    uint16_t length;

    if (!identity_pending) {
        return;
    }
    length = identity_pending_len;
    identity_pending = false;

    parsed = g920_identity_parse(&identity, identity_buffer, length);
    if (parsed != G920_IDENTITY_OK) {
        G920_LOGE(TAG, "identity: rejected (%s)",
                  g920_identity_status_name(parsed));
        return;
    }

    /*
     * Та же личность — **ничего не делаем**. Ни записи в NVS, ни, главное,
     * переподключения на шине: устройство уже представлено консоли ровно
     * этими дескрипторами, и повторять представление нечем и незачем.
     *
     * Передатчик шлёт личность при каждой своей загрузке, так что этот путь
     * — обычный, а не исключительный.
     */
    if (installed_len == length
        && memcmp(installed_identity, identity_buffer, length) == 0) {
        have_identity = true;
        G920_LOGI(TAG, "identity: same as installed — bus left alone");
        return;
    }

    G920_LOGI(TAG, "identity: %04x:%04x rev %04x, %u sections, %u bytes",
              (unsigned)identity.fingerprint.vendor_id,
              (unsigned)identity.fingerprint.product_id,
              (unsigned)identity.fingerprint.device_release,
              (unsigned)g920_identity_section_count(&identity),
              (unsigned)g920_identity_size(&identity));

    stored = store_identity(identity_buffer, g920_identity_size(&identity));
    if (stored == G920_STORE_OK) {
        have_identity = true;
        G920_LOGI(TAG, "identity: saved to nvs");
        remember_installed(length);
        g920_gip_device_set_identity(&identity);
    } else {
        G920_LOGE(TAG, "identity: store failed (%s)",
                  g920_store_status_name(stored));
    }
}

void app_main(void)
{
    char version[G920_VERSION_STR_MAX];
    g920_store_status_t status;
    bool led;
    bool on = false;
    uint16_t dumped_count = 0;
    bool dump_pending = false;
    bool was_configured = false;
    uint8_t sequence = 0;
    uint32_t since_hello_ms = 0;
    uint32_t since_report_ms = 0;
    uint32_t since_blink_ms = 0;
    uint32_t since_ask_ms = 0;
    uint32_t since_auth_ms = 0;

    if (g920_version_format(version, sizeof(version), g920_firmware_version())
        < 0) {
        version[0] = '?';
        version[1] = '\0';
    }
    G920_LOGI(TAG, "fw %s, role RX, mode gip-device (stub)", version);

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

    g920_hostlog_init(&hostlog, hostlog_storage, HOSTLOG_CAPACITY);

    if (g920_link_init(on_link_frame, NULL) != G920_LINK_OK) {
        G920_LOGE(TAG, "link init failed — identity cannot arrive");
    } else {
        G920_LOGI(TAG, "link up");
    }

    have_identity = load_identity();
    if (!have_identity) {
        G920_LOGI(TAG, "no identity yet, will ask tx for it");
    }

    /* До поднятия USB: дескрипторы читаются один раз при энумерации, и
     * личность, установленная заранее, избавляет от переподключения. */
    if (have_identity) {
        remember_installed(g920_identity_size(&identity));
        g920_gip_device_set_identity(&identity);
    }

    g920_gip_device_set_rx(on_host_message, NULL);

    if (g920_gip_device_init(&hostlog) != ESP_OK) {
        G920_LOGE(TAG, "usb device init failed");
        g920_board_led_set(G920_IND_FAULT);
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
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
        take_identity();
        tunnel_drain();

        /* Пока личности нет — просим. Как появилась, замолкаем: она в NVS и
         * переживёт следующий сброс. */
        /* Аутентификация не завершилась за отведённое время — начинаем
         * сессию заново: переподключаемся к хосту и просим TX вернуть руль
         * в начало разговора. */
        since_auth_ms += TICK_MS;
        if (auth_alive) {
            auth_alive = false;
            since_auth_ms = 0;
        }
        since_host_ms += TICK_MS;
        /*
         * ⚠ Перезапуск сессии **только пока хост жив**.
         *
         * Перезапуск делается через `g920_gip_device_set_identity`, а это
         * переподключение по USB — для консоли ровно тот сигнал, которым
         * её будит подключаемый геймпад. Пока бокс выключен, security не
         * проходит никогда, таймер срабатывает каждые 30 секунд и объявляет
         * устройство заново — и бокс включается сам. Проверено человеком
         * 03.08.2026 самым прямым способом: **без донгла бокс выключается
         * полностью и включается штатно**.
         *
         * Смысла в перезапуске при мёртвом хосте нет и по существу:
         * предохранитель заведён для случая «хост жив, но обмен встал», а
         * не «хоста нет». Молчащему переподключаться незачем.
         */
        if (g920_gip_device_configured() && g920_link_has_peer() && !auth_done
            && since_host_ms < HOST_GONE_MS
            && since_auth_ms >= AUTH_RETRY_MS) {
            since_auth_ms = 0;
            G920_LOGW(TAG, "auth did not complete in %u s — restarting the "
                           "session from scratch",
                      (unsigned)(AUTH_RETRY_MS / 1000u));
            /* Журнал на 256 событий давно полон, и новые в него не
             * попадают: без сброса каждая следующая попытка наблюдается
             * вслепую. */
            g920_gip_device_lock();
            g920_hostlog_reset(&hostlog);
            dumped_count = 0;
            g920_gip_device_unlock();
            g920_gip_device_set_identity(&identity); /* даёт переподключение */
        }

        /*
         * Статус по расписанию спеки: сразу после START, затем раз в
         * секунду первые десять секунд, дальше раз в двадцать.
         */
        if (start_seen && g920_gip_device_configured()) {
            status_since_start_ms += TICK_MS;
            if (status_due_ms <= TICK_MS) {
                if (send_status()) {
                    status_count++;
                    status_due_ms =
                        (status_since_start_ms < 10000u) ? 1000u : 20000u;
                } else {
                    status_due_ms = TICK_MS; /* эндпоинт занят — следующий такт */
                }
            } else {
                status_due_ms -= TICK_MS;
            }
        }

        /* Пульс мёртвой руки. Только при живом пире: звать отсутствующего
         * — дело пиринга, а не наше. */
        since_heartbeat_ms += TICK_MS;
        if (since_heartbeat_ms >= HEARTBEAT_MS && g920_link_has_peer()) {
            since_heartbeat_ms = 0;
            (void)g920_link_send(G920_FRAME_ALIVE, alive_seq++, 0, NULL, 0);
        }

        /*
         * Отложенные ответы руля — **вперёд всего остального**, что мы
         * кладём в тот же эндпоинт. Консоль ждёт именно их, и каждый такт
         * ожидания она может засчитать в свой таймаут security.
         *
         * Отдаём столько, сколько эндпоинт возьмёт за такт, и на первом же
         * отказе останавливаемся: следующий пойдёт через 2 мс.
         */
        while (auth_q_tail != auth_q_head) {
            const uint32_t slot = auth_q_tail % AUTH_QUEUE_LEN;

            if (!g920_gip_device_send(auth_queue[slot].data,
                                      auth_queue[slot].len)) {
                break;
            }
            auth_q_tail++;
            auth_host_ok++;
        }

        /* Пауза в потоке ввода — повторяем последний отчёт руля. */
        since_input_ms += TICK_MS;
        if (last_input_len > 0 && since_input_ms >= IDLE_REPORT_MS
            && g920_gip_device_configured()) {
            since_input_ms = 0;
            if (g920_gip_device_send(last_input, last_input_len)) {
                idle_fwd++;
            } else {
                idle_lost++;
            }
        }

        if (metadata_send_now) {
            metadata_send_now = false;
            (void)send_metadata_fragment();
        }

        since_ask_ms += TICK_MS;
        if (!have_identity && since_ask_ms >= IDENTITY_ASK_EVERY_MS) {
            const uint8_t code = CONTROL_NEED_IDENTITY;

            since_ask_ms = 0;
            send_control(code);
        }
        since_hello_ms += TICK_MS;
        since_blink_ms += TICK_MS;
        since_report_ms += TICK_MS;

        /*
         * Первый Hello — **сразу по настройке**, не дожидаясь периода.
         *
         * Это не оптимизация, а исправление замера. T2 по ROADMAP — время
         * от SET_CONFIGURATION до первого опроса IN. Приложение видит не
         * опрос, а **успешную передачу**, и она требует, чтобы нам было что
         * отдать: пока эндпоинт не взведён, хост получает NAK, а NAK до
         * приложения TinyUSB не доносит. С Hello раз в полсекунды T2
         * измерял мой собственный период — на живом маке вышло 309 мс,
         * то есть ровно «сколько осталось до следующего Hello».
         *
         * Взводим сразу — и T2 становится тем, чем задуман: задержкой
         * хоста, а не нашей.
         */
        if (g920_gip_device_configured() && !was_configured) {
            was_configured = true;
            since_hello_ms = 0;
            hello_count++;
            (void)send_hello(&sequence);
            {
                const uint8_t code = CONTROL_RESET_WHEEL;

                /* Консоль настроила нас заново — руль должен начать
                 * знакомство с начала, иначе он молчит, а она ждёт. */
                G920_LOGI(TAG, "host configured us: asking tx to reset wheel");
                send_control(code);
            }
        } else if (!g920_gip_device_configured()) {
            was_configured = false;
            /* Хост нас отпустил — мы снова в Arrival и снова представляемся. */
            host_answered = false;
        }

        /* Announce шлётся **всегда**: донгл ведёт разговор с консолью сам,
         * и без представления она к нему не обратится. Условие «только без
         * пира» осталось от сквозного туннеля и было прямой ошибкой — с
         * включённым TX консоль не видела от нас ни одного Hello. */
        if (since_hello_ms >= HELLO_EVERY_MS && !host_answered) {
            since_hello_ms = 0;
            hello_count++;
            /* Пока хост не настроил устройство, слать некуда — это не
             * ошибка, а нормальное начало любой энумерации. */
            (void)send_hello(&sequence);
        }

        if (since_blink_ms >= BLINK_EVERY_MS) {
            uint32_t t1 = 0;
            bool seen;

            since_blink_ms = 0;
            on = !on;
            /*
             * Рабочее состояние здесь — «хост с нами заговорил», а не
             * «хост нас настроил». Разница выяснилась на первом же живом
             * маке: он читает дескрипторы и на этом останавливается, потому
             * что vendor-класс некому подхватить. Ждать от индикатора
             * SET_CONFIGURATION значило бы показывать «ничего не
             * происходит» там, где на самом деле снята целая трасса.
             */
            g920_gip_device_lock();
            seen = g920_hostlog_t1_us(&hostlog, &t1);
            g920_gip_device_unlock();

            g920_board_led_set(seen ? (on ? G920_IND_OK : G920_IND_OFF)
                                    : (on ? G920_IND_DETECT : G920_IND_OFF));
        }

        if (since_report_ms >= REPORT_EVERY_MS) {
            since_report_ms = 0;
            report();
            G920_LOGI(M2,
                      "auth: to wheel %u, dropped %u, back %u | link: retries "
                      "%u, gave up %u, pending %u",
                      (unsigned)auth_sent, (unsigned)auth_dropped,
                      (unsigned)auth_back,
                      (unsigned)g920_link_reliable_retries(),
                      (unsigned)g920_link_reliable_gave_up(),
                      (unsigned)g920_link_reliable_pending());
            {
                /*
                 * Что приёмник линка **отсеял, не дойдя до приложения**.
                 *
                 * Счётчики были в линке с M3, но донгл их не печатал ни
                 * разу — и это стоило целого вечера: колбэк зовётся только
                 * для доставленных кадров, поэтому отвергнутая кнопка Xbox
                 * выглядела как «кнопка не пришла», и я четыре раза подряд
                 * искал причину не там. Вердикт по каждому кадру линк знает,
                 * приложение его получало и выбрасывало.
                 */
                g920_link_rx_counters_t rx_c;

                g920_link_rx_counters(&rx_c, false);
                G920_LOGI(M2, "link rx: delivered %u, dup %u, stale %u, "
                              "foreign %u, sessions %u, gaps %u",
                          (unsigned)rx_c.delivered, (unsigned)rx_c.duplicates,
                          (unsigned)rx_c.stale, (unsigned)rx_c.foreign,
                          (unsigned)rx_c.sessions, (unsigned)rx_c.gaps);
            }
            G920_LOGI(M2, "input to host: fresh %u ok / %u lost, "
                          "idle repeat %u ok / %u lost",
                      (unsigned)input_fwd, (unsigned)input_lost,
                      (unsigned)idle_fwd, (unsigned)idle_lost);
            G920_LOGI(M2, "wheel answers to host: %u ok / %u lost | "
                          "queue peak %u of %u, overflow %u",
                      (unsigned)auth_host_ok, (unsigned)auth_host_lost,
                      (unsigned)auth_q_max, (unsigned)AUTH_QUEUE_LEN,
                      (unsigned)auth_q_overflow);
            G920_LOGI(M2, "ffb: sent %u, refused %u | status: sent %u, start %s",
                      (unsigned)ffb_sent, (unsigned)ffb_refused,
                      (unsigned)status_count, start_seen ? "yes" : "no");
            /*
             * Полную трассу печатаем, **когда журнал изменился**, а не
             * когда хост настроил устройство: первая версия ждала
             * настройки — и промолчала ровно в том случае, ради которого
             * трасса и нужна. Хост, прочитавший дескрипторы и не пошедший
             * дальше, это не «ничего не произошло», а признак платформы.
             *
             * Повторять неизменившуюся трассу незачем: она длинная, и лог,
             * ради которого всё сделано, залился бы ею.
             */
            /*
             * Печатаются **только новые события**, и не больше горсти за
             * проход.
             *
             * Сплошная выгрузка журнала занимала секунды, и всё это время
             * главный цикл стоял, а с ним стоял `g920_link_tick` — тот
             * самый, что двигает повторы надёжной дисциплины по кадру за
             * вызов. Цена измерена на живом стенде: security-сообщение от
             * консоли шло к рулю секундами при бюджете протокола в 100 мс.
             * Инструмент наблюдения не должен ломать то, за чем наблюдает.
             */
            dump_pending = true;
        }

        if (dump_pending) {
            uint16_t count;
            int printed = 0;

            g920_gip_device_lock();
            count = g920_hostlog_count(&hostlog);
            while (dumped_count < count && printed < HOSTLOG_SLICE) {
                const g920_host_event_t *event =
                    g920_hostlog_at(&hostlog, dumped_count);
                char line[G920_HOSTLOG_LINE_MAX];

                if (event != NULL
                    && g920_hostlog_format(line, sizeof(line), event) > 0) {
                    G920_LOGI(M2, "%s", line);
                }
                dumped_count++;
                printed++;
            }
            if (dumped_count >= count) {
                dump_pending = false;
            }
            g920_gip_device_unlock();
        }
    }
}

#endif /* G920_MODE_GIP */
