#include "app_storage.h"
#include "MT6701.h"
#include "w25q64.h"
#include "main.h"

#include <stddef.h>
#include <string.h>

#define APP_SETTINGS_MAGIC 0x464F4331UL  /* "FOC1" */
#define APP_SETTINGS_VER   7U
#define APP_SETTINGS_ADDR_A 0x00000000UL
#define APP_SETTINGS_ADDR_B 0x00001000UL
#define APP_LOG_ADDR        0x00002000UL
#define APP_LOG_MAGIC       0x4C4F4731UL /* "LOG1" */

typedef struct {
    uint32_t magic;
    uint32_t seq;
    uint32_t tick;
    uint16_t event;
    uint16_t reserved;
    uint32_t value;
    uint32_t crc;
    uint32_t pad[2];
} LogRecord;

static AppSettings s_settings;
static bool s_ready;
static bool s_settings_valid;
static uint32_t s_active_addr = APP_SETTINGS_ADDR_A;
static uint32_t s_log_seq;
static uint32_t s_log_addr;

/* 主循环分步擦写，芯片内部忙时不阻塞按键与转场。 */
enum { STORE_IDLE, STORE_SETTINGS_ERASE, STORE_SETTINGS_PROGRAM,
       STORE_LOG_ERASE, STORE_LOG_PROGRAM };
static uint8_t s_store_state;
static uint32_t s_store_started;
static uint32_t s_service_at;
static AppSettings s_save_snapshot;
static uint32_t s_save_addr;
static bool s_save_pending;
static bool s_save_result_ready;
static bool s_save_success;
static bool s_save_failed;
static uint8_t s_save_retries;
static uint32_t s_flash_error;
uint32_t AppStorage_FlashError(void) { return s_flash_error; }
static const char *s_save_error="Save OK";
const char *AppStorage_SaveErrorText(void) { return s_save_error; }
#define LOG_QUEUE_LENGTH 4U
static LogRecord s_log_queue[LOG_QUEUE_LENGTH];
static uint8_t s_log_head, s_log_count;

static uint32_t crc32_bytes(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint32_t i = 0U; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0U; b < 8U; b++) {
            if (crc & 1U) {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

static void extended_defaults(AppSettings *s)
{
    s->control_config=0U;
    s->gear_ratio_x100=100;
    s->gear_offset_x100=0;
    s->axis2_flags=(30UL<<APP_POSITION_SPEED_SHIFT) |
                  ((uint32_t)APP_SPEED_LIMIT_DEFAULT<<APP_SPEED_LIMIT_SHIFT);
    s->axis2_speed_x10=s->axis2_voltage_x100=s->axis2_position_x100=0;
    s->axis2_speed_step_x10=s->axis2_torque_step_x100=s->axis2_position_step_x100=5;
}

static void settings_set_defaults(void)
{
    memset(&s_settings, 0, sizeof(s_settings));
    s_settings.magic = APP_SETTINGS_MAGIC;
    s_settings.version = APP_SETTINGS_VER;
    s_settings.size = (uint16_t)sizeof(AppSettings);
    s_settings.seq = 0U;
    s_settings.speed_x10 = 0;
    s_settings.voltage_x100 = 0;
    s_settings.position_x100 = 0;
    s_settings.speed_step_x10 = 5;
    s_settings.torque_step_x100 = 5;
    s_settings.position_step_x100 = 5;
    s_settings.flags = APP_FLAG_LED_ENABLE | (30UL << APP_POSITION_SPEED_SHIFT) |
                       ((uint32_t)APP_SPEED_LIMIT_DEFAULT << APP_SPEED_LIMIT_SHIFT);
    s_settings.led_brightness = 100U;
    extended_defaults(&s_settings);
    s_settings.crc = crc32_bytes((const uint8_t *)&s_settings,
                                 (uint32_t)offsetof(AppSettings, crc));
}

static bool settings_is_valid(const AppSettings *s)
{
    if (s->magic != APP_SETTINGS_MAGIC ||
        s->version != APP_SETTINGS_VER ||
        s->size != sizeof(AppSettings)) {
        return false;
    }
    return s->crc == crc32_bytes((const uint8_t *)s,
                                 (uint32_t)offsetof(AppSettings, crc));
}

static bool settings_read(uint32_t addr, AppSettings *out)
{
    if (!W25Q_Read(addr, (uint8_t *)out, sizeof(*out))) {
        return false;
    }
    if (out->magic == APP_SETTINGS_MAGIC &&
        (out->version >= 1U && out->version <= 6U)) {
        static const uint8_t crc_offsets[]={offsetof(AppSettings,speed_step_x10),offsetof(AppSettings,control_config),
            offsetof(AppSettings,motor_detect),offsetof(AppSettings,motor_detect_l_uh),offsetof(AppSettings,calibration),offsetof(AppSettings,custom)};
        _Static_assert(offsetof(AppSettings,custom)<256,"Legacy storage offsets");
        uint32_t old_crc=crc_offsets[out->version-1U];
        uint32_t stored_crc;
        if (out->size != old_crc+sizeof(uint32_t)) return false;
        memcpy(&stored_crc,(const uint8_t *)out+old_crc,sizeof(stored_crc));
        if (stored_crc != crc32_bytes((const uint8_t *)out,old_crc)) return false;
        if (out->version == 1U)
            out->speed_step_x10=out->torque_step_x100=out->position_step_x100=5;
        if (out->version <= 2U) extended_defaults(out);
        if(out->version<=3U) out->motor_detect=0U;
        if(out->version<=4U) out->motor_detect_l_uh=out->motor_detect_flux_uwb=0;
        if(out->version<=5U) {
            out->motor_detect&=0xFFFFFFU; /* Legacy measurements have no trusted model binding. */
            memset(out->calibration,0,sizeof(out->calibration));
        }
        out->control_config&=~APP_CONFIG_CUSTOM_MASK;
        memset(out->custom,0,sizeof(out->custom));
        out->version=APP_SETTINGS_VER;
        out->size=sizeof(*out);
        out->crc=crc32_bytes((const uint8_t *)out,(uint32_t)offsetof(AppSettings,crc));
    }
    return settings_is_valid(out);
}

static bool settings_write(uint32_t addr, const AppSettings *s)
{
    if (!W25Q_SectorErase(addr)) {
        return false;
    }
    if (!W25Q_PageProgram(addr, (const uint8_t *)s, sizeof(*s))) {
        return false;
    }

    AppSettings verify;
    if (!W25Q_Read(addr, (uint8_t *)&verify, sizeof(verify))) {
        return false;
    }
    if (memcmp(&verify, s, sizeof(verify)) != 0) {
        return false;
    }
    return settings_is_valid(&verify);
}

static bool log_read(uint32_t addr, LogRecord *out)
{
    if (!W25Q_Read(APP_LOG_ADDR + addr, (uint8_t *)out, sizeof(*out))) {
        return false;
    }
    if (out->magic != APP_LOG_MAGIC) {
        return false;
    }
    return out->crc == crc32_bytes((const uint8_t *)out,
                                   (uint32_t)offsetof(LogRecord, crc));
}

static void log_scan(void)
{
    s_log_seq = 0U;
    s_log_addr = 0U;

    for (uint32_t addr = 0U;
         addr + sizeof(LogRecord) <= W25Q64_SECTOR_SIZE;
         addr += sizeof(LogRecord)) {
        LogRecord rec;
        if (!log_read(addr, &rec)) {
            s_log_addr = addr;
            return;
        }
        s_log_seq = rec.seq;
        s_log_addr = addr + sizeof(LogRecord);
    }

    /* 整区已满：擦除后从头开始 */
    if (W25Q_SectorErase(APP_LOG_ADDR)) {
        s_log_seq = 0U;
        s_log_addr = 0U;
    }
}

bool AppStorage_Init(void)
{
    s_store_state = STORE_IDLE;
    s_save_pending = false;
    s_save_failed = false;
    s_save_result_ready = false;
    s_save_error="Save OK";
    s_flash_error=0;
    s_log_head = s_log_count = 0U;
    s_service_at = HAL_GetTick();
    /* Boot only, before motor control starts. A single failed JEDEC read
     * must not permanently disable saving. Keep CS arbitration and bounded
     * recovery: at most 3 reads, two 2ms waits, no erase/write during retry. */
    s_ready=false;
    for (unsigned attempt=0;attempt<3U && !s_ready;++attempt) {
        if (attempt) { MT6701_2_Service(HAL_GetTick()); HAL_Delay(2U); }
        s_ready=W25Q_Init();
        if (!s_ready) {
            s_flash_error=W25Q_LastFailure();
            if (!s_flash_error) s_flash_error=12UL<<24; /* ID rejected with no HAL failure. */
        }
    }
    if (!s_ready) {
        s_save_error="Save NO FLASH";
        settings_set_defaults();
        s_settings_valid = false;
        return false;
    }

    AppSettings a;
    AppSettings b;
    bool ok_a = settings_read(APP_SETTINGS_ADDR_A, &a);
    bool ok_b = settings_read(APP_SETTINGS_ADDR_B, &b);

    if (ok_a && ok_b) {
        if ((int32_t)(b.seq - a.seq) >= 0) {
            s_settings = b;
            s_active_addr = APP_SETTINGS_ADDR_B;
        } else {
            s_settings = a;
            s_active_addr = APP_SETTINGS_ADDR_A;
        }
        s_settings_valid = true;
    } else if (ok_a) {
        s_settings = a;
        s_active_addr = APP_SETTINGS_ADDR_A;
        s_settings_valid = true;
    } else if (ok_b) {
        s_settings = b;
        s_active_addr = APP_SETTINGS_ADDR_B;
        s_settings_valid = true;
    } else {
        settings_set_defaults();
        s_active_addr = APP_SETTINGS_ADDR_A;
        s_settings_valid = false;
    }

    log_scan();
    return true;
}

bool AppStorage_IsReady(void)
{
    return s_ready;
}

AppSettings *AppStorage_Get(void)
{
    return &s_settings;
}

void AppStorage_Load(void)
{
    if (!s_ready) {
        settings_set_defaults();
        return;
    }

    AppSettings a;
    AppSettings b;
    bool ok_a = settings_read(APP_SETTINGS_ADDR_A, &a);
    bool ok_b = settings_read(APP_SETTINGS_ADDR_B, &b);

    if (ok_a && ok_b) {
        s_settings = ((int32_t)(b.seq - a.seq) >= 0) ? b : a;
        s_active_addr = ((int32_t)(b.seq - a.seq) >= 0)
            ? APP_SETTINGS_ADDR_B : APP_SETTINGS_ADDR_A;
        s_settings_valid = true;
    } else if (ok_a) {
        s_settings = a;
        s_active_addr = APP_SETTINGS_ADDR_A;
        s_settings_valid = true;
    } else if (ok_b) {
        s_settings = b;
        s_active_addr = APP_SETTINGS_ADDR_B;
        s_settings_valid = true;
    }
}

bool AppStorage_Save(void)
{
    if (MT6701_2_StorageBlocked() || !s_ready || s_store_state != STORE_IDLE || s_save_pending) {
        return false;
    }

    uint32_t target = (s_active_addr == APP_SETTINGS_ADDR_A)
        ? APP_SETTINGS_ADDR_B : APP_SETTINGS_ADDR_A;

    s_settings.magic = APP_SETTINGS_MAGIC;
    s_settings.version = APP_SETTINGS_VER;
    s_settings.size = (uint16_t)sizeof(AppSettings);
    s_settings.seq++;
    s_settings.crc = crc32_bytes((const uint8_t *)&s_settings,
                                 (uint32_t)offsetof(AppSettings, crc));

    if (!settings_write(target, &s_settings)) {
        return false;
    }

    s_active_addr = target;
    s_settings_valid = true;
    AppStorage_Log(APP_LOG_SETTINGS_SAVED, s_settings.seq);
    return true;
}

void AppStorage_ResetDefaults(void)
{
    uint32_t seq = s_settings.seq;
    settings_set_defaults();
    s_settings.seq = seq;
}

void AppStorage_Log(uint16_t event, uint32_t value)
{
    if (!s_ready || s_log_count == LOG_QUEUE_LENGTH) return;
    LogRecord *rec = &s_log_queue[(s_log_head + s_log_count) % LOG_QUEUE_LENGTH];
    memset(rec, 0, sizeof(*rec));
    rec->magic = APP_LOG_MAGIC;
    rec->seq = ++s_log_seq;
    rec->tick = HAL_GetTick();
    rec->event = event;
    rec->value = value;
    rec->crc = crc32_bytes((const uint8_t *)rec, (uint32_t)offsetof(LogRecord, crc));
    ++s_log_count;
}

bool AppStorage_RequestSave(void)
{
    if (!s_ready || s_save_pending) return false;
    s_save_result_ready=false;
    s_save_retries=0U;
    s_flash_error=0;
    if (s_store_state==STORE_IDLE) s_store_started=HAL_GetTick();
    s_save_snapshot = s_settings;
    s_save_snapshot.magic = APP_SETTINGS_MAGIC;
    s_save_snapshot.version = APP_SETTINGS_VER;
    s_save_snapshot.size = sizeof(AppSettings);
    ++s_save_snapshot.seq;
    s_save_snapshot.crc = crc32_bytes((const uint8_t *)&s_save_snapshot,
                                     (uint32_t)offsetof(AppSettings, crc));
    s_save_addr = s_active_addr == APP_SETTINGS_ADDR_A ? APP_SETTINGS_ADDR_B : APP_SETTINGS_ADDR_A;
    s_save_pending = true;
    return true;
}

bool AppStorage_PollSaveResult(bool *success)
{
    if (!s_save_result_ready || success == NULL) return false;
    *success = s_save_success;
    s_save_result_ready = false;
    return true;
}

static void store_complete(bool success,bool retry)
{
    if (s_store_state == STORE_SETTINGS_ERASE || s_store_state == STORE_SETTINGS_PROGRAM) {
        /* Keep the immutable snapshot and previous A/B copy. One transient
         * SPI/readback failure gets one complete erase/program/verify retry;
         * never publish success until the bytes and CRC pass. */
        if (!success && retry && s_save_retries++==0U) {
            s_store_state=STORE_IDLE; s_store_started=HAL_GetTick(); return;
        }
        s_save_pending = false;
        s_save_result_ready = true;
        s_save_success = success;
        s_save_failed = !success;
        if (success) {
            s_save_error="Save OK";
            s_flash_error=0;
            s_active_addr = s_save_addr;
            s_settings.seq = s_save_snapshot.seq;
            s_settings_valid = true;
            AppStorage_Log(APP_LOG_SETTINGS_SAVED, s_settings.seq);
        }
    } else if (s_store_state == STORE_LOG_PROGRAM) {
        s_log_head = (s_log_head + 1U) % LOG_QUEUE_LENGTH;
        --s_log_count;
        /* 失败页可能已部分写入，下一条记录先擦除，不能覆盖编程。 */
        s_log_addr = success ? s_log_addr + sizeof(LogRecord) : W25Q64_SECTOR_SIZE;
    }
    s_store_state = STORE_IDLE;
}

bool AppStorage_IsDeferred(void) { return MT6701_2_StorageBlocked(); }

AppSaveState AppStorage_GetSaveState(void)
{
    if (!s_ready) return APP_SAVE_FAILED;
    if (s_save_pending) {
        if (AppStorage_IsDeferred()) return APP_SAVE_DEFERRED;
        return s_store_state==STORE_SETTINGS_ERASE || s_store_state==STORE_SETTINGS_PROGRAM ?
            APP_SAVE_WRITING : APP_SAVE_PENDING;
    }
    if (s_save_failed) return APP_SAVE_FAILED;
    return s_settings_valid ? APP_SAVE_SAVED : APP_SAVE_IDLE;
}

static void storage_wait(uint32_t now_ms)
{
    uint32_t timeout=(s_store_state==STORE_SETTINGS_PROGRAM || s_store_state==STORE_LOG_PROGRAM) ? 10U : 1000U;
    if ((s_store_state!=STORE_IDLE || s_save_pending) && now_ms-s_store_started>=timeout) {
        if (s_store_state==STORE_IDLE) s_store_state=STORE_SETTINGS_ERASE;
        if (s_save_pending) { s_save_error="Save BUSY TMO"; s_flash_error=(4UL<<24)|W25Q_LastStatus(); }
        store_complete(false,false);
    }
}

static void storage_step(uint32_t now_ms)
{
    if (!s_ready || now_ms == s_service_at) return;
    s_service_at = now_ms;
    if (s_store_state == STORE_IDLE && !s_save_pending && s_log_count == 0U) return;
    /* Timestamp successful command completion, then allow at least one full
     * millisecond for the internal cycle to start (2 tick edges). */
    if (s_store_state!=STORE_IDLE && now_ms-s_store_started<2U) return;
    bool busy;
    if (!W25Q_PollBusy(&busy)) {
        if (s_store_state == STORE_IDLE && s_save_pending) s_store_state = STORE_SETTINGS_ERASE;
        if (s_save_pending) { s_save_error="Save STATUS"; s_flash_error=W25Q_LastFailure(); }
        store_complete(false,true);
        return;
    }
    if (busy) {
        storage_wait(now_ms);
        return;
    }
    uint8_t previous_state=s_store_state;
    uint32_t previous_started=s_store_started;
    bool ok = true;
    switch (s_store_state) {
    case STORE_SETTINGS_ERASE:
        s_store_state = STORE_SETTINGS_PROGRAM;
        s_save_error="Save WRITE";
        ok = W25Q_BeginPageProgram(s_save_addr, (const uint8_t *)&s_save_snapshot,
                                   sizeof(s_save_snapshot));
        s_store_started=HAL_GetTick();
        break;
    case STORE_SETTINGS_PROGRAM: {
        AppSettings verify;
        s_save_error="Save READ";
        ok = W25Q_Read(s_save_addr, (uint8_t *)&verify, sizeof(verify));
        s_flash_error=ok ? 0 : W25Q_LastFailure();
        if (ok) {
            s_save_error="Save VERIFY";
            ok=memcmp(&verify, &s_save_snapshot, sizeof(verify))==0 && settings_is_valid(&verify);
        }
        store_complete(ok,true);
        return;
    }
    case STORE_LOG_ERASE:
        s_log_addr = 0U;
        s_store_state = STORE_IDLE;
        break;
    case STORE_LOG_PROGRAM:
        store_complete(true,false);
        break;
    default:
        if (s_save_pending) {
            s_store_state = STORE_SETTINGS_ERASE;
            s_save_error="Save ERASE";
            ok = W25Q_BeginSectorErase(s_save_addr);
        } else if (s_log_count != 0U) {
            if (s_log_addr + sizeof(LogRecord) > W25Q64_SECTOR_SIZE) {
                s_store_state = STORE_LOG_ERASE;
                ok = W25Q_BeginSectorErase(APP_LOG_ADDR);
            } else {
                s_store_state = STORE_LOG_PROGRAM;
                ok = W25Q_BeginPageProgram(APP_LOG_ADDR + s_log_addr,
                    (const uint8_t *)&s_log_queue[s_log_head], sizeof(LogRecord));
            }
        }
        s_store_started = HAL_GetTick();
        break;
    }
    if (!ok) {
        if (W25Q_LastFailureBusy()) {
            /* Begin did not issue the program/erase. Keep its previous phase,
             * original deadline and snapshot; BUSY is not a failed attempt. */
            s_store_state=previous_state; s_store_started=previous_started;
            if (s_save_pending) { s_save_error="Save WAIT"; s_flash_error=W25Q_LastFailure(); }
            storage_wait(HAL_GetTick()); return;
        }
        if (s_save_pending && (s_store_state==STORE_SETTINGS_ERASE || s_store_state==STORE_SETTINGS_PROGRAM))
            s_flash_error=W25Q_LastFailure();
        store_complete(false,true);
    }
}

void AppStorage_Service(uint32_t now_ms)
{
    if (MT6701_2_StorageBlocked() || !MT6701_2_FlashBegin()) return;
    (void)now_ms;
    storage_step(HAL_GetTick()); /* Main's loop timestamp may predate the last command. */
    MT6701_2_FlashEnd();
}

bool AppStorage_LedEnabled(void)
{
    return (s_settings.flags & APP_FLAG_LED_ENABLE) != 0U;
}

uint8_t AppStorage_LedBrightness(void)
{
    if (s_settings.led_brightness > 100U) {
        return 100U;
    }
    return (uint8_t)s_settings.led_brightness;
}

uint32_t AppStorage_ReadID(void)
{
    return W25Q_ReadID();
}
