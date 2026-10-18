#include "app_storage.h"
#include "w25q64.h"
#include "main.h"

#include <stddef.h>
#include <string.h>

#define APP_SETTINGS_MAGIC 0x464F4331UL  /* "FOC1" */
#define APP_SETTINGS_VER   2U
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
    /* version 1 的 CRC 位于新增步长字段之前；校验旧记录后在 RAM 中迁移，
     * 不丢失原有目标、LED、位置速度上限和序号。下次保存才写 version 2。 */
    const uint32_t legacy_crc_offset = (uint32_t)offsetof(AppSettings, speed_step_x10);
    if (out->magic == APP_SETTINGS_MAGIC && out->version == 1U &&
        out->size == legacy_crc_offset + sizeof(uint32_t)) {
        uint32_t stored_crc;
        memcpy(&stored_crc, (const uint8_t *)out + legacy_crc_offset, sizeof(stored_crc));
        if (stored_crc != crc32_bytes((const uint8_t *)out, legacy_crc_offset)) return false;
        out->version = APP_SETTINGS_VER;
        out->size = sizeof(*out);
        out->speed_step_x10 = out->torque_step_x100 = out->position_step_x100 = 5;
        out->crc = crc32_bytes((const uint8_t *)out, (uint32_t)offsetof(AppSettings, crc));
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
    s_save_result_ready = false;
    s_log_head = s_log_count = 0U;
    s_service_at = HAL_GetTick();
    s_ready = W25Q_Init();
    if (!s_ready) {
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
    if (!s_ready || s_store_state != STORE_IDLE || s_save_pending) {
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
    if (!s_ready || s_save_pending || s_save_result_ready) return false;
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

static void store_complete(bool success)
{
    if (s_store_state == STORE_SETTINGS_ERASE || s_store_state == STORE_SETTINGS_PROGRAM) {
        s_save_pending = false;
        s_save_result_ready = true;
        s_save_success = success;
        if (success) {
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

void AppStorage_Service(uint32_t now_ms)
{
    if (!s_ready || now_ms == s_service_at) return;
    s_service_at = now_ms;
    if (s_store_state == STORE_IDLE && !s_save_pending && s_log_count == 0U) return;
    bool busy;
    if (!W25Q_PollBusy(&busy)) {
        if (s_store_state == STORE_IDLE && s_save_pending) s_store_state = STORE_SETTINGS_ERASE;
        store_complete(false);
        return;
    }
    if (busy) {
        uint32_t timeout = (s_store_state == STORE_SETTINGS_PROGRAM ||
                            s_store_state == STORE_LOG_PROGRAM) ? 10U : 1000U;
        if (s_store_state != STORE_IDLE && now_ms - s_store_started >= timeout)
            store_complete(false);
        return;
    }
    bool ok = true;
    switch (s_store_state) {
    case STORE_SETTINGS_ERASE:
        s_store_state = STORE_SETTINGS_PROGRAM;
        s_store_started = now_ms;
        ok = W25Q_BeginPageProgram(s_save_addr, (const uint8_t *)&s_save_snapshot,
                                   sizeof(s_save_snapshot));
        break;
    case STORE_SETTINGS_PROGRAM: {
        AppSettings verify;
        ok = W25Q_Read(s_save_addr, (uint8_t *)&verify, sizeof(verify)) &&
             memcmp(&verify, &s_save_snapshot, sizeof(verify)) == 0 && settings_is_valid(&verify);
        store_complete(ok);
        return;
    }
    case STORE_LOG_ERASE:
        s_log_addr = 0U;
        s_store_state = STORE_IDLE;
        break;
    case STORE_LOG_PROGRAM:
        store_complete(true);
        break;
    default:
        if (s_save_pending) {
            s_store_state = STORE_SETTINGS_ERASE;
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
        s_store_started = now_ms;
        break;
    }
    if (!ok) store_complete(false);
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
