#ifndef __APP_STORAGE_H
#define __APP_STORAGE_H

#include <stdbool.h>
#include <stdint.h>

#define APP_FLAG_CURRENT_LOOP (1UL << 0)
#define APP_FLAG_LED_ENABLE   (1UL << 1)
/* 在既有 flags 的空闲位保存位置模式速度上限，兼容 version 1 的旧记录。 */
#define APP_POSITION_SPEED_SHIFT 8U
/* Keep the legacy low six bits; extend into previously unused flags[28:25].
 * Existing v1/v2/v3 records decode unchanged. Never overlap speed/model flags. */
#define APP_POSITION_SPEED_LOW_MASK (0x3FUL << APP_POSITION_SPEED_SHIFT)
#define APP_POSITION_SPEED_HIGH_SHIFT 25U
#define APP_POSITION_SPEED_MASK (APP_POSITION_SPEED_LOW_MASK | (0xFUL << APP_POSITION_SPEED_HIGH_SHIFT))
static inline uint32_t AppSettings_GetPositionSpeed(uint32_t flags)
{
    return ((flags & APP_POSITION_SPEED_LOW_MASK) >> APP_POSITION_SPEED_SHIFT) |
           (((flags >> APP_POSITION_SPEED_HIGH_SHIFT) & 0xFU) << 6U);
}
static inline uint32_t AppSettings_EncodePositionSpeed(uint32_t speed)
{
    return ((speed & 0x3FU) << APP_POSITION_SPEED_SHIFT) |
           (((speed >> 6U) & 0xFU) << APP_POSITION_SPEED_HIGH_SHIFT);
}
/* 速度模式目标上限，单位 rad/s；旧记录的空闲位为 0 时采用默认值。 */
#define APP_SPEED_LIMIT_SHIFT 14U
#define APP_SPEED_LIMIT_MASK (0x3FFUL << APP_SPEED_LIMIT_SHIFT)
#define APP_SPEED_LIMIT_DEFAULT 100
#define APP_SPEED_LIMIT_MAX 600
/* Spare v2 flags bit: old records/defaults select 2804, CRC/layout unchanged. */
#define APP_FLAG_MOTOR_4015 (1UL << 24)
/* Spare v3 control_config bit; old records select M1 without migration. */
#define APP_CONFIG_FOCUS_M2 (1UL << 5)
/* v3 spare bits: 100mA units, zero/invalid legacy values use 3.0A. */
#define APP_CONFIG_IBUS_SHIFT 8U
#define APP_CONFIG_IBUS_MASK (63UL << APP_CONFIG_IBUS_SHIFT)
#define APP_IBUS_MIN_MA 500U
#define APP_IBUS_MAX_MA 4000U
#define APP_IBUS_DEFAULT_MA 3000U
static inline uint16_t AppSettings_GetIbusLimit(uint32_t config)
{
    unsigned ma=((config & APP_CONFIG_IBUS_MASK)>>APP_CONFIG_IBUS_SHIFT)*100U;
    return ma>=APP_IBUS_MIN_MA && ma<=APP_IBUS_MAX_MA ? (uint16_t)ma : APP_IBUS_DEFAULT_MA;
}

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t seq;
    int32_t speed_x10;       /* 0.1 rad/s */
    int32_t voltage_x100;    /* 0.01 V */
    int32_t position_x100;   /* 0.01 rad */
    uint32_t flags;
    uint32_t led_brightness; /* 0..100 */
    int32_t speed_step_x10;       /* 1..100: 0.1..10.0 rad/s */
    int32_t torque_step_x100;     /* 1..100: 0.01..1.00 V or A */
    int32_t position_step_x100;   /* 1..100: 0.01..1.00 rad */
    uint32_t control_config;     /* work[2:0], HSO[3], M2 model[4], focus[5], IBUS 100mA[13:8] */
    int32_t gear_ratio_x100;     /* Signed mechanical follower ratio; default 100. */
    int32_t gear_offset_x100;    /* Relative position offset, 0.01 rad. */
    uint32_t axis2_flags;        /* Current loop and per-axis speed limits. */
    int32_t axis2_speed_x10;
    int32_t axis2_voltage_x100;
    int32_t axis2_position_x100;
    int32_t axis2_speed_step_x10;
    int32_t axis2_torque_step_x100;
    int32_t axis2_position_step_x100;
    uint32_t crc;
} AppSettings;

/* 日志事件 */
enum {
    APP_LOG_BOOT = 1,
    APP_LOG_MODE_CHANGED,
    APP_LOG_TARGET_CHANGED,
    APP_LOG_SETTINGS_SAVED,
    APP_LOG_ENCODER_ERROR,
    APP_LOG_ALIGN_DONE,
};

bool AppStorage_Init(void);
bool AppStorage_IsReady(void);
AppSettings *AppStorage_Get(void);
void AppStorage_Load(void);
bool AppStorage_Save(void); /* 同步保存，仅用于启动/维护阶段。 */
bool AppStorage_RequestSave(void);
bool AppStorage_IsDeferred(void); /* M2 real-time sampling currently owns SPI2. */
typedef enum {
    APP_SAVE_IDLE, APP_SAVE_PENDING, APP_SAVE_WRITING, APP_SAVE_DEFERRED,
    APP_SAVE_SAVED, APP_SAVE_FAILED
} AppSaveState;
AppSaveState AppStorage_GetSaveState(void);
void AppStorage_Service(uint32_t now_ms);
bool AppStorage_PollSaveResult(bool *success);
const char *AppStorage_SaveErrorText(void); /* Last settings I/O/verify stage, or Save OK. */
uint32_t AppStorage_FlashError(void); /* Last failing settings transaction; W25Q_LastFailure layout. */
/* 只恢复 RAM，应用通过 RequestSave 延迟持久化。 */
void AppStorage_ResetDefaults(void);
void AppStorage_Log(uint16_t event, uint32_t value);
bool AppStorage_LedEnabled(void);
uint8_t AppStorage_LedBrightness(void);
uint32_t AppStorage_ReadID(void);

#endif /* __APP_STORAGE_H */
