#ifndef __APP_STORAGE_H
#define __APP_STORAGE_H

#include <stdbool.h>
#include <stdint.h>

#define APP_FLAG_CURRENT_LOOP (1UL << 0)
#define APP_FLAG_LED_ENABLE   (1UL << 1)
/* 在既有 flags 的空闲位保存位置模式速度上限，兼容 version 1 的旧记录。 */
#define APP_POSITION_SPEED_SHIFT 8U
#define APP_POSITION_SPEED_MASK (0x3FUL << APP_POSITION_SPEED_SHIFT)
/* 速度模式目标上限，单位 rad/s；旧记录的空闲位为 0 时采用默认值。 */
#define APP_SPEED_LIMIT_SHIFT 14U
#define APP_SPEED_LIMIT_MASK (0x3FFUL << APP_SPEED_LIMIT_SHIFT)
#define APP_SPEED_LIMIT_DEFAULT 100
#define APP_SPEED_LIMIT_MAX 600
/* Spare v2 flags bit: old records/defaults select 2804, CRC/layout unchanged. */
#define APP_FLAG_MOTOR_4015 (1UL << 24)

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
void AppStorage_Service(uint32_t now_ms);
bool AppStorage_PollSaveResult(bool *success);
/* 只恢复 RAM，应用通过 RequestSave 延迟持久化。 */
void AppStorage_ResetDefaults(void);
void AppStorage_Log(uint16_t event, uint32_t value);
bool AppStorage_LedEnabled(void);
uint8_t AppStorage_LedBrightness(void);
uint32_t AppStorage_ReadID(void);

#endif /* __APP_STORAGE_H */
