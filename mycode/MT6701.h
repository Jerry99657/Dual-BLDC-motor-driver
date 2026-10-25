#ifndef __MT6701_H_
#define __MT6701_H_

#include "main.h"
#include "spi.h"
#include <math.h>
#include <stdbool.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define CS_Enable  HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port, MT6701_CSN_Pin, GPIO_PIN_RESET)
#define CS_Disable HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port, MT6701_CSN_Pin, GPIO_PIN_SET)

extern volatile int Angle_Raw;
extern volatile float Angle;
extern uint8_t MT6701_Data[3];

/* StartContinuous 仅使能定频采样；实际 DMA 请求由 10 kHz ADC 回调中的
 * MT6701_Service() 发起，不在 DMA 完成回调里无限自触发。 */
void MT6701_StartContinuous(void);
void MT6701_StopContinuous(void);
void MT6701_Service(void);
/* Call from main with interrupts enabled: deferred DMA abort/reset/recovery.
 * Recovery restores sampling only; drive output still requires explicit Run. */
void MT6701_MainService(void);
uint8_t MT6701_IsContinuous(void);
uint8_t MT6701_IsHealthy(void);
/* Detection-only all-zero gap near raw zero, <=50 ADC cycles / 5 ms: retain the last verified
 * angle, never refresh it or relax the normal 3-cycle FOC health threshold. */
uint8_t MT6701_HasNearZeroGap(void);
uint16_t MT6701_GetRawAngle(void);
uint8_t MT6701_GetStatus(void);
uint32_t MT6701_GetSampleCounter(void);
uint32_t MT6701_GetErrorCounter(void);
uint16_t MT6701_GetSampleAgeCycles(void);

typedef enum {
    MT6701_ERROR_NONE = 0,
    MT6701_ERROR_FRAME, /* All-zero frame, ambiguous with angle=0/status=0. */
    MT6701_ERROR_CRC,
    MT6701_ERROR_FIELD,
    MT6701_ERROR_TRACK,
    MT6701_ERROR_SPI,
    MT6701_ERROR_ONES, /* All-one frame; also included in frame_errors. */
    MT6701_ERROR_TIMEOUT /* M1 DMA timeout; counted in spi_errors. */
} MT6701Error;

typedef struct {
    uint32_t errors, crc_errors, field_errors, track_errors, spi_errors, frame_errors;
    uint16_t age_cycles, max_age_cycles;
    /* Most recent all-zero episode, frozen until the next episode starts.
     * Count is zero observations before the first subsequent valid frame;
     * intervening CRC/status faults do not resolve the episode. UINT16_MAX
     * zero_after means unresolved; zero_age is pre-event cycle age, raised to
     * at least four if wall-clock freshness expired despite a paused clock. */
    uint16_t zero_before, zero_after, zero_count, zero_age;
    uint32_t zero_started_at, zero_ended_at; /* HAL milliseconds, unsigned wrap. */
    uint8_t status;
    MT6701Error last_error;
} MT6701Diagnostics;
/* Cumulative since reset; constant-level frames are counted before CRC/status.
 * Last error persists after good frames; CRC-invalid status bits are never
 * classified as magnetic errors. max_age starts after the first valid frame. */
void MT6701_GetDiagnostics(MT6701Diagnostics *out);
const char *MT6701_ErrorText(MT6701Error error);

/* M2 encoder on SPI2 / MT6701_CSN2 (PB12). SPI2 is shared with W25Q64:
 * FastService schedules DMA at 10 kHz when M2 is enabled; main Service owns
 * abort/recovery and optional 1 kHz diagnostics when M2 is disabled.
 * Five consecutive failures stop optional polling. An enabled/required ENC2
 * probes once per 250 ms after motion releases SPI2; valid data resumes DMA.
 * A latched optional ENC2 still requires explicit Retry or enabling its axis.
 * Valid frames reset the streak; a busy SPI2/W25Q64 does not count as a failure.
 * Reserved output/align intervals defer Flash; every CS interval owns SPI2. */
typedef struct {
    uint16_t raw_angle;
    uint8_t status;
    uint8_t healthy;
    uint32_t samples;
    uint32_t errors;
    float angle;
    uint16_t age_cycles;
    uint8_t stopped;
} MT6701_2_Snapshot;
void MT6701_2_Service(uint32_t now_ms);
void MT6701_2_GetSnapshot(MT6701_2_Snapshot *out);
void MT6701_2_GetDiagnostics(MT6701Diagnostics *out);
void MT6701_2_SetRealtime(bool enable);
void MT6701_2_FastService(void);
void MT6701_2_Retry(void); /* Explicit stopped-axis retry, including optional ENC2. */
bool MT6701_2_Reserve(bool active); /* Refuses a live Flash/diagnostic CS. */
bool MT6701_2_StorageBlocked(void);
bool MT6701_2_FlashBegin(void);
void MT6701_2_FlashEnd(void);
/* Shared SPI2 recovery is deferred to main, never performed while Flash owns
 * CS or in the control IRQ. The snapshot is taken before abort clears flags. */
void MT6701_2_BusFault(void);
typedef struct {
    uint16_t cr1, cr2, sr, resets;
} MT6701_2_BusDiagnostics;
void MT6701_2_GetBusDiagnostics(MT6701_2_BusDiagnostics *out);

#endif /* __MT6701_H_ */
