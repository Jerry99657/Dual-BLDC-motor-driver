#ifndef ST_HSO_ADAPTER_H
#define ST_HSO_ADAPTER_H

#include <stdbool.h>
#include <stdint.h>
#include "motor_params.h"
#include "motor_sampling.h"

typedef struct {
    float rs_ohm;
    float ls_h;
    float flux_wb; /* Physical PM flux linkage, NOT MCSDK ratedFlux = 2*pi*flux. */
    float voltage_pole_rps;
    float sample_delay_cycles;
} StHsoConfig;

typedef enum {
    ST_HSO_OFF = 0, ST_HSO_WAIT_PARAM, ST_HSO_WAIT_SAMPLE,
    ST_HSO_OBSERVING, ST_HSO_LEARNING, ST_HSO_TRACKING,
    ST_HSO_INPUT_RANGE, ST_HSO_TIMING
} StHsoState;

typedef enum {
    ST_HSO_REASON_NONE = 0, ST_HSO_REASON_OFF, ST_HSO_REASON_PARAM,
    ST_HSO_REASON_SAMPLE, ST_HSO_REASON_SEQUENCE, ST_HSO_REASON_RANGE,
    ST_HSO_REASON_ENCODER, ST_HSO_REASON_SPEED, ST_HSO_REASON_ANGLE,
    ST_HSO_REASON_FREQUENCY, ST_HSO_REASON_FLUX, ST_HSO_REASON_NUMERIC,
    ST_HSO_REASON_TIMING
} StHsoReason;

typedef struct {
    StHsoState state;
    float angle_rad;
    float velocity_rad_s; /* Mechanical, signed. */
    float flux_wb;
    float reference_flux_wb;
    float angle_error_rad; /* HSO minus aligned encoder, wrapped +/- pi. */
    float emf_hz; /* Electrical, signed. */
    uint32_t samples;
    uint32_t max_cycles; /* Whole adapter, including initialization/learning. */
    uint32_t overruns; /* > 100 us at 170 MHz. */
    uint32_t max_control_cycles; /* Whole ADC capture/observer/FOC callback. */
    bool usable; /* Encoder-validated spinning takeover; NOT a standstill lock. */
    StHsoReason reason; /* Current qualification/hold rejection; not latched. */
    StHsoReason entry_reason; /* Strict current-frame admission check. */
    uint32_t qualified_samples;
    uint32_t bad_samples; /* Consecutive soft hold failures; 20 => lost. */
    uint32_t qualify_paused_samples; /* Small angle glitches, max 50 per attempt. */
    uint32_t qualify_bad_run; /* Max 10 consecutive paused frames. */
    float qualify_reset_angle_rad; /* Last angle-triggered admission reset, signed. */
} StHsoSnapshot;

/* Call once before starting ADC interrupts. Owns CORDIC in ADC1 ISR only. */
void StHso_Init(const MotorParams *motor);
bool StHso_SetConfig(const StHsoConfig *config);
void StHso_GetConfig(StHsoConfig *config);
void StHso_Enable(bool enable);
bool StHso_LearnFlux(void); /* Passive: requires encoder-controlled steady rotation. */
void StHso_Process(const MotorSampleFrame *frame, bool sampling_ready,
                   float encoder_angle, float encoder_velocity, bool encoder_ready);
void StHso_GetSnapshot(StHsoSnapshot *snapshot);
const char *StHso_StateText(StHsoState state);
const char *StHso_ReasonText(StHsoReason reason);
bool StHso_IsUsable(void);
bool StHso_CanTakeover(void); /* Qualified and strict current-frame entry checks. */
float StHso_Angle(void);
float StHso_Velocity(void);
void StHso_RecordControlCycles(uint32_t cycles);

#endif
