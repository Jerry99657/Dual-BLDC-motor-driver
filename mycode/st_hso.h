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
    ST_HSO_INPUT_RANGE, ST_HSO_TIMING, ST_HSO_READY, ST_HSO_ALIGN,
    ST_HSO_RAMP, ST_HSO_CAPTURE, ST_HSO_CONTROL, ST_HSO_FAULT
} StHsoState;

typedef enum {
    ST_HSO_REASON_NONE = 0, ST_HSO_REASON_OFF, ST_HSO_REASON_PARAM,
    ST_HSO_REASON_SAMPLE, ST_HSO_REASON_SEQUENCE, ST_HSO_REASON_RANGE,
    ST_HSO_REASON_ENCODER, ST_HSO_REASON_SPEED, ST_HSO_REASON_ANGLE,
    ST_HSO_REASON_FREQUENCY, ST_HSO_REASON_FLUX, ST_HSO_REASON_NUMERIC,
    ST_HSO_REASON_TIMING, ST_HSO_REASON_STARTUP, ST_HSO_REASON_STANDSTILL,
    ST_HSO_REASON_SETTLE, ST_HSO_REASON_EMF_LOW, ST_HSO_REASON_FLUX_RIPPLE,
    ST_HSO_REASON_LEARN_TIMEOUT, ST_HSO_REASON_CAPTURE_TIMEOUT, ST_HSO_REASON_PHASE_DRIFT
} StHsoReason;

typedef struct {
    StHsoState state;
    float angle_rad;
    float velocity_rad_s; /* Mechanical, signed. */
    float flux_wb; /* Learning: last whole-cycle EMF estimate; thereafter HSO flux. */
    float reference_flux_wb;
    float angle_error_rad; /* Startup: HSO minus I/F angle; run: angle-step residual. Never encoder. */
    float emf_hz; /* Electrical, signed. */
    uint32_t samples;
    uint32_t max_cycles; /* Whole adapter, including initialization/learning. */
    uint32_t overruns; /* > 100 us at 170 MHz. */
    uint32_t max_control_cycles; /* Whole ADC capture/observer/FOC callback. */
    bool usable; /* Electrically qualified sensorless control, not zero-speed holding. */
    StHsoReason reason; /* Current qualification/hold rejection; not latched. */
    StHsoReason entry_reason; /* Admission check; last learning rejection retained for timeout diagnostics. */
    uint32_t qualified_samples;
    uint32_t bad_samples; /* Consecutive soft hold failures; 20 => lost. */
    uint32_t capture_resets;
    StHsoReason capture_reason; /* Last qualification reset; survives good frames/Stop. */
    float capture_angle_rad, capture_frequency_error_hz, capture_emf_ratio;
    float capture_drift_rad; /* Last reset: phase change from frozen window anchor. */
    float emf_offset_v,emf_offset_limit_v; /* Max absolute alpha/beta DC correction / per-axis limit, volts. */
} StHsoSnapshot;

/* Call once before starting ADC interrupts. Owns CORDIC in ADC1 ISR only. */
void StHso_Init(const MotorParams *motor);
bool StHso_SetConfig(const StHsoConfig *config);
void StHso_GetConfig(StHsoConfig *config);
void StHso_Enable(bool enable);
bool StHso_LearnFlux(void); /* Stopped only: clear flux, next I/F start re-estimates it. */
bool StHso_Start(float mechanical_target); /* Explicit start; requires quiet idle sampling. */
void StHso_Stop(void);
bool StHso_IsStarting(void);
bool StHso_IsRunning(void);
float StHso_ControlAngle(void);
float StHso_ControlVelocity(void);
float StHso_CurrentD(void); /* Remaining I/F current-vector amplitude, before frame rotation. */
void StHso_BlendCurrent(float *d,float *q); /* q input is speed-PI request; outputs are control-frame dq. */
float StHso_MinSpeed(void); /* Nonzero command floor, rounded upward to a 5-rad/s multiple. */
bool StHso_AtRest(void);
bool StHso_ValidTarget(float target);
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
