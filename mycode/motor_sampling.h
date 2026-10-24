#ifndef MOTOR_SAMPLING_H
#define MOTOR_SAMPLING_H

#include <stdbool.h>
#include <stdint.h>
#include "motor_params.h"

/* 固定对应当前 CubeMX 注入组，保留原始值供 HSO/第二路控制使用。 */
typedef struct {
    uint16_t m1_current_raw[3];
    uint16_t m2_current_raw[2];
    uint16_t m1_voltage_raw[3];
    float m1_current[3];
    float m2_current[2];
    float m1_voltage[3];
    uint32_t sequence;
    uint32_t m1_sixstep_stamp;
    bool m1_sixstep_valid;
    bool m1_sixstep_off; /* OFF-center sampling: all driven phases LOW. */
    bool m1_voltage_high_plateau; /* All M1 high sides stayed on across voltage ranks; sampled before arming. */
} MotorSampleFrame;

typedef enum {
    MOTOR_SAMPLE_FAULT_NONE = 0,
    MOTOR_SAMPLE_FAULT_INIT,
    MOTOR_SAMPLE_FAULT_ADC,
    MOTOR_SAMPLE_FAULT_PAIR,
    MOTOR_SAMPLE_FAULT_FRAME_TIMEOUT,
    MOTOR_SAMPLE_FAULT_BUS_TIMEOUT,
    MOTOR_SAMPLE_FAULT_BUS_LOST,
    MOTOR_SAMPLE_FAULT_BUS_RANGE,
    MOTOR_SAMPLE_FAULT_OVERCURRENT,
    MOTOR_SAMPLE_FAULT_CURRENT_RANGE,
    MOTOR_SAMPLE_FAULT_OVERLOAD,
    MOTOR_SAMPLE_FAULT_REFERENCE,
    MOTOR_SAMPLE_FAULT_IBUS,
    MOTOR_SAMPLE_FAULT_IBUS_INIT
} MotorSamplingFault;

typedef struct {
    MotorSamplingFault fault;
    uint16_t bus_raw;
    uint32_t bus_samples;
    uint32_t frame_samples;
    uint32_t bus_age_ms;
    uint32_t frame_age_ms;
    uint32_t adc1_error;
    uint32_t adc2_error;
    float bus_voltage;
    bool bus_fresh;
    bool power_enabled;
    uint16_t current_raw[3];       /* Live U/V/W ADC counts. */
    uint16_t current_zero[3];      /* Calibrated zero, rounded ADC counts. */
    uint16_t fault_current_raw[3]; /* First current fault, never overwritten. */
    uint8_t fault_phase;          /* 0: none/vector fault; 1/2/3: U/V/W. */
    bool fault_during_zero;
    uint16_t silicon_revision;
    float reference_voltage;     /* Filtered estimate of actual ADC VREF+. */
    uint16_t reference_raw;       /* Latest VREFINT ADC code, not VREF+ code. */
    uint16_t reference_cal;       /* Factory 12-bit code at 3.0 V VREF+. */
    uint32_t reference_updates;
    uint32_t reference_errors;
    uint32_t reference_age_ms;
    bool reference_fresh;
    uint16_t ibus_limit_ma, ibus_dac_code;
    float ibus_threshold_v;
    uint32_t ibus_trips;
    bool ibus_ready, ibus_latched, ibus_high;
} MotorSamplingDiagnostics;

/* Starts continuous measurement even at 0 V. Both bridges wait for current
 * zero calibration and stable VBUS; the axis controllers explicitly arm them. */
bool MotorSampling_Start(void);
void MotorSampling_Stop(void);
void MotorSampling_Service(void);
bool MotorSampling_IsReady(void);
bool MotorSampling_IsCalibrated(void);
/* Gate only M1's six power outputs. CH4/ADC and all protection stay active.
 * Enabling requires readiness; two injected frames preload fresh PWM first. */
bool MotorSampling_SetOutputEnabled(bool enable);
bool MotorSampling_IsOutputRequested(void);
bool MotorSampling_IsOutputEnabled(void);
/* Axis current faults are independent; shared ADC/bus/reference faults stop
 * both bridges. M2 W current is reconstructed from its two measured phases. */
bool MotorSampling_IsAxisReady(MotorAxis axis);
bool MotorSampling_SetAxisOutputEnabled(MotorAxis axis, bool enable);
bool MotorSampling_IsAxisOutputRequested(MotorAxis axis);
bool MotorSampling_IsAxisOutputEnabled(MotorAxis axis);
void MotorSampling_GetAxisDiagnostics(MotorAxis axis, MotorSamplingDiagnostics *out);
const char *MotorSampling_GetAxisStatusText(MotorAxis axis);
float MotorSampling_GetBusVoltage(void);
void MotorSampling_GetDiagnostics(MotorSamplingDiagnostics *out);
const char *MotorSampling_GetStatusText(void);
/* 单一 ADC1 JEOS 回调中调用；成功后才计算控制环。 */
bool MotorSampling_Capture(void);
/* 指针仅供同一控制 ISR 使用；主循环应使用 CopyFrame。 */
const MotorSampleFrame *MotorSampling_GetFrame(void);
void MotorSampling_CopyFrame(MotorSampleFrame *out);

/* Physical M1 IBUS only: 5mOhm / INA240A2 / half-reference zero / COMP4.
 * Threshold changes require M1 power OFF. Clear never requests PWM or clears
 * unrelated ADC/current faults; caller must stop its controller first. */
bool MotorSampling_SetIbusLimit(uint16_t milliamps);
uint16_t MotorSampling_GetIbusLimit(void);
bool MotorSampling_IsIbusTripped(void);
bool MotorSampling_ClearIbusTrip(void);

/* KEY1 global coast latch. IRQ only disconnects power and records the event;
 * main-loop control consumes it before a fresh explicit start may rearm.
 * Release alone never resumes; require a stable 30 ms release. */
void MotorSampling_GlobalStopIRQ(void);
bool MotorSampling_IsGlobalStopped(void);
bool MotorSampling_TakeGlobalStop(void);
bool MotorSampling_ResumeGlobalStop(void);
/* M1 six-step owns only its output mode and spare TIM7. Mode changes require
 * stopped outputs; the shared ADC/PWM clocks and M2 stay unchanged. */
bool MotorSampling_SelectSixStep(bool enable);
/* 0..5: two-phase conduction, third floating. 6: all three complementary
 * PWM1 pairs for encoder-free sine startup; caller sets individual CCRs. */
bool MotorSampling_SixStepPattern(uint8_t sector,float duty);
bool MotorSampling_SixStepTimer(uint32_t microseconds);

#endif
