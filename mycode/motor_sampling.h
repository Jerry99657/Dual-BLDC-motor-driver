#ifndef MOTOR_SAMPLING_H
#define MOTOR_SAMPLING_H

#include <stdbool.h>
#include <stdint.h>

/* 固定对应当前 CubeMX 注入组，保留原始值供 HSO/第二路控制使用。 */
typedef struct {
    uint16_t m1_current_raw[3];
    uint16_t m2_current_raw[2];
    uint16_t m1_voltage_raw[3];
    float m1_current[3];
    float m2_current[2];
    float m1_voltage[3];
    uint32_t sequence;
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
    MOTOR_SAMPLE_FAULT_REFERENCE
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
} MotorSamplingDiagnostics;

/* Starts continuous measurement even at 0 V. M1 power waits for current zero
 * calibration and stable VBUS; M2 power always stays off. */
bool MotorSampling_Start(void);
void MotorSampling_Stop(void);
void MotorSampling_Service(void);
bool MotorSampling_IsReady(void);
bool MotorSampling_IsCalibrated(void);
float MotorSampling_GetBusVoltage(void);
void MotorSampling_GetDiagnostics(MotorSamplingDiagnostics *out);
const char *MotorSampling_GetStatusText(void);
/* 单一 ADC1 JEOS 回调中调用；成功后才计算控制环。 */
bool MotorSampling_Capture(void);
/* 指针仅供同一控制 ISR 使用；主循环应使用 CopyFrame。 */
const MotorSampleFrame *MotorSampling_GetFrame(void);
void MotorSampling_CopyFrame(MotorSampleFrame *out);

#endif
