#ifndef MOTOR_DETECT_H
#define MOTOR_DETECT_H
#include "motor_sampling.h"
/* Single M1, stopped encoder feedback only. R/poles, aligned equivalent L and
 * PM flux/Ke identification; no preset electrical parameters are used by the
 * excitation. Results are measurements, never auto-applied. */
bool MotorDetect_Start(unsigned milliamps);
bool MotorDetect_Active(void);
void MotorDetect_Stop(void);
void MotorDetect_Reset(void);
bool MotorDetect_Process(const MotorSampleFrame *frame); /* Owns this ADC cycle. */
void MotorDetect_Service(uint32_t now);
const char *MotorDetect_Status(void);
/* Signed pole count in byte 0 (encoder direction), equivalent-star Rs in
 * milliohms in bits[23:8]. Zero means no complete, validated measurement. */
uint32_t MotorDetect_Result(void);
/* Equivalent aligned inductance (uH) and phase PM flux (uWb). Published with
 * Result only after the complete run; zero also denotes a legacy R/poles record. */
uint32_t MotorDetect_Inductance(void);
uint32_t MotorDetect_Flux(void);
MotorProfile MotorDetect_Profile(void); /* Captured at start, independent of later UI changes. */
#endif
