#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <stdbool.h>
#include "JerryFOC.h"

typedef enum {
    MOTOR_WORK_SINGLE_M1=0, MOTOR_WORK_SINGLE_M2, MOTOR_WORK_INDEPENDENT,
    MOTOR_WORK_SYNC_SPEED, MOTOR_WORK_SYNC_POSITION,
    MOTOR_WORK_MASTER_M1, MOTOR_WORK_MASTER_M2, MOTOR_WORK_COUNT
} MotorWorkMode;

typedef struct {
    JerryFOC_ControlMode mode;
    float velocity, position, electrical_angle, iq, id, ia, ib;
    float target_velocity, target_position, target_iq;
    bool healthy, aligned, active, aligning, current_loop, ready;
    const char *status;
} MotorAxisSnapshot;

void MotorControl_Init(void);
void MotorControl_Service(uint32_t now);
void MotorControl_Fast(void);  /* ADC JEOS, after the complete paired frame. */
void MotorControl_PrepareOuter(void); /* Both new measurements, then shared references. */
void MotorControl_Outer(void); /* TIM2, 1 kHz; after M1 encoder/outer update. */
void MotorControl_RecordCycles(uint32_t cycles);
void MotorControl_Telemetry(float data[25]);
bool MotorControl_AxisEnabled(MotorAxis axis);
bool MotorControl_IsCoupled(void);
bool MotorControl_IsHandFollow(void);
bool MotorControl_IsPassive(MotorAxis axis);
bool MotorControl_StartFollow(void); /* Capture both shaft origins; power only the follower. */
/* Position correction plus trajectory velocity/damping in hand-follow;
 * ordinary position modes retain the supplied position controller. */
float MotorControl_PositionVelocity(MotorAxis axis, PIDController *position, float error);
bool MotorControl_IsFastFollow(MotorAxis axis);
float MotorControl_SpeedReference(MotorAxis axis,float previous,float target);
float MotorControl_VelocityControl(MotorAxis axis, PIDController *velocity,
                                  float reference, float measured);
bool MotorControl_HsoAllowed(void);
bool MotorControl_MotionAllowed(MotorAxis axis);
bool MotorControl_SelectWork(MotorWorkMode mode);
bool MotorControl_SelectFeedback(bool hso);
bool MotorControl_SelectProfile(MotorAxis axis, MotorProfile profile);
bool MotorControl_SetGear(int32_t ratio_x100, int32_t offset_x100);
bool MotorControl_RequestAlign(MotorAxis axis);
bool MotorControl_Command(MotorAxis axis, JerryFOC_ControlMode mode, float target,
                          float position_speed, bool current_loop);
void MotorControl_Stop(MotorAxis axis);
void MotorControl_StopAll(void);
void MotorControl_GetAxis(MotorAxis axis, MotorAxisSnapshot *out);
MotorWorkMode MotorControl_GetWork(void);
const char *MotorControl_WorkText(void);
const char *MotorControl_CommandText(void);
const char *MotorControl_EncoderLostText(MotorAxis axis);
MotorAxis MotorControl_GetFocus(void);
void MotorControl_SetFocus(MotorAxis axis); /* View only, never starts/stops. */
bool MotorControl_ModeAllowed(JerryFOC_ControlMode mode);
bool MotorControl_IsGroupActive(void);
float MotorControl_GroupError(void);
uint32_t MotorControl_MaxCycles(void);
#endif
