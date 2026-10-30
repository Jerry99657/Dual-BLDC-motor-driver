#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <stdbool.h>
#include "JerryFOC.h"

typedef enum {
    MOTOR_WORK_SINGLE_M1=0, MOTOR_WORK_SINGLE_M2, MOTOR_WORK_INDEPENDENT,
    MOTOR_WORK_SYNC_SPEED, MOTOR_WORK_SYNC_POSITION,
    MOTOR_WORK_MASTER_M1, MOTOR_WORK_MASTER_M2, MOTOR_WORK_COUNT
} MotorWorkMode;

typedef enum { MOTOR_DRIVE_ENCODER, MOTOR_DRIVE_HSO, MOTOR_DRIVE_SIXSTEP } MotorDriveMode;
typedef enum { MOTOR_INPUT_NONE, MOTOR_INPUT_LOCAL, MOTOR_INPUT_USB,
               MOTOR_INPUT_UART, MOTOR_INPUT_FOLLOW } MotorInput;

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
bool MotorControl_OuterRequired(void); /* Powered normal FOC; excludes alignment/detection/open loop. */
void MotorControl_TimingFault(const char *reason); /* Both gates first; reset-only latch. */
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
bool MotorControl_SixStepAllowed(void);
MotorDriveMode MotorControl_GetDrive(MotorAxis axis);
/* Storage/UI drafts affect control only after this atomic publication.
 * Current-loop changes require stopped outputs; encoder speed caps may change live. */
bool MotorControl_SetOptions(MotorAxis axis,uint32_t flags);
uint32_t MotorControl_GetOptions(MotorAxis axis);
float MotorControl_LimitSpeed(MotorAxis axis,float value);
bool MotorControl_SelectSixStep(bool enable);
bool MotorControl_MotionAllowed(MotorAxis axis);
bool MotorControl_SelectWork(MotorWorkMode mode);
bool MotorControl_SelectFeedback(bool hso);
bool MotorControl_SelectProfile(MotorAxis axis, MotorProfile profile);
bool MotorControl_ApplyCalibration(MotorAxis axis,const MotorCalibration *record);
bool MotorControl_SaveCustom(unsigned slot,const MotorCalibration *record);
bool MotorControl_SelectGeneric(void);
bool MotorControl_SetGear(int32_t ratio_x100, int32_t offset_x100);
bool MotorControl_RequestAlign(MotorAxis axis);
bool MotorControl_CommandFrom(MotorAxis axis,JerryFOC_ControlMode mode,float target,
                             float position_speed,bool current_loop,MotorInput input);
static inline bool MotorControl_Command(MotorAxis axis,JerryFOC_ControlMode mode,float target,
                                        float position_speed,bool current_loop)
{
    return MotorControl_CommandFrom(axis,mode,target,position_speed,current_loop,MOTOR_INPUT_LOCAL);
}
/* Legacy startup paths claim only inside their own bounded command critical section. */
bool MotorControl_ClaimInput(MotorAxis axis,MotorInput input);
MotorInput MotorControl_GetInput(MotorAxis axis);
const char *MotorControl_InputText(MotorAxis axis);
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
float MotorControl_GroupTarget(void); /* Accepted master reference for Sync modes. */
uint32_t MotorControl_MaxCycles(void);
#endif
