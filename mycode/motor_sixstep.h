#ifndef MOTOR_SIXSTEP_H
#define MOTOR_SIXSTEP_H
#include "motor_sampling.h"

typedef enum { SIX_IDLE, SIX_ALIGN, SIX_RAMP, SIX_SYNC, SIX_RUN, SIX_FAULT } SixStepState;
typedef enum { SIX_REJECT_NONE, SIX_REJECT_EARLY, SIX_REJECT_LATE,
               SIX_REJECT_SHORT, SIX_REJECT_LONG } SixStepReject;
typedef struct {
    SixStepState state;
    float target,velocity,reference,duty,bemf; /* velocity/reference: generic electrical Hz, preset mechanical rad/s. */
    uint32_t crossings;
    uint8_t sector,quality;
    char gate; /* C/E/S current/EMF/settle during sine; W/P/Z/K/T during six-step. */
    bool speed_control; /* target: signed rad/s when true, 0..100% throttle otherwise. */
    uint16_t phase_pct,period_pct; /* Candidate position / current interval; interval / previous interval, %. */
    uint16_t startup_ms;
    uint8_t period_fraction,reject; /* Tenths of period_pct; frozen rejection cause. */
    float startup_emf; /* Settled I/F estimate, not measured floating-phase residual. */
    uint16_t period_10us[6]; /* Oldest first; includes the rejected candidate. */
    bool sampling_off;
    const char *status;
} SixStepSnapshot;
/* Physical M1 only; no encoder dependency. Called from the paired ADC ISR. */
void SixStep_Process(const MotorSampleFrame *frame);
bool SixStep_Start(float target);
bool SixStep_Throttle(float percent);
void SixStep_Stop(void);
void SixStep_OnTimer(void); /* TIM7 one-shot commutation / loss watchdog. */
bool SixStep_AtRest(void);
bool SixStep_IsGeneric(void);
void SixStep_SetGeneric(bool enable); /* Coordinator only, with outputs stopped. */
float SixStep_MinSpeed(void);
float SixStep_MaxSpeed(void);
void SixStep_GetSnapshot(SixStepSnapshot *out);
#endif
