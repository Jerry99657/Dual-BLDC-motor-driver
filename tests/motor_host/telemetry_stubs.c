#include "protocol.h"
#include "motor_control.h"
#include "motor_sixstep.h"
#include "MT6701.h"
#include "st_hso.h"
#include "app_storage.h"
AppSaveState AppStorage_GetSaveState(void) { return APP_SAVE_IDLE; }
bool telemetry_host_stopped;
bool MotorSampling_IsGlobalStopped(void) { return telemetry_host_stopped; }
uint32_t telemetry_host_tick;
uint32_t HAL_GetTick(void) { return telemetry_host_tick; }
bool MotorSampling_IsBusFresh(void) { return true; }
float MotorSampling_GetBusVoltage(void) { return 12; }
void MotorSampling_CopyFrame(MotorSampleFrame *out) { *out=(MotorSampleFrame){.m1_current={0.1f,-0.2f,0.1f}}; }
void StHso_GetSnapshot(StHsoSnapshot *s) { *s=(StHsoSnapshot){0}; }
float StHso_ControlVelocity(void) { return JerryFOC_getVelocity(); }
bool StHso_IsRunning(void) { return false; }
void SixStep_GetSnapshot(SixStepSnapshot *s) { *s=(SixStepSnapshot){.status="READY"}; }
void MT6701_GetDiagnostics(MT6701Diagnostics *s) { *s=(MT6701Diagnostics){.errors=16777217U}; }
void MT6701_2_GetSnapshot(MT6701_2_Snapshot *s) { *s=(MT6701_2_Snapshot){0}; }
uint16_t MT6701_GetRawAngle(void) { return 123; }
uint8_t MT6701_IsHealthy(void) { return 1; }
