#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "JerryFOC.h"
#include "motor_sampling.h"
#include "adc.h"
#include "angle_src_encoder.h"
#include "st_hso.h"
#include "protocol.h"
#include "usart.h"
#include "usb_device.h"

UART_HandleTypeDef huart1;
USBD_HandleTypeDef hUsbDeviceFS;
volatile uint8_t g_usb_cdc_ready;
static float telemetry_reference;
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h,uint8_t *p,uint16_t n) {
    (void)h;
    if(n==104) memcpy(&telemetry_reference,p+7*sizeof(float),sizeof(float));
    return HAL_OK;
}
uint8_t CDC_Transmit_FS(uint8_t *p,uint16_t n) { (void)p; (void)n; return 0; }
uint8_t CDC_TxReady_FS(void) { return 0; }
void StHso_Enable(bool en) { (void)en; }
bool StHso_LearnFlux(void) { return true; }
void StHso_GetConfig(StHsoConfig *out) { *out=(StHsoConfig){0}; }
bool StHso_SetConfig(const StHsoConfig *c) { (void)c; return true; }
static void command(const char *s) {
    Protocol_Feed(PROTO_SRC_USB,(const uint8_t *)s,(uint32_t)strlen(s));
    Protocol_Feed(PROTO_SRC_USB,(const uint8_t *)"\n",1);
}

HostDwt host_dwt;
uint32_t host_primask;
ADC_TypeDef adc1,adc2;
ADC_HandleTypeDef hadc1={.Instance=ADC1},hadc2={.Instance=ADC2};
TIM_TypeDef tim1,tim8,tim6;
TIM_HandleTypeDef htim1={&tim1},htim8={&tim8},htim6={&tim6};
static MotorSampleFrame frame;
static float bus=12.0f;
static bool ready=true, calibrated=true, complete=true;
static unsigned captures, encoder_services, aligns;
static uint8_t healthy=1, aligned=1;
static bool hso_usable;
static bool hso_entry_ok=true;
static unsigned hso_processes;
static bool timing_loss_after_control;
bool StHso_IsUsable(void) { return hso_usable; }
bool StHso_CanTakeover(void) { return hso_usable && hso_entry_ok; }
float StHso_Angle(void) { return 0.1f; }
static float hso_velocity=110.0f;
float StHso_Velocity(void) { return hso_velocity; }
static StHsoReason reject_reason=ST_HSO_REASON_ANGLE;
void StHso_GetSnapshot(StHsoSnapshot *out) { *out=(StHsoSnapshot){.reason=reject_reason}; }
const char *StHso_ReasonText(StHsoReason reason) { return reason==ST_HSO_REASON_ANGLE ? "ANGLE" : "TIMING"; }
void StHso_Process(const MotorSampleFrame *f,bool r,float a,float v,bool e) {
    (void)f; (void)r; (void)a; (void)v; (void)e; ++hso_processes;
}
void StHso_RecordControlCycles(uint32_t cycles) {
    (void)cycles;
    if(timing_loss_after_control) { hso_usable=false; reject_reason=ST_HSO_REASON_TIMING; }
}
static float encoder_velocity;
static float zero(void) { return 0; }
static float encoder_speed(void) { return encoder_velocity; }
static AngleSource source={zero,zero,encoder_speed,1};
uint32_t HAL_GetTick(void) { return 0; }
void HAL_Delay(uint32_t ms) { (void)ms; }
bool MotorSampling_IsReady(void) { return ready; }
bool MotorSampling_IsCalibrated(void) { return calibrated; }
float MotorSampling_GetBusVoltage(void) { return bus; }
bool MotorSampling_Capture(void) { ++captures; return complete; }
const MotorSampleFrame *MotorSampling_GetFrame(void) { return &frame; }
uint8_t MT6701_IsContinuous(void) { return 1; }
uint8_t MT6701_IsHealthy(void) { return healthy; }
uint16_t MT6701_GetRawAngle(void) { return 1000; }
uint8_t MT6701_GetStatus(void) { return 0; }
uint16_t MT6701_GetSampleAgeCycles(void) { return 0; }
uint32_t MT6701_GetErrorCounter(void) { return 0; }
void MT6701_Service(void) { ++encoder_services; }
void AngleSrc_Encoder_Align(float angle) { (void)angle; ++aligns; }
void AngleSrc_Encoder_Update(float dt) { (void)dt; }
uint8_t AngleSrc_Encoder_IsSource(const AngleSource *s) { return s==&source; }
uint8_t AngleSrc_Encoder_IsAligned(void) { return aligned; }
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);
static void zero_pwm(void) { assert(tim1.CCR1==tim1.CCR2 && tim1.CCR2==tim1.CCR3); }
int main(void) {
    tim1.ARR=8499;
    JerryFOC_setAngleSource(&source);
    JerryFOC_setPhaseVoltage(3,0,0);
    float spread12=(float)(tim1.CCR2-tim1.CCR3);
    bus=24;
    JerryFOC_setPhaseVoltage(3,0,0);
    float spread24=(float)(tim1.CCR2-tim1.CCR3);
    assert(fabsf(spread12-2*spread24)<=2.0f);
    assert(JerryFOC_getBusVoltage()==24 && JerryFOC_isDriveReady());
    /* Keep the motor's configured voltage ceiling even on a 24 V bus. */
    JerryFOC_setPhaseVoltage(100,0,0);
    float applied=(float)(tim1.CCR2-tim1.CCR3)/tim1.ARR*bus;
    assert(applied<=MOTOR_2804.bus_voltage*0.95f+0.01f);
    calibrated=false; JerryFOC_setPhaseVoltage(3,0,0); zero_pwm();
    JerryFOC_alignStart(); assert(aligns==0); assert(!JerryFOC_isDriveReady());
    calibrated=true;
    frame.m1_current[0]=0.5f; frame.m1_current[1]=-0.25f;
    adc1.JDR1=4095; adc2.JDR1=0; /* Old direct reads would give the wrong current. */
    JerryFOC_setCurrent(1); HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    assert(captures==1 && encoder_services==1 && hso_processes==1);
    assert(JerryFOC_getPhaseCurrent_A()==0.5f && JerryFOC_getPhaseCurrent_B()==-0.25f);
    complete=false; frame.m1_current[0]=1.5f;
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    assert(captures==2 && encoder_services==1 && JerryFOC_getPhaseCurrent_A()==0.5f);
    HAL_ADCEx_InjectedConvCpltCallback(&hadc2); assert(captures==2);
    complete=true; JerryFOC_alignStart();
    float align_ud=(float)(tim1.CCR1-tim1.CCR2)*bus/(1.5f*tim1.ARR);
    assert(fabsf(align_ud-MOTOR_2804.Rs*MOTOR_2804.rated_current)<0.01f);
    unsigned duty=tim1.CCR1;
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    assert(captures==3 && encoder_services==2 && tim1.CCR1==duty); /* Align voltage is not overwritten. */
    ready=false; JerryFOC_alignFinish(); zero_pwm(); assert(aligns==0);
    ready=true; JerryFOC_alignStart(); JerryFOC_alignFinish(); zero_pwm(); assert(aligns==1);
    calibrated=false; JerryFOC_run(); zero_pwm(); assert(JerryFOC_Target_Iq==0);
    calibrated=true; healthy=0; JerryFOC_setCurrent(3); JerryFOC_run(); zero_pwm(); assert(JerryFOC_Target_Iq==0);
    healthy=1; aligned=0; JerryFOC_setCurrent(3); JerryFOC_run(); zero_pwm(); assert(JerryFOC_Target_Iq==0);
    aligned=1; ready=false;
    JerryFOC_setMode(JERRYFOC_MODE_VELOCITY); JerryFOC_setVelocity(100);
    TIM_HandleTypeDef outer={TIM2}; HAL_TIM_PeriodElapsedCallback(&outer);
    assert(JerryFOC_Target_Iq==0 && vel_loop_M0.integral_prev==0);
    ready=true;
    /* UI and protocol are not the only callers: enforce the same continuous
     * target in setter, outer speed PI and the fast current-loop boundary. */
    JerryFOC_useCurrentLoop(1);
    assert(vel_loop_M0.limit==0.5f);
    JerryFOC_setCurrent(10); assert(JerryFOC_Target_Iq==0.5f);
    JerryFOC_setCurrent(-10); assert(JerryFOC_Target_Iq==-0.5f);
    JerryFOC_setCurrent(NAN); assert(JerryFOC_Target_Iq==0);
    JerryFOC_setVelocity(100);
    for(unsigned i=0;i<1000;++i) HAL_TIM_PeriodElapsedCallback(&outer);
    assert(fabsf(JerryFOC_Target_Iq)<=0.5f);
    JerryFOC_Target_Iq=10; JerryFOC_run(); assert(JerryFOC_Target_Iq==0.5f);
    JerryFOC_Target_Iq=-10; JerryFOC_run(); assert(JerryFOC_Target_Iq==-0.5f);
    JerryFOC_useCurrentLoop(0);
    JerryFOC_setCurrent(3); assert(JerryFOC_Target_Iq==3); /* Voltage command units stay V. */
    assert(!JerryFOC_useHSO(1)); /* No observer lock. */
    hso_usable=true;
    JerryFOC_setMode(JERRYFOC_MODE_POSITION);
    assert(!JerryFOC_useHSO(1)); /* Absolute position stays on encoder. */
    JerryFOC_setMode(JERRYFOC_MODE_TORQUE);
    encoder_velocity=100;
    M0_Vel_Flt.y_prev=filtered_velocity_global=100;
    command("Speed:100");
    hso_entry_ok=false;
    assert(!JerryFOC_useHSO(1) && !JerryFOC_hasHSOFault());
    hso_entry_ok=true;
    assert(JerryFOC_useHSO(1) && JerryFOC_isHSOActive());
    /* No speed step on entry: speed uses the same 10 ms blend as angle. */
    M0_Vel_Flt.y_prev=100;
    HAL_TIM_PeriodElapsedCallback(&outer);
    assert(fabsf(filtered_velocity_global-100)<1e-6f);
    for(unsigned i=0;i<50;++i) JerryFOC_run();
    M0_Vel_Flt.y_prev=100; HAL_TIM_PeriodElapsedCallback(&outer);
    assert(fabsf(filtered_velocity_global-(100+5.0f/6.0f))<0.001f);
    for(unsigned i=0;i<100;++i) JerryFOC_run();
    assert(fabsf(JerryFOC_getElectricalAngle()-0.1f)<1e-5f);
    M0_Vel_Flt.y_prev=100; HAL_TIM_PeriodElapsedCallback(&outer);
    assert(fabsf(filtered_velocity_global-(100+10.0f/6.0f))<0.001f);
    /* Both small and large above-boundary changes remain on HSO. */
    command("Speed:95"); JerryFOC_run();
    assert(JerryFOC_isHSOActive() && !JerryFOC_isHSOReturning());
    command("Speed:100"); command("Speed:20");
    assert(JerryFOC_isHSOActive() && !JerryFOC_isHSOReturning() && JerryFOC_Target_Velocity==20);
    for(unsigned i=0;i<10;++i) HAL_TIM_PeriodElapsedCallback(&outer);
    float before_deceleration=telemetry_reference;
    for(unsigned i=0;i<10;++i) HAL_TIM_PeriodElapsedCallback(&outer);
    assert(fabsf(before_deceleration-telemetry_reference-0.5f)<0.001f); /* 50 rad/s^2 */
    for(unsigned i=0;i<110;++i) JerryFOC_run();
    assert(JerryFOC_isHSOActive() && !JerryFOC_isHSOReturning());
    assert(JerryFOC_Target_Velocity==20 && JerryFOC_isDriveReady());
    assert(strcmp(JerryFOC_getHSOFaultText(),"--")==0);
    hso_usable=true; command("Speed:100"); command("HSO:Use");
    assert(JerryFOC_isHSOActive());
    /* A loss already pending when the command arrives must be processed
     * BEFORE applying that fresh command, never by the following ADC frame. */
    hso_usable=false; reject_reason=ST_HSO_REASON_ANGLE;
    command("Speed:20"); JerryFOC_run();
    assert(!JerryFOC_isHSOActive() && JerryFOC_Target_Velocity==20);
    assert(JerryFOC_getMode()==JERRYFOC_MODE_VELOCITY && JerryFOC_isDriveReady());
    assert(strcmp(JerryFOC_getHSOFaultText(),"ANGLE")==0);
    hso_usable=true; command("Speed:100"); command("HSO:Use");
    for(unsigned i=0;i<110;++i) JerryFOC_run();
    /* Speed:0 must decelerate under HSO until ACTUAL speed reaches 45 rpm,
     * not return merely because the target is now below that boundary. */
    command("Speed:0"); JerryFOC_run();
    assert(JerryFOC_isHSOActive() && !JerryFOC_isHSOReturning());
    for(unsigned speed=100;speed>=5;--speed) {
        encoder_velocity=hso_velocity=(float)speed;
        JerryFOC_run();
        assert(JerryFOC_isHSOActive() && !JerryFOC_isHSOReturning());
    }
    const float boundary=45.0f*(2.0f*PI)/60.0f;
    encoder_velocity=boundary; hso_velocity=6; JerryFOC_run(); /* Observer LP may lag. */
    assert(JerryFOC_isHSOReturning() && JerryFOC_Target_Velocity==0);
    assert(strcmp(JerryFOC_getFeedbackText(),"TO ENCODER")==0);
    float previous_angle=JerryFOC_getElectricalAngle();
    hso_usable=false; reject_reason=ST_HSO_REASON_SPEED;
    hso_velocity=NAN; /* Return must not consume invalid observer outputs. */
    for(unsigned i=0;i<110;++i) {
        JerryFOC_run();
        float a=JerryFOC_getElectricalAngle();
        assert(a<=previous_angle+0.00001f); previous_angle=a;
    }
    assert(!JerryFOC_isHSOActive() && !JerryFOC_hasHSOFault() && JerryFOC_isDriveReady());
    assert(JerryFOC_getMode()==JERRYFOC_MODE_VELOCITY && JerryFOC_Target_Velocity==0);
    assert(strcmp(JerryFOC_getFeedbackText(),"ENCODER")==0);
    assert(strcmp(JerryFOC_getHSOFaultText(),"--")==0);
    command("Speed:100"); HAL_TIM_PeriodElapsedCallback(&outer); JerryFOC_run();
    assert(JerryFOC_Target_Velocity==100 && !JerryFOC_isHSOActive());
    hso_usable=true; encoder_velocity=100; hso_velocity=110;
    command("HSO:Use"); assert(Protocol_GetHsoResult()==1 && JerryFOC_isHSOActive());
    for(unsigned i=0;i<110;++i) JerryFOC_run();
    /* Natural deceleration in torque mode also exits before the 5 Hz floor. */
    JerryFOC_setMode(JERRYFOC_MODE_TORQUE); encoder_velocity=hso_velocity=7;
    JerryFOC_run(); assert(JerryFOC_isHSOActive() && !JerryFOC_isHSOReturning());
    encoder_velocity=hso_velocity=4.6f;
    JerryFOC_run(); assert(JerryFOC_isHSOReturning());
    for(unsigned i=0;i<110;++i) JerryFOC_run();
    assert(!JerryFOC_isHSOActive() && !JerryFOC_hasHSOFault());
    encoder_velocity=hso_velocity=boundary; command("Speed:100");
    command("HSO:Use"); assert(Protocol_GetHsoResult()==-3);
    encoder_velocity=hso_velocity=4.8f; command("HSO:Use"); assert(JerryFOC_isHSOActive());
    command("HSO:Encoder");
    encoder_velocity=100; hso_velocity=110; command("Speed:100"); command("HSO:Use");
    assert(JerryFOC_isHSOActive());
    command("Speed:-100"); JerryFOC_run();
    assert(JerryFOC_isHSOActive() && !JerryFOC_isHSOReturning());
    encoder_velocity=hso_velocity=5; JerryFOC_run(); assert(!JerryFOC_isHSOReturning());
    encoder_velocity=hso_velocity=4.6f; JerryFOC_run(); assert(JerryFOC_isHSOReturning());
    for(unsigned i=0;i<110;++i) JerryFOC_run();
    assert(!JerryFOC_isHSOActive() && JerryFOC_Target_Velocity==-100);
    /* Same mechanical boundary for reverse rotation. */
    encoder_velocity=hso_velocity=-100; command("HSO:Use"); assert(JerryFOC_isHSOActive());
    command("Speed:0"); JerryFOC_run(); assert(!JerryFOC_isHSOReturning());
    encoder_velocity=hso_velocity=-5; JerryFOC_run(); assert(!JerryFOC_isHSOReturning());
    encoder_velocity=hso_velocity=-4.6f; JerryFOC_run(); assert(JerryFOC_isHSOReturning());
    for(unsigned i=0;i<110;++i) JerryFOC_run();
    assert(!JerryFOC_isHSOActive() && JerryFOC_Target_Velocity==0);
    encoder_velocity=100; hso_velocity=110;
    command("Speed:100"); command("HSO:Use"); assert(JerryFOC_isHSOActive());
    reject_reason=ST_HSO_REASON_ANGLE;
    hso_usable=false; JerryFOC_run();
    assert(!JerryFOC_hasHSOFault() && JerryFOC_isDriveReady() && !JerryFOC_isHSOActive()); zero_pwm();
    assert(JerryFOC_Target_Iq==0 && JerryFOC_Target_Velocity==0);
    assert(strcmp(JerryFOC_getHSOFaultText(),"ANGLE")==0);
    reject_reason=ST_HSO_REASON_TIMING;
    hso_usable=true;
    for(unsigned i=0;i<10;++i) { JerryFOC_run(); HAL_TIM_PeriodElapsedCallback(&outer); }
    zero_pwm(); /* Observer recovery cannot restart the motor. */
    assert(strcmp(JerryFOC_getHSOFaultText(),"ANGLE")==0);
    assert(!JerryFOC_useHSO(1));
    command("Speed:nan"); command("Speed:inf"); command("Speed:");
    command("Speed:100junk"); JerryFOC_run(); zero_pwm();
    assert(JerryFOC_Target_Velocity==0);
    command("Speed:100"); HAL_TIM_PeriodElapsedCallback(&outer); JerryFOC_run();
    assert(JerryFOC_Target_Velocity==100 && JerryFOC_isDriveReady());
    command("HSO:Use"); assert(JerryFOC_isHSOActive());
    assert(strcmp(JerryFOC_getHSOFaultText(),"--")==0);
    timing_loss_after_control=true;
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1); zero_pwm();
    assert(!JerryFOC_isHSOActive() && JerryFOC_Target_Velocity==0);
    assert(strcmp(JerryFOC_getHSOFaultText(),"TIMING")==0);
    timing_loss_after_control=false; hso_usable=true;
    command("Speed:100"); command("HSO:Use"); assert(JerryFOC_isHSOActive());
    /* Unhealthy encoder / real sampling faults can never auto-return to drive. */
    healthy=0; JerryFOC_run(); zero_pwm();
    assert(JerryFOC_hasHSOFault() && !JerryFOC_isDriveReady());
    command("Speed:100"); healthy=1; JerryFOC_run(); zero_pwm();
    assert(JerryFOC_hasHSOFault()); /* Recovery alone does not resume stored target. */
    command("Speed:100"); assert(JerryFOC_isDriveReady());
    command("HSO:Use"); ready=false; JerryFOC_run(); zero_pwm();
    command("Speed:100"); assert(!JerryFOC_isDriveReady());
    JerryFOC_run(); zero_pwm(); ready=true;
    JerryFOC_run(); zero_pwm(); assert(JerryFOC_hasHSOFault());
    command("Speed:100"); assert(JerryFOC_isDriveReady());
    assert(JerryFOC_useHSO(0) && !JerryFOC_isHSOActive() && !JerryFOC_hasHSOFault());
    assert(JerryFOC_Target_Iq==0 && JerryFOC_Target_Velocity==0);
    command("Speed:100"); assert(JerryFOC_useHSO(1));
    JerryFOC_setMode(JERRYFOC_MODE_POSITION);
    assert(!JerryFOC_isHSOActive());
    puts("PASS: real protocol+FOC Speed:0/restart/reverse, low-speed HSO return, safe loss stop/fresh-command recovery, hardware fault gates, timing stop, 12/24V scaling, current clamps");
    return 0;
}
