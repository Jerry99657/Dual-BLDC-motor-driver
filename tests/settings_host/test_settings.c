/* Links only the real settings service and motor presets; no KK_UI, OLED,
 * UI app, protocol, compiler mocks of settings logic, or presentation hooks. */
#include "app_settings.h"
#include "motor_sampling.h"
#include "motor_detect.h"
#include "motor_sixstep.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

uint32_t host_primask;
static uint32_t now,options[2];
static AppSettings settings,snapshot;
static AppSaveState state;
static bool storage_ready=true,deferred,request_ok=true,options_ok=true,gear_ok=true,ibus_ok=true;
static bool detect_active,detect_ok=true,pause,diag_ok=true,profile_ok=true;
static bool requested[2],aligning[2],hso,six,generic;
static unsigned saves,publications,ibus_calls,stops,detect_current,clears;
static uint32_t detect_result;
static MotorWorkMode work;
static MotorAxis focus;
static MotorInput owner[2];
static uint16_t ibus=APP_IBUS_DEFAULT_MA;
uint32_t HAL_GetTick(void) { return now; }
AppSettings *AppStorage_Get(void) { return &settings; }
bool AppStorage_IsReady(void) { return storage_ready; }
bool AppStorage_IsDeferred(void) { return deferred; }
AppSaveState AppStorage_GetSaveState(void) { return state; }
bool AppStorage_RequestSave(void) {
    if(!request_ok || state==APP_SAVE_WRITING) return false;
    snapshot=settings;++saves;state=APP_SAVE_WRITING;return true;
}
void AppStorage_ResetDefaults(void) {
    memset(&settings,0,sizeof(settings));settings.led_brightness=100;
    settings.gear_ratio_x100=100;settings.flags=APP_FLAG_LED_ENABLE;
}
uint32_t MotorControl_GetOptions(MotorAxis axis) { return options[axis]; }
void MotorControl_GetAxis(MotorAxis axis,MotorAxisSnapshot *out) { *out=(MotorAxisSnapshot){.aligning=aligning[axis]}; }
bool MotorControl_SetOptions(MotorAxis axis,uint32_t flags) {
    if(!options_ok) return false;
    options[axis]=flags;++publications;return true;
}
MotorInput MotorControl_GetInput(MotorAxis axis) { return owner[axis]; }
bool MotorControl_HsoAllowed(void) { return hso; }
bool MotorControl_SixStepAllowed(void) { return six; }
MotorDriveMode MotorControl_GetDrive(MotorAxis axis) { return axis==MOTOR_AXIS_2 ? MOTOR_DRIVE_ENCODER : hso ? MOTOR_DRIVE_HSO : six ? MOTOR_DRIVE_SIXSTEP : MOTOR_DRIVE_ENCODER; }
bool MotorControl_IsCoupled(void) { return work>=MOTOR_WORK_SYNC_SPEED; }
void MotorControl_Stop(MotorAxis axis) { requested[axis]=false;owner[axis]=MOTOR_INPUT_NONE;++stops; }
void MotorControl_StopAll(void) { MotorControl_Stop(MOTOR_AXIS_1);MotorControl_Stop(MOTOR_AXIS_2);detect_active=false; }
bool MotorControl_SelectWork(MotorWorkMode value) { work=value;return true; }
bool MotorControl_SelectFeedback(bool value) { hso=value;six=false;return true; }
void MotorControl_SetFocus(MotorAxis axis) { focus=axis; }
static void clear_profile_settings(MotorAxis axis) {
    options[axis]=(settings.flags & APP_FLAG_LED_ENABLE)|
        (MotorParams_GetAxisProfile(axis)==MOTOR_PROFILE_4015 ? APP_FLAG_MOTOR_4015 : 0U)|
        (100U<<APP_SPEED_LIMIT_SHIFT)|AppSettings_EncodePositionSpeed(30);
    if(!axis) {
        settings.flags=options[axis];settings.speed_x10=settings.voltage_x100=settings.position_x100=0;
        settings.control_config&=~APP_CONFIG_THROTTLE_MASK;
    } else { settings.axis2_flags=options[axis];settings.axis2_speed_x10=settings.axis2_voltage_x100=settings.axis2_position_x100=0; }
}
bool MotorControl_SelectProfile(MotorAxis axis,MotorProfile profile) {
    if(!profile_ok) return false;
    bool ok=MotorParams_SelectAxis(axis,profile);
    if(!axis) generic=false;
    clear_profile_settings(axis);
    return ok;
}
bool MotorControl_SelectGeneric(void) { generic=six;if(generic) clear_profile_settings(MOTOR_AXIS_1);return generic; }
bool SixStep_IsGeneric(void) { return generic; }
bool MotorControl_SetGear(int32_t ratio,int32_t offset) {
    if(!gear_ok || !ratio || ratio< -1000 || ratio>1000 || offset< -6283 || offset>6283) return false;
    settings.gear_ratio_x100=ratio;settings.gear_offset_x100=offset;return true;
}
bool MotorSampling_IsAxisOutputRequested(MotorAxis axis) { return requested[axis]; }
bool MotorSampling_SetIbusLimit(uint16_t value) { ++ibus_calls;if(!ibus_ok)return false;ibus=value;return true; }
bool MotorSampling_ClearIbusTrip(void) { return ibus_ok; }
void MotorSampling_GlobalStopIRQ(void) { pause=true;requested[0]=requested[1]=false; }
uint8_t JerryFOC_hasHSOFault(void) { return 0; }
uint8_t JerryFOC_useHSO(uint8_t value) { (void)value;return 1; }
bool MotorDetect_Active(void) { return detect_active; }
bool MotorDetect_Start(unsigned value) { detect_current=value;detect_active=detect_ok;return detect_ok; }
uint32_t MotorDetect_Result(void) { return detect_result; }
uint32_t MotorDetect_Inductance(void) { return 440; }
uint32_t MotorDetect_Flux(void) { return 9290; }
bool BoardDiag_Clear(void) { if(diag_ok) ++clears;return diag_ok; }
static void reset(void) {
    now=100;AppStorage_ResetDefaults();memset(options,0,sizeof(options));
    MotorParams_SelectAxis(MOTOR_AXIS_1,MOTOR_PROFILE_2804);MotorParams_SelectAxis(MOTOR_AXIS_2,MOTOR_PROFILE_2804);
    state=APP_SAVE_IDLE;storage_ready=request_ok=options_ok=gear_ok=ibus_ok=profile_ok=diag_ok=detect_ok=true;
    deferred=detect_active=pause=hso=six=generic=false;detect_result=0;
    saves=publications=ibus_calls=stops=clears=0;work=MOTOR_WORK_SINGLE_M1;focus=MOTOR_AXIS_1;
    memset(owner,0,sizeof(owner));memset(requested,0,sizeof(requested));memset(aligning,0,sizeof(aligning));AppSettings_ResetSave(now);
}
static void equal(const AppSettings *a,const AppSettings *b) { assert(!memcmp(a,b,sizeof(*a))); }
static void rejected_boundaries(void) {
    const unsigned keys[]={APP_SET_POSITION_SPEED,APP_SET_SPEED_LIMIT,APP_SET_SPEED_STEP,
        APP_SET_TORQUE_STEP,APP_SET_POSITION_STEP,APP_SET_CURRENT_LOOP};
    const int32_t bad[]={INT32_MIN,-1,1000,INT32_MAX};
    for(unsigned a=1;a<=2;++a) for(unsigned k=0;k<sizeof(keys)/sizeof(keys[0]);++k)
        for(unsigned b=0;b<sizeof(bad)/sizeof(bad[0]);++b) {
            reset();AppSettings before=settings;
            assert(AppSettings_Apply(a,keys[k],bad[b])==-1);
            equal(&before,&settings);assert(!publications && App_SaveState()==APP_SAVE_IDLE);
        }
    for(unsigned key=APP_ACTION_DEFAULTS;key<=APP_ACTION_DIAG_CLEAR;++key) {
        if(key==APP_ACTION_DETECT_START) continue;
        reset();AppSettings before=settings;
        assert(AppSettings_Apply(0,key,1)==-1);equal(&before,&settings);
        assert(!stops && !clears && !saves && !pause && App_SaveState()==APP_SAVE_IDLE);
    }
    reset();AppAxisSettings v;assert(AppSettings_ReadAxis(MOTOR_AXIS_1,&v));
    v.targets[0]=INT32_MIN;assert(AppSettings_StoreAxis(MOTOR_AXIS_1,&v) && settings.speed_x10==-1000);
    v.targets[0]=INT32_MAX;assert(AppSettings_StoreAxis(MOTOR_AXIS_1,&v) && settings.speed_x10==1000);
}
int main(void) {
    rejected_boundaries();
    reset();AppAxisSettings value;AppSettings before=settings;
    assert(AppSettings_ReadAxis(MOTOR_AXIS_1,&value));
    assert(value.speed_limit==100 && value.position_speed==30 && value.steps[0]==5);
    equal(&before,&settings);assert(App_SaveState()==APP_SAVE_IDLE);
    assert(!AppSettings_ReadAxis(MOTOR_AXIS_COUNT,&value) && !AppSettings_ReadAxis(MOTOR_AXIS_1,NULL));
    assert(!AppSettings_StoreAxis(MOTOR_AXIS_COUNT,&value) && !AppSettings_StoreAxis(MOTOR_AXIS_1,NULL));
    AppSettings_StoreTargets(MOTOR_AXIS_COUNT,value.targets,0);equal(&before,&settings);
    assert(AppSettings_Apply(0,APP_SET_SPEED_LIMIT,100)==-2);
    assert(AppSettings_Apply(2,APP_SET_LED_ENABLE,1)==-2 && AppSettings_Apply(3,0,1)==-2 && AppSettings_Apply(0,18,0)==-2);
    for(unsigned a=1;a<=2;++a) {
        assert(AppSettings_Apply(a,APP_SET_SPEED_LIMIT,600)==1);
        assert(AppSettings_Apply(a,APP_SET_POSITION_SPEED,600)==1);
        assert(AppSettings_Apply(a,APP_SET_SPEED_STEP,17)==1 && AppSettings_Apply(a,APP_SET_TORQUE_STEP,23)==1);
        assert(AppSettings_Apply(a,APP_SET_POSITION_STEP,37)==1);
        assert(AppSettings_ReadAxis((MotorAxis)(a-1),&value));
        value.targets[0]=5000;assert(AppSettings_StoreAxis((MotorAxis)(a-1),&value));
        assert(AppSettings_Apply(a,APP_SET_SPEED_LIMIT,120)==1);
        assert(AppSettings_ReadAxis((MotorAxis)(a-1),&value));
        assert(value.position_speed==120 && value.targets[0]==1200 && value.steps[0]==17 && value.steps[1]==23 && value.steps[2]==37);
        before=settings;options_ok=false;
        assert(AppSettings_Apply(a,APP_SET_SPEED_LIMIT,200)==-1);equal(&before,&settings);options_ok=true;
        assert(AppSettings_Apply(a,APP_SET_POSITION_SPEED,121)==-1 && AppSettings_Apply(a,APP_SET_SPEED_LIMIT,601)==-1);
        assert(AppSettings_Apply(a,APP_SET_SPEED_STEP,0)==-1 && AppSettings_Apply(a,APP_SET_CURRENT_LOOP,2)==-1);
        equal(&before,&settings);
    }
    before=settings;hso=true;assert(AppSettings_Apply(1,APP_SET_CURRENT_LOOP,1)==-1);equal(&before,&settings);
    assert(AppSettings_Apply(2,APP_SET_CURRENT_LOOP,1)==1);hso=false;
    six=true;assert(AppSettings_Apply(1,APP_SET_CURRENT_LOOP,0)==-1);six=false;
    assert(AppSettings_Apply(0,APP_SET_GEAR_RATIO,-150)==1 && AppSettings_Apply(0,APP_SET_GEAR_OFFSET,23)==1);
    before=settings;assert(AppSettings_Apply(0,APP_SET_GEAR_RATIO,0)==-1);equal(&before,&settings);
    assert(AppSettings_Apply(0,APP_SET_LED_BRIGHTNESS,61)==1 && AppSettings_Apply(0,APP_SET_LED_ENABLE,0)==1);
    assert(settings.led_brightness==61 && !(settings.flags&APP_FLAG_LED_ENABLE));
    for(int ma=-100;ma<=4100;ma+=50) {
        before=settings;unsigned calls=ibus_calls;int result=AppSettings_Apply(0,APP_SET_IBUS_LIMIT,ma);
        bool valid=ma>=500 && ma<=4000 && ma%100==0;
        assert(result==(valid?1:-1));assert(ibus_calls==calls+(unsigned)valid);
        if(valid) assert(ibus==ma && AppSettings_GetIbusLimit(settings.control_config)==ma);
        else equal(&before,&settings);
    }
    before=settings;ibus_ok=false;assert(AppSettings_Apply(0,APP_SET_IBUS_LIMIT,2000)==-1);equal(&before,&settings);ibus_ok=true;
    reset();assert(AppSettings_Apply(0,APP_SET_LED_BRIGHTNESS,42)==1 && App_SaveState()==APP_SAVE_PENDING);
    now+=1999;assert(!AppSettings_SaveService(now,true));++now;
    assert(!AppSettings_SaveService(now,false));AppSettings_Input(now);
    now+=1999;assert(!AppSettings_SaveService(now,true));++now;request_ok=false;
    assert(!AppSettings_SaveService(now,true) && App_SaveState()==APP_SAVE_PENDING);
    request_ok=true;assert(AppSettings_SaveService(now,true) && saves==1 && snapshot.led_brightness==42);
    assert(AppSettings_Apply(0,APP_SET_LED_BRIGHTNESS,64)==1 && AppSettings_Apply(0,APP_ACTION_SAVE,0)==1);
    assert(!AppSettings_SaveService(now,false) && saves==1 && snapshot.led_brightness==42);
    state=APP_SAVE_SAVED;assert(AppSettings_SaveService(now,false) && saves==2 && snapshot.led_brightness==64);
    state=APP_SAVE_SAVED;AppSettings_Changed(now);deferred=true;assert(App_SaveState()==APP_SAVE_DEFERRED);
    deferred=false;now+=2000;assert(AppSettings_SaveService(now,true));
    reset();now=UINT32_MAX-1000U;AppSettings_ResetSave(now);
    assert(AppSettings_Apply(0,APP_SET_LED_ENABLE,1)==1);
    assert(!AppSettings_SaveService(now-1U,true));now+=2000U;
    assert(AppSettings_SaveService(now,true) && saves==1);
    reset();assert(AppSettings_Apply(0,APP_ACTION_DETECT_START,250)==1 && detect_active && detect_current==250);
    detect_result=11U|(1920U<<8);assert(!AppSettings_SaveDetection());detect_active=false;
    assert(AppSettings_Apply(0,APP_ACTION_DETECT_SAVE,0)==1);
    assert(settings.motor_detect==detect_result && settings.motor_detect_l_uh==440 && settings.motor_detect_flux_uwb==9290);
    assert(AppSettings_Apply(0,APP_ACTION_DIAG_CLEAR,0)==1 && clears==1);diag_ok=false;
    assert(AppSettings_Apply(0,APP_ACTION_DIAG_CLEAR,0)==-1 && clears==1);
    requested[0]=requested[1]=true;assert(AppSettings_Apply(0,APP_ACTION_PAUSE,0)==1 && pause && !requested[0] && !requested[1]);
    reset();settings.speed_x10=300;settings.axis2_speed_x10=400;settings.speed_step_x10=17;
    owner[0]=MOTOR_INPUT_USB;requested[0]=true;profile_ok=false;
    assert(!AppSettings_SelectMotor(MOTOR_AXIS_1,MOTOR_PROFILE_4015) && requested[0] && stops==0);
    owner[0]=MOTOR_INPUT_NONE;requested[0]=false;profile_ok=true;
    aligning[0]=true;assert(!AppSettings_SelectMotor(MOTOR_AXIS_1,MOTOR_PROFILE_4015) && stops==0);
    aligning[0]=false;
    assert(AppSettings_SelectMotor(MOTOR_AXIS_1,MOTOR_PROFILE_4015));
    assert(settings.speed_x10==0 && settings.voltage_x100==0 && settings.position_x100==0 && settings.axis2_speed_x10==400);
    assert(settings.speed_step_x10==17 && MotorParams_GetAxisProfile(MOTOR_AXIS_1)==MOTOR_PROFILE_4015);
    six=true;assert(AppSettings_SelectMotor(MOTOR_AXIS_1,MOTOR_PROFILE_COUNT) && generic);
    assert(!AppSettings_SelectMotor(MOTOR_AXIS_2,MOTOR_PROFILE_COUNT));
    requested[0]=requested[1]=true;assert(AppSettings_Apply(0,APP_ACTION_DEFAULTS,0)==1);
    assert(!requested[0] && !requested[1] && work==MOTOR_WORK_SINGLE_M1 && focus==MOTOR_AXIS_1);
    assert(settings.led_brightness==100 && !settings.motor_detect && App_SaveState()==APP_SAVE_PENDING);
    puts("PASS: real settings service without UI/LCD: both axes, limits/steps, atomic rejected publication, HSO/six-step preferences, IBUS range/DAC rejection, deferred/forced/in-flight save, stale/wrapped time, defaults/profile/results and pause/clear gates");
}
