#include "app_settings.h"
#include "board_diag.h"
#include "motor_sampling.h"
#include "motor_sixstep.h"
#include "motor_detect.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static bool dirty,force_save;
static uint32_t changed_at,input_at;
static int32_t limit(int32_t x,int32_t lo,int32_t hi) { return x<lo ? lo : x>hi ? hi : x; }

/* Byte copies keep the unchanged v5 storage/wire layout without aliasing
 * AppSettings as a different structure or walking unrelated members. */
#define TRIPLE(a,b,c) _Static_assert(offsetof(AppSettings,b)==offsetof(AppSettings,a)+4 && offsetof(AppSettings,c)==offsetof(AppSettings,a)+8,"Settings triple")
TRIPLE(speed_x10,voltage_x100,position_x100);
TRIPLE(axis2_speed_x10,axis2_voltage_x100,axis2_position_x100);
TRIPLE(speed_step_x10,torque_step_x100,position_step_x100);
TRIPLE(axis2_speed_step_x10,axis2_torque_step_x100,axis2_position_step_x100);
#undef TRIPLE
_Static_assert(sizeof(AppSettings)<256,"Compact settings offsets");
static char *axis_data(MotorAxis axis,bool steps)
{
    static const uint8_t offsets[2][2]={
        {offsetof(AppSettings,speed_x10),offsetof(AppSettings,speed_step_x10)},
        {offsetof(AppSettings,axis2_speed_x10),offsetof(AppSettings,axis2_speed_step_x10)}};
    return (char *)AppStorage_Get()+offsets[axis][steps];
}
bool AppSettings_ReadAxis(MotorAxis axis,AppAxisSettings *out)
{
    if((unsigned)axis>=MOTOR_AXIS_COUNT || !out) return false;
    uint32_t flags=MotorControl_GetOptions(axis);
    const MotorParams *motor=MotorParams_GetAxis(axis);
    out->speed_limit=(int32_t)((flags & APP_SPEED_LIMIT_MASK)>>APP_SPEED_LIMIT_SHIFT);
    if(out->speed_limit<1 || out->speed_limit>APP_SPEED_LIMIT_MAX) out->speed_limit=APP_SPEED_LIMIT_DEFAULT;
    out->speed_limit=limit(out->speed_limit,1,(int32_t)motor->max_speed_mech);
    out->position_speed=(int32_t)AppSettings_GetPositionSpeed(flags);
    if(out->position_speed<1 || out->position_speed>APP_SPEED_LIMIT_MAX) out->position_speed=30;
    out->position_speed=limit(out->position_speed,1,out->speed_limit);
    out->current_loop=(flags & APP_FLAG_CURRENT_LOOP)!=0;
    memcpy(out->targets,axis_data(axis,false),sizeof(out->targets));
    memcpy(out->steps,axis_data(axis,true),sizeof(out->steps));
    out->targets[0]=limit(out->targets[0],-out->speed_limit*10,out->speed_limit*10);
    int32_t voltage=(int32_t)roundf(motor->vel_limit*100.0f);
    out->targets[1]=limit(out->targets[1],-voltage,voltage);
    out->targets[2]=limit(out->targets[2],-314,314);
    for(unsigned i=0;i<3;++i) if(out->steps[i]<1 || out->steps[i]>100) out->steps[i]=5;
    return true;
}
bool AppSettings_StoreAxis(MotorAxis axis,const AppAxisSettings *value)
{
    if((unsigned)axis>=MOTOR_AXIS_COUNT || !value || (uint32_t)value->speed_limit-1U>=APP_SPEED_LIMIT_MAX ||
        value->speed_limit>(int32_t)MotorParams_GetAxis(axis)->max_speed_mech ||
        (uint32_t)value->position_speed-1U>=(uint32_t)value->speed_limit) return false;
    for(unsigned i=0;i<3;++i) if((uint32_t)value->steps[i]-1U>=100U) return false;
    AppSettings *st=AppStorage_Get();
    uint32_t flags=(value->current_loop ? APP_FLAG_CURRENT_LOOP : 0U) |
        (st->flags & APP_FLAG_LED_ENABLE) | AppSettings_EncodePositionSpeed((uint32_t)value->position_speed) |
        ((uint32_t)value->speed_limit<<APP_SPEED_LIMIT_SHIFT) |
        (MotorParams_GetAxisProfile(axis)==MOTOR_PROFILE_4015 ? APP_FLAG_MOTOR_4015 : 0U);
    if(!MotorControl_SetOptions(axis,flags)) return false;
    if(axis==MOTOR_AXIS_1) st->flags=flags; else st->axis2_flags=flags;
    int32_t targets[]={limit(value->targets[0],-value->speed_limit*10,value->speed_limit*10),value->targets[1],value->targets[2]};
    memcpy(axis_data(axis,false),targets,sizeof(targets));
    memcpy(axis_data(axis,true),value->steps,sizeof(value->steps));
    return true;
}
void AppSettings_StoreTargets(MotorAxis axis,const int32_t targets[3],unsigned throttle)
{
    if((unsigned)axis>=MOTOR_AXIS_COUNT || !targets) return;
    memcpy(axis_data(axis,false),targets,12);
    if(axis==MOTOR_AXIS_1) {
        AppSettings *st=AppStorage_Get();
        st->control_config=(st->control_config & ~APP_CONFIG_THROTTLE_MASK) |
            ((uint32_t)(throttle>100U ? 100U : throttle)<<APP_CONFIG_THROTTLE_SHIFT);
    }
}
void AppSettings_Changed(uint32_t now) { dirty=true;changed_at=now; }
void AppSettings_Input(uint32_t now) { input_at=now; }
void AppSettings_ResetSave(uint32_t now) { dirty=force_save=false;changed_at=input_at=now; }
unsigned App_SaveState(void)
{
    unsigned state=AppStorage_GetSaveState();
    if(dirty && AppStorage_IsReady() && state!=APP_SAVE_WRITING)
        state=AppStorage_IsDeferred() ? APP_SAVE_DEFERRED : APP_SAVE_PENDING;
    return state;
}
bool AppSettings_SaveService(uint32_t now,bool settled)
{
    if(!dirty || (!force_save && (!settled || (int32_t)(now-changed_at)<2000 || (int32_t)(now-input_at)<2000))) return false;
    if(AppStorage_IsReady() && !AppStorage_RequestSave()) return false;
    dirty=force_save=false;return true;
}
bool AppSettings_SaveDetection(void)
{
    if(MotorDetect_Active() || !MotorDetect_Result()) return false;
    AppSettings *st=AppStorage_Get();st->motor_detect=MotorDetect_Result();
    st->motor_detect_l_uh=MotorDetect_Inductance();st->motor_detect_flux_uwb=MotorDetect_Flux();
    AppSettings_Changed(HAL_GetTick());return true;
}
bool AppSettings_SelectMotor(MotorAxis axis,MotorProfile profile)
{
    if((unsigned)axis>=MOTOR_AXIS_COUNT || (unsigned)profile>MOTOR_PROFILE_COUNT) return false;
    if(profile==MotorParams_GetAxisProfile(axis) && !(axis==MOTOR_AXIS_1 && SixStep_IsGeneric())) return true;
    MotorAxisSnapshot state;MotorControl_GetAxis(axis,&state);
    if(state.aligning) return false;
    bool sensorless=MotorControl_GetDrive(axis)!=MOTOR_DRIVE_ENCODER;
    if((!sensorless || MotorSampling_IsAxisOutputRequested(axis)) && MotorControl_GetInput(axis)<=MOTOR_INPUT_LOCAL)
        MotorControl_Stop(axis);
    bool ok=profile==MOTOR_PROFILE_COUNT ? axis==MOTOR_AXIS_1 && MotorControl_SelectGeneric() : MotorControl_SelectProfile(axis,profile);
    if(!ok) return false;
    /* SelectProfile/SelectGeneric already clear confirmed targets and publish
     * the axis defaults while both outputs are certified stopped. */
    AppSettings_Changed(HAL_GetTick());return true;
}
static bool defaults(void)
{
    MotorControl_StopAll();
    if(!MotorControl_SelectProfile(MOTOR_AXIS_1,MOTOR_PROFILE_2804) ||
        !MotorControl_SelectProfile(MOTOR_AXIS_2,MOTOR_PROFILE_2804) ||
        !MotorControl_SelectWork(MOTOR_WORK_SINGLE_M1) || !MotorControl_SelectFeedback(false)) return false;
    AppStorage_ResetDefaults();(void)MotorSampling_SetIbusLimit(APP_IBUS_DEFAULT_MA);
    (void)MotorControl_SetGear(100,0);MotorControl_SetFocus(MOTOR_AXIS_1);
    AppSettings *st=AppStorage_Get();
    (void)MotorControl_SetOptions(MOTOR_AXIS_1,st->flags);
    (void)MotorControl_SetOptions(MOTOR_AXIS_2,st->axis2_flags);
    return true;
}
bool AppSettings_ValidRequest(unsigned axis,unsigned key)
{
    if(axis>2 || key>APP_ACTION_DIAG_CLEAR) return false;
    const unsigned axis_keys=(1U<<1)|(1U<<2)|(1U<<6)|(1U<<8)|(1U<<9)|(1U<<10);
    return (((axis_keys>>key)&1U)!=0)==(axis!=0);
}
int8_t AppSettings_Apply(unsigned axis,unsigned key,int32_t value)
{
    if(!AppSettings_ValidRequest(axis,key)) return -2;
    /* Action payloads are zero, except the explicit detection test current. */
    if(key>=APP_ACTION_DEFAULTS && key!=APP_ACTION_DETECT_START && value!=0) return -1;
    AppSettings *st=AppStorage_Get();bool ok=false;
    if(axis) {
        AppAxisSettings data;MotorAxis a=(MotorAxis)(axis-1U);AppSettings_ReadAxis(a,&data);
        if(key==APP_SET_POSITION_SPEED) data.position_speed=value;
        else if(key==APP_SET_SPEED_LIMIT) {
            data.speed_limit=value;
            if(data.position_speed>value) data.position_speed=value;
        } else if(key==APP_SET_CURRENT_LOOP) {
            if((uint32_t)value>1U || MotorControl_GetDrive(a)!=MOTOR_DRIVE_ENCODER) return -1;
            data.current_loop=value!=0;
        } else data.steps[key-APP_SET_SPEED_STEP]=value; /* ValidRequest certified a step key. */
        ok=AppSettings_StoreAxis(a,&data);
    } else switch(key) {
    case APP_SET_LED_BRIGHTNESS: if(value>=0 && value<=100) { st->led_brightness=(uint32_t)value;ok=true; } break;
    case APP_SET_LED_ENABLE: if(value==0 || value==1) { st->flags=(st->flags & ~APP_FLAG_LED_ENABLE)|(value ? APP_FLAG_LED_ENABLE : 0U);ok=true; } break;
    case APP_SET_GEAR_RATIO: ok=MotorControl_SetGear(value,st->gear_offset_x100);break;
    case APP_SET_GEAR_OFFSET: ok=MotorControl_SetGear(st->gear_ratio_x100,value);break;
    case APP_SET_IBUS_LIMIT:
        if(value>=(int32_t)APP_IBUS_MIN_MA && value<=(int32_t)APP_IBUS_MAX_MA && value%100==0 && MotorSampling_SetIbusLimit((uint16_t)value)) {
            st->control_config=(st->control_config & ~APP_CONFIG_IBUS_MASK)|((uint32_t)(value/100)<<APP_CONFIG_IBUS_SHIFT);ok=true;
        } break;
    case APP_ACTION_DEFAULTS: ok=defaults();break;
    case APP_ACTION_CLEAR_IBUS:
        MotorControl_Stop(MOTOR_AXIS_1);ok=MotorSampling_ClearIbusTrip();
        if(ok && MotorControl_HsoAllowed() && JerryFOC_hasHSOFault()) (void)JerryFOC_useHSO(0);
        return ok ? 1 : -1;
    case APP_ACTION_DETECT_START: return value>=100 && value<=500 && MotorDetect_Start((unsigned)value) ? 1 : -1;
    case APP_ACTION_PAUSE:
        MotorSampling_GlobalStopIRQ();MotorControl_StopAll();return 1;
    case APP_ACTION_DETECT_SAVE: return AppSettings_SaveDetection() ? 1 : -1;
    case APP_ACTION_SAVE: force_save=true;ok=true;break;
    case APP_ACTION_DIAG_CLEAR: return BoardDiag_Clear() ? 1 : -1;
    default:break;
    }
    if(ok) AppSettings_Changed(HAL_GetTick());
    return ok ? 1 : -1;
}
