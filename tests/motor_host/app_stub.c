/* Parser/control suites isolate the app bridge; UI suites exercise its real
 * implementation and storage/controller publication independently. */
#include "app_control.h"
#include "app_storage.h"
#include <string.h>
static bool dirty;
bool app_host_config_allowed=true;
unsigned app_host_axis,app_host_key;
int32_t app_host_value;
unsigned app_host_config_calls;
AppConfigAction app_host_config_action;
MotorAxis app_host_config_axis;
int32_t app_host_config_value,app_host_config_extra;
int8_t App_Control(unsigned axis,unsigned key,int32_t value)
{ app_host_axis=axis;app_host_key=key;app_host_value=value;return 1; }
bool App_CanConfigure(void) { return app_host_config_allowed; }
bool App_Configure(AppConfigAction action,MotorAxis axis,int32_t value,int32_t extra)
{
    ++app_host_config_calls;app_host_config_action=action;app_host_config_axis=axis;
    app_host_config_value=value;app_host_config_extra=extra;
    bool ok=false;
    switch(action) {
    case APP_CFG_WORK: ok=MotorControl_SelectWork((MotorWorkMode)value);break;
    case APP_CFG_MOTOR: ok=App_SelectMotor(axis,(MotorProfile)value);break;
    case APP_CFG_FEEDBACK: ok=MotorControl_SelectFeedback(value!=0);break;
    case APP_CFG_SIXSTEP: ok=MotorControl_SelectSixStep(value!=0);break;
    case APP_CFG_GEAR: ok=MotorControl_SetGear(value,extra);break;
    case APP_CFG_FOCUS: MotorControl_SetFocus(axis);ok=true;break;
    case APP_CFG_ALIGN: return MotorControl_RequestAlign(axis);
    }
    if(ok) dirty=true;
    return ok;
}
bool App_SelectMotor(MotorAxis axis,MotorProfile profile)
{ return profile==MOTOR_PROFILE_ESC ? MotorControl_SelectGeneric() : MotorControl_SelectProfile(axis,profile); }
void App_ConfigChanged(void) { dirty=true; }
void App_SaveService(uint32_t now) { (void)now; }
unsigned App_SaveState(void) { return dirty ? APP_SAVE_PENDING : AppStorage_GetSaveState(); }
void App_RemoteMotion(MotorAxis axis,bool detect) { (void)axis;(void)detect; }
void App_CancelNavigation(void) {}
uint8_t App_NavigationEvent(uint32_t now) { (void)now;return 0; }
int App_ReadRow(unsigned axis,unsigned group,unsigned row,char *label,char *value)
{ if(axis<1 || axis>2 || group>7 || row>1)return -2;strcpy(label,"Test");strcpy(value,"24.0 V");return 2; }
