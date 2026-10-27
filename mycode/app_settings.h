#ifndef APP_SETTINGS_H
#define APP_SETTINGS_H
#include "app_storage.h"
#include "motor_control.h"

/* Main-loop settings business. No LCD, KK_UI, editor bindings or heap. */
enum {
    APP_SET_LED_BRIGHTNESS,APP_SET_POSITION_SPEED,APP_SET_SPEED_LIMIT,
    APP_SET_GEAR_RATIO,APP_SET_GEAR_OFFSET,APP_SET_IBUS_LIMIT,
    APP_SET_CURRENT_LOOP,APP_SET_LED_ENABLE,APP_SET_SPEED_STEP,
    APP_SET_TORQUE_STEP,APP_SET_POSITION_STEP,APP_ACTION_DEFAULTS,
    APP_ACTION_CLEAR_IBUS,APP_ACTION_DETECT_START,APP_ACTION_PAUSE,
    APP_ACTION_DETECT_SAVE,APP_ACTION_SAVE,APP_ACTION_DIAG_CLEAR
};
typedef struct {
    int32_t targets[3]; /* Speed x10, Uq x100, position x100; never an Iq target. */
    int32_t steps[3];   /* Speed, torque, position, in their native Settings units. */
    int32_t position_speed,speed_limit;
    bool current_loop;  /* Saved encoder preference; HSO's effective ON is separate. */
} AppAxisSettings;

bool AppSettings_ReadAxis(MotorAxis axis,AppAxisSettings *out);
bool AppSettings_StoreAxis(MotorAxis axis,const AppAxisSettings *value);
void AppSettings_StoreTargets(MotorAxis axis,const int32_t targets[3],unsigned throttle);
bool AppSettings_ValidRequest(unsigned axis,unsigned key);
int8_t AppSettings_Apply(unsigned axis,unsigned key,int32_t value);
bool AppSettings_SaveDetection(void);
bool AppSettings_SelectMotor(MotorAxis axis,MotorProfile profile);
void AppSettings_Changed(uint32_t now);
void AppSettings_Input(uint32_t now);
void AppSettings_ResetSave(uint32_t now);
/* settled describes foreground editing/activity, not a storage-bus promise.
 * Accepted changes are already published; saving never commits UI drafts. */
bool AppSettings_SaveService(uint32_t now,bool settled);
unsigned App_SaveState(void);
#endif
