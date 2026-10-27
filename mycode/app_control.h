#ifndef APP_CONTROL_H
#define APP_CONTROL_H
#include "motor_control.h"
/* Main-loop application bridge. Axis: 0 global, 1 M1, 2 M2. Values are
 * integers in the native Settings units, never raw storage offsets. */
enum {
    APP_SET_LED_BRIGHTNESS,APP_SET_POSITION_SPEED,APP_SET_SPEED_LIMIT,
    APP_SET_GEAR_RATIO,APP_SET_GEAR_OFFSET,APP_SET_IBUS_LIMIT,
    APP_SET_CURRENT_LOOP,APP_SET_LED_ENABLE,APP_SET_SPEED_STEP,
    APP_SET_TORQUE_STEP,APP_SET_POSITION_STEP,APP_ACTION_DEFAULTS,
    APP_ACTION_CLEAR_IBUS,APP_ACTION_DETECT_START,APP_ACTION_PAUSE,
    APP_ACTION_DETECT_SAVE,APP_ACTION_SAVE,APP_ACTION_DIAG_CLEAR
};
int8_t App_Control(unsigned axis,unsigned key,int32_t value);
bool App_CanConfigure(void);
bool App_SelectMotor(MotorAxis axis,MotorProfile profile);
void App_ConfigChanged(void);
void App_SaveService(uint32_t now);
unsigned App_SaveState(void);
void App_RemoteMotion(MotorAxis axis,bool detect);
void App_CancelNavigation(void); /* Stop ISR-safe: only clears a byte. */
uint8_t App_NavigationKeys(uint32_t now);
/* Main-loop only. Caller supplies label[24], value[32]; returns page count or
 * negative error. Live topic formatting never changes View, editors or settings. */
int App_ReadRow(unsigned axis,unsigned group,unsigned row,char *label,char *value);
#endif
