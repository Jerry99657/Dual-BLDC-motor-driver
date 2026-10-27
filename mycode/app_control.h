#ifndef APP_CONTROL_H
#define APP_CONTROL_H
#include "app_settings.h"
/* Main-loop application bridge. Axis: 0 global, 1 M1, 2 M2. Values are
 * integers in the native Settings units, never raw storage offsets. */
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
