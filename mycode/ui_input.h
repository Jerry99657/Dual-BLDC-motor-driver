#ifndef __UI_INPUT_H
#define __UI_INPUT_H

#include <stdbool.h>
#include <stdint.h>
#include "kk_ui.h"

void ui_input_init(void);
/* Sole physical-key reader and recognizer: existing TIM2 1kHz vector. */
void ui_input_tick(void);
/* Main-only: cancel old events on page/axis/mode changes; stop paths cancel too. */
bool ui_input_context(uint16_t context);
typedef enum { UI_INPUT_ACCEPT, UI_INPUT_CANCEL, UI_INPUT_DISPLAY_ERROR } UiInputResult;
void ui_input_complete(UiInputResult result);
/* Main-only: peek normalized input, without reading physical GPIO. KK_UI
 * acknowledges it after display checks; failed rendering cannot replay it. */
KK_UI_Input ui_input_poll(void);

/* Latest timer sample; KEY1 remains outside the UI input path. */
uint8_t ui_input_raw_keys(void);

#endif /* __UI_INPUT_H */
