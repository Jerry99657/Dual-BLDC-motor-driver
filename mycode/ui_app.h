#ifndef __UI_APP_H
#define __UI_APP_H

#include <stdbool.h>
#include <stdint.h>

#include "kk_ui.h"

enum {
    UI_PAGE_HOME = 1,
    UI_PAGE_RUN,
    UI_PAGE_SETTINGS,
    UI_PAGE_STATUS,
    UI_PAGE_ABOUT,
    UI_PAGE_RUN_STEPS,
    UI_PAGE_MOTOR,
    UI_PAGE_SYSTEM,
    UI_PAGE_WORK,
    UI_PAGE_DRIVE,
    UI_PAGE_MOTION,
    UI_PAGE_ENCODER,
    UI_PAGE_ADC,
    UI_PAGE_HSO,
    UI_PAGE_DIAGNOSTICS,
};

const KK_UI_App *ui_app_get(void);
void ui_app_reset(uint32_t now_ms);
void ui_app_update(uint32_t now_ms);
void ui_app_process_events(void);
void ui_app_note_display_error(void);

#endif /* __UI_APP_H */
