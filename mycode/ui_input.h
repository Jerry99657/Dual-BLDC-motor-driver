#ifndef __UI_INPUT_H
#define __UI_INPUT_H

#include <stdbool.h>
#include <stdint.h>

void ui_input_init(void);

/* 六个独立原始键位，由 KK_UI 统一去抖和连发。KEY2 对应 BACK。 */
uint8_t ui_input_poll(void);

#endif /* __UI_INPUT_H */
