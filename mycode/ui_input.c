#include "ui_input.h"
#include "kk_ui.h"
#include "main.h"

/* 上拉输入、按下接地；开机按住按键不会改变极性。
 * 高有效硬件可在构建中定义对应的 KK_UI_KEY_* 位。 */
#ifndef UI_INPUT_ACTIVE_HIGH_MASK
#define UI_INPUT_ACTIVE_HIGH_MASK 0U
#endif

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t key;
} InputPin;

static const InputPin s_pins[] = {
    {KEY_Up_GPIO_Port, KEY_Up_Pin, KK_UI_KEY_UP},
    {KEY_Down_GPIO_Port, KEY_Down_Pin, KK_UI_KEY_DOWN},
    {KEY_OK_GPIO_Port, KEY_OK_Pin, KK_UI_KEY_OK},
    {KEY_Left_GPIO_Port, KEY_Left_Pin, KK_UI_KEY_LEFT},
    {KEY_Right_GPIO_Port, KEY_Right_Pin, KK_UI_KEY_RIGHT},
    {KEY2_GPIO_Port, KEY2_Pin, KK_UI_KEY_BACK},
};

static uint8_t s_raw_keys;

uint8_t ui_input_raw_keys(void) { return s_raw_keys; }

void ui_input_init(void)
{
#ifdef KEY1_Pin
    GPIO_InitTypeDef reserved_pin={0};
    reserved_pin.Pin=KEY1_Pin; reserved_pin.Mode=GPIO_MODE_INPUT; reserved_pin.Pull=GPIO_PULLUP;
    HAL_GPIO_Init(KEY1_GPIO_Port,&reserved_pin);
#endif
    s_raw_keys=0U;
    for (unsigned i = 0U; i < sizeof(s_pins) / sizeof(s_pins[0]); ++i) {
        GPIO_InitTypeDef init = {0};
        init.Pin = s_pins[i].pin;
        init.Mode = GPIO_MODE_INPUT;
        init.Pull = (UI_INPUT_ACTIVE_HIGH_MASK & s_pins[i].key) ?
                    GPIO_PULLDOWN : GPIO_PULLUP;
        HAL_GPIO_Init(s_pins[i].port, &init);
    }
}

uint8_t ui_input_poll(void)
{
    uint8_t keys = 0U;
    for (unsigned i = 0U; i < sizeof(s_pins) / sizeof(s_pins[0]); ++i) {
        bool high = HAL_GPIO_ReadPin(s_pins[i].port, s_pins[i].pin) == GPIO_PIN_SET;
        bool active_high = (UI_INPUT_ACTIVE_HIGH_MASK & s_pins[i].key) != 0U;
        if (high == active_high) keys |= s_pins[i].key;
    }
    s_raw_keys=keys;
    return keys;
}

