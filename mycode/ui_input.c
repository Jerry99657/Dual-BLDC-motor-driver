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

static bool axis_raw,axis_stable,axis_pending;
static uint32_t axis_changed_at;

bool ui_input_take_axis_change(void)
{
    bool result=axis_pending; axis_pending=false; return result;
}

void ui_input_init(void)
{
#ifdef KEY1_Pin
    GPIO_InitTypeDef axis_pin={0};
    axis_pin.Pin=KEY1_Pin; axis_pin.Mode=GPIO_MODE_INPUT; axis_pin.Pull=GPIO_PULLUP;
    HAL_GPIO_Init(KEY1_GPIO_Port,&axis_pin);
    axis_raw=axis_stable=HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==GPIO_PIN_RESET;
#endif
    axis_pending=false; axis_changed_at=HAL_GetTick();
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
#ifdef KEY1_Pin
    bool pressed=HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==GPIO_PIN_RESET;
    uint32_t now=HAL_GetTick();
    if (pressed!=axis_raw) { axis_raw=pressed; axis_changed_at=now; }
    if (axis_stable!=axis_raw && now-axis_changed_at>=20U) {
        axis_stable=axis_raw;
        if (axis_stable) axis_pending=true;
    }
#endif
    uint8_t keys = 0U;
    for (unsigned i = 0U; i < sizeof(s_pins) / sizeof(s_pins[0]); ++i) {
        bool high = HAL_GPIO_ReadPin(s_pins[i].port, s_pins[i].pin) == GPIO_PIN_SET;
        bool active_high = (UI_INPUT_ACTIVE_HIGH_MASK & s_pins[i].key) != 0U;
        if (high == active_high) keys |= s_pins[i].key;
    }
    return keys;
}

