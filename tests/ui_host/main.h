#ifndef UI_HOST_MAIN_H
#define UI_HOST_MAIN_H
#include <stdint.h>
typedef struct { uint32_t pins; } GPIO_TypeDef;
typedef struct { uint32_t Pin, Mode, Pull; } GPIO_InitTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
extern GPIO_TypeDef host_gpio;
#define GPIO_MODE_INPUT 0
#define GPIO_PULLUP 1
#define GPIO_PULLDOWN 2
#define KEY_Up_Pin (1U << 8)
#define KEY_Down_Pin (1U << 10)
#define KEY_OK_Pin (1U << 12)
#define KEY_Left_Pin (1U << 11)
#define KEY_Right_Pin (1U << 9)
#define KEY2_Pin (1U << 14)
#define KEY_Up_GPIO_Port (&host_gpio)
#define KEY_Down_GPIO_Port (&host_gpio)
#define KEY_OK_GPIO_Port (&host_gpio)
#define KEY_Left_GPIO_Port (&host_gpio)
#define KEY_Right_GPIO_Port (&host_gpio)
#define KEY2_GPIO_Port (&host_gpio)
uint32_t HAL_GetTick(void);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; }
#endif
