#include "../ui_host/main.h"
#define TFT3_CS_GPIO_Port (&host_gpio)
#define TFT3_DC_GPIO_Port (&host_gpio)
#define TFT3_RESET_GPIO_Port (&host_gpio)
#define TFT3_CS_Pin (1U << 0)
#define TFT3_DC_Pin (1U << 1)
#define TFT3_RESET_Pin (1U << 2)
void HAL_GPIO_WritePin(GPIO_TypeDef *port,uint16_t pin,GPIO_PinState state);
void HAL_Delay(uint32_t delay);
