#ifndef CUBEMX_IRQ_H
#define CUBEMX_IRQ_H
/* Include ONLY in stm32g4xx_it.c USER CODE Includes. CubeMX regenerates each
 * vector's HAL call with its new handle. Redirect that generated call, never
 * pin a device to an old physical vector in a preserved USER CODE block. */
#include "spi_port.h"
#include "motor_irq.h"
#include "tim.h"
#include "ws2812.h"
static inline void CubeMX_DMAIRQ(DMA_HandleTypeDef *dma)
{
    if (dma==htim3.hdma[TIM_DMA_ID_CC1]) WS2812_DMAIRQ(dma);
    else SpiPort_DMAIRQ(dma); /* SPI LL, or the unchanged ADC HAL dispatcher. */
}
#define HAL_DMA_IRQHandler(handle) CubeMX_DMAIRQ(handle)
/* Self reference deliberately calls the real HAL function for other pins.
 * Constant generated KEY1_Pin calls fold to the application dispatcher. */
#define HAL_GPIO_EXTI_IRQHandler(pin) do { \
    if ((pin)==KEY1_Pin) MotorIRQ_Key1(); \
    else HAL_GPIO_EXTI_IRQHandler(pin); \
} while (0)
#endif
