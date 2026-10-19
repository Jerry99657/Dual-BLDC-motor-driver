#ifndef MOTOR_HOST_MAIN_H
#define MOTOR_HOST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t pins; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
extern GPIO_TypeDef gpio_a, gpio_b;
#define MT6701_CSN_GPIO_Port (&gpio_a)
#define MT6701_CSN_Pin (1U << 4)
#define MT6701_CSN2_GPIO_Port (&gpio_b)
#define MT6701_CSN2_Pin (1U << 12)
#define W25_NSS_GPIO_Port (&gpio_b)
#define W25_NSS_Pin (1U << 15)
#define SET_BIT(reg, bits) ((reg) |= (bits))
#define CLEAR_BIT(reg, bits) ((reg) &= ~(bits))
#define MODIFY_REG(reg, mask, value) ((reg) = ((reg) & ~(mask)) | (value))
extern uint32_t host_primask;
typedef struct { uint32_t CYCCNT; } HostDwt;
extern HostDwt host_dwt;
#define DWT (&host_dwt)
#define __HAL_RCC_CORDIC_CLK_ENABLE() ((void)0)
static inline uint32_t __get_PRIMASK(void) { return host_primask; }
static inline void __disable_irq(void) { host_primask = 1; }
static inline void __enable_irq(void) { host_primask = 0; }
static inline void __set_PRIMASK(uint32_t mask) { host_primask = mask; }
uint32_t HAL_GetTick(void);
static inline uint32_t HAL_GetREVID(void) { return 0x2002U; }
void HAL_Delay(uint32_t ms);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t);
#endif
