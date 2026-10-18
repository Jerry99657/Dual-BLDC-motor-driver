#ifndef __MT6701_H_
#define __MT6701_H_

#include "main.h"
#include "spi.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define CS_Enable  HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port, MT6701_CSN_Pin, GPIO_PIN_RESET)
#define CS_Disable HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port, MT6701_CSN_Pin, GPIO_PIN_SET)

extern volatile int Angle_Raw;
extern volatile float Angle;
extern uint8_t MT6701_Data[3];

/* StartContinuous 仅使能定频采样；实际 DMA 请求由 10 kHz ADC 回调中的
 * MT6701_Service() 发起，不在 DMA 完成回调里无限自触发。 */
void MT6701_StartContinuous(void);
void MT6701_StopContinuous(void);
void MT6701_Service(void);
uint8_t MT6701_IsContinuous(void);
uint8_t MT6701_IsHealthy(void);
uint16_t MT6701_GetRawAngle(void);
uint8_t MT6701_GetStatus(void);
uint32_t MT6701_GetSampleCounter(void);
uint32_t MT6701_GetErrorCounter(void);
uint16_t MT6701_GetSampleAgeCycles(void);

#endif /* __MT6701_H_ */
