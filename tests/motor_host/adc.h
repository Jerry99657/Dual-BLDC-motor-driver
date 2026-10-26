#ifndef MOTOR_HOST_ADC_H
#define MOTOR_HOST_ADC_H
#include "main.h"
typedef struct { uint32_t JDR1, JDR2, JDR3, JDR4, ISR, IER; } ADC_TypeDef;
typedef struct { ADC_TypeDef *Instance; uint32_t ErrorCode,State; } ADC_HandleTypeDef;
extern ADC_TypeDef adc1, adc2;
extern ADC_HandleTypeDef hadc1, hadc2;
#define ADC1 (&adc1)
#define ADC2 (&adc2)
extern uint16_t host_vrefint_cal;
#define VREFINT_CAL_ADDR (&host_vrefint_cal)
#define VREFINT_CAL_VREF 3000UL
#define ADC_SINGLE_ENDED 0U
#define ADC_FLAG_JEOC (1U << 5)
#define ADC_FLAG_JEOS (1U << 6)
#define __HAL_ADC_GET_FLAG(h, flag) (((h)->Instance->ISR & (flag)) != 0U)
#define __HAL_ADC_CLEAR_FLAG(h, flags) ((h)->Instance->ISR &= ~(flags))
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *, uint32_t *, uint32_t);
HAL_StatusTypeDef HAL_ADCEx_InjectedStart(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADCEx_InjectedStop(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *);
#endif
