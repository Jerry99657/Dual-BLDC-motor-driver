#ifndef IRQ_HOST_ADC_H
#define IRQ_HOST_ADC_H
#include "main.h"
#include "irq_constants.h"
typedef struct { uint32_t ISR,IER; } ADC_TypeDef;
typedef struct { ADC_TypeDef *Instance; uint32_t State,ErrorCode; } ADC_HandleTypeDef;
extern ADC_TypeDef irq_adc1,irq_adc2;
extern ADC_HandleTypeDef hadc1,hadc2;
#define ADC1 (&irq_adc1)
#define ADC2 (&irq_adc2)
#endif
