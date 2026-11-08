#ifndef ADC_PORT_H
#define ADC_PORT_H
#include "adc.h"

/* External-triggered regular scans, circular halfword DMA. CubeMX owns setup. */
HAL_StatusTypeDef AdcPort_Start(ADC_HandleTypeDef *adc, volatile uint16_t *buffer, uint32_t count);
HAL_StatusTypeDef AdcPort_Stop(ADC_HandleTypeDef *adc);
void AdcPort_DMAIRQ(DMA_HandleTypeDef *dma);
#endif
