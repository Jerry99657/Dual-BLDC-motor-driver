#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "adc_constants.h"
typedef enum {HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT} HAL_StatusTypeDef;
typedef enum {HAL_UNLOCKED,HAL_LOCKED} HAL_LockTypeDef;
typedef enum {HAL_DMA_STATE_RESET,HAL_DMA_STATE_READY,HAL_DMA_STATE_BUSY,HAL_DMA_STATE_TIMEOUT} HAL_DMA_StateTypeDef;
typedef struct {uint32_t ISR,IFCR;} DMA_TypeDef;
typedef struct {uint32_t CCR,CNDTR;uintptr_t CPAR,CMAR;} DMA_Channel_TypeDef;
typedef struct {DMA_Channel_TypeDef *Instance;DMA_TypeDef *DmaBaseAddress;uint32_t ChannelIndex;void *Parent;HAL_DMA_StateTypeDef State;HAL_LockTypeDef Lock;uint32_t ErrorCode;} DMA_HandleTypeDef;
typedef struct {uint32_t CR,CFGR,IER,ISR,DR;} ADC_TypeDef;
typedef struct {ADC_TypeDef *Instance;DMA_HandleTypeDef *DMA_Handle;uint32_t State,ErrorCode;} ADC_HandleTypeDef;
extern ADC_HandleTypeDef hadc1;
extern uint32_t host_primask;
static inline uint32_t __get_PRIMASK(void) {return host_primask;}
static inline void __disable_irq(void) {host_primask=1;}
static inline void __set_PRIMASK(uint32_t mask) {host_primask=mask;}
void HostWrite(void *address,size_t width,uintptr_t value);
#define READ_REG(reg) (reg)
#define WRITE_REG(reg,value) HostWrite(&(reg),sizeof(reg),(uintptr_t)(value))
#define SET_BIT(reg,bits) ((reg)|=(bits))
#define CLEAR_BIT(reg,bits) ((reg)&=~(bits))
#define ADC_FLAG_EOC ADC_ISR_EOC
#define ADC_FLAG_EOS ADC_ISR_EOS
#define ADC_FLAG_OVR ADC_ISR_OVR
#define __HAL_ADC_CLEAR_FLAG(adc,flags) ((adc)->Instance->ISR&=~(flags))
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *);
