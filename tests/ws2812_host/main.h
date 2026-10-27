#pragma once
#include <stdint.h>
#include <stdbool.h>
#define __IO volatile
#define __STATIC_INLINE static inline
#include "ws_constants.h"
#include "ws_registers.h"
typedef struct { uint32_t MemDataAlignment; } DMA_InitTypeDef;
typedef struct { DMA_Channel_TypeDef *Instance; DMA_TypeDef *DmaBaseAddress;
                 uint32_t ChannelIndex; void *Parent; DMA_InitTypeDef Init; } DMA_HandleTypeDef;
typedef struct { TIM_TypeDef *Instance; DMA_HandleTypeDef *hdma[1]; } TIM_HandleTypeDef;
#define TIM3 ((TIM_TypeDef *)0x40000400UL)
#define DMA1 ((DMA_TypeDef *)0x40020000UL)
#define DMA2 ((DMA_TypeDef *)0x40020400UL)
#define DMA1_BASE 0x40020000UL
#define DMA1_Channel1_BASE (DMA1_BASE+8U)
#define DMA1_Channel2_BASE (DMA1_BASE+28U)
#define DMA1_Channel3_BASE (DMA1_BASE+48U)
#define DMA1_Channel4_BASE (DMA1_BASE+68U)
#define DMA1_Channel5_BASE (DMA1_BASE+88U)
#define DMA1_Channel6_BASE (DMA1_BASE+108U)
uint32_t HostRead(volatile const uint32_t *reg);
void HostWrite(volatile uint32_t *reg,uint32_t value);
#define READ_REG(reg) HostRead(&(reg))
#define READ_BIT(reg,bits) (HostRead(&(reg))&(bits))
#define WRITE_REG(reg,value) HostWrite(&(reg),(value))
#define SET_BIT(reg,bits) HostWrite(&(reg),HostRead(&(reg))|(bits))
#define CLEAR_BIT(reg,bits) HostWrite(&(reg),HostRead(&(reg))&~(bits))
#define MODIFY_REG(reg,clear,set) HostWrite(&(reg),(HostRead(&(reg))&~(clear))|(set))
extern uint32_t host_primask;
static inline uint32_t __get_PRIMASK(void) { return host_primask; }
static inline void __disable_irq(void) { host_primask=1U; }
void __set_PRIMASK(uint32_t mask);
uint32_t HAL_GetTick(void);
