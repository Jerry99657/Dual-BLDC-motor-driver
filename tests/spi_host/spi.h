#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "spi_constants.h"
typedef struct { volatile uint32_t CR1,CR2,SR,DR,CRCPR,RXCRCR,TXCRCR,I2SCFGR,I2SPR; } SPI_TypeDef;
typedef struct { volatile uint32_t ISR,IFCR; } DMA_TypeDef;
typedef struct { volatile uint32_t CCR,CNDTR,CPAR,CMAR; } DMA_Channel_TypeDef;
typedef struct { DMA_Channel_TypeDef *Instance; DMA_TypeDef *DmaBaseAddress; uint32_t ChannelIndex; void *Parent; } DMA_HandleTypeDef;
typedef struct { SPI_TypeDef *Instance; volatile HAL_SPI_StateTypeDef State; volatile uint32_t ErrorCode;
                 DMA_HandleTypeDef *hdmarx,*hdmatx; } SPI_HandleTypeDef;
extern SPI_HandleTypeDef ports[3];
#define hspi1 ports[0]
#define hspi2 ports[1]
#define hspi3 ports[2]
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *d);

typedef enum { HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT } HAL_StatusTypeDef;
#define DMA1_BASE 0x40020000UL
#define DMA2_BASE 0x40020400UL
#define DMA1_Channel1_BASE (DMA1_BASE+0x08U)
#define DMA1_Channel2_BASE (DMA1_BASE+0x1CU)
#define DMA1_Channel3_BASE (DMA1_BASE+0x30U)
#define DMA1_Channel4_BASE (DMA1_BASE+0x44U)
#define DMA1_Channel5_BASE (DMA1_BASE+0x58U)
#define DMA1_Channel6_BASE (DMA1_BASE+0x6CU)
#define DMA1 ((DMA_TypeDef *)DMA1_BASE)
#define DMA2 ((DMA_TypeDef *)DMA2_BASE)
#define SPI1 ((SPI_TypeDef *)0x40013000UL)
#define SPI2 ((SPI_TypeDef *)0x40003800UL)
#define SPI3 ((SPI_TypeDef *)0x40003C00UL)
#define __IO volatile
#define __STATIC_INLINE static inline
uint32_t HostRead(volatile const uint32_t *reg);
void HostWrite(volatile uint32_t *reg,uint32_t value);
void HostReceived(SPI_TypeDef *spi);
void HostTransmitted(SPI_TypeDef *spi,uint8_t value);
void HostClearOverrun(const SPI_TypeDef *spi);
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
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h);
void HostResetSPI3(void);
#define __HAL_RCC_SPI3_FORCE_RESET() HostResetSPI3()
#define __HAL_RCC_SPI3_RELEASE_RESET() ((void)0)
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *h);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *h);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *h);
