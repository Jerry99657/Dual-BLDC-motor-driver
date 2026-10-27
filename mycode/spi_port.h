#ifndef SPI_PORT_H
#define SPI_PORT_H
#include "spi.h"
#include <stdbool.h>
#include <stdint.h>

/* Board-specific 8-bit master runtime. HAL owns setup/reinitialization only.
 * Application owns CS and SPI2 arbitration. No HAL runtime API may share a
 * migrated port. State/ErrorCode are runtime status, not HAL DMA ownership.
 * Completion/error use the existing application HAL_SPI_*Callback hooks.
 * Service finishes the last wire bit without waiting inside an interrupt. */
HAL_StatusTypeDef SpiPort_ExchangeDMA(SPI_HandleTypeDef *h,const uint8_t *tx,uint8_t *rx,uint16_t length);
HAL_StatusTypeDef SpiPort_WriteDMA(SPI_HandleTypeDef *h,const uint8_t *tx,uint16_t length);
HAL_StatusTypeDef SpiPort_Exchange(SPI_HandleTypeDef *h,const uint8_t *tx,uint8_t *rx,uint16_t length,uint32_t timeout);
HAL_StatusTypeDef SpiPort_Write(SPI_HandleTypeDef *h,const uint8_t *tx,uint16_t length,uint32_t timeout);
HAL_StatusTypeDef SpiPort_Read(SPI_HandleTypeDef *h,uint8_t *rx,uint16_t length,uint32_t timeout);
HAL_StatusTypeDef SpiPort_Abort(SPI_HandleTypeDef *h);
HAL_StatusTypeDef SpiPort_DMAStop(SPI_HandleTypeDef *h);
uint32_t SpiPort_GetError(const SPI_HandleTypeDef *h);
void SpiPort_Service(SPI_HandleTypeDef *h);
void SpiPort_IRQHandler(SPI_HandleTypeDef *h);
/* Generated DMA vectors pass their own handle, including HAL-owned DMA. */
void SpiPort_DMAIRQ(DMA_HandleTypeDef *dma);
#endif
