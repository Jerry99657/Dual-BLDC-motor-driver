#ifndef MOTOR_HOST_SPI_H
#define MOTOR_HOST_SPI_H
#include "main.h"
typedef struct { uint32_t CR1, CR2, SR; } SPI_TypeDef;
typedef struct { uint32_t CLKPhase,BaudRatePrescaler,NSSPMode; } SPI_InitTypeDef;
typedef struct { SPI_TypeDef *Instance; uint32_t State; SPI_InitTypeDef Init; } SPI_HandleTypeDef;
extern SPI_TypeDef spi1, spi2, spi3;
extern SPI_HandleTypeDef hspi1, hspi2, hspi3;
#define SPI1 (&spi1)
#define SPI2 (&spi2)
#define SPI3 (&spi3)
#define HAL_SPI_STATE_READY 1U
#define SPI_CR1_CPHA 1U
#define SPI_CR1_CPOL 2U
#define SPI_CR1_BR (7U << 3)
#define SPI_CR1_SPE (1U << 6)
#define SPI_SR_BSY (1U << 7)
#define SPI_SR_FRLVL (3U << 9)
#define SPI_SR_FTLVL (3U << 11)
#define SPI_SR_OVR (1U << 6)
#define SPI_SR_MODF (1U << 5)
#define SPI_SR_FRE (1U << 8)
#define SPI_MODE_MASTER ((1U << 2) | (1U << 8))
#define SPI_NSS_SOFT (1U << 9)
#define SPI_DATASIZE_8BIT (7U << 8)
#define SPI_RXFIFO_THRESHOLD_QF (1U << 12)
#define SPI_NSS_PULSE_DISABLE 0U
#define SPI_PHASE_1EDGE 0U
#define SPI_PHASE_2EDGE SPI_CR1_CPHA
#define SPI_BAUDRATEPRESCALER_8 (2U << 3)
#define SPI_BAUDRATEPRESCALER_256 SPI_CR1_BR
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *, const uint8_t *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, const uint8_t *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *, const uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef *);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *);
uint32_t HAL_SPI_GetError(const SPI_HandleTypeDef *);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *);
void Host_SPI1_ForceReset(void);
void Host_SPI1_ReleaseReset(void);
#define __HAL_RCC_SPI1_FORCE_RESET() Host_SPI1_ForceReset()
#define __HAL_RCC_SPI1_RELEASE_RESET() Host_SPI1_ReleaseReset()
void Host_SPI2_ForceReset(void);
void Host_SPI2_ReleaseReset(void);
#define __HAL_RCC_SPI2_FORCE_RESET() Host_SPI2_ForceReset()
#define __HAL_RCC_SPI2_RELEASE_RESET() Host_SPI2_ReleaseReset()
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *);
#endif
