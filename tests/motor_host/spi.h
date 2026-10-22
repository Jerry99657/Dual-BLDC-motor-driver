#ifndef MOTOR_HOST_SPI_H
#define MOTOR_HOST_SPI_H
#include "main.h"
typedef struct { uint32_t CR1, CR2, SR; } SPI_TypeDef;
typedef struct { SPI_TypeDef *Instance; uint32_t State; } SPI_HandleTypeDef;
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
#define SPI_PHASE_2EDGE SPI_CR1_CPHA
#define SPI_BAUDRATEPRESCALER_256 SPI_CR1_BR
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *, const uint8_t *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, const uint8_t *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *, const uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef *);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *);
void Host_SPI1_ForceReset(void);
void Host_SPI1_ReleaseReset(void);
#define __HAL_RCC_SPI1_FORCE_RESET() Host_SPI1_ForceReset()
#define __HAL_RCC_SPI1_RELEASE_RESET() Host_SPI1_ReleaseReset()
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *);
#endif
