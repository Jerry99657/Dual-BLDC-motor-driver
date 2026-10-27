#ifndef OLED_HOST_SPI_H
#define OLED_HOST_SPI_H
#include <stdint.h>
typedef struct { void *Instance; } SPI_HandleTypeDef;
typedef enum { HAL_OK,HAL_ERROR } HAL_StatusTypeDef;
#define SPI3 ((void *)3)
extern SPI_HandleTypeDef hspi3;
HAL_StatusTypeDef SpiPort_Write(SPI_HandleTypeDef *spi,const uint8_t *data,uint16_t length,uint32_t timeout);
HAL_StatusTypeDef SpiPort_WriteDMA(SPI_HandleTypeDef *spi,const uint8_t *data,uint16_t length);
HAL_StatusTypeDef SpiPort_Abort(SPI_HandleTypeDef *spi);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *spi);
#endif
