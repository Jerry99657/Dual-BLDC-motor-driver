#ifndef HOST_USART_H
#define HOST_USART_H
#include "main.h"
typedef struct { unsigned gState; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart1;
#define HAL_UART_STATE_READY 0U
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
#endif
