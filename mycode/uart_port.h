#ifndef UART_PORT_H
#define UART_PORT_H
#include <stdbool.h>
#include <stdint.h>

/* USART1 only: HAL owns fixed 115200/8N1 setup; this module owns runtime.
 * Call Init once after MX_USART1_UART_Init, before protocol service. */
void Uart1_Init(void);
void Uart1_IRQHandler(void);
/* Main-loop producer. Copies the frame; false leaves an active TX intact.
 * Busy includes the final stop bit, not just an empty data register. */
bool Uart1_TrySend(const uint8_t *data,uint16_t length);
#endif
