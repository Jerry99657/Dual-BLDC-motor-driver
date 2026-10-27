#ifndef UART_HOST_MAIN_H
#define UART_HOST_MAIN_H
#include <stdint.h>
#include <stddef.h>
#include "uart_constants.h"
typedef struct { uint32_t CR1,CR3,ISR,ICR,RDR,TDR; } USART_TypeDef;
extern USART_TypeDef uart_regs;
#define USART1 (&uart_regs)
#define __STATIC_INLINE static inline
uint32_t HostRead(const uint32_t *reg);
void HostWrite(uint32_t *reg,uint32_t value);
#define READ_REG(reg) HostRead(&(reg))
#define READ_BIT(reg,bits) (HostRead(&(reg))&(bits))
#define WRITE_REG(reg,value) HostWrite(&(reg),(value))
#define ATOMIC_SET_BIT(reg,bits) ((reg)|=(bits))
#define ATOMIC_CLEAR_BIT(reg,bits) ((reg)&=~(bits))
extern uint32_t host_primask;
static inline uint32_t __get_PRIMASK(void) { return host_primask; }
static inline void __disable_irq(void) { host_primask=1; }
void __set_PRIMASK(uint32_t mask);
#endif
