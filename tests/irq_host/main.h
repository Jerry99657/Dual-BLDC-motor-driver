#ifndef IRQ_HOST_MAIN_H
#define IRQ_HOST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef struct { uint32_t CYCCNT; } HostDwt;
extern HostDwt host_dwt;
#define DWT (&host_dwt)
#define TIM_CR1_DIR 0x10U
typedef struct { uint32_t PR1; } EXTI_TypeDef;
extern EXTI_TypeDef irq_exti;
#define EXTI (&irq_exti)
#define KEY1_Pin (1U<<13)
#define READ_REG(reg) (reg)
#define READ_BIT(reg,bits) ((reg)&(bits))
#define CLEAR_BIT(reg,bits) ((reg)&=~(bits))
#define __STATIC_INLINE static inline
void HostRegisterWrite(uint32_t *reg,uint32_t value);
#define WRITE_REG(reg,value) HostRegisterWrite(&(reg),(value))
#endif
