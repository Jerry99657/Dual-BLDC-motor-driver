#ifndef MOTOR_IRQ_H
#define MOTOR_IRQ_H

/* Fixed board configuration: HAL owns initialization/start/stop and DMA;
 * these LL dispatchers own only the EXTI, TIM2, TIM1 BRK and ADC12 vectors.
 * Called from CubeMX USER CODE blocks before its generated HAL fallback. */
void MotorIRQ_Key1(void);
void MotorIRQ_Tim2(void);
void MotorIRQ_Break(void);
void MotorIRQ_Adc12(void);

#endif
