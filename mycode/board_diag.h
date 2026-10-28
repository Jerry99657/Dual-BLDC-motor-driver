#ifndef BOARD_DIAG_H
#define BOARD_DIAG_H
#include <stdint.h>
#include <stdbool.h>

enum { DIAG_SAMPLE=1, DIAG_ENCODER, DIAG_HSO, DIAG_SIXSTEP, DIAG_CONTROL, DIAG_DETECT };
typedef struct {
    float target_speed,target_position,target_q,speed,position;
    uint32_t at;
    uint8_t mode;
} BoardDiagMotion;
typedef struct {
    uint32_t at,sequence,cycles;
    BoardDiagMotion motion[2];
    float bus,reference,current[2][3];
    uint32_t enc_errors[2];
    uint16_t raw[2][3],enc_age[2];
    uint8_t enc_status[2],enc_error[2],work,drive[2],model[2],axis,source;
    char reason[24];
} BoardDiagFault;
typedef struct {
    uint32_t kind,stage,pc,lr,sp,xpsr,exc_return,cfsr,hfsr,bfar,mmfar;
} BoardDiagCrash;
typedef struct {
    uint32_t adc_max,outer_max,adc_jitter,outer_jitter,center_age,overruns;
} BoardDiagTiming;

void BoardDiag_Init(void);
bool BoardDiag_Clear(void); /* Both bridges must be stopped; never clears drive faults. */
void BoardDiag_Motion(unsigned axis,unsigned mode,float target_speed,float target_position,
                      float target_q,float speed,float position);
/* Call after disconnecting dangerous output, before controller targets reset.
 * First event remains frozen across restart/reset until explicit Clear. */
void BoardDiag_Fault(unsigned axis,unsigned source,const char *reason);
void BoardDiag_Timing(unsigned outer,uint32_t entered,uint32_t center_age);
const BoardDiagFault *BoardDiag_LastFault(void);
const BoardDiagCrash *BoardDiag_LastCrash(void);
const BoardDiagTiming *BoardDiag_Times(void);
/* Extra rows appended to the existing Diagnostics info page / GetRow group6. */
#define BOARD_DIAG_ROWS 29U
const char *BoardDiag_Row(unsigned axis,unsigned row,char value[32]);

#if defined(__arm__) || defined(__thumb__)
/* Attributes on declarations also apply to CubeMX's generated definitions. */
void NMI_Handler(void) __attribute__((naked));
void HardFault_Handler(void) __attribute__((naked));
void MemManage_Handler(void) __attribute__((naked));
void BusFault_Handler(void) __attribute__((naked));
void UsageFault_Handler(void) __attribute__((naked));
#define BOARD_DIAG_VECTOR(kind) __asm volatile("movs r2, #" #kind "\n b BoardDiag_ExceptionEntry"); __builtin_unreachable()
#endif
#endif
