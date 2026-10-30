#include "board_diag.h"
#include "compact_text.h"
#include "main.h"
#include "tim.h"
#include "motor_sampling.h"
#include "motor_control.h"
#include "MT6701.h"
#include "ui_text.h"
#include "st_hso.h"
#include "motor_sixstep.h"
#include <string.h>
#include <stddef.h>

#define DIAG_MAGIC 0x44313631UL /* R116 record layout; reject stale/torn RAM. */
static struct {
    uint32_t magic,check;
    BoardDiagCrash crash;
    BoardDiagFault fault;
} retained __attribute__((section(".noinit"),used,aligned(8)));
static BoardDiagMotion motion[2];
static BoardDiagTiming timing;
static uint32_t previous[2];
static uint8_t seen;
static uint32_t outer_completed;
/* 0: stopped, 1: first completion pending, 2: healthy, 3/4: consecutive
 * misses. Keeping warm-up and streak in one word makes reset atomic. */
static uint32_t outer_watch;

static uint32_t checksum(void)
{
    const uint8_t *p=(const uint8_t *)&retained.crash;
    uint32_t sum=2166136261UL;
    for (unsigned n=0;n<sizeof(retained)-8U;++n) sum=(sum^p[n])*16777619UL;
    return sum;
}
static void commit(void)
{
    retained.check=checksum();
    __DMB(); retained.magic=DIAG_MAGIC;
}
static bool loader_debug_stop(const BoardDiagCrash *c)
{
    /* ST-Link may execute its RAM programming algorithm while application
     * vectors still point here. Its return breakpoint can leave a DEBUGEVT
     * HardFault record, then Reset starts the application with SRAM retained.
     * Match only the observed low-RAM loader context, never DEBUGEVT alone:
     * no valid main stage, no application PC, no real fault-status bits, and
     * a basic thread/MSP frame. Every other context retains boot inhibition. */
    return c->kind==3U && c->stage-1U>=7U && c->pc==0x20000000UL &&
        c->lr>=0x20000001UL && c->lr<0x20001000UL && (c->lr&1U) &&
        c->sp>=0x20000100UL && c->sp<=0x20000FE0UL && !(c->sp&7U) &&
        c->exc_return==0xFFFFFFF9UL && (c->xpsr&0x010001FFUL)==0x01000000UL &&
        c->cfsr==0U && c->hfsr==0x80000000UL; /* HFSR.DEBUGEVT only. */
}
void BoardDiag_Init(void)
{
    if (retained.magic!=DIAG_MAGIC || retained.check!=checksum()) memset(&retained,0,sizeof(retained));
    if (loader_debug_stop(&retained.crash)) memset(&retained.crash,0,sizeof(retained.crash));
    commit();
}
const BoardDiagFault *BoardDiag_LastFault(void) { return &retained.fault; }
const BoardDiagCrash *BoardDiag_LastCrash(void) { return &retained.crash; }
const BoardDiagTiming *BoardDiag_Times(void) { return &timing; }
void BoardDiag_OuterReset(void)
{
    /* Both stop callers hold PRIMASK; one aligned word clears the guard. */
    outer_watch=0;
}
bool BoardDiag_Clear(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool ok=!MotorSampling_IsOutputRequested() && !MotorSampling_IsAxisOutputRequested(MOTOR_AXIS_2);
    if (ok) {
        retained.magic=0; memset(&retained.crash,0,sizeof(retained)-8U); commit();
        memset(&timing,0,sizeof(timing)); seen=outer_watch=0;
    }
    __set_PRIMASK(mask); return ok;
}
void BoardDiag_Motion(unsigned axis,unsigned mode,float speed_target,float position_target,
                      float q,float speed,float position)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    motion[axis]=(BoardDiagMotion){speed_target,position_target,q,speed,position,HAL_GetTick(),(uint8_t)mode};
    __set_PRIMASK(mask);
}
void BoardDiag_Fault(unsigned axis,unsigned source,const char *reason)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    BoardDiagFault *f=&retained.fault;
    if (!f->source) {
        /* Reserve first so a diagnostic getter's protection poll cannot recurse. */
        retained.magic=0; f->source=(uint8_t)source; f->axis=(uint8_t)axis;
        f->at=HAL_GetTick(); f->cycles=timing.adc_max; f->work=(uint8_t)MotorControl_GetWork();
        memcpy(f->motion,motion,sizeof(motion));
        if (MotorControl_GetDrive(MOTOR_AXIS_1)==MOTOR_DRIVE_SIXSTEP) {
            SixStepSnapshot six; SixStep_GetSnapshot(&six);
            f->motion[0]=(BoardDiagMotion){.target_speed=six.target,.target_q=six.duty,
                .speed=six.velocity,.at=f->at,.mode=six.speed_control ? 1U : 3U};
        }
        const MotorSampleFrame *s=MotorSampling_GetFrame(); f->sequence=s->sequence;
        f->bus=MotorSampling_GetBusVoltage();
        MotorSamplingDiagnostics adc; MotorSampling_GetAxisDiagnostics(MOTOR_AXIS_1,&adc);
        f->reference=adc.reference_voltage;
        for (unsigned a=0;a<2;++a) {
            f->drive[a]=(uint8_t)MotorControl_GetDrive((MotorAxis)a);
            f->model[a]=(uint8_t)MotorParams_GetAxisProfile((MotorAxis)a);
            MT6701Diagnostics e;
            if (a) MT6701_2_GetDiagnostics(&e); else MT6701_GetDiagnostics(&e);
            f->enc_age[a]=e.age_cycles; f->enc_status[a]=e.status;
            f->enc_error[a]=(uint8_t)e.last_error; f->enc_errors[a]=e.errors;
            f->enc_health[a]=e.health|((uint32_t)e.wall_ms<<8)|((uint32_t)e.failures<<16);
            for (unsigned p=0;p<3;++p) {
                f->current[a][p]=a ? (p<2 ? s->m2_current[p] : -s->m2_current[0]-s->m2_current[1]) : s->m1_current[p];
                f->raw[a][p]=a ? (p<2 ? s->m2_current_raw[p] : 0U) : s->m1_current_raw[p];
            }
        }
        if (f->drive[0]==MOTOR_DRIVE_SIXSTEP && SixStep_IsGeneric()) f->model[0]=MOTOR_PROFILE_ESC;
        unsigned n=0; if (reason) for (;n<sizeof(f->reason)-1U && reason[n];++n) f->reason[n]=reason[n];
        f->reason[n]=0; commit();
    }
    __set_PRIMASK(mask);
}
__attribute__((optimize("Os")))
void BoardDiag_Timing(unsigned outer,uint32_t entered,uint32_t center_age)
{
    uint32_t now=DWT->CYCCNT,elapsed=now-entered,interval=0;
    const char *fault=NULL;
    /* ADC and TIM2 priorities are generated, not assumed. Protect only the
     * small integer monitor; restore before fault capture/controller cleanup. */
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    uint32_t *maximum=outer ? &timing.outer_max : &timing.adc_max;
    uint32_t *jitter=outer ? &timing.outer_jitter : &timing.adc_jitter;
    if (elapsed>*maximum) *maximum=elapsed;
    if (seen&(1U<<outer)) {
        interval=entered-previous[outer];
        uint32_t period=outer ? 170000U : 17000U;
        uint32_t error=interval>period ? interval-period : period-interval;
        if (error>*jitter) *jitter=error;
    }
    seen|=(uint8_t)(1U<<outer); previous[outer]=entered;
    if (!MotorControl_OuterRequired()) outer_watch=0;
    else {
        if (!outer_watch) { outer_watch=1;outer_completed=now; }
        if (outer) {
            bool over=elapsed>=170000U;
            bool gap=outer_watch>=2U && interval-85000U>170000U;
            if (over) ++timing.outer_overruns;
            if (gap) ++timing.outer_gaps;
            if (outer_watch==1U) outer_watch=2U;
            outer_watch=over || gap ? outer_watch+1U : 2U;
            outer_completed=now;
            if (elapsed>=340000U || outer_watch>=5U)
                fault=over ? "OUTER OVERRUN" : "OUTER GAP";
        } else if (now-outer_completed>=340000U) {
            ++timing.outer_stalls;
            fault="OUTER STALE";
        }
    }
    __set_PRIMASK(mask);
    if (fault) MotorControl_TimingFault(fault);
    if (!outer) {
        if (center_age>timing.center_age) timing.center_age=center_age;
        if (elapsed>=17000U) ++timing.overruns;
        StHso_RecordControlCycles(elapsed);
        MotorControl_RecordCycles(elapsed);
    }
}

static int32_t milli(float value,bool *valid)
{
    /* Faults may contain NaN or runaway numbers; display must stay defined. */
    if (!(value>=-2000000.0f && value<=2000000.0f)) { *valid=false; return 0; }
    return (int32_t)(value*1000.0f);
}
const char *BoardDiag_Row(unsigned axis,unsigned row,char value[32])
{
    static const char names[]=
        "ADC IRQ Max\0"
        "ADC Jitter\0"
        "PWM Age\0"
        "Outer Max\0"
        "Outer Jitter\0"
        "IRQ Overrun\0"
        "First\0"
        "F Time\0"
        "F W/D/P/C\0"
        "F Bus/Ref\0"
        "F Cmd/Act\0"
        "F Position\0"
        "F Current\0"
        "F ADC\0"
        "F Enc\0"
        "F Counts\0"
        "F Age/IRQ\0"
        "CPU Fault\0"
        "CPU PC\0"
        "CPU LR\0"
        "CPU SP\0"
        "CPU CFSR\0"
        "CPU HFSR\0"
        "CPU B/M\0"
        "CPU XPSR\0"
        "CPU Return\0"
        "Stop Src\0"
        "Stop At\0"
        "Stop K/P\0"
        "F Enc Gate\0"
        "Outer E/G/S";
    _Static_assert(BOARD_DIAG_ROWS==31,"Diagnostic rows");
    const BoardDiagFault *f=&retained.fault; const BoardDiagCrash *c=&retained.crash;
    bool valid=true;
    if (axis>1 || row>=BOARD_DIAG_ROWS) return NULL;
    const BoardDiagMotion *m=&f->motion[axis];
    if (row==30U) {
        UiText_Format(value,32,"%lu/%lu/%lu",(unsigned long)timing.outer_overruns,
            (unsigned long)timing.outer_gaps,(unsigned long)timing.outer_stalls);
    } else if (row==29U) {
        if (!f->source) strcpy(value,"--");
        else {
            uint32_t h=f->enc_health[axis];
            UiText_Format(value,32,"E%u H%02X T%lu F%lu",axis+1U,(unsigned)(h&255U),
                (unsigned long)((h>>8)&255U),(unsigned long)(h>>16));
        }
    } else if (row>=26U) {
        MotorGlobalStopRecord stop; MotorSampling_GetGlobalStop(&stop);
        static const char sources[]=
        "--\0"
        "CPU BOOT\0"
        "KEY1 EDGE\0"
        "KEY1 LOW\0"
        "KEY1 PEND\0"
        "COMMAND";
        if (!stop.source) strcpy(value,"--");
        else if (row==26U) strcpy(value,CompactText_At(sources,stop.source));
        else if (row==27U) UiText_Format(value,32,"%lu ms S%lu",(unsigned long)stop.at,(unsigned long)stop.stage);
        else UiText_Format(value,32,"%u/%u Now %u/%u",stop.key&1U,(stop.key>>1U)&1U,stop.now&1U,(stop.now>>1U)&1U);
    } else if ((row>=6U && row<=16U && !f->source) || (row>=17U && !c->kind)) strcpy(value,"--");
    else if (row<6U) {
        static const uint8_t offsets[]={offsetof(BoardDiagTiming,adc_max),offsetof(BoardDiagTiming,adc_jitter),
            offsetof(BoardDiagTiming,center_age),offsetof(BoardDiagTiming,outer_max),offsetof(BoardDiagTiming,outer_jitter),offsetof(BoardDiagTiming,overruns)};
        uint32_t time;memcpy(&time,(const char *)&timing+offsets[row],sizeof(time));
        if (row==5U) UiText_Format(value,32,"%lu",(unsigned long)time);
        else UiText_Fixed(value,32,(int32_t)(time/17U),1U,"us");
    } else switch (row) {
    case 6: UiText_Format(value,32,"%u M%u %s",f->source,f->axis,f->reason); break;
    case 7: UiText_Format(value,32,"%lu ms",(unsigned long)f->at); break;
    case 8: {
        static const char drives[]=
        "ENCODER\0"
        "HSO\0"
        "6STEP";
        UiText_Format(value,32,"%u/%s/%u/%u",f->work,CompactText_At(drives,f->drive[axis]%3U),f->model[axis],m->mode); break;
    }
    case 9: UiText_Format(value,32,"%ld/%ld mV",(long)milli(f->bus,&valid),(long)milli(f->reference,&valid)); break;
    case 10:
        if (m->mode==3U) UiText_Format(value,32,"%ld%%/%ld %s",(long)(milli(m->target_speed,&valid)/1000),(long)milli(m->speed,&valid),f->model[axis]==MOTOR_PROFILE_ESC ? "mHz" : "mrad/s");
        else UiText_Format(value,32,"%ld/%ld mrad/s",(long)milli(m->target_speed,&valid),(long)milli(m->speed,&valid));
        break;
    case 11:
        if (f->drive[axis]==MOTOR_DRIVE_SIXSTEP) strcpy(value,"--");
        else UiText_Format(value,32,"%ld/%ld mrad",(long)milli(m->target_position,&valid),(long)milli(m->position,&valid));
        break;
    case 12: UiText_Format(value,32,"%ld/%ld/%ld mA",(long)milli(f->current[axis][0],&valid),(long)milli(f->current[axis][1],&valid),(long)milli(f->current[axis][2],&valid)); break;
    case 13: UiText_Format(value,32,"%u/%u/%u",f->raw[axis][0],f->raw[axis][1],f->raw[axis][2]); break;
    case 14: UiText_Format(value,32,"E%u %s %u/%02X",axis+1U,MT6701_ErrorText((MT6701Error)f->enc_error[axis]),f->enc_age[axis],f->enc_status[axis]); break;
    case 15: UiText_Format(value,32,"%lu/%lu",(unsigned long)f->sequence,(unsigned long)f->enc_errors[axis]); break;
    case 16: UiText_Format(value,32,"%lu ms/%lu us",(unsigned long)(f->at-m->at),(unsigned long)(f->cycles/170U)); break;
    case 17: UiText_Format(value,32,"%lu stage %lu",(unsigned long)c->kind,(unsigned long)c->stage); break;
    default: {
        static const uint8_t regs[]={offsetof(BoardDiagCrash,pc),offsetof(BoardDiagCrash,lr),offsetof(BoardDiagCrash,sp),offsetof(BoardDiagCrash,cfsr),offsetof(BoardDiagCrash,hfsr),offsetof(BoardDiagCrash,bfar),offsetof(BoardDiagCrash,xpsr),offsetof(BoardDiagCrash,exc_return)};
        uint32_t reg;memcpy(&reg,(const char *)c+regs[row-18U],sizeof(reg));
        UiText_Format(value,32,"%08lX",(unsigned long)reg);
        if (row==23U) UiText_Format(value,32,"%08lX/%08lX",(unsigned long)c->bfar,(unsigned long)c->mmfar);
        break;
    }
    }
    if (!valid) strcpy(value,"INVALID");
    return CompactText_At(names,row);
}

#if defined(__arm__) || defined(__thumb__)
/* Do not depend on a possibly corrupted MSP, the HAL or floating point in
 * the fatal path. The stack is reserved independently of normal MSP/PSP. */
static uint32_t fault_stack[64] __attribute__((used,aligned(8)));
extern volatile uint32_t g_boot_stage;
__attribute__((naked,used)) void BoardDiag_ExceptionEntry(void)
{
    __asm volatile("cpsid i\n tst lr,#4\n ite eq\n mrseq r0,msp\n mrsne r0,psp\n"
                   "mov r1,lr\n ldr r3,=fault_stack+256\n msr msp,r3\n b BoardDiag_Fatal");
}
__attribute__((noreturn,used,noinline,optimize("Os"),target("general-regs-only")))
void BoardDiag_Fatal(uint32_t *stack,uint32_t exc_return,uint32_t kind)
{
    /* Output gate first, independent of initialization/handle state. */
    if (RCC->APB2ENR&RCC_APB2ENR_TIM1EN) { TIM1->BDTR&=~TIM_BDTR_MOE; TIM1->CCER&=~0x555UL; }
    if (RCC->APB2ENR&RCC_APB2ENR_TIM8EN) { TIM8->BDTR&=~TIM_BDTR_MOE; TIM8->CCER&=~0x555UL; }
    BoardDiag_Init();
    if (!retained.crash.kind) {
        retained.magic=0;
        BoardDiagCrash *c=&retained.crash;
        c->kind=kind; c->stage=g_boot_stage; c->sp=(uint32_t)(uintptr_t)stack; c->exc_return=exc_return;
        c->cfsr=SCB->CFSR; c->hfsr=SCB->HFSR; c->bfar=SCB->BFAR; c->mmfar=SCB->MMFAR;
        /* Extended FP context precedes the basic frame. Stacking faults or
         * invalid pointers must never cause a second fault while reading it. */
        uintptr_t frame=(uintptr_t)stack+((exc_return&16U) ? 0U : 72U);
        if (!(c->cfsr&0x3838U) && !(frame&3U) && frame>=0x20000000UL && frame<=0x20008000UL-32U) {
            const uint32_t *p=(const uint32_t *)frame;
            c->lr=p[5]; c->pc=p[6]; c->xpsr=p[7];
        }
        commit();
    }
    __DSB(); for (;;) __NOP();
}
#endif
