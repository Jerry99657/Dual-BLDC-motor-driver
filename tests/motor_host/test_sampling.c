#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "adc.h"
#include "tim.h"
#include "motor_sampling.h"
#include "motor_params.h"

ADC_TypeDef adc1, adc2;
ADC_HandleTypeDef hadc1 = {.Instance=ADC1}, hadc2 = {.Instance=ADC2};
TIM_TypeDef tim1, tim8, tim6;
TIM_HandleTypeDef htim1 = {&tim1}, htim8 = {&tim8}, htim6 = {&tim6};
uint32_t host_primask;
static uint32_t tick, calls, fail_at;
static bool dma, bus_timer, slave_armed, master_armed, publish_enabled, fault_after_arming;
static bool model_bus_trigger;
static unsigned bus_triggers;
static uint16_t model_bus_raw;
static volatile uint16_t *buffer;
static bool irq_on_tick;
static void inject_irq(void);

uint32_t HAL_GetTick(void) {
    uint32_t now=tick;
    if (irq_on_tick) { irq_on_tick=false; inject_irq(); }
    return now;
}
static HAL_StatusTypeDef status(void) { return ++calls == fail_at ? HAL_ERROR : HAL_OK; }
static void publish(uint16_t raw) {
    assert(dma && buffer);
    buffer[0] = buffer[1] = raw;
    HAL_ADC_ConvHalfCpltCallback(&hadc1);
    HAL_ADC_ConvCpltCallback(&hadc1);
}
void HAL_Delay(uint32_t ms) {
    tick += ms;
    if (publish_enabled && bus_timer && dma) publish(1354U);
}
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h, uint32_t mode) {
    (void)h; (void)mode;
    assert(!(tim1.CR1 & TIM_CR1_CEN) && !(tim8.CR1 & TIM_CR1_CEN));
    assert(!(tim1.BDTR & TIM_BDTR_MOE) && !(tim8.BDTR & TIM_BDTR_MOE));
    return status();
}
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *h, uint32_t *p, uint32_t n) {
    assert(h == &hadc1 && n == 2U);
    if (status() != HAL_OK) return HAL_ERROR;
    buffer = (volatile uint16_t *)p; dma = true; return HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_InjectedStart(ADC_HandleTypeDef *h) {
    assert(h == &hadc2 && !master_armed);
    if (status() != HAL_OK) return HAL_ERROR;
    slave_armed = true; return HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef *h) {
    assert(h == &hadc1 && slave_armed);
    if (status() != HAL_OK) return HAL_ERROR;
    master_armed = true;
    if (fault_after_arming) HAL_ADC_ErrorCallback(&hadc1);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *h) {
    assert(h != &htim6); /* A free-running regular trigger must never return. */
    assert(h == &htim1 && slave_armed && master_armed && dma);
    assert(!(tim6.CR1 & TIM_CR1_CEN));
    assert(host_primask == 1U);
    assert(!(tim8.CR1 & TIM_CR1_CEN));
    assert(!(tim1.CR1 & TIM_CR1_DIR) && !(tim8.CR1 & TIM_CR1_DIR));
    assert(!(tim1.BDTR & TIM_BDTR_MOE) && !(tim8.BDTR & TIM_BDTR_MOE));
    assert((tim8.CCER & 0x555U) == 0x555U);
    assert((tim1.CCER & 0x555U) == 0U); /* Only CH4 may be enabled during zeroing. */
    assert(tim1.CCR1 == tim1.CCR2 && tim1.CCR2 == tim1.CCR3);
    assert(tim8.CCR1 == tim8.CCR2 && tim8.CCR2 == tim8.CCR3);
    if (status() != HAL_OK) return HAL_ERROR;
    tim1.CR1 |= TIM_CR1_CEN;
    tim8.CR1 |= TIM_CR1_CEN; /* Hardware ITR0 from the master's CEN. */
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *h) {
    /* Real HAL does not clear CEN while PWM CCER bits remain enabled. */
    if (!h->Instance->CCER) h->Instance->CR1 &= ~TIM_CR1_CEN;
    if (h == &htim6) bus_timer = false;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT(ADC_HandleTypeDef *h) {
    assert(h == &hadc1); master_armed = false; return HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_InjectedStop(ADC_HandleTypeDef *h) {
    assert(h == &hadc2 && !master_armed); slave_armed = false; return HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *h) {
    assert(h == &hadc1); dma = false; return HAL_OK;
}
static void reset(void) {
    MotorSampling_Stop();
    memset(&tim1,0,sizeof(tim1)); memset(&tim8,0,sizeof(tim8)); memset(&tim6,0,sizeof(tim6));
    tim1.ARR = tim8.ARR = 8499U;
    /* Restart from a previous down-counting phase. */
    tim1.CR1 = tim8.CR1 = TIM_CR1_CMS | TIM_CR1_DIR;
    tick = calls = fail_at = host_primask = 0U; publish_enabled = true; fault_after_arming = false;
    irq_on_tick=false; hadc1.ErrorCode=hadc2.ErrorCode=0;
    model_bus_trigger=false; bus_triggers=0; model_bus_raw=1354;
}
static bool capture_with_bus(void) {
    bool ok=MotorSampling_Capture();
    if (tim6.EGR & TIM_EVENTSOURCE_UPDATE) {
        assert(!(tim6.CR1 & TIM_CR1_CEN));
        assert(!(tim1.CR1 & TIM_CR1_DIR) && tim1.CNT<=tim1.ARR/2);
        tim6.EGR=0; /* Hardware self-clears UG and emits TRGO. */
        ++bus_triggers;
        if(model_bus_trigger && publish_enabled) {
            if(bus_triggers & 1U) {
                buffer[0]=model_bus_raw; HAL_ADC_ConvHalfCpltCallback(&hadc1);
            } else {
                buffer[1]=model_bus_raw; HAL_ADC_ConvCpltCallback(&hadc1);
            }
        }
    }
    return ok;
}
#define MotorSampling_Capture capture_with_bus
static void frame(int delta) {
    adc1.JDR1 = 2000 + delta; adc2.JDR1 = 2050 - delta;
    adc1.JDR2 = 2100 + 2*delta; adc1.JDR3 = 1111;
    adc2.JDR2 = 2222; adc2.JDR3 = 3333;
    adc1.JDR4 = 1900 - 2*delta; adc2.JDR4 = 1950 + delta/2;
    adc2.ISR |= ADC_FLAG_JEOS | ADC_FLAG_JEOC;
}
static void inject_irq(void) {
    ++tick; publish(1354); frame(0); assert(MotorSampling_Capture());
}
static void off(void) {
    assert(!(tim8.BDTR & TIM_BDTR_MOE));
    assert(!(tim1.BDTR & TIM_BDTR_MOE) || !(tim1.CCER & 0x555U));
    assert(!MotorSampling_IsReady());
}
static void near(float a,float b) { assert(fabsf(a-b)<0.0001f); }
static void sample_ms(unsigned ms,uint16_t raw) {
    for (unsigned m=0;m<ms;++m) {
        ++tick; publish(raw);
        for (unsigned n=0;n<10;++n) {
            frame(0);
            if (raw==0) { /* Model an analog front end powered by the late motor supply. */
                adc1.JDR1=adc2.JDR1=adc1.JDR2=adc1.JDR4=adc2.JDR4=0;
            }
            assert(MotorSampling_Capture());
        }
        MotorSampling_Service();
    }
}
static MotorSamplingDiagnostics diagnostics(void) {
    MotorSamplingDiagnostics out;
    MotorSampling_GetDiagnostics(&out);
    assert(out.silicon_revision==0x2002U);
    return out;
}
static void loaded_ms(unsigned ms,int counts) {
    for(unsigned m=0;m<ms;++m) {
        ++tick; publish(1354);
        for(unsigned n=0;n<10;++n) {
            frame(0); adc1.JDR1=2000+counts;
            adc2.JDR1=2050-counts/2; adc1.JDR2=2100-counts/2;
            assert(MotorSampling_Capture());
        }
        MotorSampling_Service();
    }
}
static void clocked_ms(unsigned ms) {
    for(unsigned m=0;m<ms;++m) {
        ++tick;
        for(unsigned n=0;n<10;++n) {
            tim1.CNT=680; /* ~4 us after the PWM1 CH4 rising trigger. */
            frame(0); assert(MotorSampling_Capture());
        }
        MotorSampling_Service();
    }
}
int main(void) {
    /* Drive the real scheduling through JEOS -> stopped TIM6 UPDATE -> DMA.
     * No asynchronous/manual bus publish: this must work before zeroing,
     * after late VBUS connection, and after an actual current fault. */
    reset(); assert(MotorSampling_Start()); model_bus_trigger=true; model_bus_raw=0;
    clocked_ms(1000);
    assert(bus_triggers==2000 && diagnostics().bus_samples==2000);
    assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_NONE && diagnostics().bus_fresh);
    assert(!MotorSampling_IsReady() && !MotorSampling_IsCalibrated());
    model_bus_raw=1354; clocked_ms(301); assert(MotorSampling_IsReady());
    assert(bus_triggers==2602 && diagnostics().bus_samples==2602);
    /* Timing risk is skipped, never started near the next injected edge. */
    unsigned triggered=bus_triggers;
    for(unsigned i=0;i<5;++i) {
        tim1.CNT=8000; frame(0); assert(MotorSampling_Capture());
    }
    assert(bus_triggers==triggered);
    for(unsigned i=0;i<5;++i) {
        tim1.CR1|=TIM_CR1_DIR; tim1.CNT=680;
        frame(0); assert(MotorSampling_Capture());
    }
    assert(bus_triggers==triggered);
    tim1.CR1&=~TIM_CR1_DIR; clocked_ms(1);
    assert(bus_triggers==triggered+2 && MotorSampling_IsReady());
    frame(0); adc1.JDR1=0; assert(MotorSampling_Capture()); off();
    assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_CURRENT_RANGE);
    triggered=bus_triggers; clocked_ms(20); off();
    assert(bus_triggers==triggered+40 && diagnostics().bus_fresh);
    assert(diagnostics().fault_current_raw[0]==0);

    reset(); assert(MotorSampling_Start());
    off(); assert(!MotorSampling_IsCalibrated());
    assert(calls==6 && host_primask==0);
    assert((tim1.CR1 & TIM_CR1_CEN) && (tim8.CR1 & TIM_CR1_CEN));
    assert(!diagnostics().bus_fresh && strcmp(MotorSampling_GetStatusText(),"WAIT ADC")==0);
    sample_ms(299,1354); off(); assert(!MotorSampling_IsCalibrated());
    sample_ms(1,1354); assert(MotorSampling_IsReady() && MotorSampling_IsCalibrated());
    assert((tim1.BDTR & TIM_BDTR_MOE) && !(tim8.BDTR & TIM_BDTR_MOE));
    assert((tim1.CCER & 0x555U)==0x555U);
    near(MotorSampling_GetBusVoltage(),1354.0f*36.3f/4095.0f);
    frame(100); assert(MotorSampling_Capture());
    MotorSampleFrame out; host_primask=1; MotorSampling_CopyFrame(&out); assert(host_primask==1); host_primask=0;
    MotorSampling_CopyFrame(NULL);
    assert(out.sequence==3001 && !adc2.ISR);
    assert(out.m1_current_raw[0]==2100 && out.m1_current_raw[1]==1950 && out.m1_current_raw[2]==2300);
    assert(out.m2_current_raw[0]==1700 && out.m2_current_raw[1]==2000);
    const float gain=3.3f/4095.0f/0.5f;
    near(out.m1_current[0],100*gain); near(out.m1_current[1],-100*gain); near(out.m1_current[2],200*gain);
    near(out.m2_current[0],-200*gain); near(out.m2_current[1],50*gain);
    near(out.m1_voltage[0],1111*36.3f/4095.0f);
    near(out.m1_voltage[1],2222*36.3f/4095.0f);
    near(out.m1_voltage[2],3333*36.3f/4095.0f);
    publish(2707); near(MotorSampling_GetBusVoltage(),2707*36.3f/4095.0f);
    /* Reproduces the old GetTick()-ISR_timestamp underflow: an ADC/DMA IRQ
     * publishes a timestamp in the next tick before GetTick returns the old one. */
    irq_on_tick=true; MotorSampling_Service(); assert(MotorSampling_IsReady());
    assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_NONE);
    MotorSampling_GetDiagnostics(NULL);
    /* An incomplete paired sequence must never become a new frame. */
    uint32_t before_pair_error=MotorSampling_GetFrame()->sequence;
    assert(!MotorSampling_Capture()); off(); assert(MotorSampling_GetFrame()->sequence==before_pair_error);
    assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_PAIR);
    sample_ms(20,2707); off();
    assert(diagnostics().bus_fresh && diagnostics().bus_samples>600);
    assert(diagnostics().frame_samples>3001);
    near(MotorSampling_GetBusVoltage(),2707*36.3f/4095.0f);

    /* MCU first, 0 V for one second, then connect the motor supply. */
    reset(); assert(MotorSampling_Start()); sample_ms(1000,0); off();
    assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_NONE && diagnostics().bus_samples==2000);
    assert(diagnostics().bus_fresh && strcmp(MotorSampling_GetStatusText(),"WAIT BUS")==0);
    assert(!MotorSampling_IsCalibrated()); near(MotorSampling_GetBusVoltage(),0);
    sample_ms(299,1354); off(); sample_ms(1,1354); assert(MotorSampling_IsReady());
    frame(100); assert(MotorSampling_Capture());
    near(MotorSampling_GetFrame()->m1_current[0],100*gain); /* No zero-offset pollution from unpowered input. */
    assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_NONE);
    /* A true loss while powered shuts down power, but never shuts off VBUS sampling. */
    publish(0); off(); assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_BUS_LOST);
    sample_ms(200,1354); off();
    near(MotorSampling_GetBusVoltage(),1354*36.3f/4095.0f);
    assert(diagnostics().bus_fresh && diagnostics().fault==MOTOR_SAMPLE_FAULT_BUS_LOST);

    reset(); assert(MotorSampling_Start()); sample_ms(300,1354);
    tick+=6; MotorSampling_Service(); off(); near(MotorSampling_GetBusVoltage(),0);
    assert(!diagnostics().bus_fresh && diagnostics().fault==MOTOR_SAMPLE_FAULT_FRAME_TIMEOUT);
    reset(); assert(MotorSampling_Start()); sample_ms(300,1354);
    tick+=6; publish(1354); MotorSampling_Service(); off();
    assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_FRAME_TIMEOUT);
    assert(diagnostics().bus_fresh); /* Real 12 V with a stopped injected sequence. */
    reset(); assert(MotorSampling_Start()); sample_ms(300,1354);
    tick+=6; publish(1354); frame(0); assert(MotorSampling_Capture()); off();
    assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_FRAME_TIMEOUT);
    reset(); assert(MotorSampling_Start()); sample_ms(300,1354); publish(4095); off();
    assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_BUS_RANGE);
    near(MotorSampling_GetBusVoltage(),36.3f);
    reset(); assert(MotorSampling_Start()); sample_ms(300,1354);
    hadc2.ErrorCode=0x10U; HAL_ADC_ErrorCallback(&hadc2); off();
    assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_ADC && diagnostics().adc2_error==0x10U);
    sample_ms(1,1354); off(); assert(diagnostics().bus_fresh);

    /* Both polarities of every phase trip at the nameplate peak, latch,
     * and leave CH4, the counters and all ADC measurements running. */
    assert(MOTOR_2804.rated_current==0.5f && MOTOR_2804.max_current==2.0f);
    for(unsigned phase=0;phase<3;++phase) for(int sign=-1;sign<=1;sign+=2) {
        reset(); assert(MotorSampling_Start()); sample_ms(300,1354);
        const int offsets[3]={2000,2050,2100};
        volatile uint32_t *regs[3]={&adc1.JDR1,&adc2.JDR1,&adc1.JDR2};
        int below=(int)(MOTOR_2804.max_current/gain);
        frame(0); *regs[phase]=(uint32_t)(offsets[phase]+sign*below);
        assert(MotorSampling_Capture() && MotorSampling_IsReady());
        frame(0); *regs[phase]=(uint32_t)(offsets[phase]+sign*(below+1));
        uint32_t before=MotorSampling_GetFrame()->sequence;
        assert(MotorSampling_Capture()); off();
        assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_OVERCURRENT);
        assert(diagnostics().fault_phase==phase+1 && !diagnostics().fault_during_zero);
        assert(diagnostics().fault_current_raw[phase]==(uint16_t)(offsets[phase]+sign*(below+1)));
        assert(strcmp(MotorSampling_GetStatusText(),"M1 OVERCURRENT")==0);
        assert((tim1.CCER & 0x555U)==0 && (tim1.CCER & TIM_CCER_CC4E));
        assert(tim1.BDTR & TIM_BDTR_MOE); /* Internal trigger path stays live. */
        sample_ms(20,1354); off();
        assert(MotorSampling_GetFrame()->sequence>before+1);
        assert(diagnostics().bus_fresh && dma && !bus_timer && master_armed && slave_armed);
        assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_OVERCURRENT);
    }
    for(unsigned phase=0;phase<3;++phase) for(unsigned rail=0;rail<2;++rail) {
        reset(); assert(MotorSampling_Start()); sample_ms(300,1354);
        assert(diagnostics().fault_phase==0);
        frame(0);
        volatile uint32_t *regs[3]={&adc1.JDR1,&adc2.JDR1,&adc1.JDR2};
        *regs[phase]=rail?4095:0;
        assert(MotorSampling_Capture()); off();
        assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_CURRENT_RANGE);
        const char *messages[]={"M1 ADC U","M1 ADC V","M1 ADC W"};
        assert(strcmp(MotorSampling_GetStatusText(),messages[phase])==0);
        assert(diagnostics().fault_phase==phase+1 && !diagnostics().fault_during_zero);
        assert(diagnostics().fault_current_raw[phase]==(rail?4095:0));
        sample_ms(10,1354); off();
        assert(diagnostics().bus_fresh && diagnostics().fault==MOTOR_SAMPLE_FAULT_CURRENT_RANGE);
        assert(diagnostics().current_raw[phase]==2000+50*phase);
        assert(diagnostics().current_zero[phase]==2000+50*phase);
        assert(diagnostics().fault_current_raw[phase]==(rail?4095:0));
        /* A subsequent bad frame must not overwrite the first fault. */
        frame(0); adc1.JDR1=adc2.JDR1=adc1.JDR2=rail?0:4095;
        assert(MotorSampling_Capture());
        assert(diagnostics().fault_current_raw[phase]==(rail?4095:0));
    }
    /* Manual loading: separate 0.8 A reference, 3 s excess-I^2 allowance.
     * A ~1 A vector now lasts ~5.3 s; it still cannot run indefinitely. */
    near(MOTOR_2804.overload_current,0.8f); near(MOTOR_2804.overload_budget_s,3.0f);
    reset(); assert(MotorSampling_Start()); sample_ms(300,1354);
    loaded_ms(5200,620); assert(MotorSampling_IsReady());
    loaded_ms(200,620); off();
    assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_OVERLOAD);
    assert(strcmp(MotorSampling_GetStatusText(),"M1 OVERLOAD")==0);
    sample_ms(50,1354); off(); assert(diagnostics().bus_fresh);
    reset(); assert(MotorSampling_Start()); sample_ms(300,1354);
    loaded_ms(1000,620); loaded_ms(600,0); /* Below-reference current drains the budget. */
    loaded_ms(5200,-620); assert(MotorSampling_IsReady());
    loaded_ms(200,-620); off(); assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_OVERLOAD);
    /* Higher loads still trip quickly, in either direction. */
    for(int sign=-1;sign<=1;sign+=2) {
        reset(); assert(MotorSampling_Start()); sample_ms(300,1354);
        loaded_ms(1100,sign*930); assert(MotorSampling_IsReady()); /* ~1.5 A */
        loaded_ms(150,sign*930); off();
        assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_OVERLOAD);
    }
    /* Repeated brief loading separated by unloaded operation is allowed. */
    reset(); assert(MotorSampling_Start()); sample_ms(300,1354);
    for(unsigned i=0;i<5;++i) {
        loaded_ms(500,930); assert(MotorSampling_IsReady());
        loaded_ms(1500,0); assert(MotorSampling_IsReady());
    }
    /* Immediately below the new reference does not accumulate overload. */
    loaded_ms(10000,496); assert(MotorSampling_IsReady()); /* ~0.799 A */
    reset(); assert(MotorSampling_Start()); sample_ms(300,1354);
    loaded_ms(2000,310); assert(MotorSampling_IsReady()); /* ~0.5 A continuous. */
    /* A failed current front end cannot calibrate zero at an ADC rail and
     * briefly enable outputs before the first powered range check. */
    reset(); assert(MotorSampling_Start());
    for(unsigned m=0;m<301;++m) {
        ++tick; publish(1354);
        for(unsigned n=0;n<10;++n) {
            frame(0); adc1.JDR1=adc2.JDR1=adc1.JDR2=0;
            assert(MotorSampling_Capture());
        }
        MotorSampling_Service();
    }
    off(); assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_CURRENT_RANGE);
    assert(diagnostics().fault_during_zero && diagnostics().fault_phase==1);
    assert(strcmp(MotorSampling_GetStatusText(),"M1 ZERO U")==0);
    assert(diagnostics().current_zero[0]==0);
    sample_ms(10,1354);
    assert(diagnostics().current_raw[0]==2000 && diagnostics().fault_current_raw[0]==0);

    /* No DMA callbacks is different from valid DMA samples measuring 0 V. */
    reset(); assert(MotorSampling_Start());
    for(unsigned n=0;n<6;++n) {
        ++tick; frame(0); assert(MotorSampling_Capture()); MotorSampling_Service();
    }
    assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_BUS_TIMEOUT); off();
    reset(); assert(MotorSampling_Start()); MotorSampling_Stop(); off();
    assert(!(tim1.CR1 & TIM_CR1_CEN) && !(tim8.CR1 & TIM_CR1_CEN) && !dma && !bus_timer);
    for(unsigned failure=1;failure<=6;++failure) {
        reset(); fail_at=failure; assert(!MotorSampling_Start()); off();
        assert(!(tim1.CR1 & TIM_CR1_CEN) && !(tim8.CR1 & TIM_CR1_CEN) && !dma && !bus_timer);
    }
    reset(); fault_after_arming=true; assert(!MotorSampling_Start()); off(); assert(calls==5);
    reset(); tick=UINT32_MAX-150; assert(MotorSampling_Start());
    sample_ms(300,1354); assert(MotorSampling_IsReady() && diagnostics().fault==MOTOR_SAMPLE_FAULT_NONE);
    puts("PASS: continuous sampling, late VBUS, timestamp race, phase mapping, +/-2A trip on all phases, ADC rails/zero failure, sustained overload/cooling, latched power-off, startup cleanup, tick wrap");
    return 0;
}
