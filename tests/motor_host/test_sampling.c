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
uint16_t host_vrefint_cal=1650;
static uint32_t tick, calls, fail_at;
static bool dma, bus_timer, slave_armed, master_armed, publish_enabled, fault_after_arming;
static bool model_bus_trigger;
static unsigned bus_triggers;
static uint16_t model_bus_raw;
static uint16_t reference_raw=1500; /* 1650 * 3.0 / 1500 = 3.3 V. */
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
    buffer[0] = buffer[2] = raw;
    buffer[1] = buffer[3] = reference_raw;
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
    assert(h == &hadc1 && n == 4U);
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
    assert((tim8.CCER & 0x555U) == 0U);
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
    reference_raw=1500; host_vrefint_cal=1650;
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
                buffer[0]=model_bus_raw; buffer[1]=reference_raw;
                HAL_ADC_ConvHalfCpltCallback(&hadc1);
            } else {
                buffer[2]=model_bus_raw; buffer[3]=reference_raw;
                HAL_ADC_ConvCpltCallback(&hadc1);
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
static void check_reference_estimation(void) {
    reset(); reference_raw=1518; assert(MotorSampling_Start());
    sample_ms(31,1354);
    assert(!diagnostics().reference_fresh && diagnostics().reference_updates==0);
    off();
    sample_ms(1,1354);
    const float measured=3.0f*1650.0f/1518.0f;
    near(diagnostics().reference_voltage,measured);
    assert(diagnostics().reference_fresh && diagnostics().reference_updates==1);
    assert(diagnostics().reference_raw==1518 && diagnostics().reference_cal==1650);
    sample_ms(268,1354); assert(MotorSampling_IsReady());
    frame(100); assert(MotorSampling_Capture());
    const MotorSampleFrame *f=MotorSampling_GetFrame();
    const float amps_per_code=measured/4095.0f/0.5f;
    near(f->m1_current[0],100*amps_per_code);
    near(f->m1_current[1],-100*amps_per_code);
    near(f->m1_current[2],200*amps_per_code);
    near(f->m2_current[0],-200*amps_per_code);
    near(f->m2_current[1],50*amps_per_code);
    for(unsigned i=0;i<3;++i)
        near(f->m1_voltage[i],(float)(1111*(i+1))*measured*11.0f/4095.0f);
    near(MotorSampling_GetBusVoltage(),1354*measured*11.0f/4095.0f);
    /* Reject individual rail/out-of-range samples without changing the gain. */
    const uint16_t invalid[]={0,4095,1000,2000};
    for(unsigned i=0;i<sizeof(invalid)/sizeof(*invalid);++i) {
        reference_raw=invalid[i]; sample_ms(1,1354);
        near(diagnostics().reference_voltage,measured);
        assert(MotorSampling_IsReady());
    }
    assert(diagnostics().reference_errors==8);
    uint32_t updates=diagnostics().reference_updates;
    /* A complete jittering block has the same mean, not a jittering gain. */
    for(unsigned i=0;i<32;++i) { reference_raw=(i&1U)?1517:1519; sample_ms(1,1354); }
    near(diagnostics().reference_voltage,measured);
    assert(diagnostics().reference_updates==updates+1U);
    /* A real slow rail change is averaged first, then low-pass filtered. */
    reference_raw=1650; sample_ms(31,1354);
    near(diagnostics().reference_voltage,measured);
    sample_ms(1,1354);
    float expected=measured+0.125f*(3.0f-measured);
    near(diagnostics().reference_voltage,expected);
    sample_ms(32,1354); expected+=0.125f*(3.0f-expected);
    near(diagnostics().reference_voltage,expected);
    sample_ms(2400,1354); near(diagnostics().reference_voltage,3.0f);
    frame(100); assert(MotorSampling_Capture());
    near(f->m1_current[0],100*3.0f/4095.0f/0.5f);
    near(f->m1_voltage[0],1111*3.0f*11.0f/4095.0f);
    near(MotorSampling_GetBusVoltage(),1354*3.0f*11.0f/4095.0f);
    frame(0); assert(MotorSampling_Capture());
    near(f->m1_current[0],0); /* Keep zero in ADC code space; never fixed 1.65 V. */
    /* A frozen valid gain must not silently authorize indefinite operation. */
    reference_raw=0; sample_ms(201,1354); off();
    assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_REFERENCE);
    assert(!diagnostics().reference_fresh && diagnostics().bus_fresh);
    assert(strcmp(MotorSampling_GetStatusText(),"ADC REF")==0);
    reference_raw=1518; sample_ms(300,1354); off();
    assert(diagnostics().reference_fresh && diagnostics().fault==MOTOR_SAMPLE_FAULT_REFERENCE);
    /* No first valid block: wait without enabling power, then latch a fault. */
    reset(); reference_raw=0; assert(MotorSampling_Start());
    sample_ms(1,1354); assert(strcmp(MotorSampling_GetStatusText(),"WAIT VREF")==0);
    sample_ms(200,1354); off();
    assert(!MotorSampling_IsCalibrated() && diagnostics().reference_updates==0);
    assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_REFERENCE);
    /* Missing factory calibration must never fall back to a guessed scale. */
    const uint16_t invalid_cal[]={0,65535};
    for(unsigned i=0;i<2;++i) {
        reset(); host_vrefint_cal=invalid_cal[i]; assert(!MotorSampling_Start());
        off(); assert(calls==0 && diagnostics().fault==MOTOR_SAMPLE_FAULT_REFERENCE);
    }
    reset(); host_vrefint_cal=1500; reference_raw=1380;
    assert(MotorSampling_Start()); sample_ms(32,1354);
    near(diagnostics().reference_voltage,3.0f*1500.0f/1380.0f);
    sample_ms(268,1354); assert(MotorSampling_IsReady());
    /* Protection still trips at physical amperes with a non-3.3 V scale. */
    float gain=diagnostics().reference_voltage/4095.0f/0.5f;
    int below=(int)(MOTOR_2804.max_current/gain);
    frame(0); adc1.JDR1=2000+below;
    assert(MotorSampling_Capture() && MotorSampling_IsReady());
    frame(0); adc1.JDR1=2000+below+1;
    assert(MotorSampling_Capture()); off();
    assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_OVERCURRENT);
    puts("PASS: VREFINT factory calibration, 64-sample averaging/IIR, first-block gate, shared current/phase/bus scale, outlier rejection, stale stop, latched recovery");
}
static void check_voltage_plateau(void) {
    reset();assert(MotorSampling_Start());sample_ms(301,1354);
    assert(MotorSampling_SetOutputEnabled(false));
    tim1.CCR1=tim1.CCR2=tim1.CCR3=4250;
    frame(0);assert(MotorSampling_Capture());assert(!MotorSampling_GetFrame()->m1_voltage_high_plateau);
    assert(MotorSampling_SetOutputEnabled(true));
    for(unsigned i=0;i<2;++i) {frame(0);assert(MotorSampling_Capture());assert(!MotorSampling_GetFrame()->m1_voltage_high_plateau);}
    frame(0);assert(MotorSampling_Capture());assert(MotorSampling_GetFrame()->m1_voltage_high_plateau);
    for(unsigned phase=0;phase<3;++phase) {
        uint32_t *ccr[]={&tim1.CCR1,&tim1.CCR2,&tim1.CCR3};
        *ccr[phase]=800;frame(0);assert(MotorSampling_Capture());
        assert(!MotorSampling_GetFrame()->m1_voltage_high_plateau);
        *ccr[phase]=801;frame(0);assert(MotorSampling_Capture());
        assert(MotorSampling_GetFrame()->m1_voltage_high_plateau);*ccr[phase]=4250;
    }
    tim1.BDTR &= ~TIM_BDTR_MOE;frame(0);assert(MotorSampling_Capture());
    assert(!MotorSampling_GetFrame()->m1_voltage_high_plateau);
    puts("PASS: voltage plateau metadata snapshots real MOE/CCER and all three PWM widths, edge guard, coast and both pre-arm frames");
}
int main(void) {
    check_reference_estimation();
    check_voltage_plateau();
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
    /* The selected profile, rather than the old global 2804 constants,
     * controls every phase and both overload/instantaneous protections. */
    MotorParams_Select(MOTOR_PROFILE_4015);
    for(unsigned phase=0;phase<3;++phase) for(int sign=-1;sign<=1;sign+=2) {
        reset(); assert(MotorSampling_Start()); sample_ms(300,1354);
        const int offsets[3]={2000,2050,2100};
        volatile uint32_t *regs[3]={&adc1.JDR1,&adc2.JDR1,&adc1.JDR2};
        int below=(int)(MOTOR_4015.max_current/gain);
        frame(0); *regs[phase]=(uint32_t)(offsets[phase]+sign*below);
        assert(MotorSampling_Capture() && MotorSampling_IsReady());
        frame(0); *regs[phase]=(uint32_t)(offsets[phase]+sign*(below+1));
        assert(MotorSampling_Capture()); off();
        assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_OVERCURRENT && diagnostics().fault_phase==phase+1);
        MotorParams_Select(MOTOR_PROFILE_2804); sample_ms(1,1354); off();
        assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_OVERCURRENT);
        MotorParams_Select(MOTOR_PROFILE_4015);
    }
    reset(); assert(MotorSampling_Start()); sample_ms(300,1354);
    loaded_ms(10000,744); assert(MotorSampling_IsReady()); /* 1.2 A, below 4015 reference. */
    loaded_ms(4400,1116); assert(MotorSampling_IsReady()); /* ~1.8 A, 4.6 s allowance. */
    loaded_ms(400,1116); off(); assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_OVERLOAD);
    MotorParams_Select(MOTOR_PROFILE_2804);
    reset(); assert(MotorSampling_Start()); sample_ms(300,1354);
    loaded_ms(600,930);
    MotorParams_Select(MOTOR_PROFILE_4015); MotorParams_Select(MOTOR_PROFILE_2804);
    loaded_ms(650,930); off(); assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_OVERLOAD);
    /* A coast stop disconnects all six gates without stopping CH4/timers,
     * DMA or readiness. Service must never reconnect it in the background. */
    MotorParams_Select(MOTOR_PROFILE_4015);
    reset(); assert(MotorSampling_Start()); sample_ms(300,1354);
    assert(MotorSampling_IsOutputEnabled());
    uint32_t seq=MotorSampling_GetFrame()->sequence, bus_count=diagnostics().bus_samples;
    assert(MotorSampling_SetOutputEnabled(false));
    assert(!MotorSampling_IsOutputEnabled() && !MotorSampling_IsOutputRequested());
    assert((tim1.CCER & 0x555U)==0 && (tim1.CCER & TIM_CCER_CC4E));
    assert((tim1.BDTR & TIM_BDTR_MOE) && (tim1.CR1 & TIM_CR1_CEN) && (tim8.CR1 & TIM_CR1_CEN));
    sample_ms(100,1354);
    assert(MotorSampling_IsReady() && !MotorSampling_IsOutputEnabled());
    assert(MotorSampling_GetFrame()->sequence>seq && diagnostics().bus_samples>bus_count);
    assert(MotorSampling_SetOutputEnabled(true) && !MotorSampling_IsOutputEnabled());
    frame(0); assert(MotorSampling_Capture() && !MotorSampling_IsOutputEnabled());
    frame(0); assert(MotorSampling_Capture() && MotorSampling_IsOutputEnabled());
    assert(MotorSampling_SetOutputEnabled(false));
    /* Stop followed immediately by a new command and another Stop cancels
     * pending activation; the ADC ISR cannot resurrect the old request. */
    assert(MotorSampling_SetOutputEnabled(true));
    assert(MotorSampling_SetOutputEnabled(false)); sample_ms(2,1354);
    assert(!MotorSampling_IsOutputEnabled());
    /* The unchanged 2.5 A threshold also trips during coast: no blind time. */
    frame(0); adc1.JDR1=2000+(uint32_t)(MOTOR_4015.max_current/gain)+1;
    assert(MotorSampling_Capture());
    assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_OVERCURRENT);
    assert(!MotorSampling_SetOutputEnabled(true) && !MotorSampling_IsOutputRequested());
    sample_ms(2,1354); assert(!MotorSampling_IsOutputEnabled());
    MotorParams_Select(MOTOR_PROFILE_2804);
    /* Two bridges: independent arming and current-fault latches. */
    MotorParams_Select(MOTOR_PROFILE_2804);
    MotorParams_SelectAxis(MOTOR_AXIS_2,MOTOR_PROFILE_4015);
    reset(); assert(MotorSampling_Start()); sample_ms(301,1354);
    assert(MotorSampling_IsAxisReady(MOTOR_AXIS_2));
    assert(MotorSampling_SetAxisOutputEnabled(MOTOR_AXIS_1,true));
    assert(MotorSampling_SetAxisOutputEnabled(MOTOR_AXIS_2,true));
    frame(0); assert(MotorSampling_Capture()); assert(!(tim8.CCER & 0x555U));
    frame(0); assert(MotorSampling_Capture()); assert(MotorSampling_IsAxisOutputEnabled(MOTOR_AXIS_2));
    assert(MotorSampling_IsOutputEnabled());
    /* Each measured M2 phase is below 2.5 A; reconstructed W exceeds it. */
    frame(0); adc1.JDR4=2750; adc2.JDR4=2800; assert(MotorSampling_Capture());
    MotorSamplingDiagnostics second_diag; MotorSampling_GetAxisDiagnostics(MOTOR_AXIS_2,&second_diag);
    assert(second_diag.fault==MOTOR_SAMPLE_FAULT_OVERCURRENT && second_diag.fault_phase==3);
    assert(MotorSampling_IsReady() && MotorSampling_IsOutputEnabled());
    assert(!MotorSampling_IsAxisOutputEnabled(MOTOR_AXIS_2));
    assert(!MotorSampling_SetAxisOutputEnabled(MOTOR_AXIS_2,true));
    sample_ms(10,1354); assert(!MotorSampling_IsAxisOutputEnabled(MOTOR_AXIS_2));
    /* Conversely M1's fault leaves M2 available in independent mode. */
    reset(); assert(MotorSampling_Start()); sample_ms(301,1354);
    assert(MotorSampling_SetAxisOutputEnabled(MOTOR_AXIS_2,true)); sample_ms(1,1354);
    frame(0); adc1.JDR1=4095; assert(MotorSampling_Capture());
    assert(!MotorSampling_IsReady() && MotorSampling_IsAxisReady(MOTOR_AXIS_2));
    assert(MotorSampling_IsAxisOutputEnabled(MOTOR_AXIS_2));
    publish(0);
    assert(!MotorSampling_IsAxisOutputEnabled(MOTOR_AXIS_2));
    assert(!MotorSampling_SetAxisOutputEnabled(MOTOR_AXIS_2,true));
    /* M2 overload uses its own model/budget, including while coasting. */
    reset(); assert(MotorSampling_Start()); sample_ms(301,1354);
    assert(MotorSampling_SetAxisOutputEnabled(MOTOR_AXIS_2,true)); sample_ms(1,1354);
    for(unsigned m=0;m<5500;++m) {
        ++tick; publish(1354);
        for(unsigned j=0;j<10;++j) {
            frame(0); adc1.JDR4=3016; adc2.JDR4=1392;
            assert(MotorSampling_Capture());
        }
        MotorSampling_Service();
    }
    MotorSampling_GetAxisDiagnostics(MOTOR_AXIS_2,&second_diag);
    assert(second_diag.fault==MOTOR_SAMPLE_FAULT_OVERLOAD && MotorSampling_IsReady());
    puts("PASS: two-bridge independent arming/trips, reconstructed M2 W overcurrent, per-axis overload and shared-bus stop");
    puts("PASS: sampling/protection for both profiles, coast retains ADC/CH4, no automatic re-enable, fresh PWM arming delay, unchanged trip during coast and fault restart refusal");
    return 0;
}
