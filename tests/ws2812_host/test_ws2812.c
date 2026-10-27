#include "ws2812.h"
#include "tim.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#else
#define _GNU_SOURCE
#include <sys/mman.h>
#endif

static DMA_HandleTypeDef dma;
TIM_HandleTypeDef htim3={.Instance=TIM3,.hdma={&dma}};
uint32_t host_primask;
static uint32_t tick,active_ccr;
static unsigned starts,transfers,high_width[24],rise_at[24],pulses,clocks,high_ticks;
static bool output,automatic_irq=true,early;
static uint8_t power_snapshot[0xC00];
static uint32_t dma_config,cr1_config,cr2_config,ccmr_config,other_flags[2],other_ccr[12];
static DMA_Channel_TypeDef *channel(unsigned slot)
{
    return (DMA_Channel_TypeDef *)((slot<6 ? (uintptr_t)DMA1 : (uintptr_t)DMA2)+8U+20U*(slot%6U));
}
uint32_t HostRead(volatile const uint32_t *reg) { return *reg; }
void HostWrite(volatile uint32_t *reg,uint32_t value)
{
    if(reg==&DMA1->IFCR || reg==&DMA2->IFCR) {
        DMA_TypeDef *base=reg==&DMA1->IFCR ? DMA1 : DMA2;
        uint32_t clear=value;
        for(unsigned n=0;n<6;++n) if(value&(1UL<<(4U*n))) clear|=15UL<<(4U*n);
        base->ISR&=~clear;
    } else {
        uint32_t before=*reg;*reg=value;
        if(reg==&TIM3->EGR && (value&TIM_EGR_UG)) { active_ccr=TIM3->CCR1;TIM3->CNT=0;TIM3->SR|=TIM_SR_UIF; }
        if(reg==&TIM3->CR1 && !(before&TIM_CR1_CEN) && (value&TIM_CR1_CEN)) {
            assert(dma.Instance->CCR&DMA_CCR_EN);
            assert((dma.Instance->CCR&(DMA_CCR_TCIE|DMA_CCR_TEIE|DMA_CCR_HTIE))==(DMA_CCR_TCIE|DMA_CCR_TEIE));
            assert(TIM3->DIER&TIM_DIER_CC1DE);assert(TIM3->CCER&TIM_CCER_CC1E);
            assert(dma.Instance->CPAR==(uint32_t)(uintptr_t)&TIM3->CCR1);
            assert(dma.Instance->CNDTR==90 && !(dma.Instance->CMAR&3U));
            transfers=0;++starts;
        }
    }
}
static void step(void)
{
    if(!(TIM3->CR1&TIM_CR1_CEN)) return;
    if(TIM3->CNT==active_ccr) {
        TIM3->SR|=TIM_SR_CC1IF;
        if((TIM3->DIER&TIM_DIER_CC1DE) && (dma.Instance->CCR&DMA_CCR_EN) && dma.Instance->CNDTR) {
            unsigned width=1U<<((dma.Instance->CCR>>10U)&3U);
            uint32_t data=0;memcpy(&data,(const void *)((uintptr_t)dma.Instance->CMAR+transfers*width),width);
            HostWrite(&TIM3->CCR1,data&0xFFFFU);++transfers;--dma.Instance->CNDTR;
            if(dma.Instance->CNDTR==45) dma.DmaBaseAddress->ISR|=5UL<<dma.ChannelIndex;
            if(!dma.Instance->CNDTR) {
                dma.DmaBaseAddress->ISR|=3UL<<dma.ChannelIndex;
                if(automatic_irq && !host_primask) WS2812_DMAIRQ(&dma);
            }
        }
    }
    bool level=(TIM3->CCER&TIM_CCER_CC1E) && (TIM3->CR1&TIM_CR1_CEN) && TIM3->CNT<active_ccr;
    if(level && !output) { assert(pulses<24);rise_at[pulses]=clocks; }
    if(!level && output) { high_width[pulses++]=high_ticks;high_ticks=0; }
    if(level) ++high_ticks;
    output=level;++clocks;
    if(TIM3->CR1&TIM_CR1_CEN) {
        if(++TIM3->CNT>TIM3->ARR) { TIM3->CNT=0;active_ccr=TIM3->CCR1;TIM3->SR|=TIM_SR_UIF; }
    }
}
static void run_frame(uint32_t grb)
{
    for(unsigned n=0;n<25000 && (TIM3->CR1&TIM_CR1_CEN);++n) step();
    assert(transfers==90 && pulses==24 && !output);
    for(unsigned n=0;n<24;++n) {
        assert(high_width[n]==((grb&(1UL<<(23-n))) ? 136U : 68U));
        if(n) assert(rise_at[n]-rise_at[n-1]==213);
    }
    /* Include preload latency and IRQ timing: the physical low reset lasts
     * at least 50 us, even if the next frame starts immediately after TC. */
    assert(clocks-(rise_at[23]+high_width[23])>=50U*170U);
}
void __set_PRIMASK(uint32_t mask)
{
    host_primask=mask;
    if(early && !mask) { early=false;run_frame(0xAA5501); }
}
uint32_t HAL_GetTick(void) { return tick; }
static void clear_wave(void)
{
    clocks=pulses=high_ticks=0;output=false;
    memset(high_width,0,sizeof(high_width));memset(rise_at,0,sizeof(rise_at));
}
static void check_unchanged(void)
{
    assert(!memcmp(power_snapshot,(void *)0x40012C00UL,sizeof(power_snapshot)));
    assert((dma.Instance->CCR&~(DMA_CCR_EN|DMA_CCR_TCIE|DMA_CCR_HTIE|DMA_CCR_TEIE))==dma_config);
    assert((TIM3->CR1&~TIM_CR1_CEN)==cr1_config && TIM3->CR2==cr2_config && TIM3->CCMR1==ccmr_config);
    assert((DMA1->ISR&~(dma.DmaBaseAddress==DMA1 ? 15UL<<dma.ChannelIndex : 0U))==other_flags[0]);
    assert((DMA2->ISR&~(dma.DmaBaseAddress==DMA2 ? 15UL<<dma.ChannelIndex : 0U))==other_flags[1]);
    for(unsigned s=0;s<12;++s) if(channel(s)!=dma.Instance) assert(channel(s)->CCR==other_ccr[s]);
    for(unsigned n=0;n<12;++n) assert(((uint32_t *)0x40020800UL)[n]==0xA5A5A5A5U);
}
static void configure(unsigned slot,unsigned align,unsigned priority)
{
    memset((void *)0x40000000UL,0,0x30000U);memset(&dma,0,sizeof(dma));
    memset((void *)0x40012C00UL,0x5A,sizeof(power_snapshot));memcpy(power_snapshot,(void *)0x40012C00UL,sizeof(power_snapshot));
    memset((void *)0x40020800UL,0xA5,48);
    dma.Instance=channel(slot);dma.DmaBaseAddress=slot<6 ? DMA1 : DMA2;dma.ChannelIndex=4U*(slot%6U);dma.Parent=&htim3;
    dma.Init.MemDataAlignment=align&1U ? DMA_MDATAALIGN_WORD : DMA_MDATAALIGN_HALFWORD;
    dma_config=DMA_CCR_DIR|DMA_CCR_MINC|(align&1U ? DMA_CCR_MSIZE_1 : DMA_CCR_MSIZE_0)|
               (align&2U ? DMA_CCR_PSIZE_1 : DMA_CCR_PSIZE_0)|(priority<<12U);
    for(unsigned s=0;s<12;++s) { channel(s)->CCR=0xA800U+s;other_ccr[s]=channel(s)->CCR; }
    dma.Instance->CCR=dma_config;
    DMA1->ISR=DMA2->ISR=0x00FFFFFFU;dma.DmaBaseAddress->ISR&=~(15UL<<dma.ChannelIndex);
    other_flags[0]=DMA1->ISR;other_flags[1]=DMA2->ISR;
    cr1_config=TIM3->CR1=TIM_CR1_ARPE;cr2_config=TIM3->CR2=TIM_CR2_MMS_0;
    ccmr_config=TIM3->CCMR1=TIM_CCMR1_OC1PE|TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2;
    TIM3->ARR=212;active_ccr=tick=0;automatic_irq=true;early=false;host_primask=0;clear_wave();
}
static void check_matrix(void)
{
    for(unsigned s=0;s<12;++s) for(unsigned a=0;a<4;++a) for(unsigned p=0;p<4;++p) {
        configure(s,a,p);WS2812_Init();assert(WS2812_IsBusy());run_frame(0);assert(!WS2812_IsBusy());
        clear_wave();WS2812_SetRgb(0,0x55,0xAA,0x01);WS2812_Show();unsigned before=starts;
        WS2812_SetRgb(0,0x12,0x34,0x56);WS2812_Show();assert(starts==before);run_frame(0xAA5501);
        clear_wave();WS2812_Service();assert(starts==before+1);run_frame(0x341256);check_unchanged();
        clear_wave();WS2812_SetRgb(0,0x55,0xAA,1);early=true;WS2812_Show();assert(!WS2812_IsBusy());
        check_unchanged();
    }
    puts("PASS: real ST LL WS2812: 192 DMA1/2 channel/width/priority combinations, physical PWM GRB pulses and >=50 us reset, preload timing, pending/latest colors, early IRQ, other DMA/DMAMUX and motor timer isolation");
}
static void check_errors(void)
{
    configure(8,0,3);WS2812_Init();automatic_irq=false;run_frame(0);assert(WS2812_IsBusy());
    WS2812_Service();assert(!WS2812_IsBusy()); /* Delayed/lost IRQ recovered by service. */
    for(unsigned fault=0;fault<3;++fault) {
        clear_wave();WS2812_Show();
        dma.DmaBaseAddress->ISR|=(fault==0 ? 8UL : fault==1 ? 10UL : 2UL)<<dma.ChannelIndex;
        /* A TC with a nonzero remaining count is rejected as a truncated frame. */
        tick=UINT32_MAX;WS2812_DMAIRQ(&dma);assert(WS2812_IsBusy());
        assert(TIM3->CCR1==0 && active_ccr==0 && !(TIM3->CR1&TIM_CR1_CEN) && !(TIM3->CCER&TIM_CCER_CC1E));
        assert(!(dma.Instance->CCR&(DMA_CCR_EN|DMA_CCR_TEIE|DMA_CCR_TCIE|DMA_CCR_HTIE)));
        unsigned before=starts;WS2812_Show();WS2812_Service();assert(starts==before);
        tick=0;WS2812_Service();assert(starts==before);tick=1;clear_wave();WS2812_Service();assert(starts==before+1);
        automatic_irq=true;run_frame(0);assert(!WS2812_IsBusy());check_unchanged();
    }
    clear_wave();WS2812_Show();
    dma.DmaBaseAddress->ISR|=8UL<<dma.ChannelIndex;dma.Instance->CCR&=~DMA_CCR_TEIE;
    WS2812_DMAIRQ(&dma);assert(TIM3->CR1&TIM_CR1_CEN); /* Disabled source cannot abort. */
    dma.Instance->CCR|=DMA_CCR_TEIE;WS2812_DMAIRQ(&dma);
    tick+=2;clear_wave();WS2812_Service();run_frame(0);
    clear_wave();host_primask=1;WS2812_Show();assert(host_primask==1);WS2812_DMAIRQ(&dma);assert(host_primask==1);
    host_primask=0;run_frame(0);WS2812_DMAIRQ(&dma);assert(!WS2812_IsBusy());
    clear_wave();WS2812_Show();WS2812_Init();clear_wave();run_frame(0);check_unchanged();
    puts("PASS: WS2812 DMA TE/TC/invalid completion, disabled/stale sources, safe low output, >=1 ms error reset/retry across tick wrap, lost IRQ service, reinitialization and PRIMASK preservation");
}
int main(void)
{
#ifdef _WIN32
    assert(VirtualAlloc((void *)0x40000000UL,0x30000U,MEM_COMMIT|MEM_RESERVE,PAGE_READWRITE)==(void *)0x40000000UL);
#else
    assert(mmap((void *)0x40000000UL,0x30000U,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED,-1,0)==(void *)0x40000000UL);
#endif
    check_matrix();check_errors();return 0;
}
