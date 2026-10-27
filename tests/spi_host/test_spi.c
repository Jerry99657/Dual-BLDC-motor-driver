#include <assert.h>
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#else
#define _GNU_SOURCE
#include <sys/mman.h>
#endif
#include "spi_port.h"

uint32_t host_primask;
static SPI_HandleTypeDef ports[3];
static DMA_HandleTypeDef rx_dma[2],tx_dma[3];
static unsigned complete[3],errors[3],resets,overrun_clears;
static uint32_t tick,tick_step,reads,inject_at,inject_error;
static bool stall,early,restart;
static bool preempt_on_quiesce,preempt_pending;
static uint8_t wire[4096];
static unsigned wire_count,received_count;
static uint8_t *const tx=(uint8_t *)0x40028000UL;
static uint8_t *const rx=(uint8_t *)0x40029000UL;
static unsigned index_of(const SPI_HandleTypeDef *h) { assert(h>=ports && h<ports+3); return (unsigned)(h-ports); }
static DMA_TypeDef *controller(unsigned port) { return port==1 ? DMA2 : DMA1; }
static unsigned tx_channel(unsigned port) { return port==2 ? 4U : 2U; }
static DMA_Channel_TypeDef *channel(DMA_TypeDef *dma,unsigned n)
{
    assert(n>=1 && n<=6); return (DMA_Channel_TypeDef *)((uintptr_t)dma+8U+(n-1U)*20U);
}
static uint32_t flags(unsigned n,unsigned value) { return (uint32_t)value<<((n-1U)*4U); }
uint32_t HostRead(volatile const uint32_t *reg)
{
    for (unsigned i=0;i<3;++i) if (reg==&ports[i].Instance->SR && ports[i].State==HAL_SPI_STATE_BUSY) {
        if (inject_at && ++reads==inject_at) ports[i].Instance->SR|=inject_error;
        if (stall) return 0U;
    }
    return *reg;
}
void HostWrite(volatile uint32_t *reg,uint32_t value)
{
    if (reg==&DMA1->IFCR || reg==&DMA2->IFCR) {
        DMA_TypeDef *dma=reg==&DMA1->IFCR ? DMA1 : DMA2;
        uint32_t clear=value;
        for (unsigned n=1;n<=6;++n) if (value&flags(n,1)) clear|=flags(n,15);
        dma->ISR&=~clear;
    } else {
        uint32_t old=*reg;*reg=value;
        if (preempt_on_quiesce && reg==&SPI1->CR2 &&
            (old&(SPI_CR2_RXDMAEN|SPI_CR2_TXDMAEN)) && !(value&(SPI_CR2_RXDMAEN|SPI_CR2_TXDMAEN))) {
            preempt_on_quiesce=false;
            if (host_primask) preempt_pending=true;
            else SpiPort_Service(ports);
        }
    }
}
void HostTransmitted(SPI_TypeDef *s,uint8_t value)
{
    assert(wire_count<sizeof(wire));wire[wire_count++]=value;
    /* A byte clocks in while being shifted out, including writes. */
    s->DR=(uint8_t)(value^0xA5U);s->SR|=SPI_SR_TXE|SPI_SR_RXNE;
}
void HostReceived(SPI_TypeDef *s)
{
    ++received_count;s->SR&=~(SPI_SR_RXNE|SPI_SR_FRLVL);
}
void HostClearOverrun(const SPI_TypeDef *s)
{
    ++overrun_clears;((SPI_TypeDef *)s)->SR&=~SPI_SR_OVR;
}
uint32_t HAL_GetTick(void) { tick+=tick_step;return tick; }
void HostResetSPI3(void) { ++resets;memset(SPI3,0,sizeof(*SPI3)); }
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h)
{
    assert(h==ports+2);h->State=HAL_SPI_STATE_READY;h->Instance->SR=SPI_SR_TXE;return HAL_OK;
}
static void finish_dma(unsigned port,bool busy)
{
    SPI_HandleTypeDef *h=ports+port;DMA_TypeDef *dma=controller(port);
    h->hdmatx->Instance->CNDTR=0U;
    unsigned owner=tx_channel(port);
    if (port!=2) {
        h->hdmarx->Instance->CNDTR=0U;owner=1;
        memcpy((void *)(uintptr_t)h->hdmarx->Instance->CMAR,(void *)(uintptr_t)h->hdmatx->Instance->CMAR,3);
    }
    dma->ISR|=flags(owner,3);
    h->Instance->SR=SPI_SR_TXE|(busy ? SPI_SR_BSY : 0U);
    SpiPort_DMAIRQ(h,owner);
}
void __set_PRIMASK(uint32_t mask)
{
    host_primask=mask;
    if (preempt_pending && !mask) { preempt_pending=false;SpiPort_Service(ports); }
    if (early && !mask) { early=false;finish_dma(0,false); }
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *h)
{
    unsigned i=index_of(h);assert(i<2 && h->State==HAL_SPI_STATE_READY);
    assert(!(h->Instance->SR&SPI_SR_BSY));++complete[i];
    if (restart) { restart=false;assert(SpiPort_ExchangeDMA(h,tx,rx,3)==HAL_OK); }
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *h)
{
    assert(h==ports+2 && h->State==HAL_SPI_STATE_READY && !(h->Instance->SR&(SPI_SR_BSY|SPI_SR_OVR)));
    ++complete[2];
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *h)
{
    assert(h->State==HAL_SPI_STATE_ERROR && h->ErrorCode);++errors[index_of(h)];
    assert(!(h->Instance->CR2&(SPI_CR2_ERRIE|SPI_CR2_TXDMAEN|SPI_CR2_RXDMAEN)));
}
static void reset(void)
{
    memset((void *)0x40000000UL,0,0x30000U);memset(ports,0,sizeof(ports));
    memset(complete,0,sizeof(complete));memset(errors,0,sizeof(errors));
    host_primask=reads=inject_at=inject_error=wire_count=received_count=resets=overrun_clears=0;
    tick=tick_step=0;stall=early=restart=false;
    preempt_on_quiesce=preempt_pending=false;
    for (unsigned i=0;i<3;++i) {
        SPI_HandleTypeDef *h=ports+i;h->Instance=i==0 ? SPI1 : i==1 ? SPI2 : SPI3;
        h->State=HAL_SPI_STATE_READY;h->Instance->SR=SPI_SR_TXE;
        h->Instance->CR1=SPI_CR1_MSTR|SPI_CR1_SSM|SPI_CR1_SSI;
        h->Instance->CR2=(7U<<SPI_CR2_DS_Pos)|SPI_CR2_FRXTH;
        DMA_TypeDef *dma=controller(i);
        tx_dma[i].Instance=channel(dma,tx_channel(i));h->hdmatx=tx_dma+i;
        h->hdmatx->Instance->CCR=DMA_CCR_MINC|DMA_CCR_DIR|DMA_CCR_PL;
        if(i<2) {
            rx_dma[i].Instance=channel(dma,1);h->hdmarx=rx_dma+i;
            h->hdmarx->Instance->CCR=DMA_CCR_MINC|DMA_CCR_PL;
        }
    }
    for(unsigned i=0;i<2048;++i) tx[i]=(uint8_t)i;
    /* ADC and WS2812 remain live, including pending IRQ flags. */
    channel(DMA1,3)->CCR=0xAA55U;channel(DMA1,5)->CCR=0xBB66U;
    DMA1->ISR=flags(3,15)|flags(5,15);
}
static void check_other_channels(void)
{
    assert(channel(DMA1,3)->CCR==0xAA55U && channel(DMA1,5)->CCR==0xBB66U);
    assert((DMA1->ISR&(flags(3,15)|flags(5,15)))==(flags(3,15)|flags(5,15)));
}
static void check_dma(void)
{
    for(unsigned i=0;i<3;++i) {
        reset();SPI_HandleTypeDef *h=ports+i;DMA_TypeDef *dma=controller(i);
        uint32_t cr1=h->Instance->CR1,cr2=h->Instance->CR2;
        host_primask=1;
        assert((i==2 ? SpiPort_WriteDMA(h,tx,1280) : SpiPort_ExchangeDMA(h,tx,rx,3))==HAL_OK);
        assert(host_primask==1);host_primask=0;
        assert(h->hdmatx->Instance->CMAR==(uint32_t)(uintptr_t)tx);
        assert(h->hdmatx->Instance->CPAR==(uint32_t)(uintptr_t)&h->Instance->DR);
        assert(h->hdmatx->Instance->CNDTR==(i==2 ? 1280U : 3U));
        assert((h->hdmatx->Instance->CCR&(DMA_CCR_MINC|DMA_CCR_DIR|DMA_CCR_PL))==(DMA_CCR_MINC|DMA_CCR_DIR|DMA_CCR_PL));
        assert(!(h->hdmatx->Instance->CCR&DMA_CCR_HTIE));
        assert((h->Instance->CR1&~SPI_CR1_SPE)==cr1);
        assert((h->Instance->CR2&~(SPI_CR2_ERRIE|SPI_CR2_RXDMAEN|SPI_CR2_TXDMAEN))==cr2);
        assert((i==2 ? SpiPort_WriteDMA(h,tx,3) : SpiPort_ExchangeDMA(h,tx,rx,3))==HAL_BUSY);
        if(i<2) {
            assert(h->hdmarx->Instance->CMAR==(uint32_t)(uintptr_t)rx);
            assert(h->hdmarx->Instance->CCR&DMA_CCR_TCIE);
            assert(!(h->hdmatx->Instance->CCR&DMA_CCR_TCIE));
            dma->ISR|=flags(2,3);SpiPort_DMAIRQ(h,2);
            assert(!complete[i] && (h->Instance->CR2&SPI_CR2_RXDMAEN));
        }
        finish_dma(i,true);assert(!complete[i] && h->State!=HAL_SPI_STATE_READY);
        /* DMA complete with BSY still high never releases the application. */
        for(unsigned n=0;n<5;++n) SpiPort_Service(h);
        assert(!complete[i]);h->Instance->SR=SPI_SR_TXE;
        if(i==2) h->Instance->SR|=SPI_SR_OVR;
        SpiPort_Service(h);assert(complete[i]==1 && !errors[i]);
        if(i==2) assert(overrun_clears==1);
        SpiPort_Service(h);SpiPort_DMAIRQ(h,i==2 ? 4 : 1);assert(complete[i]==1);
        check_other_channels();
    }
    reset();early=true;assert(SpiPort_ExchangeDMA(ports,tx,rx,3)==HAL_OK && complete[0]==1);
    reset();assert(SpiPort_ExchangeDMA(ports,tx,rx,3)==HAL_OK);restart=true;
    finish_dma(0,false);assert(complete[0]==1 && ports[0].State==HAL_SPI_STATE_BUSY_TX_RX);
    finish_dma(0,false);assert(complete[0]==2);
    reset();assert(SpiPort_ExchangeDMA(ports,tx,rx,3)==HAL_OK);
    restart=preempt_on_quiesce=true;finish_dma(0,false);
    assert(!preempt_on_quiesce && !preempt_pending && complete[0]==1);
    assert((ports[0].hdmarx->Instance->CCR&(DMA_CCR_EN|DMA_CCR_TCIE|DMA_CCR_TEIE))==(DMA_CCR_EN|DMA_CCR_TCIE|DMA_CCR_TEIE));
    assert((ports[0].hdmatx->Instance->CCR&(DMA_CCR_EN|DMA_CCR_TEIE))==(DMA_CCR_EN|DMA_CCR_TEIE));
    finish_dma(0,false);assert(complete[0]==2); /* Old IRQ never disables new DMA. */
    reset();
    for(unsigned i=0;i<2;++i) assert(SpiPort_ExchangeDMA(ports+i,tx,rx+i*4,3)==HAL_OK);
    assert(SpiPort_WriteDMA(ports+2,tx,1280)==HAL_OK);
    finish_dma(0,false);assert(complete[0]==1 && !complete[1] && !complete[2]);
    assert(ports[2].hdmatx->Instance->CCR&DMA_CCR_EN);
    finish_dma(1,false);finish_dma(2,false);assert(complete[1]==1 && complete[2]==1);check_other_channels();
    puts("PASS: actual LL SPI DMA1/2 channel addressing, RX-before-TX setup, priorities, three simultaneous ports, TC/BSY boundary, callback restart/early completion, stale IRQs and ADC/WS2812 flag isolation");
}
static void check_errors(void)
{
    for(unsigned i=0;i<3;++i) for(unsigned which=0;which<(i==2 ? 1U : 2U);++which) {
        reset();SPI_HandleTypeDef *h=ports+i;
        assert((i==2 ? SpiPort_WriteDMA(h,tx,3) : SpiPort_ExchangeDMA(h,tx,rx,3))==HAL_OK);
        unsigned n=which ? 2U : i==2 ? 4U : 1U;
        controller(i)->ISR|=flags(n,11);
        SpiPort_DMAIRQ(h,n);assert(errors[i]==1 && !complete[i] && SpiPort_GetError(h)==HAL_SPI_ERROR_DMA);
        SpiPort_DMAIRQ(h,n);assert(errors[i]==1);
        assert((i==2 ? SpiPort_WriteDMA(h,tx,3) : SpiPort_ExchangeDMA(h,tx,rx,3))==HAL_BUSY);
        if(i==2) { SpiPort_Service(h);assert(resets==1 && h->State==HAL_SPI_STATE_READY); }
        else { assert(SpiPort_Abort(h)==HAL_OK);assert(h->State==HAL_SPI_STATE_READY); }
        check_other_channels();
    }
    const uint32_t faults[]={SPI_SR_OVR,SPI_SR_MODF,SPI_SR_FRE};
    for(unsigned i=0;i<2;++i) for(unsigned n=0;n<3;++n) {
        reset();SPI_HandleTypeDef *h=ports+i;assert(SpiPort_ExchangeDMA(h,tx,rx,3)==HAL_OK);
        h->Instance->SR|=faults[n];SpiPort_IRQHandler(h);
        assert(errors[i]==1 && (h->Instance->SR&faults[n])); /* Snapshot before cleanup. */
        assert(!(h->hdmatx->Instance->CCR&DMA_CCR_EN) && !(h->hdmarx->Instance->CCR&DMA_CCR_EN));
        SpiPort_IRQHandler(h);assert(errors[i]==1);check_other_channels();
    }
    reset();ports[0].Instance->SR|=SPI_SR_OVR;SpiPort_IRQHandler(ports);assert(!errors[0]);
    assert(SpiPort_ExchangeDMA(ports,tx,rx,3)==HAL_ERROR && (SpiPort_GetError(ports)&HAL_SPI_ERROR_OVR));
    reset();assert(SpiPort_WriteDMA(ports+2,tx,3)==HAL_OK);SPI3->SR|=SPI_SR_MODF;
    SpiPort_Service(ports+2);assert(errors[2]==1);SpiPort_Service(ports+2);assert(resets==1);
    reset();host_primask=1;assert(SpiPort_Abort(ports)==HAL_OK && host_primask==1);check_other_channels();
    reset();assert(SpiPort_ExchangeDMA(ports+2,tx,rx,3)==HAL_ERROR);
    assert(SpiPort_WriteDMA(ports,tx,3)==HAL_ERROR);
    assert(SpiPort_ExchangeDMA(ports,tx,NULL,3)==HAL_ERROR);
    assert(SpiPort_ExchangeDMA(ports,NULL,rx,3)==HAL_ERROR);
    assert(SpiPort_ExchangeDMA(ports,tx,rx,0)==HAL_ERROR);
    reset();assert(SpiPort_ExchangeDMA(ports,tx,rx,3)==HAL_OK);
    DMA1->ISR|=flags(1,3);SpiPort_DMAIRQ(ports,1);
    assert(errors[0]==1 && !complete[0]); /* Never publish a partial buffer. */
    reset();assert(SpiPort_ExchangeDMA(ports,tx,rx,3)==HAL_OK);
    preempt_on_quiesce=true;DMA1->ISR|=flags(1,11);SpiPort_DMAIRQ(ports,1);
    assert(!preempt_on_quiesce && !preempt_pending && errors[0]==1 && !complete[0]);
    puts("PASS: RX/TX DMA TE dominates TC, OVR/MODF/FRE snapshots, disabled/stale vectors, no duplicate callbacks, LCD reset after errors and scoped abort/IRQ-mask preservation");
}
static void check_polling(void)
{
    const unsigned lengths[]={1,2,3,4,128,256,1280};
    for(unsigned i=0;i<3;++i) for(unsigned n=0;n<sizeof(lengths)/sizeof(lengths[0]);++n) {
        reset();unsigned length=lengths[n];SPI_HandleTypeDef *h=ports+i;
        assert(SpiPort_Exchange(h,tx,rx,(uint16_t)length,100)==HAL_OK);
        assert(wire_count==length && received_count==length && !memcmp(wire,tx,length));
        for(unsigned j=0;j<length;++j) assert(rx[j]==(uint8_t)(tx[j]^0xA5U));
        assert(h->State==HAL_SPI_STATE_READY && !h->ErrorCode);
        assert(!(h->Instance->SR&(SPI_SR_RXNE|SPI_SR_OVR)));check_other_channels();
    }
    reset();assert(SpiPort_Write(ports+2,tx,256,100)==HAL_OK && received_count==256);
    wire_count=received_count=0;assert(SpiPort_Read(ports+1,rx,256,100)==HAL_OK);
    assert(wire_count==256 && received_count==256);
    for(unsigned i=0;i<256;++i) assert(wire[i]==255 && rx[i]==0x5A);
    reset();assert(SpiPort_ExchangeDMA(ports,tx,rx,3)==HAL_OK);
    assert(SpiPort_Exchange(ports,tx,rx,3,100)==HAL_BUSY && !wire_count);
    assert(SpiPort_Write(ports,tx,0,100)==HAL_ERROR && SpiPort_Read(ports,NULL,3,100)==HAL_ERROR);
    reset();stall=true;tick=UINT32_MAX-2U;tick_step=1;
    assert(SpiPort_Exchange(ports+1,tx,rx,3,5)==HAL_TIMEOUT && !wire_count);
    assert(ports[1].State==HAL_SPI_STATE_ERROR && SpiPort_GetError(ports+1)==HAL_SPI_ERROR_FLAG);
    reset();inject_at=3;inject_error=SPI_SR_OVR;
    assert(SpiPort_Exchange(ports+1,tx,rx,4,100)==HAL_ERROR && wire_count<4);
    assert(SpiPort_GetError(ports+1)==HAL_SPI_ERROR_OVR);
    puts("PASS: real LL polling duplex/write/read 1..1280 bytes, dummy clocks, RX draining, BUSY/invalid rejection, hardware faults and timeout across tick wrap");
}
int main(void)
{
#ifdef _WIN32
    assert(VirtualAlloc((void *)0x40000000UL,0x30000U,MEM_COMMIT|MEM_RESERVE,PAGE_READWRITE)==(void *)0x40000000UL);
#else
    assert(mmap((void *)0x40000000UL,0x30000U,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED,-1,0)==(void *)0x40000000UL);
#endif
    check_dma();check_errors();check_polling();return 0;
}
