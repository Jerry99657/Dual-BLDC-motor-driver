#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "MT6701.h"
#include "w25q64.h"
#include "app_storage.h"

GPIO_TypeDef gpio_a={0xFFFFU},gpio_b={0xFFFFU};
SPI_TypeDef spi1,spi2,spi3;
SPI_HandleTypeDef hspi1={SPI1,HAL_SPI_STATE_READY},hspi2={SPI2,HAL_SPI_STATE_READY},hspi3={SPI3,HAL_SPI_STATE_READY};
uint32_t host_primask;
static uint32_t tick, payload=0x0FA006U, flash_cr1, flash_cr2;
static unsigned reads, aborts, display_errors, dma_starts;
static uint8_t preempt_on_tick;
static uint8_t command, *dma_rx, *dma2_rx;
static unsigned dma2_starts;
static bool early2,preempt2,flash_probe;
static HAL_StatusTypeDef poll_result=HAL_OK;
static HAL_StatusTypeDef dma_result=HAL_OK;
static uint8_t flash[3U*W25Q64_SECTOR_SIZE];
static uint32_t flash_addr, flash_busy_until;
static bool write_enabled;
uint32_t HAL_GetTick(void) {
    if(preempt2) { preempt2=false; MT6701_2_FastService(); }
    if(preempt_on_tick) { preempt_on_tick=0; MT6701_Service(); }
    return tick;
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p,uint16_t pin) { return (p->pins & pin)?GPIO_PIN_SET:GPIO_PIN_RESET; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p,uint16_t pin,GPIO_PinState state) {
    if(state==GPIO_PIN_RESET) {
        if(p==MT6701_CSN2_GPIO_Port && pin==MT6701_CSN2_Pin) assert(gpio_b.pins & W25_NSS_Pin);
        if(p==W25_NSS_GPIO_Port && pin==W25_NSS_Pin) assert(gpio_b.pins & MT6701_CSN2_Pin);
        if(p==W25_NSS_GPIO_Port && pin==W25_NSS_Pin) {
            command=0;
            if (flash_probe) { unsigned before=dma2_starts; MT6701_2_FastService(); assert(dma2_starts==before); }
        }
        p->pins &= ~pin;
    } else p->pins |= pin;
}
static void receive_frame(uint8_t *p) { p[0]=payload>>16; p[1]=payload>>8; p[2]=payload; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *h,const uint8_t *tx,uint8_t *rx,uint16_t n) {
    (void)tx; assert(n==3);
    if (h==&hspi2) {
        assert(!(gpio_b.pins & MT6701_CSN2_Pin) && (gpio_b.pins & W25_NSS_Pin));
        assert((spi2.CR1 & (SPI_CR1_CPOL|SPI_CR1_CPHA|SPI_CR1_BR))==(SPI_PHASE_2EDGE|SPI_BAUDRATEPRESCALER_256));
        dma2_rx=rx; ++dma2_starts; h->State=2;
        spi2.CR2 ^= 0x1000U;
        if (early2) { receive_frame(rx); h->State=HAL_SPI_STATE_READY; HAL_SPI_TxRxCpltCallback(h); }
        return dma_result;
    }
    assert(h==&hspi1); dma_rx=rx; ++dma_starts; return dma_result;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h,const uint8_t *tx,uint8_t *rx,uint16_t n,uint32_t timeout) {
    (void)tx; assert(h==&hspi2 && n==3 && timeout==1);
    assert(!(gpio_b.pins & MT6701_CSN2_Pin) && (gpio_b.pins & W25_NSS_Pin));
    assert((spi2.CR1 & (SPI_CR1_CPOL|SPI_CR1_CPHA|SPI_CR1_BR))==(SPI_PHASE_2EDGE|SPI_BAUDRATEPRESCALER_256));
    spi2.CR1 |= SPI_CR1_SPE; spi2.CR2 ^= 0x1000U; /* HAL modifies RX FIFO threshold. */
    ++reads; receive_frame(rx);
    if(poll_result!=HAL_OK) hspi2.State=2; /* Abort must release HAL ownership too. */
    return poll_result;
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h,const uint8_t *p,uint16_t n,uint32_t timeout) {
    (void)timeout; assert(h==&hspi2);
    assert(spi2.CR1==flash_cr1 && spi2.CR2==flash_cr2);
    assert(!(gpio_b.pins & W25_NSS_Pin) && (gpio_b.pins & MT6701_CSN2_Pin));
    if(command==0) {
        assert(n==1 || n==4); command=p[0];
        if(n==4) flash_addr=((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];
        if(command==0x06U) { assert((int32_t)(tick-flash_busy_until)>=0); write_enabled=true; }
        if(command==0x20U) {
            assert(write_enabled && flash_addr+W25Q64_SECTOR_SIZE<=sizeof(flash));
            assert(flash_addr%W25Q64_SECTOR_SIZE==0);
            memset(flash+flash_addr,255,W25Q64_SECTOR_SIZE);
            write_enabled=false; flash_busy_until=tick+40U;
        }
    } else {
        assert(command==0x02U && write_enabled && flash_addr+n<=sizeof(flash));
        assert(flash_addr%W25Q64_PAGE_SIZE+n<=W25Q64_PAGE_SIZE);
        for(unsigned i=0;i<n;++i) flash[flash_addr+i]&=p[i];
        write_enabled=false; flash_busy_until=tick+3U;
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *h,uint8_t *p,uint16_t n,uint32_t timeout) {
    (void)timeout; assert(h==&hspi2);
    memset(p,0,n); if(command==0x9FU) { assert(n==3); p[0]=0xEF; p[1]=0x40; p[2]=0x17; }
    else if(command==0x05U) { assert(n==1); p[0]=(int32_t)(tick-flash_busy_until)<0; }
    else if(command==0x03U) {
        assert((int32_t)(tick-flash_busy_until)>=0 && flash_addr+n<=sizeof(flash));
        memcpy(p,flash+flash_addr,n);
    } else assert(0);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef *h) { assert(h==&hspi1 || h==&hspi2); return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *h) {
    assert(h==&hspi2); ++aborts; h->State=HAL_SPI_STATE_READY; return HAL_OK;
}
void OLED_DriverHandleSpiError(void) { ++display_errors; }
static void poll(void) {
    MT6701_2_Service(tick);
    assert(spi2.CR1==flash_cr1 && spi2.CR2==flash_cr2);
    assert(gpio_b.pins & MT6701_CSN2_Pin);
    assert(W25Q_ReadID()==0xEF4017U); /* A Flash transaction still uses its original mode. */
}
static void check_absent_encoder(const char *scenario) {
    bool timeout=strcmp(scenario,"timeout")==0;
    if(strcmp(scenario,"missing-wrap")==0) tick=UINT32_MAX-2U;
    else assert(strcmp(scenario,"missing")==0 || timeout);
    flash_busy_until=tick;
    assert(AppStorage_Init());
    AppStorage_Get()->speed_x10=123;
    assert(AppStorage_RequestSave());
    payload=0xFFFFFFU; poll_result=timeout?HAL_TIMEOUT:HAL_OK;
    for(unsigned i=0;i<1000;++i) {
        ++tick;
        AppStorage_Service(tick); /* Same ordering as main.c. */
        poll();
    }
    MT6701_2_Snapshot out; MT6701_2_GetSnapshot(&out);
    assert(reads==5 && out.samples==0 && out.errors==5 && !out.healthy);
    assert(aborts==(timeout?5U:0U) && hspi2.State==HAL_SPI_STATE_READY);
    bool success; assert(AppStorage_PollSaveResult(&success) && success);
    assert(AppStorage_Init() && AppStorage_Get()->speed_x10==123);
    /* Another actual save/readback while ENC2 is latched off; no SPI2 reprobe. */
    poll_result=HAL_OK; payload=0x0FA006U;
    AppStorage_Get()->speed_x10=456;
    assert(AppStorage_RequestSave());
    for(unsigned i=0;i<100;++i) { ++tick; AppStorage_Service(tick); poll(); }
    assert(reads==5);
    assert(AppStorage_PollSaveResult(&success) && success);
    assert(AppStorage_Init() && AppStorage_Get()->speed_x10==456);
    printf("PASS: ENC2 %s stops after 5 failures; real W25Q64 + AppStorage saves/readback/reload, CS/HAL/mode release\n",scenario);
}
int main(int argc,char **argv) {
    flash_cr1=spi2.CR1=SPI_CR1_SPE | (2U<<3) | 0x300U;
    flash_cr2=spi2.CR2=0x700U;
    memset(flash,255,sizeof(flash));
    if(argc>1) { check_absent_encoder(argv[1]); return 0; }
    assert(W25Q_Init());
    MT6701_2_Snapshot out;
    MT6701_2_GetSnapshot(&out); assert(!out.healthy);
    tick=1; poll(); MT6701_2_GetSnapshot(&out);
    assert(out.healthy && out.raw_angle==1000 && out.samples==1 && out.errors==0);
    poll(); assert(reads==1); /* Same millisecond: no extra transaction. */
    tick=2; payload=0x1F400CU; poll(); MT6701_2_GetSnapshot(&out); assert(out.raw_angle==2000);
    tick=3; payload^=1U; poll(); MT6701_2_GetSnapshot(&out); assert(out.samples==2 && out.errors==1);
    tick=4; poll_result=HAL_TIMEOUT; poll(); MT6701_2_GetSnapshot(&out);
    assert(out.errors==2 && aborts==1 && out.raw_angle==2000);
    tick=8; MT6701_2_GetSnapshot(&out); assert(!out.healthy);
    poll_result=HAL_OK; payload=0x2EE03AU;
    hspi2.State=2; MT6701_2_Service(++tick); assert(reads==4);
    hspi2.State=HAL_SPI_STATE_READY; spi2.SR=SPI_SR_BSY; MT6701_2_Service(++tick); assert(reads==4);
    spi2.SR=0; gpio_b.pins &= ~W25_NSS_Pin; MT6701_2_Service(++tick); assert(reads==4);
    gpio_b.pins |= W25_NSS_Pin; ++tick; poll(); MT6701_2_GetSnapshot(&out); assert(out.healthy && out.raw_angle==3000);
    MT6701_2_GetSnapshot(NULL);
    /* Original M1 DMA path still decodes the same CRC/status format. */
    payload=0x0FA006U; MT6701_StartContinuous(); assert(dma_rx);
    receive_frame(dma_rx); unsigned before=dma_starts; preempt_on_tick=1;
    HAL_SPI_TxRxCpltCallback(&hspi1);
    assert(dma_starts==before); /* ADC cannot reuse the DMA buffer during publication. */
    assert(MT6701_IsHealthy() && MT6701_GetRawAngle()==1000 && MT6701_GetSampleCounter()==1);
    tick+=6; assert(!MT6701_IsHealthy()); /* ADC scheduler stopped: age cycles alone cannot detect this. */
    MT6701_Service(); receive_frame(dma_rx); HAL_SPI_TxRxCpltCallback(&hspi1); assert(MT6701_IsHealthy());
    for(unsigned i=0;i<4;++i) MT6701_Service();
    assert(!MT6701_IsHealthy());
    payload^=1U; receive_frame(dma_rx); HAL_SPI_TxRxCpltCallback(&hspi1); assert(MT6701_GetErrorCounter()==1);
    HAL_SPI_ErrorCallback(&hspi1); assert(MT6701_GetErrorCounter()==2);
    HAL_SPI_ErrorCallback(&hspi3); assert(display_errors==1);
    MT6701_StopContinuous(); assert(!MT6701_IsContinuous());
    /* Decode magnetic status only after validating CRC. A bad wire frame
     * cannot be reported as a weak/strong magnet just because bits flipped. */
    MT6701Diagnostics diag;
    MT6701_GetDiagnostics(NULL);
    MT6701_GetDiagnostics(&diag);
    assert(diag.errors==2 && diag.crc_errors==1 && diag.spi_errors==1 && diag.max_age_cycles>=4);
    const uint32_t bad_frames[]={0x0FA045U,0x0FA080U,0x0FA21EU,0x0FA044U,0U,0xFFFFFFU};
    MT6701_StartContinuous();
    for(unsigned i=0;i<sizeof(bad_frames)/sizeof(bad_frames[0]);++i) {
        MT6701_Service(); payload=bad_frames[i]; receive_frame(dma_rx);
        HAL_SPI_TxRxCpltCallback(&hspi1);
    }
    MT6701_GetDiagnostics(&diag);
    assert(diag.errors==8 && diag.crc_errors==2 && diag.field_errors==2 && diag.track_errors==1);
    assert(diag.frame_errors==2 && diag.last_error==MT6701_ERROR_FRAME);
    /* The push-button flag is valid data, never a magnetic-field fault. */
    MT6701_Service(); payload=0x0FA10AU; receive_frame(dma_rx); HAL_SPI_TxRxCpltCallback(&hspi1);
    assert(MT6701_IsHealthy() && MT6701_GetStatus()==4);
    MT6701_GetDiagnostics(&diag); assert(diag.errors==8 && diag.age_cycles==0);
    assert(strcmp(MT6701_ErrorText(MT6701_ERROR_CRC),"CRC")==0);
    assert(strcmp(MT6701_ErrorText(MT6701_ERROR_FIELD),"FIELD")==0);
    assert(strcmp(MT6701_ErrorText(MT6701_ERROR_TRACK),"TRACK")==0);
    assert(strcmp(MT6701_ErrorText(MT6701_ERROR_SPI),"SPI")==0);
    assert(strcmp(MT6701_ErrorText(MT6701_ERROR_NONE),"--")==0);
    host_primask=1; MT6701_GetDiagnostics(&diag); assert(host_primask==1); host_primask=0;
    MT6701_StopContinuous(); dma_result=HAL_ERROR; MT6701_StartContinuous();
    MT6701_GetDiagnostics(&diag); assert(diag.spi_errors==2 && (gpio_a.pins & MT6701_CSN_Pin));
    dma_result=HAL_OK; MT6701_StopContinuous();
    /* Valid samples reset consecutive failures, even after four failures. */
    payload=0;
    for(unsigned i=0;i<4;++i) { ++tick; poll(); }
    payload=0x0FA006U; ++tick; poll();
    MT6701_2_GetSnapshot(&out); assert(out.healthy);
    /* A later unplug/CRC failure is also bounded. Bus contention does not
     * count against these five actual failures. */
    hspi2.State=2;
    unsigned prior=reads;
    for(unsigned i=0;i<20;++i) MT6701_2_Service(++tick);
    assert(reads==prior); hspi2.State=HAL_SPI_STATE_READY;
    uint32_t errors=out.errors;
    payload=0xFFFFFFU;
    for(unsigned i=0;i<5;++i) { ++tick; poll(); }
    MT6701_2_GetSnapshot(&out);
    assert(!out.healthy && out.errors==errors+5U && reads==prior+5U);
    prior=reads;
    payload=0x0FA006U;
    for(unsigned i=0;i<100;++i) { ++tick; poll(); }
    MT6701_2_GetSnapshot(&out);
    assert(reads==prior && out.errors==errors+5U && !out.healthy);
    /* Real M2 DMA, including a callback before DMA-start returns. */
    MT6701_2_Retry(); MT6701_2_SetRealtime(true); payload=0x0FA006U;
    MT6701_2_FastService(); assert(dma2_starts==1 && dma2_rx);
    assert(!MT6701_2_FlashBegin());
    MT6701_2_Retry(); /* Explicit retry does not release an in-flight CS. */
    assert(!(gpio_b.pins & MT6701_CSN2_Pin));
    receive_frame(dma2_rx); hspi2.State=HAL_SPI_STATE_READY;
    unsigned before2=dma2_starts; preempt2=true;
    HAL_SPI_TxRxCpltCallback(&hspi2); assert(dma2_starts==before2);
    assert(spi2.CR1==flash_cr1 && spi2.CR2==flash_cr2);
    MT6701_2_GetSnapshot(&out); assert(out.healthy && out.raw_angle==1000);
    early2=true; MT6701_2_FastService(); early2=false;
    assert(spi2.CR1==flash_cr1 && spi2.CR2==flash_cr2);
    assert(MT6701_2_Reserve(true));
    assert(!MT6701_2_FlashBegin());
    MT6701_2_Reserve(false);
    assert(MT6701_2_FlashBegin());
    assert(!MT6701_2_Reserve(true));
    before2=dma2_starts; MT6701_2_FastService(); assert(dma2_starts==before2);
    MT6701_2_FlashEnd();
    /* Save defers while real-time output is reserved, then resumes unchanged. */
    flash_probe=true; assert(AppStorage_Init());
    AppStorage_Get()->control_config=18; AppStorage_Get()->axis2_speed_x10=321;
    assert(AppStorage_RequestSave()); assert(MT6701_2_Reserve(true));
    for(unsigned i=0;i<100;++i) AppStorage_Service(++tick);
    bool saved; assert(AppStorage_IsDeferred() && !AppStorage_PollSaveResult(&saved));
    MT6701_2_Reserve(false);
    for(unsigned i=0;i<100;++i) AppStorage_Service(++tick);
    assert(AppStorage_PollSaveResult(&saved) && saved);
    assert(AppStorage_Init());
    assert(AppStorage_Get()->control_config==18 && AppStorage_Get()->axis2_speed_x10==321);
    /* A hung DMA is cleaned up in main and releases CS/mode/HAL state. */
    MT6701_2_FastService(); assert(!(gpio_b.pins & MT6701_CSN2_Pin));
    unsigned old_aborts=aborts;
    for(unsigned i=0;i<4;++i) MT6701_2_FastService();
    MT6701_2_Service(++tick); assert(aborts==old_aborts+1);
    assert(spi2.CR1==flash_cr1 && spi2.CR2==flash_cr2);
    /* Five invalid frames stop requests until an explicit retry. */
    early2=true; payload=0;
    for(unsigned i=0;i<5;++i) MT6701_2_FastService();
    before2=dma2_starts;
    for(unsigned i=0;i<100;++i) MT6701_2_FastService();
    MT6701_2_GetSnapshot(&out); assert(out.stopped && dma2_starts==before2);
    assert(W25Q_ReadID()==0xEF4017U);
    MT6701_2_Retry(); payload=0x0FA006U; MT6701_2_FastService();
    MT6701_2_GetSnapshot(&out); assert(out.healthy && !out.stopped);
    early2=false; MT6701_2_SetRealtime(false);
    puts("PASS: real M2 10kHz DMA, early/preempted completion, Flash mutual exclusion, deferred saves, hung-DMA cleanup, five-failure stop and explicit retry");
    puts("PASS: M2 SPI2 CS/mode restoration, shared W25Q64 transactions, CRC, timeout/abort, stale input, M1 DMA regression");
    return 0;
}
