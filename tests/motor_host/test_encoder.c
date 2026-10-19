#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "MT6701.h"
#include "w25q64.h"

GPIO_TypeDef gpio_a={0xFFFFU},gpio_b={0xFFFFU};
SPI_TypeDef spi1,spi2,spi3;
SPI_HandleTypeDef hspi1={SPI1,HAL_SPI_STATE_READY},hspi2={SPI2,HAL_SPI_STATE_READY},hspi3={SPI3,HAL_SPI_STATE_READY};
uint32_t host_primask;
static uint32_t tick, payload=0x0FA006U, flash_cr1, flash_cr2;
static unsigned reads, aborts, display_errors, dma_starts;
static uint8_t preempt_on_tick;
static uint8_t command, *dma_rx;
static HAL_StatusTypeDef poll_result=HAL_OK;
uint32_t HAL_GetTick(void) {
    if(preempt_on_tick) { preempt_on_tick=0; MT6701_Service(); }
    return tick;
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p,uint16_t pin) { return (p->pins & pin)?GPIO_PIN_SET:GPIO_PIN_RESET; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p,uint16_t pin,GPIO_PinState state) {
    if(state==GPIO_PIN_RESET) {
        if(p==MT6701_CSN2_GPIO_Port && pin==MT6701_CSN2_Pin) assert(gpio_b.pins & W25_NSS_Pin);
        if(p==W25_NSS_GPIO_Port && pin==W25_NSS_Pin) assert(gpio_b.pins & MT6701_CSN2_Pin);
        p->pins &= ~pin;
    } else p->pins |= pin;
}
static void receive_frame(uint8_t *p) { p[0]=payload>>16; p[1]=payload>>8; p[2]=payload; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *h,const uint8_t *tx,uint8_t *rx,uint16_t n) {
    (void)tx; assert(h==&hspi1 && n==3); dma_rx=rx; ++dma_starts; return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h,const uint8_t *tx,uint8_t *rx,uint16_t n,uint32_t timeout) {
    (void)tx; assert(h==&hspi2 && n==3 && timeout==1);
    assert(!(gpio_b.pins & MT6701_CSN2_Pin) && (gpio_b.pins & W25_NSS_Pin));
    assert((spi2.CR1 & (SPI_CR1_CPOL|SPI_CR1_CPHA|SPI_CR1_BR))==(SPI_PHASE_2EDGE|SPI_BAUDRATEPRESCALER_256));
    spi2.CR1 |= SPI_CR1_SPE; spi2.CR2 ^= 0x1000U; /* HAL modifies RX FIFO threshold. */
    ++reads; receive_frame(rx); return poll_result;
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h,const uint8_t *p,uint16_t n,uint32_t timeout) {
    (void)n; (void)timeout; assert(h==&hspi2);
    assert(spi2.CR1==flash_cr1 && spi2.CR2==flash_cr2);
    assert(!(gpio_b.pins & W25_NSS_Pin) && (gpio_b.pins & MT6701_CSN2_Pin));
    command=p[0]; return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *h,uint8_t *p,uint16_t n,uint32_t timeout) {
    (void)h; (void)timeout;
    memset(p,0,n); if(command==0x9FU) { assert(n==3); p[0]=0xEF; p[1]=0x40; p[2]=0x17; }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef *h) { assert(h==&hspi1); return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *h) { assert(h==&hspi2); ++aborts; return HAL_OK; }
void OLED_DriverHandleSpiError(void) { ++display_errors; }
static void poll(void) {
    MT6701_2_Service(tick);
    assert(spi2.CR1==flash_cr1 && spi2.CR2==flash_cr2);
    assert(gpio_b.pins & MT6701_CSN2_Pin);
    assert(W25Q_ReadID()==0xEF4017U); /* A Flash transaction still uses its original mode. */
}
int main(void) {
    flash_cr1=spi2.CR1=SPI_CR1_SPE | (2U<<3) | 0x300U;
    flash_cr2=spi2.CR2=0x700U;
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
    puts("PASS: M2 SPI2 CS/mode restoration, shared W25Q64 transactions, CRC, timeout/abort, stale input, M1 DMA regression");
    return 0;
}
