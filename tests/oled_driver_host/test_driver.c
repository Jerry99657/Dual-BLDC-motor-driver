#include <assert.h>
#include <stdio.h>
#include "main.h"
#include "spi.h"
#include "kk_oled_internal.h"
GPIO_TypeDef host_gpio;
SPI_HandleTypeDef hspi3={SPI3};
static uint32_t tick;
static bool immediate=true,dma_pending;
static unsigned starts,fail_start;
void HAL_GPIO_WritePin(GPIO_TypeDef *port,uint16_t pin,GPIO_PinState state) {
    if(state==GPIO_PIN_SET) port->pins|=pin; else port->pins&=~pin;
}
void HAL_Delay(uint32_t delay) { tick+=delay; }
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi,uint8_t *data,uint16_t length,uint32_t timeout) {
    (void)spi;(void)data;(void)length;(void)timeout;return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *spi,uint8_t *data,uint16_t length) {
    (void)data; assert(length>0); ++starts;
    if(starts==fail_start) return HAL_ERROR;
    dma_pending=true;
    if(immediate) { dma_pending=false; HAL_SPI_TxCpltCallback(spi); }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *spi) {
    (void)spi;dma_pending=false;return HAL_OK;
}
static void complete(void) {
    while(dma_pending) { dma_pending=false;HAL_SPI_TxCpltCallback(&hspi3); }
    OLED_DriverService();
}
int main(void) {
    assert(OLED_Init()==OLED_OK);
    OLED_DrawPixel(1,1);
    assert(OLED_UpdateDMA()==OLED_OK); /* ISR has already fired before this returns. */
    OLED_DriverService();
    if(OLED_IsBusy()) fprintf(stderr,"Early DMA completion lost: OLED stays BUSY after service\n");
    assert(!OLED_IsBusy() && OLED_GetLastStatus()==OLED_OK);
    OLED_DrawBox(0,0,80,160); starts=0;fail_start=2;
    assert(OLED_UpdateDMA()==OLED_OK); OLED_DriverService();
    assert(!OLED_IsBusy() && OLED_GetLastStatus()==OLED_ERROR);
    fail_start=0;immediate=false;
    assert(OLED_UpdateDMA()==OLED_OK && OLED_IsBusy());
    OLED_DriverService();assert(OLED_IsBusy()); complete();
    assert(!OLED_IsBusy() && OLED_GetLastStatus()==OLED_OK);
    OLED_Clear(); OLED_DrawPixel(2,2); assert(OLED_UpdateDMA()==OLED_OK);
    OLED_Abort();OLED_DriverService();assert(!OLED_IsBusy() && OLED_GetLastStatus()==OLED_ERROR);
    OLED_DrawPixel(3,3); assert(OLED_UpdateDMA()==OLED_OK);
    OLED_DriverHandleSpiError();OLED_DriverService();
    assert(!OLED_IsBusy() && OLED_GetLastStatus()==OLED_ERROR); dma_pending=false;
    starts=0;fail_start=1;
    assert(OLED_UpdateDMA()==OLED_ERROR && !OLED_IsBusy());
    fail_start=0;immediate=true;
    assert(OLED_UpdateDMA()==OLED_OK); OLED_DriverService();
    assert(!OLED_IsBusy() && OLED_GetLastStatus()==OLED_OK);
    OLED_Clear();assert(OLED_Update()==OLED_OK && !OLED_IsBusy());
    puts("PASS: real ST7735 driver, early DMA completion/error, async transfer, abort, blocking refresh");
}
