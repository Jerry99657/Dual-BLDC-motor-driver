#include "spi_port.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_dma.h"

#define SPI_ERRORS (SPI_SR_OVR|SPI_SR_MODF|SPI_SR_FRE)
#define SPI_EVENTS (SPI_CR2_RXDMAEN|SPI_CR2_TXDMAEN|SPI_CR2_ERRIE|SPI_CR2_RXNEIE|SPI_CR2_TXEIE)

/* HAL_DMA_Init and __HAL_LINKDMA publish the actual CubeMX mapping.
 * RX and TX may be on different DMA controllers and arbitrary channels. */
static uint32_t channel(const DMA_HandleTypeDef *d) { return d->ChannelIndex>>2U; }
static uint32_t dma_flags(const DMA_HandleTypeDef *d)
{
    return LL_DMA_ReadReg(d->DmaBaseAddress,ISR)>>d->ChannelIndex;
}
static void stop_channel(DMA_HandleTypeDef *d)
{
    if (!d) return;
    LL_DMA_DisableChannel(d->DmaBaseAddress,channel(d));
    LL_DMA_DisableIT_TC(d->DmaBaseAddress,channel(d));
    LL_DMA_DisableIT_HT(d->DmaBaseAddress,channel(d));
    LL_DMA_DisableIT_TE(d->DmaBaseAddress,channel(d));
    LL_DMA_WriteReg(d->DmaBaseAddress,IFCR,15UL<<d->ChannelIndex);
}
static void quiesce(SPI_HandleTypeDef *h)
{
    CLEAR_BIT(h->Instance->CR2,SPI_EVENTS);
    stop_channel(h->hdmatx);
    stop_channel(h->hdmarx);
}
static bool dma_busy(const SPI_HandleTypeDef *h)
{
    return h->State==HAL_SPI_STATE_BUSY_TX_RX || h->State==HAL_SPI_STATE_BUSY_TX;
}
static uint32_t error_code(uint32_t sr)
{
    uint32_t error=0U;
    if (sr&SPI_SR_OVR) error|=HAL_SPI_ERROR_OVR;
    if (sr&SPI_SR_MODF) error|=HAL_SPI_ERROR_MODF;
    if (sr&SPI_SR_FRE) error|=HAL_SPI_ERROR_FRE;
    return error;
}
static void fail(SPI_HandleTypeDef *h,uint32_t error)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    h->ErrorCode=error;
    h->State=HAL_SPI_STATE_ERROR;
    quiesce(h);
    __set_PRIMASK(mask);
    /* Keep SPI SR intact until the application records its bus snapshot. */
    HAL_SPI_ErrorCallback(h);
}
uint32_t SpiPort_GetError(const SPI_HandleTypeDef *h) { return h->ErrorCode; }

void SpiPort_Service(SPI_HandleTypeDef *h)
{
    /* Only main calls LCD service after an error; DMA IRQ returns at fail.
     * Recover queued LCD bytes before a later dirty refresh reuses SPI3. */
    if (h->Instance==SPI3 && h->State==HAL_SPI_STATE_ERROR) {
        (void)SpiPort_Abort(h);
        return;
    }
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    uint32_t sr=LL_SPI_ReadReg(h->Instance,SR);
    bool active=dma_busy(h);
    uint32_t error=error_code(sr);
    if (h->Instance==SPI3) error&=~HAL_SPI_ERROR_OVR;
    if (active && error) {
        fail(h,error);
        __set_PRIMASK(mask); return;
    }
    bool done=active && !(h->Instance->CR2&(SPI_CR2_RXDMAEN|SPI_CR2_TXDMAEN)) &&
        (sr&SPI_SR_TXE) && !(sr&(SPI_SR_BSY|SPI_SR_FTLVL));
    HAL_SPI_StateTypeDef state=h->State;
    if (done) {
        quiesce(h);
        if (h->Instance==SPI3) {
            /* LCD is physically full duplex, MISO unused. Its TX-only DMA
             * may overrun RX; drain only after the final clock. */
            for (unsigned n=0;n<4U && LL_SPI_IsActiveFlag_RXNE(h->Instance);++n)
                (void)LL_SPI_ReceiveData8(h->Instance);
            LL_SPI_ClearFlag_OVR(h->Instance);
        }
        h->State=HAL_SPI_STATE_READY;
    }
    __set_PRIMASK(mask);
    if (done) {
        if (state==HAL_SPI_STATE_BUSY_TX_RX) HAL_SPI_TxRxCpltCallback(h);
        else HAL_SPI_TxCpltCallback(h);
    }
}

void SpiPort_IRQHandler(SPI_HandleTypeDef *h)
{
    if ((h->Instance->CR2&SPI_CR2_ERRIE) && dma_busy(h)) {
        uint32_t error=error_code(LL_SPI_ReadReg(h->Instance,SR));
        if (h->Instance==SPI3) error&=~HAL_SPI_ERROR_OVR;
        if (error) fail(h,error);
    }
}
void SpiPort_DMAIRQ(DMA_HandleTypeDef *d)
{
    SPI_HandleTypeDef *h=d->Parent;
    /* This is also the generated ADC/WS2812 DMA vector entry. Their HAL
     * ownership, callbacks and newly generated channel routes are intact. */
    if ((h!=&hspi1 && h!=&hspi2 && h!=&hspi3) ||
        (d!=h->hdmarx && d!=h->hdmatx)) {
        HAL_DMA_IRQHandler(d);
        return;
    }
    uint32_t enabled=d->Instance->CCR;
    if (!dma_busy(h) || !(enabled&DMA_CCR_EN)) return;
    uint32_t flags=dma_flags(d);
    uint32_t errors=dma_flags(h->hdmatx)&8U;
    if (h->hdmarx) errors|=dma_flags(h->hdmarx)&8U;
    if ((enabled&DMA_CCR_TEIE) && errors) { fail(h,HAL_SPI_ERROR_DMA); return; }
    if ((enabled&DMA_CCR_TCIE) && (flags&2U)) {
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        if (h->hdmatx->Instance->CNDTR ||
            (h->hdmarx && h->hdmarx->Instance->CNDTR)) {
            fail(h,HAL_SPI_ERROR_DMA); __set_PRIMASK(mask); return;
        }
        /* Disable BOTH linked channels atomically before ADC can publish a
         * completed frame and start the next one. No wait for BSY in IRQ. */
        quiesce(h);
        __set_PRIMASK(mask);
        SpiPort_Service(h);
    }
}

static HAL_StatusTypeDef claim(SPI_HandleTypeDef *h,HAL_SPI_StateTypeDef state)
{
    if (h->State!=HAL_SPI_STATE_READY) return HAL_BUSY;
    uint32_t sr=LL_SPI_ReadReg(h->Instance,SR);
    if (sr&(SPI_ERRORS|SPI_SR_BSY|SPI_SR_FRLVL|SPI_SR_FTLVL)) {
        h->ErrorCode=error_code(sr)|HAL_SPI_ERROR_FLAG;
        h->State=HAL_SPI_STATE_ERROR;
        return HAL_ERROR;
    }
    h->ErrorCode=HAL_SPI_ERROR_NONE;
    h->State=state;
    return HAL_OK;
}
static void setup_dma(DMA_HandleTypeDef *d,const SPI_TypeDef *spi,
                      const void *memory,uint16_t length,bool complete)
{
    stop_channel(d);
    LL_DMA_SetPeriphAddress(d->DmaBaseAddress,channel(d),LL_SPI_DMA_GetRegAddr(spi));
    LL_DMA_SetMemoryAddress(d->DmaBaseAddress,channel(d),(uint32_t)(uintptr_t)memory);
    LL_DMA_SetDataLength(d->DmaBaseAddress,channel(d),length);
    LL_DMA_EnableIT_TE(d->DmaBaseAddress,channel(d));
    if (complete) LL_DMA_EnableIT_TC(d->DmaBaseAddress,channel(d));
    LL_DMA_EnableChannel(d->DmaBaseAddress,channel(d));
}
static HAL_StatusTypeDef start_dma(SPI_HandleTypeDef *h,const uint8_t *tx,uint8_t *rx,uint16_t length)
{
    if (!tx || !length || ((h->Instance==SPI3)!=(rx==NULL))) return HAL_ERROR;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    HAL_StatusTypeDef result=claim(h,rx ? HAL_SPI_STATE_BUSY_TX_RX : HAL_SPI_STATE_BUSY_TX);
    if (result==HAL_OK) {
        quiesce(h);
        if (rx) {
            setup_dma(h->hdmarx,h->Instance,rx,length,true);
            LL_SPI_EnableDMAReq_RX(h->Instance);
        }
        setup_dma(h->hdmatx,h->Instance,tx,length,!rx);
        /* ERRIE on LCD would continuously interrupt on expected RX overrun.
         * DMA TE is still armed; MODF/FRE are checked at completion/service. */
        if (rx) LL_SPI_EnableIT_ERR(h->Instance);
        LL_SPI_Enable(h->Instance);
        LL_SPI_EnableDMAReq_TX(h->Instance); /* Publish last, RX already armed. */
    }
    __set_PRIMASK(mask);
    return result;
}
HAL_StatusTypeDef SpiPort_ExchangeDMA(SPI_HandleTypeDef *h,const uint8_t *tx,uint8_t *rx,uint16_t length)
{
    return start_dma(h,tx,rx,length);
}
HAL_StatusTypeDef SpiPort_WriteDMA(SPI_HandleTypeDef *h,const uint8_t *tx,uint16_t length)
{
    return start_dma(h,tx,NULL,length);
}

/* Main-loop polling, one outstanding byte. Always drain full-duplex RX,
 * including writes and dummy clocks for Flash reads; never synthesize OVR. */
HAL_StatusTypeDef SpiPort_Exchange(SPI_HandleTypeDef *h,const uint8_t *tx,uint8_t *rx,uint16_t length,uint32_t timeout)
{
    if ((!tx && !rx) || !length) return HAL_ERROR;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    HAL_StatusTypeDef result=claim(h,HAL_SPI_STATE_BUSY);
    __set_PRIMASK(mask);
    if (result!=HAL_OK) return result;
    uint32_t at=HAL_GetTick();
    uint16_t sent=0U,received=0U;
    LL_SPI_Enable(h->Instance);
    for (;;) {
        uint32_t sr=LL_SPI_ReadReg(h->Instance,SR);
        uint32_t error=error_code(sr);
        if (error) { h->ErrorCode=error; result=HAL_ERROR; break; }
        if (received<sent && (sr&SPI_SR_RXNE)) {
            uint8_t byte=LL_SPI_ReceiveData8(h->Instance);
            if (rx) rx[received]=byte;
            ++received;
        }
        if (sent<length && sent==received && (sr&SPI_SR_TXE)) {
            LL_SPI_TransmitData8(h->Instance,tx ? tx[sent] : 255U);
            ++sent;
        }
        if (received==length && (sr&SPI_SR_TXE) && !(sr&(SPI_SR_BSY|SPI_SR_FTLVL))) break;
        if (HAL_GetTick()-at>=timeout) {
            h->ErrorCode=HAL_SPI_ERROR_FLAG;
            result=HAL_TIMEOUT; break;
        }
    }
    h->State=result==HAL_OK ? HAL_SPI_STATE_READY : HAL_SPI_STATE_ERROR;
    return result;
}
HAL_StatusTypeDef SpiPort_Write(SPI_HandleTypeDef *h,const uint8_t *tx,uint16_t length,uint32_t timeout)
{
    if (!tx) return HAL_ERROR;
    return SpiPort_Exchange(h,tx,NULL,length,timeout);
}
HAL_StatusTypeDef SpiPort_Read(SPI_HandleTypeDef *h,uint8_t *rx,uint16_t length,uint32_t timeout)
{
    if (!rx) return HAL_ERROR;
    return SpiPort_Exchange(h,NULL,rx,length,timeout);
}
HAL_StatusTypeDef SpiPort_Abort(SPI_HandleTypeDef *h)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    quiesce(h);
    LL_SPI_Disable(h->Instance);
    /* Encoder main-service recovery already resets/reinitializes SPI1/2.
     * LCD timeout recovery also discards any queued TX data before reuse. */
    if (h->Instance==SPI3) {
        __HAL_RCC_SPI3_FORCE_RESET();
        __HAL_RCC_SPI3_RELEASE_RESET();
    }
    h->State=HAL_SPI_STATE_READY;
    h->ErrorCode=HAL_SPI_ERROR_NONE;
    __set_PRIMASK(mask);
    return h->Instance==SPI3 ? HAL_SPI_Init(h) : HAL_OK;
}
HAL_StatusTypeDef SpiPort_DMAStop(SPI_HandleTypeDef *h) { return SpiPort_Abort(h); }
