#include "adc_port.h"

#define DMA_EVENTS (DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE)

static void stop_dma(DMA_HandleTypeDef *dma)
{
    CLEAR_BIT(dma->Instance->CCR, DMA_CCR_EN | DMA_EVENTS);
    WRITE_REG(dma->DmaBaseAddress->IFCR, 15UL << dma->ChannelIndex);
    dma->State = HAL_DMA_STATE_READY;
    dma->Lock = HAL_UNLOCKED;
}

HAL_StatusTypeDef AdcPort_Start(ADC_HandleTypeDef *adc, volatile uint16_t *buffer, uint32_t count)
{
    DMA_HandleTypeDef *dma = adc->DMA_Handle;
    if (!dma || dma->Parent != adc || !buffer || !count || count > 65535U) return HAL_ERROR;
    if ((adc->Instance->CR & ADC_CR_ADSTART) || (dma->Instance->CCR & DMA_CCR_EN) ||
        dma->State == HAL_DMA_STATE_BUSY) return HAL_BUSY;

    /* Only transfer addresses/count and interrupt ownership change here.
     * CCR format, DMAMUX request and physical route come from HAL_DMA_Init. */
    stop_dma(dma);
    WRITE_REG(dma->Instance->CPAR, (uintptr_t)&adc->Instance->DR);
    WRITE_REG(dma->Instance->CMAR, (uintptr_t)buffer);
    WRITE_REG(dma->Instance->CNDTR, count);
    dma->ErrorCode = HAL_DMA_ERROR_NONE;
    dma->State = HAL_DMA_STATE_BUSY;
    dma->Lock = HAL_LOCKED;
    SET_BIT(dma->Instance->CCR, DMA_EVENTS | DMA_CCR_EN);
    __HAL_ADC_CLEAR_FLAG(adc, ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR);
    SET_BIT(adc->Instance->IER, ADC_IER_OVRIE);
    SET_BIT(adc->Instance->CFGR, ADC_CFGR_DMAEN);

    /* HAL retains ADC enable/ready delays, error state and timeout handling.
     * The application's TIM6 trigger is stopped until injection is armed. */
    HAL_StatusTypeDef status = HAL_ADC_Start(adc);
    if (status != HAL_OK) {
        CLEAR_BIT(adc->Instance->IER, ADC_IER_OVRIE);
        CLEAR_BIT(adc->Instance->CFGR, ADC_CFGR_DMAEN);
        stop_dma(dma);
    }
    return status;
}

HAL_StatusTypeDef AdcPort_Stop(ADC_HandleTypeDef *adc)
{
    /* The caller has stopped PWM/regular triggers and the injected groups.
     * HAL keeps its conversion-stop/disable timeouts. Always retire DMA,
     * including a failed ADC stop, so no stale IRQ can publish another scan. */
    HAL_StatusTypeDef status = HAL_ADC_Stop(adc);
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    CLEAR_BIT(adc->Instance->IER, ADC_IER_OVRIE);
    CLEAR_BIT(adc->Instance->CFGR, ADC_CFGR_DMAEN);
    if (adc->DMA_Handle) stop_dma(adc->DMA_Handle);
    __set_PRIMASK(mask);
    return status;
}

void AdcPort_DMAIRQ(DMA_HandleTypeDef *dma)
{
    ADC_HandleTypeDef *adc = &hadc1;
    uint32_t flags = READ_REG(dma->DmaBaseAddress->ISR) >> dma->ChannelIndex;
    uint32_t enabled = READ_REG(dma->Instance->CCR);
    /* TE clears EN in hardware. Handle it before HT/TC so a corrupted block
     * cannot refresh bus/reference validity or leave an interrupt storm. */
    if (dma != adc->DMA_Handle || dma->Parent != adc ||
        ((flags & 8U) && (enabled & DMA_CCR_TEIE))) {
        stop_dma(dma);
        dma->ErrorCode = HAL_DMA_ERROR_TE;
        adc->State |= HAL_ADC_STATE_ERROR_DMA;
        adc->ErrorCode |= HAL_ADC_ERROR_DMA;
        HAL_ADC_ErrorCallback(adc);
        return;
    }
    if (dma->State != HAL_DMA_STATE_BUSY || !(enabled & DMA_CCR_EN)) {
        stop_dma(dma);
        return;
    }
    uint32_t pending = 0U;
    if ((flags & 4U) && (enabled & DMA_CCR_HTIE)) pending |= 4U;
    if ((flags & 2U) && (enabled & DMA_CCR_TCIE)) pending |= 2U;
    if (!pending) return;
    WRITE_REG(dma->DmaBaseAddress->IFCR, pending << dma->ChannelIndex);
    if (adc->State & (HAL_ADC_STATE_ERROR_INTERNAL | HAL_ADC_STATE_ERROR_DMA)) {
        HAL_ADC_ErrorCallback(adc);
        return;
    }
    if (pending & 4U) HAL_ADC_ConvHalfCpltCallback(adc);
    if (pending & 2U) {
        adc->State |= HAL_ADC_STATE_REG_EOC;
        dma->Lock = HAL_UNLOCKED;
        HAL_ADC_ConvCpltCallback(adc);
    }
}
