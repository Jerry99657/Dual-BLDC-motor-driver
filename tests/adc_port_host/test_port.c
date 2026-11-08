#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "adc_port.h"

static ADC_TypeDef adc;
ADC_HandleTypeDef hadc1;
uint32_t host_primask;
static DMA_TypeDef controllers[2];
static DMA_Channel_TypeDef channels[2][6];
static DMA_HandleTypeDef dma;
static uint16_t buffer[4];
static unsigned halves,fulls,errors,starts,stops;
static HAL_StatusTypeDef start_result,stop_result;
static uint32_t format;

void HostWrite(void *address,size_t width,uintptr_t value)
{
    for(unsigned n=0;n<2;n++) if(address==&controllers[n].IFCR) {
        uint32_t clear=(uint32_t)value;
        for(unsigned ch=0;ch<6;ch++) if(value&(1UL<<(4U*ch))) clear|=15UL<<(4U*ch);
        controllers[n].ISR&=~clear;
        controllers[n].IFCR=(uint32_t)value;
        return;
    }
    if(width==sizeof(uintptr_t)) *(uintptr_t *)address=value;
    else {assert(width==sizeof(uint32_t));*(uint32_t *)address=(uint32_t)value;}
}
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *h)
{
    assert(h==&hadc1);++starts;
    assert(dma.Instance->CNDTR==4 && dma.Instance->CMAR==(uintptr_t)buffer);
    assert(dma.Instance->CPAR==(uintptr_t)&adc.DR);
    assert((dma.Instance->CCR&format)==format);
    assert(adc.CFGR&ADC_CFGR_DMAEN);assert(adc.IER&ADC_IER_OVRIE);
    if(start_result==HAL_OK) {
        adc.CR|=ADC_CR_ADSTART;
        /* Match HAL_ADC_Start: conversion result bits clear; DMA/internal
         * error state survives. A normal start is not a fault reset. */
        h->State&=~(HAL_ADC_STATE_READY|HAL_ADC_STATE_REG_EOC|HAL_ADC_STATE_REG_OVR|HAL_ADC_STATE_REG_EOSMP);
        h->State|=HAL_ADC_STATE_REG_BUSY;
        if(h->State&HAL_ADC_STATE_INJ_BUSY) h->ErrorCode&=~(HAL_ADC_ERROR_OVR|HAL_ADC_ERROR_DMA);
        else h->ErrorCode=HAL_ADC_ERROR_NONE;
    }
    return start_result;
}
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *h)
{
    assert(h==&hadc1);++stops;
    if(stop_result==HAL_OK) {
        adc.CR&=~ADC_CR_ADSTART;
        h->State&=~(HAL_ADC_STATE_REG_BUSY|HAL_ADC_STATE_INJ_BUSY);
        h->State|=HAL_ADC_STATE_READY;
    }
    return stop_result;
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *h)
{
    assert(h==&hadc1 && buffer[0]==1354 && buffer[1]==1500);++halves;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *h)
{
    assert(h==&hadc1 && buffer[2]==1400 && buffer[3]==1510);++fulls;
    assert(h->State&HAL_ADC_STATE_REG_EOC);
}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *h)
{
    assert(h==&hadc1);++errors;
}
static void reset(unsigned controller,unsigned channel)
{
    memset(controllers,0,sizeof(controllers));memset(channels,0,sizeof(channels));memset(&adc,0,sizeof(adc));
    dma=(DMA_HandleTypeDef){.Instance=&channels[controller][channel],.DmaBaseAddress=&controllers[controller],
        .ChannelIndex=4*channel,.Parent=&hadc1,.State=HAL_DMA_STATE_READY};
    hadc1=(ADC_HandleTypeDef){.Instance=&adc,.DMA_Handle=&dma};
    format=DMA_CCR_CIRC|DMA_CCR_MINC|DMA_CCR_PSIZE_0|DMA_CCR_MSIZE_0|DMA_CCR_PL_1;
    dma.Instance->CCR=format;adc.CFGR=ADC_CFGR_DMACFG;
    halves=fulls=errors=starts=stops=0;start_result=stop_result=HAL_OK;
    buffer[0]=1354;buffer[1]=1500;buffer[2]=1400;buffer[3]=1510;
}
static void raise(unsigned flags)
{
    dma.DmaBaseAddress->ISR|=flags<<dma.ChannelIndex;
    if(flags&8U) dma.Instance->CCR&=~DMA_CCR_EN;
    AdcPort_DMAIRQ(&dma);
}
int main(void)
{
    for(unsigned controller=0;controller<2;controller++)for(unsigned channel=0;channel<6;channel++)for(unsigned mask=0;mask<2;mask++) {
        reset(controller,channel);host_primask=mask;
        unsigned other=4U*((channel+1U)%6U);uint32_t neighbors=15UL<<other;
        dma.DmaBaseAddress->ISR=neighbors|(15UL<<dma.ChannelIndex);
        assert(AdcPort_Start(&hadc1,buffer,4)==HAL_OK);
        assert(dma.DmaBaseAddress->ISR==neighbors && host_primask==mask);
        assert(dma.State==HAL_DMA_STATE_BUSY && dma.Lock==HAL_LOCKED);
        assert(AdcPort_Start(&hadc1,buffer,4)==HAL_BUSY && starts==1);
        hadc1.State|=HAL_ADC_STATE_INJ_BUSY;
        raise(4);assert(halves==1 && fulls==0 && errors==0);
        raise(2);assert(halves==1 && fulls==1 && errors==0);
        assert((hadc1.State&HAL_ADC_STATE_INJ_BUSY) && dma.State==HAL_DMA_STATE_BUSY);
        raise(6);assert(halves==2 && fulls==2);
        AdcPort_DMAIRQ(&dma);assert(halves==2 && fulls==2);
        assert(dma.DmaBaseAddress->ISR==neighbors);
        assert(AdcPort_Stop(&hadc1)==HAL_OK && host_primask==mask);
        raise(6);assert(halves==2 && fulls==2 && !errors);
        assert(AdcPort_Start(&hadc1,buffer,4)==HAL_OK);
        raise(6);assert(halves==3 && fulls==3 && !errors);
        halves=fulls=2;
        raise(14);assert(errors==1 && halves==2 && fulls==2);
        assert(!(dma.Instance->CCR&(DMA_CCR_EN|DMA_CCR_TEIE|DMA_CCR_HTIE|DMA_CCR_TCIE)));
        assert(hadc1.State&HAL_ADC_STATE_ERROR_DMA);assert(hadc1.ErrorCode&HAL_ADC_ERROR_DMA);
        assert(dma.ErrorCode==HAL_DMA_ERROR_TE && dma.State==HAL_DMA_STATE_READY);
        assert(dma.DmaBaseAddress->ISR==neighbors);
        assert(AdcPort_Stop(&hadc1)==HAL_OK && host_primask==mask);
        raise(6);assert(halves==2 && fulls==2 && errors==1);
        assert(dma.DmaBaseAddress->ISR==neighbors);
        assert(AdcPort_Start(&hadc1,buffer,4)==HAL_OK);
        assert(hadc1.State&HAL_ADC_STATE_ERROR_DMA);
        raise(6);assert(halves==2 && fulls==2 && errors==2);
        assert(AdcPort_Stop(&hadc1)==HAL_OK);
        /* A reset/reinitialization establishes a clean ADC state. The port
         * must not clear the previous error on its own. */
        hadc1.State=HAL_ADC_STATE_READY;hadc1.ErrorCode=HAL_ADC_ERROR_NONE;
        assert(AdcPort_Start(&hadc1,buffer,4)==HAL_OK);
        raise(6);assert(halves==3 && fulls==3 && errors==2);
        assert((adc.CFGR&ADC_CFGR_DMACFG) && (dma.Instance->CCR&format)==format);
        assert(AdcPort_Stop(&hadc1)==HAL_OK);
    }
    for(unsigned result=HAL_ERROR;result<=HAL_TIMEOUT;result++) {
        reset(1,5);start_result=(HAL_StatusTypeDef)result;
        assert(AdcPort_Start(&hadc1,buffer,4)==start_result);
        assert(!(dma.Instance->CCR&DMA_CCR_EN) && !(adc.CFGR&ADC_CFGR_DMAEN) && !(adc.IER&ADC_IER_OVRIE));
        assert(dma.State==HAL_DMA_STATE_READY);
        reset(1,5);assert(AdcPort_Start(&hadc1,buffer,4)==HAL_OK);stop_result=(HAL_StatusTypeDef)result;
        assert(AdcPort_Stop(&hadc1)==stop_result);
        assert(!(dma.Instance->CCR&(DMA_CCR_EN|DMA_CCR_TEIE|DMA_CCR_HTIE|DMA_CCR_TCIE)));
        assert(!(adc.IER&ADC_IER_OVRIE) && dma.State==HAL_DMA_STATE_READY);
    }
    reset(0,0);dma.Parent=NULL;assert(AdcPort_Start(&hadc1,buffer,4)==HAL_ERROR && starts==0);
    reset(0,0);assert(AdcPort_Start(&hadc1,buffer,4)==HAL_OK);dma.Instance->CCR&=~DMA_CCR_HTIE;
    raise(4);assert(!halves && !errors);assert(dma.DmaBaseAddress->ISR&4U);
    hadc1.State|=HAL_ADC_STATE_ERROR_INTERNAL;raise(2);assert(errors==1 && !fulls);
    puts("PASS: ADC regular DMA follows both controllers/all 12 channel routes, preserves generated format/neighbors, paired HT/TC, coalescing, TE-before-data with hardware EN clear, stale IRQ retirement, normal restart, HAL error-state retention, HAL failures and PRIMASK");
}
