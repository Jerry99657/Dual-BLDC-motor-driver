#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "usbd_cdc_if.h"
#include "protocol.h"

uint32_t host_primask;
USBD_HandleTypeDef hUsbDeviceFS;
extern volatile uint8_t g_usb_cdc_ready;
static USBD_CDC_HandleTypeDef cdc;
static unsigned submits, receives, commands;
static uint8_t result;
static bool complete_early;

uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *d,uint8_t *p,uint32_t n) {
    assert(d->pClassData==&cdc);
    cdc.TxBuffer=p; cdc.TxLength=n; return USBD_OK;
}
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *d,uint8_t *p) {
    assert(d->pClassData==&cdc); cdc.RxBuffer=p; return USBD_OK;
}
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *d) {
    assert(d->dev_state==USBD_STATE_CONFIGURED && host_primask==1);
    assert(cdc.TxState==0); ++submits;
    if(result!=USBD_OK) return result;
    cdc.TxState=1;
    if(complete_early) {
        cdc.TxState=0;
        USBD_Interface_fops_FS.TransmitCplt(cdc.TxBuffer,&cdc.TxLength,1);
    }
    return USBD_OK;
}
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *d) {
    assert(d->pClassData==&cdc); ++receives; return USBD_OK;
}
void Protocol_Feed(uint8_t source,const uint8_t *p,uint32_t n) {
    assert(source==PROTO_SRC_USB && n==6 && memcmp(p,"Stop\r\n",6)==0);
    ++commands;
}
int main(void) {
    uint8_t payload[104]={0};
    assert(!CDC_TxReady_FS());
    assert(CDC_Transmit_FS(payload,sizeof(payload))==USBD_FAIL);
    hUsbDeviceFS.pClassData=&cdc;
    assert(USBD_Interface_fops_FS.Init()==USBD_OK);
    assert(g_usb_cdc_ready && !CDC_TxReady_FS()); /* Init precedes configured. */
    hUsbDeviceFS.dev_state=USBD_STATE_CONFIGURED;
    assert(CDC_TxReady_FS());
    assert(CDC_Transmit_FS(NULL,1)==USBD_FAIL);
    assert(CDC_Transmit_FS(payload,0)==USBD_FAIL);
    assert(CDC_Transmit_FS(payload,sizeof(payload))==USBD_OK && submits==1);
    assert(!CDC_TxReady_FS());
    assert(CDC_Transmit_FS(payload,sizeof(payload))==USBD_BUSY && submits==1);
    uint32_t n=6;
    USBD_Interface_fops_FS.Receive((uint8_t *)"Stop\r\n",&n);
    assert(commands==1 && receives==1); /* RX still works while TX busy. */
    cdc.TxState=0;
    USBD_Interface_fops_FS.TransmitCplt(payload,&cdc.TxLength,1);
    assert(CDC_TxReady_FS());
    complete_early=true;
    for(unsigned i=0;i<1000;++i) {
        assert(CDC_Transmit_FS(payload,sizeof(payload))==USBD_OK);
        assert(CDC_TxReady_FS());
    }
    hUsbDeviceFS.dev_state=USBD_STATE_SUSPENDED;
    assert(!CDC_TxReady_FS());
    assert(CDC_Transmit_FS(payload,sizeof(payload))==USBD_FAIL);
    hUsbDeviceFS.dev_state=USBD_STATE_CONFIGURED;
    assert(CDC_TxReady_FS());
    host_primask=1;
    assert(CDC_TxReady_FS() && host_primask==1);
    assert(CDC_Transmit_FS(payload,sizeof(payload))==USBD_OK && host_primask==1);
    host_primask=0;
    assert(USBD_Interface_fops_FS.DeInit()==USBD_OK && !g_usb_cdc_ready);
    hUsbDeviceFS.pClassData=NULL;
    assert(!CDC_TxReady_FS());
    assert(CDC_Transmit_FS(payload,sizeof(payload))==USBD_FAIL);
    hUsbDeviceFS.pClassData=&cdc;
    USBD_Interface_fops_FS.Init();
    assert(CDC_TxReady_FS());
    result=USBD_FAIL;
    assert(CDC_Transmit_FS(payload,sizeof(payload))==USBD_FAIL);
    assert(CDC_TxReady_FS() && host_primask==0);
    puts("PASS: real CDC interface guards, TX state, RX while busy, early completion, suspend/reconnect, IRQ mask preservation");
}
