/* Exercise unchanged ST PCD/USB objects with both values of its public option.
 * Python models endpoint toggle/W0C registers and host packet completions. */
#include "stm32g4xx_hal.h"
#include <string.h>

PCD_HandleTypeDef test_pcd;
uint8_t test_tx[192],test_rx[192],test_setup[8];
uint32_t test_events[8];

void Test_Init(void)
{
    memset(&test_pcd,0,sizeof(test_pcd));
    memset(test_events,0,sizeof(test_events));
    memset(test_rx,0,sizeof(test_rx));
    test_pcd.Instance=USB;
    test_pcd.State=HAL_PCD_STATE_READY;
    test_pcd.Init.dev_endpoints=8;
    for(unsigned i=0;i<sizeof(test_tx);++i) test_tx[i]=(uint8_t)(i^0xA5U);
    HAL_PCDEx_PMAConfig(&test_pcd,0x00,PCD_SNG_BUF,0x18);
    HAL_PCDEx_PMAConfig(&test_pcd,0x80,PCD_SNG_BUF,0x58);
    HAL_PCDEx_PMAConfig(&test_pcd,0x81,PCD_SNG_BUF,0xC0);
    HAL_PCDEx_PMAConfig(&test_pcd,0x01,PCD_SNG_BUF,0x110);
    HAL_PCD_EP_Open(&test_pcd,0x00,64,EP_TYPE_CTRL);
    HAL_PCD_EP_Open(&test_pcd,0x80,64,EP_TYPE_CTRL);
    HAL_PCD_EP_Open(&test_pcd,0x81,64,EP_TYPE_BULK);
    HAL_PCD_EP_Open(&test_pcd,0x01,64,EP_TYPE_BULK);
}
unsigned Test_Tx(unsigned endpoint,unsigned length)
{ return HAL_PCD_EP_Transmit(&test_pcd,(uint8_t)(0x80U|endpoint),test_tx,length); }
unsigned Test_Rx(unsigned endpoint,unsigned length)
{ return HAL_PCD_EP_Receive(&test_pcd,(uint8_t)endpoint,test_rx,length); }
void Test_Irq(void) { HAL_PCD_IRQHandler(&test_pcd); }
void Test_Stall(unsigned endpoint) { HAL_PCD_EP_SetStall(&test_pcd,(uint8_t)endpoint); }
void Test_ClearStall(unsigned endpoint) { HAL_PCD_EP_ClrStall(&test_pcd,(uint8_t)endpoint); }
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *h,uint8_t endpoint)
{ (void)h; ++test_events[endpoint ? 0 : 2]; }
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *h,uint8_t endpoint)
{ (void)h; ++test_events[endpoint ? 1 : 3]; }
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *h)
{ ++test_events[4];memcpy(test_setup,h->Setup,sizeof(test_setup)); }
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *h) { (void)h;++test_events[5]; }
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *h) { (void)h;++test_events[6]; }
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *h) { (void)h;++test_events[7]; }
