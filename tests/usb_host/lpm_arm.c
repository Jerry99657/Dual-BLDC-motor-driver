/* Unchanged ST USB Device/CDC stack and generated descriptors; only the
 * physical USBD_LL transport is modeled to capture control replies. */
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_desc.h"

static USBD_HandleTypeDef device;
static USBD_CDC_HandleTypeDef allocated;
static uint8_t rx[128];
uint8_t test_reply[512];
uint32_t test_meta[5];
static uint32_t address;

static int8_t iface_init(void) { USBD_CDC_SetRxBuffer(&device,rx);return 0; }
static int8_t iface_deinit(void) { return 0; }
static int8_t iface_control(uint8_t cmd,uint8_t *p,uint16_t size)
{ (void)cmd;(void)p;(void)size;return 0; }
static int8_t iface_rx(uint8_t *p,uint32_t *size) { (void)p;(void)size;return 0; }
static int8_t iface_tx(uint8_t *p,uint32_t *size,uint8_t ep)
{ (void)p;(void)size;(void)ep;return 0; }
static USBD_CDC_ItfTypeDef iface={iface_init,iface_deinit,iface_control,iface_rx,iface_tx};

void Test_Init(void)
{
    memset(&device,0,sizeof(device));address=0;
    USBD_Init(&device,&CDC_Desc,DEVICE_FS);
    USBD_RegisterClass(&device,&USBD_CDC);
    USBD_CDC_RegisterInterface(&device,&iface);
    USBD_LL_SetSpeed(&device,USBD_SPEED_FULL);
    device.ep_in[0].maxpacket=device.ep_out[0].maxpacket=64;
    device.dev_state=USBD_STATE_DEFAULT;
}
void Test_Request(uint32_t low,uint32_t high)
{
    uint32_t setup[2]={low,high};
    memset(test_reply,0,sizeof(test_reply));
    test_meta[0]=UINT32_MAX;test_meta[1]=0;
    USBD_LL_SetupStage(&device,(uint8_t *)setup);
    test_meta[2]=address;test_meta[3]=device.dev_state;test_meta[4]=device.ep0_state;
}
uint32_t Test_SuspendResume(void)
{
    uint32_t result=device.dev_state;
    USBD_LL_Suspend(&device);result|=(uint32_t)device.dev_state<<8;
    USBD_LL_Resume(&device);result|=(uint32_t)device.dev_state<<16;return result;
}
void *USBD_static_malloc(uint32_t size) { return size<=sizeof(allocated) ? &allocated : NULL; }
void USBD_static_free(void *p) { (void)p; }
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *p) { (void)p;return USBD_OK; }
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *p) { (void)p;return USBD_OK; }
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *p) { (void)p;return USBD_OK; }
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *p) { (void)p;return USBD_OK; }
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *p,uint8_t ep,uint8_t type,uint16_t size)
{ (void)p;(void)ep;(void)type;(void)size;return USBD_OK; }
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *p,uint8_t ep)
{ (void)p;(void)ep;return USBD_OK; }
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *p,uint8_t ep)
{ (void)p;(void)ep;return USBD_OK; }
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *p,uint8_t ep)
{ (void)p;test_meta[1]|=ep&128U ? 2U : 1U;return USBD_OK; }
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *p,uint8_t ep)
{ (void)p;(void)ep;return USBD_OK; }
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *p,uint8_t ep)
{ (void)p;(void)ep;return 0; }
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *p,uint8_t value)
{ (void)p;address=value;return USBD_OK; }
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *p,uint8_t ep,uint8_t *data,uint32_t size)
{ (void)p;(void)ep;test_meta[0]=size;if(size<=sizeof(test_reply) && size) memcpy(test_reply,data,size);return USBD_OK; }
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *p,uint8_t ep,uint8_t *data,uint32_t size)
{ (void)p;(void)ep;(void)data;(void)size;return USBD_OK; }
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *p,uint8_t ep)
{ (void)p;(void)ep;return 0; }
void USBD_LL_Delay(uint32_t delay) { (void)delay; }
