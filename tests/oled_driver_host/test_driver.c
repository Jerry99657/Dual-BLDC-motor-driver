#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "spi.h"
#include "kk_oled_internal.h"
GPIO_TypeDef host_gpio;
SPI_HandleTypeDef hspi3={SPI3};
static uint32_t tick;
static bool immediate=true,dma_pending;
static unsigned starts,fail_start;
/* Model ST7735S command/data traffic and full 132x162 GRAM. Resets do NOT
 * clear display memory (datasheet RAMWR defaults). Seed it with color noise. */
#define GRAM_WIDTH 132U
#define GRAM_HEIGHT 162U
/* User-supplied BOE module address(): CASET 0x0018..0x0067,
 * RASET 0x0000..0x009F with MADCTL=0x08. Independent of driver constants. */
#define PANEL_COL 24U
#define PANEL_ROW 0U
static uint16_t gram[GRAM_HEIGHT][GRAM_WIDTH];
static uint16_t x0,x1,y0,y1,cursor_x,cursor_y;
static uint32_t pixels_written,window_pixels;
static uint8_t command;
static bool display_on;
static uint8_t *dma_data;
static uint16_t dma_length;

static void receive_bytes(const uint8_t *data,uint16_t length) {
    assert(!(host_gpio.pins & TFT3_CS_Pin));
    if(!(host_gpio.pins & TFT3_DC_Pin)) {
        assert(length==1); command=data[0];
        if(command==0x01 || command==0x28) display_on=false;
        if(command==0x29) display_on=true;
        if(command==0x2C) {
            assert(x0<=x1 && x1<GRAM_WIDTH && y0<=y1 && y1<GRAM_HEIGHT);
            cursor_x=x0; cursor_y=y0; pixels_written=0;
            window_pixels=(uint32_t)(x1-x0+1U)*(y1-y0+1U);
        }
    } else if(command==0x2A || command==0x2B) {
        assert(length==4);
        uint16_t first=((uint16_t)data[0]<<8)|data[1];
        uint16_t last=((uint16_t)data[2]<<8)|data[3];
        if(command==0x2A) { x0=first; x1=last; }
        else { y0=first; y1=last; }
    } else if(command==0x2C) {
        assert(length>0 && (length&1U)==0);
        for(unsigned i=0;i<length;i+=2) {
            assert(pixels_written<window_pixels && cursor_y<=y1);
            uint16_t pixel=((uint16_t)data[i]<<8)|data[i+1];
            assert(pixel==0 || pixel==0xFFFFU);
            gram[cursor_y][cursor_x]=pixel; ++pixels_written;
            if(++cursor_x>x1) { cursor_x=x0; ++cursor_y; }
        }
    }
}
static void assert_cleared_gram(void) {
    assert(display_on);
    for(unsigned y=0;y<GRAM_HEIGHT;++y) for(unsigned x=0;x<GRAM_WIDTH;++x) {
        if(gram[y][x]!=0)
            fprintf(stderr,"Uninitialized LCD GRAM at (%u,%u): 0x%04X\n",x,y,gram[y][x]);
        assert(gram[y][x]==0);
    }
}
static void assert_frame(void) {
    const uint8_t *fb=OLED_InternalTestGetStableBuffer();
    for(unsigned y=0;y<OLED_PHYSICAL_HEIGHT;++y) for(unsigned x=0;x<OLED_PHYSICAL_WIDTH;++x) {
        uint16_t expected=(fb[(y/8U)*OLED_PHYSICAL_WIDTH+x] & (1U<<(y%8U)))?0xFFFFU:0;
        if(gram[y+PANEL_ROW][x+PANEL_COL]!=expected)
            fprintf(stderr,"BOE panel pixel (%u,%u) mapped incorrectly: expected 0x%04X got 0x%04X\n",
                    x,y,expected,gram[y+PANEL_ROW][x+PANEL_COL]);
        assert(gram[y+PANEL_ROW][x+PANEL_COL]==expected);
    }
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port,uint16_t pin,GPIO_PinState state) {
    if(state==GPIO_PIN_SET) port->pins|=pin; else port->pins&=~pin;
}
void HAL_Delay(uint32_t delay) { tick+=delay; }
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi,uint8_t *data,uint16_t length,uint32_t timeout) {
    (void)timeout; assert(spi==&hspi3); receive_bytes(data,length); return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *spi,uint8_t *data,uint16_t length) {
    assert(length>0); ++starts;
    if(starts==fail_start) return HAL_ERROR;
    dma_pending=true; dma_data=data; dma_length=length;
    if(immediate) { dma_pending=false; receive_bytes(data,length); HAL_SPI_TxCpltCallback(spi); }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *spi) {
    (void)spi;dma_pending=false;return HAL_OK;
}
static void complete(void) {
    while(dma_pending) {
        dma_pending=false; receive_bytes(dma_data,dma_length); HAL_SPI_TxCpltCallback(&hspi3);
    }
    OLED_DriverService();
}
int main(void) {
    for(unsigned y=0;y<GRAM_HEIGHT;++y) for(unsigned x=0;x<GRAM_WIDTH;++x) gram[y][x]=0xA55AU;
    assert(OLED_Init()==OLED_OK);
    assert_cleared_gram();
    OLED_DrawPixel(1,1);
    assert(OLED_UpdateDMA()==OLED_OK); /* ISR has already fired before this returns. */
    OLED_DriverService();
    if(OLED_IsBusy()) fprintf(stderr,"Early DMA completion lost: OLED stays BUSY after service\n");
    assert(!OLED_IsBusy() && OLED_GetLastStatus()==OLED_OK);
    assert_frame();
    OLED_DrawBox(0,0,80,160); starts=0;fail_start=2;
    assert(OLED_UpdateDMA()==OLED_OK); OLED_DriverService();
    assert(!OLED_IsBusy() && OLED_GetLastStatus()==OLED_ERROR);
    fail_start=0;immediate=false;
    assert(OLED_UpdateDMA()==OLED_OK && OLED_IsBusy());
    OLED_DriverService();assert(OLED_IsBusy()); complete();
    assert(!OLED_IsBusy() && OLED_GetLastStatus()==OLED_OK);
    assert_frame();
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
    assert_frame();
    OLED_Clear();assert(OLED_Update()==OLED_OK && !OLED_IsBusy());
    assert_frame();
    /* All four logical edges must map into native GRAM with inclusive window
     * ends; dirty and full updates must never touch pixels outside their window. */
    for(unsigned rotation=OLED_ROTATION_0;rotation<=OLED_ROTATION_270;++rotation) {
        OLED_SetRotation((OLED_Rotation)rotation);
        OLED_Clear();
        int16_t w=(int16_t)OLED_GetWidth(),h=(int16_t)OLED_GetHeight();
        OLED_DrawFrame(0,0,w,h);
        assert(OLED_UpdateDMA()==OLED_OK); OLED_DriverService();
        assert_frame(); assert(!OLED_IsBusy() && pixels_written==window_pixels);
        OLED_Clear(); assert(OLED_Update()==OLED_OK); assert_frame();
    }
    /* Recovery initialization must clear old off-window data as well. */
    gram[0][0]=gram[161][131]=0x1234U;
    assert(OLED_Init()==OLED_OK); assert_cleared_gram();
    puts("PASS: real ST7735 driver, early DMA completion/error, async transfer, abort, blocking refresh");
    puts("PASS: ST7735S full GRAM initialization, exact RGB565 window lengths, four-edge mapping at all rotations, dirty refresh and recovery clear");
}
