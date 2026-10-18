#include "kk_oled_driver.h"
#include "kk_oled_internal.h"

#include "main.h"
#include "spi.h"

#include <string.h>

/* ================= ST7735S 硬件绑定 ================= */
#define TFT_CS_LOW()   HAL_GPIO_WritePin(TFT3_CS_GPIO_Port, TFT3_CS_Pin, GPIO_PIN_RESET)
#define TFT_CS_HIGH()  HAL_GPIO_WritePin(TFT3_CS_GPIO_Port, TFT3_CS_Pin, GPIO_PIN_SET)
#define TFT_DC_CMD()   HAL_GPIO_WritePin(TFT3_DC_GPIO_Port, TFT3_DC_Pin, GPIO_PIN_RESET)
#define TFT_DC_DATA()  HAL_GPIO_WritePin(TFT3_DC_GPIO_Port, TFT3_DC_Pin, GPIO_PIN_SET)
#define TFT_RST_LOW()  HAL_GPIO_WritePin(TFT3_RESET_GPIO_Port, TFT3_RESET_Pin, GPIO_PIN_RESET)
#define TFT_RST_HIGH() HAL_GPIO_WritePin(TFT3_RESET_GPIO_Port, TFT3_RESET_Pin, GPIO_PIN_SET)

#define TFT_SPI_TIMEOUT_MS 100U

/* 与参考工程同款 80x160 模组：列偏移 26、行偏移 1、BGR、INVERT */
#define TFT_COL_OFFSET 26U
#define TFT_ROW_OFFSET 1U
#define TFT_FG_COLOR   0xFFFFU
#define TFT_BG_COLOR   0x0000U

/* 每块最多 8 行，1280 字节 RGB565，避免占用 25.6 KB 全屏缓冲 */
#define TFT_CHUNK_ROWS 8U

/* ST7735 命令 */
#define CMD_SWRESET 0x01U
#define CMD_SLPOUT  0x11U
#define CMD_NORON   0x13U
#define CMD_INVOFF  0x20U
#define CMD_INVON   0x21U
#define CMD_DISPOFF 0x28U
#define CMD_DISPON  0x29U
#define CMD_CASET   0x2AU
#define CMD_RASET   0x2BU
#define CMD_RAMWR   0x2CU
#define CMD_MADCTL  0x36U
#define CMD_COLMOD  0x3AU
#define CMD_FRMCTR1 0xB1U
#define CMD_FRMCTR2 0xB2U
#define CMD_FRMCTR3 0xB3U
#define CMD_INVCTR  0xB4U
#define CMD_PWCTR1  0xC0U
#define CMD_PWCTR2  0xC1U
#define CMD_PWCTR3  0xC2U
#define CMD_PWCTR4  0xC3U
#define CMD_PWCTR5  0xC4U
#define CMD_VMCTR1  0xC5U
#define CMD_GMCTRP1 0xE0U
#define CMD_GMCTRN1 0xE1U

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint16_t delay_ms;
    uint8_t data[16];
} InitCmd;

static const InitCmd s_init_cmds[] = {
    {CMD_SWRESET, 0, 150, {0}},
    {CMD_SLPOUT,  0, 255, {0}},
    {CMD_FRMCTR1, 3, 0, {0x01, 0x2C, 0x2D}},
    {CMD_FRMCTR2, 3, 0, {0x01, 0x2C, 0x2D}},
    {CMD_FRMCTR3, 6, 0, {0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D}},
    {CMD_INVCTR,  1, 0, {0x07}},
    {CMD_PWCTR1,  3, 0, {0xA2, 0x02, 0x84}},
    {CMD_PWCTR2,  1, 0, {0xC5}},
    {CMD_PWCTR3,  2, 0, {0x0A, 0x00}},
    {CMD_PWCTR4,  2, 0, {0x8A, 0x2A}},
    {CMD_PWCTR5,  2, 0, {0x8A, 0xEE}},
    {CMD_VMCTR1,  1, 0, {0x0E}},
    {CMD_COLMOD,  1, 0, {0x05}},
    {CMD_GMCTRP1, 16, 0, {0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D,
                          0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10}},
    {CMD_GMCTRN1, 16, 0, {0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
                          0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10}},
    {CMD_NORON,   0, 10, {0}},
};

/* RGB565 大端发送缓冲，最多 8 行 */
static uint16_t s_chunk[OLED_PHYSICAL_WIDTH * TFT_CHUNK_ROWS];
static const uint16_t s_fg = TFT_FG_COLOR;
static const uint16_t s_bg = TFT_BG_COLOR;

/* DMA 状态机 */
static volatile uint8_t s_dma_active;
static volatile uint8_t s_completion_pending;
static volatile OLED_Status s_completion_status;
static uint16_t s_rect_x0;
static uint16_t s_rect_x1;
static uint16_t s_dma_y;
static uint16_t s_dma_end_y;
static uint16_t s_dma_rows;

static bool tft_write_cmd(uint8_t cmd)
{
    return HAL_SPI_Transmit(&hspi3, &cmd, 1, TFT_SPI_TIMEOUT_MS) == HAL_OK;
}

static bool tft_write_data(const uint8_t *data, uint16_t len)
{
    return HAL_SPI_Transmit(&hspi3, (uint8_t *)data, len,
                            TFT_SPI_TIMEOUT_MS) == HAL_OK;
}

static bool tft_cmd(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    TFT_CS_LOW();
    TFT_DC_CMD();
    bool ok = tft_write_cmd(cmd);
    if (ok && len > 0U) {
        TFT_DC_DATA();
        ok = tft_write_data(data, len);
    }
    TFT_CS_HIGH();
    return ok;
}

/* 设置窗口并发送 RAMWR；成功后保持 CS 低，调用者继续发像素，最后拉高 CS。 */
static bool tft_begin_window(uint16_t x0, uint16_t y0,
                             uint16_t x1, uint16_t y1)
{
    x0 += TFT_COL_OFFSET;
    x1 += TFT_COL_OFFSET;
    y0 += TFT_ROW_OFFSET;
    y1 += TFT_ROW_OFFSET;

    const uint8_t caset[4] = {
        (uint8_t)(x0 >> 8), (uint8_t)x0,
        (uint8_t)(x1 >> 8), (uint8_t)x1
    };
    const uint8_t raset[4] = {
        (uint8_t)(y0 >> 8), (uint8_t)y0,
        (uint8_t)(y1 >> 8), (uint8_t)y1
    };

    TFT_CS_LOW();
    TFT_DC_CMD();
    if (!tft_write_cmd(CMD_CASET)) return false;
    TFT_DC_DATA();
    if (!tft_write_data(caset, 4U)) return false;
    TFT_DC_CMD();
    if (!tft_write_cmd(CMD_RASET)) return false;
    TFT_DC_DATA();
    if (!tft_write_data(raset, 4U)) return false;
    TFT_DC_CMD();
    if (!tft_write_cmd(CMD_RAMWR)) return false;
    TFT_DC_DATA();
    return true;
}

/* 从核心的 1-bit 页缓冲取矩形，转换为 RGB565 行块。 */
static void fill_chunk(uint16_t x0, uint16_t x1, uint16_t y, uint16_t rows)
{
    const uint8_t *fb = OLED_InternalGetTransferBuffer();
    uint16_t out = 0U;

    for (uint16_t row = 0U; row < rows; row++) {
        uint16_t yy = (uint16_t)(y + row);
        const uint8_t *page = &fb[(yy >> 3) * OLED_PHYSICAL_WIDTH];
        uint8_t mask = (uint8_t)(1U << (yy & 7U));
        for (uint16_t x = x0; x <= x1; x++) {
            s_chunk[out++] = (page[x] & mask) ? s_fg : s_bg;
        }
    }
}

static bool compute_rect(uint16_t *x0_out, uint16_t *x1_out,
                         uint16_t *y0_out, uint16_t *y1_out)
{
    uint16_t x0 = OLED_PHYSICAL_WIDTH;
    uint16_t x1 = 0U;
    uint16_t p0 = OLED_PHYSICAL_PAGES;
    uint16_t p1 = 0U;

    for (uint8_t page = 0U; page < OLED_PHYSICAL_PAGES; page++) {
        uint8_t min_x = OLED_InternalGetTransferMinX(page);
        if (min_x >= OLED_PHYSICAL_WIDTH) {
            continue;
        }
        uint8_t max_x = OLED_InternalGetTransferMaxX(page);
        if (min_x < x0) x0 = min_x;
        if (max_x > x1) x1 = max_x;
        if (page < p0) p0 = page;
        p1 = page;
    }

    if (p0 >= OLED_PHYSICAL_PAGES) {
        return false;
    }

    *x0_out = x0;
    *x1_out = x1;
    *y0_out = (uint16_t)(p0 * 8U);
    *y1_out = (uint16_t)(p1 * 8U + 7U);
    return true;
}

static bool start_next_dma(void)
{
    uint16_t remain = (uint16_t)(s_dma_end_y - s_dma_y + 1U);
    uint16_t rows = remain > TFT_CHUNK_ROWS ? TFT_CHUNK_ROWS : remain;

    fill_chunk(s_rect_x0, s_rect_x1, s_dma_y, rows);
    s_dma_rows = rows;

    uint16_t bytes = (uint16_t)((s_rect_x1 - s_rect_x0 + 1U) * rows * 2U);
    return HAL_SPI_Transmit_DMA(&hspi3, (uint8_t *)s_chunk, bytes) == HAL_OK;
}

/* ================= 驱动接口 ================= */
OLED_Status OLED_DriverInit(void)
{
    s_dma_active = 0U;
    s_completion_pending = 0U;

    /* 硬件复位 */
    TFT_CS_HIGH();
    TFT_RST_LOW();
    HAL_Delay(10);
    TFT_RST_HIGH();
    HAL_Delay(120);

    for (size_t i = 0U; i < sizeof(s_init_cmds) / sizeof(s_init_cmds[0]); i++) {
        const InitCmd *c = &s_init_cmds[i];
        if (!tft_cmd(c->cmd, c->data, c->len)) {
            return OLED_ERROR;
        }
        if (c->delay_ms != 0U) {
            HAL_Delay(c->delay_ms);
        }
    }

    /* 原生竖屏方向，BGR，打开反显 */
    const uint8_t madctl = 0x08U;
    if (!tft_cmd(CMD_MADCTL, &madctl, 1U)) return OLED_ERROR;
    if (!tft_cmd(CMD_INVON, NULL, 0U)) return OLED_ERROR;

    /* 用背景色填满屏幕，和核心初始全 0 帧保持一致 */
    if (!tft_begin_window(0U, 0U,
                          OLED_PHYSICAL_WIDTH - 1U,
                          OLED_PHYSICAL_HEIGHT - 1U)) {
        TFT_CS_HIGH();
        return OLED_ERROR;
    }
    for (uint16_t i = 0U; i < sizeof(s_chunk) / sizeof(s_chunk[0]); i++) {
        s_chunk[i] = s_bg;
    }
    for (uint16_t y = 0U; y < OLED_PHYSICAL_HEIGHT; y += TFT_CHUNK_ROWS) {
        uint16_t bytes = (uint16_t)(OLED_PHYSICAL_WIDTH * TFT_CHUNK_ROWS * 2U);
        if (!tft_write_data((const uint8_t *)s_chunk, bytes)) {
            TFT_CS_HIGH();
            return OLED_ERROR;
        }
    }
    TFT_CS_HIGH();

    if (!tft_cmd(CMD_DISPON, NULL, 0U)) return OLED_ERROR;
    HAL_Delay(20);
    return OLED_OK;
}

OLED_Status OLED_DriverWriteBlocking(void)
{
    uint16_t x0, x1, y0, y1;
    if (!compute_rect(&x0, &x1, &y0, &y1)) {
        return OLED_OK;
    }

    if (!tft_begin_window(x0, y0, x1, y1)) {
        TFT_CS_HIGH();
        return OLED_ERROR;
    }

    for (uint16_t y = y0; y <= y1; ) {
        uint16_t remain = (uint16_t)(y1 - y + 1U);
        uint16_t rows = remain > TFT_CHUNK_ROWS ? TFT_CHUNK_ROWS : remain;
        fill_chunk(x0, x1, y, rows);
        uint16_t bytes = (uint16_t)((x1 - x0 + 1U) * rows * 2U);
        if (!tft_write_data((const uint8_t *)s_chunk, bytes)) {
            TFT_CS_HIGH();
            return OLED_ERROR;
        }
        y = (uint16_t)(y + rows);
    }

    TFT_CS_HIGH();
    return OLED_OK;
}

OLED_Status OLED_DriverWriteDMA(void)
{
    if (OLED_DriverIsBusy()) {
        return OLED_BUSY;
    }

    uint16_t x0, x1, y0, y1;
    if (!compute_rect(&x0, &x1, &y0, &y1)) {
        return OLED_ERROR;
    }

    s_rect_x0 = x0;
    s_rect_x1 = x1;
    s_dma_y = y0;
    s_dma_end_y = y1;

    if (!tft_begin_window(x0, y0, x1, y1)) {
        TFT_CS_HIGH();
        return OLED_ERROR;
    }

    /* 先置 busy，避免极短传输在返回核心之前就完成中断 */
    s_dma_active = 1U;
    if (!start_next_dma()) {
        s_dma_active = 0U;
        TFT_CS_HIGH();
        return OLED_ERROR;
    }

    return OLED_OK;
}

OLED_Status OLED_DriverWriteIT(void)
{
    return OLED_UNSUPPORTED;
}

bool OLED_DriverIsBusy(void)
{
    return s_dma_active != 0U || s_completion_pending != 0U;
}

void OLED_DriverService(void)
{
    if (s_completion_pending != 0U) {
        OLED_Status status = s_completion_status;
        s_completion_pending = 0U;
        OLED_InternalTransferFinished(status);
    }
}

/* 显示刷新超时后的兜底：终止 SPI/DMA，释放片选和忙标志。
 * HAL_SPI_Abort() 内部可能回调错误处理，因此这里允许重复调用。 */
void OLED_DriverAbort(void)
{
    if (!OLED_DriverIsBusy()) {
        return;
    }

    if (s_dma_active != 0U) (void)HAL_SPI_Abort(&hspi3);
    TFT_CS_HIGH();
    s_dma_active = 0U;
    s_completion_pending = 0U;
    OLED_InternalTransferFinished(OLED_ERROR);
}

OLED_Status OLED_DriverSetContrast(uint8_t value)
{
    (void)value;
    return OLED_UNSUPPORTED;
}

OLED_Status OLED_DriverSetPowerSave(bool enable)
{
    return tft_cmd(enable ? CMD_DISPOFF : CMD_DISPON, NULL, 0U)
        ? OLED_OK : OLED_ERROR;
}

/* ================= HAL 回调 ================= */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI3 || !s_dma_active) {
        return;
    }

    s_dma_y = (uint16_t)(s_dma_y + s_dma_rows);
    if (s_dma_y > s_dma_end_y) {
        TFT_CS_HIGH();
        s_dma_active = 0U;
        /* IRQ 可能早于 OLED_UpdateDMA() 返回，直接通知会被尚未置位的
         * async_pending 吞掉。由主循环在启动完成后恰好交付一次。 */
        s_completion_status = OLED_OK;
        s_completion_pending = 1U;
        return;
    }

    if (!start_next_dma()) {
        TFT_CS_HIGH();
        s_dma_active = 0U;
        s_completion_status = OLED_ERROR;
        s_completion_pending = 1U;
    }
}

void OLED_DriverHandleSpiError(void)
{
    if (s_dma_active) {
        TFT_CS_HIGH();
        s_dma_active = 0U;
        s_completion_status = OLED_ERROR;
        s_completion_pending = 1U;
    }
}
