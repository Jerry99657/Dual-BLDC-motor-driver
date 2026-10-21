#include "w25q64.h"

#include "main.h"
#include "spi.h"
#include "MT6701.h"

#define W25_CS_LOW()  HAL_GPIO_WritePin(W25_NSS_GPIO_Port, W25_NSS_Pin, GPIO_PIN_RESET)
#define W25_CS_HIGH() HAL_GPIO_WritePin(W25_NSS_GPIO_Port, W25_NSS_Pin, GPIO_PIN_SET)

#define W25_CMD_WRITE_ENABLE  0x06U
#define W25_CMD_READ_STATUS1  0x05U
#define W25_CMD_READ_DATA     0x03U
#define W25_CMD_PAGE_PROGRAM  0x02U
#define W25_CMD_SECTOR_ERASE  0x20U
#define W25_CMD_JEDEC_ID      0x9FU

#define W25_SPI_TIMEOUT_MS    100U
#define W25_ERASE_TIMEOUT_MS  1000U

static uint32_t s_jedec_id;
static bool s_present;

static bool w25_send_cmd_addr(uint8_t cmd, uint32_t addr)
{
    uint8_t hdr[4] = {
        cmd,
        (uint8_t)(addr >> 16),
        (uint8_t)(addr >> 8),
        (uint8_t)addr
    };
    return HAL_SPI_Transmit(&hspi2, hdr, sizeof(hdr),
                            W25_SPI_TIMEOUT_MS) == HAL_OK;
}

static bool w25_write_enable(void)
{
    uint8_t cmd = W25_CMD_WRITE_ENABLE;
    if (!MT6701_2_FlashBegin()) return false;
    W25_CS_LOW();
    bool ok = HAL_SPI_Transmit(&hspi2, &cmd, 1,
                               W25_SPI_TIMEOUT_MS) == HAL_OK;
    W25_CS_HIGH();
    MT6701_2_FlashEnd();
    return ok;
}

bool W25Q_PollBusy(bool *busy)
{
    uint8_t cmd = W25_CMD_READ_STATUS1;
    uint8_t status;
    if (busy == NULL) return false;
    if (!MT6701_2_FlashBegin()) return false;
    W25_CS_LOW();
    bool ok = HAL_SPI_Transmit(&hspi2, &cmd, 1U, W25_SPI_TIMEOUT_MS) == HAL_OK;
    if (ok) ok = HAL_SPI_Receive(&hspi2, &status, 1U, W25_SPI_TIMEOUT_MS) == HAL_OK;
    W25_CS_HIGH();
    MT6701_2_FlashEnd();
    if (ok) *busy = (status & 1U) != 0U;
    return ok;
}

bool W25Q_WaitBusy(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    do {
        bool busy;
        if (!W25Q_PollBusy(&busy)) return false;
        if (!busy) return true;
    } while (HAL_GetTick() - start < timeout_ms);
    return false;
}

bool W25Q_Init(void)
{
    s_jedec_id = W25Q_ReadID();
    /* W25Q 系列厂商 ID = 0xEF；容量缺省按 W25Q64 使用 */
    s_present = ((s_jedec_id >> 16) == 0xEFU);
    return s_present;
}

bool W25Q_IsPresent(void)
{
    return s_present;
}

uint32_t W25Q_ReadID(void)
{
    uint8_t cmd = W25_CMD_JEDEC_ID;
    uint8_t id[3] = {0U, 0U, 0U};

    if (!MT6701_2_FlashBegin()) return false;
    W25_CS_LOW();
    bool ok = HAL_SPI_Transmit(&hspi2, &cmd, 1,
                               W25_SPI_TIMEOUT_MS) == HAL_OK;
    if (ok) {
        ok = HAL_SPI_Receive(&hspi2, id, sizeof(id),
                             W25_SPI_TIMEOUT_MS) == HAL_OK;
    }
    W25_CS_HIGH();
    MT6701_2_FlashEnd();

    if (!ok) {
        return 0U;
    }
    return ((uint32_t)id[0] << 16) |
           ((uint32_t)id[1] << 8) |
           (uint32_t)id[2];
}

bool W25Q_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0U) {
        return false;
    }

    if (!MT6701_2_FlashBegin()) return false;
    W25_CS_LOW();
    bool ok = w25_send_cmd_addr(W25_CMD_READ_DATA, addr);
    if (ok) {
        ok = HAL_SPI_Receive(&hspi2, buf, (uint16_t)len,
                             W25_SPI_TIMEOUT_MS) == HAL_OK;
    }
    W25_CS_HIGH();
    MT6701_2_FlashEnd();
    return ok;
}

bool W25Q_BeginPageProgram(uint32_t addr, const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U || len > W25Q64_PAGE_SIZE) {
        return false;
    }
    if ((addr % W25Q64_PAGE_SIZE) + len > W25Q64_PAGE_SIZE) {
        return false;
    }
    if (!w25_write_enable()) {
        return false;
    }

    if (!MT6701_2_FlashBegin()) return false;
    W25_CS_LOW();
    bool ok = w25_send_cmd_addr(W25_CMD_PAGE_PROGRAM, addr);
    if (ok) {
        ok = HAL_SPI_Transmit(&hspi2, (uint8_t *)data, len,
                              W25_SPI_TIMEOUT_MS) == HAL_OK;
    }
    W25_CS_HIGH();
    MT6701_2_FlashEnd();

    return ok;
}

bool W25Q_BeginSectorErase(uint32_t addr)
{
    if (!w25_write_enable()) {
        return false;
    }

    if (!MT6701_2_FlashBegin()) return false;
    W25_CS_LOW();
    bool ok = w25_send_cmd_addr(W25_CMD_SECTOR_ERASE, addr);
    W25_CS_HIGH();
    MT6701_2_FlashEnd();

    return ok;
}

bool W25Q_PageProgram(uint32_t addr, const uint8_t *data, uint16_t len)
{
    return W25Q_BeginPageProgram(addr, data, len) && W25Q_WaitBusy(10U);
}

bool W25Q_SectorErase(uint32_t addr)
{
    return W25Q_BeginSectorErase(addr) && W25Q_WaitBusy(W25_ERASE_TIMEOUT_MS);
}
