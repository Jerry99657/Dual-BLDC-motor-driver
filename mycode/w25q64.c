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
static uint8_t s_discard[W25Q64_PAGE_SIZE] __attribute__((aligned(4)));
static uint8_t s_sr1;
static uint32_t s_failure;
uint32_t W25Q_LastFailure(void) { return s_failure; }
bool W25Q_LastFailureBusy(void) { return (s_failure&0xFF000001UL)==0x04000001UL; }
uint8_t W25Q_LastStatus(void) { return s_sr1; }

static bool w25_result(unsigned op,HAL_StatusTypeDef result)
{
    if (result==HAL_OK) return true;
    /* Capture before Abort clears the HAL error. SR1 is the last successful
     * status read, not a claim that it was sampled at the moment of failure. */
    s_failure=(op<<24)|((uint32_t)result<<16)|((HAL_SPI_GetError(&hspi2)&255U)<<8)|s_sr1;
    return false;
}

static bool w25_send(unsigned op,const uint8_t *data,uint16_t len)
{
    /* SPI2 is physically full duplex. Drain RX while sending commands and
     * page data; do not deliberately overflow RX with HAL_SPI_Transmit and
     * defer FIFO/OVR cleanup until after a high-priority control interrupt. */
    return w25_result(op,HAL_SPI_TransmitReceive(&hspi2,data,s_discard,len,W25_SPI_TIMEOUT_MS));
}

static bool w25_end(bool ok)
{
    W25_CS_HIGH();
    /* Keep new owners blocked until main aborts DMA and resets the peripheral.
     * The failure snapshot must precede any cleanup of HAL/SPI error flags. */
    if (!ok) MT6701_2_BusFault();
    MT6701_2_FlashEnd();
    return ok;
}

static bool w25_read_status(uint8_t *status);

static bool w25_send_cmd_addr(unsigned op,uint8_t cmd, uint32_t addr)
{
    uint8_t hdr[4] = {
        cmd,
        (uint8_t)(addr >> 16),
        (uint8_t)(addr >> 8),
        (uint8_t)addr
    };
    return w25_send(op,hdr,sizeof(hdr));
}

static bool w25_write_enable(void)
{
    uint8_t cmd = W25_CMD_WRITE_ENABLE;
    uint8_t status;
    if (!w25_read_status(&status)) return false;
    if (status&1U) { s_failure=(4UL<<24)|status; return false; }
    if (!MT6701_2_FlashBegin()) return w25_result(11,HAL_BUSY);
    W25_CS_LOW();
    bool ok = w25_send(1,&cmd,1);
    if (!w25_end(ok)) return false;
    /* HAL_OK only means 0x06 left the MCU, not that Flash accepted WREN. */
    if (!w25_read_status(&status)) return false;
    if ((status & 3U)==2U) return true;
    s_failure=(4UL<<24)|status; /* Transfer succeeded; WEL/BUSY admission did not. */
    if (!(status&1U)) MT6701_2_BusFault(); /* Resync bus before the bounded save retry. */
    return false;
}

static bool w25_read_status(uint8_t *status)
{
    uint8_t cmd = W25_CMD_READ_STATUS1;
    uint8_t samples[2]={255,255};
    if (!MT6701_2_FlashBegin()) return w25_result(11,HAL_BUSY);
    W25_CS_LOW();
    bool ok = w25_send(2,&cmd,1);
    if (ok) ok=w25_result(3,HAL_SPI_Receive(&hspi2,samples,sizeof(samples),W25_SPI_TIMEOUT_MS));
    if (ok) {
        /* SR1 repeats under the same CS. Require both samples idle, and both
         * WEL bits set; a transitioning/single bad byte must not admit a write.
         * Other bits retain the newest sample. Diagnostics use this decision. */
        *status=(samples[1]&~3U)|((samples[0]|samples[1])&1U)|((samples[0]&samples[1])&2U);
        s_sr1=*status;
    }
    return w25_end(ok);
}

bool W25Q_PollBusy(bool *busy)
{
    uint8_t status;
    if (!busy) return false;
    bool ok=w25_read_status(&status);
    if (ok) *busy=(status & 1U)!=0U;
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

    if (!MT6701_2_FlashBegin()) { (void)w25_result(11,HAL_BUSY); return 0; }
    W25_CS_LOW();
    bool ok = w25_send(8,&cmd,1);
    if (ok) {
        ok=w25_result(9,HAL_SPI_Receive(&hspi2,id,sizeof(id),W25_SPI_TIMEOUT_MS));
    }
    if (!w25_end(ok)) {
        return 0U;
    }
    return ((uint32_t)id[0] << 16) |
           ((uint32_t)id[1] << 8) |
           (uint32_t)id[2];
}

bool W25Q_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0U) {
        s_failure=10UL<<24; return false;
    }

    if (!MT6701_2_FlashBegin()) return w25_result(11,HAL_BUSY);
    W25_CS_LOW();
    bool ok = w25_send_cmd_addr(8,W25_CMD_READ_DATA, addr);
    if (ok) {
        ok=w25_result(9,HAL_SPI_Receive(&hspi2,buf,(uint16_t)len,W25_SPI_TIMEOUT_MS));
    }
    return w25_end(ok);
}

bool W25Q_BeginPageProgram(uint32_t addr, const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U || len > W25Q64_PAGE_SIZE) {
        s_failure=10UL<<24; return false;
    }
    if ((addr % W25Q64_PAGE_SIZE) + len > W25Q64_PAGE_SIZE) {
        s_failure=10UL<<24; return false;
    }
    if (!w25_write_enable()) {
        return false;
    }

    if (!MT6701_2_FlashBegin()) return w25_result(11,HAL_BUSY);
    W25_CS_LOW();
    bool ok = w25_send_cmd_addr(5,W25_CMD_PAGE_PROGRAM, addr);
    if (ok) {
        ok=w25_send(6,data,len);
    }
    return w25_end(ok);
}

bool W25Q_BeginSectorErase(uint32_t addr)
{
    if (!w25_write_enable()) {
        return false;
    }

    if (!MT6701_2_FlashBegin()) return w25_result(11,HAL_BUSY);
    W25_CS_LOW();
    bool ok = w25_send_cmd_addr(7,W25_CMD_SECTOR_ERASE, addr);
    return w25_end(ok);
}

bool W25Q_PageProgram(uint32_t addr, const uint8_t *data, uint16_t len)
{
    return W25Q_BeginPageProgram(addr, data, len) && W25Q_WaitBusy(10U);
}

bool W25Q_SectorErase(uint32_t addr)
{
    return W25Q_BeginSectorErase(addr) && W25Q_WaitBusy(W25_ERASE_TIMEOUT_MS);
}
