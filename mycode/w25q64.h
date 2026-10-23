#ifndef __W25Q64_H
#define __W25Q64_H

#include <stdbool.h>
#include <stdint.h>

#define W25Q64_SECTOR_SIZE 4096U
#define W25Q64_PAGE_SIZE   256U

/* Last failing operation/HAL status/HAL error/last confirmed SR1, one byte each.
 * Operations: 1 WREN, 2 SR cmd, 3 SR read, 4 WEL check, 5 PP cmd,
 * 6 PP data, 7 erase cmd, 8 read cmd, 9 read data, 10 argument, 11 bus lock. */
uint32_t W25Q_LastFailure(void);
bool W25Q_LastFailureBusy(void); /* Last Begin deferred because confirmed BUSY=1. */
uint8_t W25Q_LastStatus(void); /* Conservative SR1: BUSY from either byte, WEL from both. */

bool W25Q_Init(void);
bool W25Q_IsPresent(void);
uint32_t W25Q_ReadID(void);
bool W25Q_Read(uint32_t addr, uint8_t *buf, uint32_t len);
bool W25Q_PageProgram(uint32_t addr, const uint8_t *data, uint16_t len);
bool W25Q_SectorErase(uint32_t addr);
bool W25Q_WaitBusy(uint32_t timeout_ms);
/* 运行时服务使用 Begin + Poll，擦写内部忙时立即返回主循环。 */
bool W25Q_BeginPageProgram(uint32_t addr, const uint8_t *data, uint16_t len);
bool W25Q_BeginSectorErase(uint32_t addr);
bool W25Q_PollBusy(bool *busy);

#endif /* __W25Q64_H */
