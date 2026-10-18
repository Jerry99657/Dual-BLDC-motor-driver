#ifndef __W25Q64_H
#define __W25Q64_H

#include <stdbool.h>
#include <stdint.h>

#define W25Q64_SECTOR_SIZE 4096U
#define W25Q64_PAGE_SIZE   256U

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
