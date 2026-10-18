#ifndef KK_OLED_DRIVER_H
#define KK_OLED_DRIVER_H

#include "kk_oled.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * ST7735S 0.96" 80x160 TFT，接 STM32G431 SPI3。
 * 面板原生竖屏 80x160；应用通过 OLED_SetRotation(90/270) 得到 160x80 横屏。
 */
#define OLED_PHYSICAL_WIDTH  80U
#define OLED_PHYSICAL_HEIGHT 160U
#define OLED_PHYSICAL_PAGES  (OLED_PHYSICAL_HEIGHT / 8U)

OLED_Status OLED_DriverInit(void);
OLED_Status OLED_DriverWriteBlocking(void);
OLED_Status OLED_DriverWriteIT(void);
OLED_Status OLED_DriverWriteDMA(void);
bool OLED_DriverIsBusy(void);
void OLED_DriverAbort(void);
/* 主循环调用；将 IRQ 完成通知交给已完成启动交接的图形核心。 */
void OLED_DriverService(void);
OLED_Status OLED_DriverSetContrast(uint8_t value);
OLED_Status OLED_DriverSetPowerSave(bool enable);

/* 由全局 HAL_SPI_ErrorCallback 在 SPI3 出错时转发 */
void OLED_DriverHandleSpiError(void);

#endif /* KK_OLED_DRIVER_H */
