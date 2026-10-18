#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

#define PROTO_SRC_USART 0U
#define PROTO_SRC_USB   1U

/* 命令行输入：USART1 和 USB CDC 共用 */
void Protocol_Feed(uint8_t source, const uint8_t *data, uint32_t len);
void Protocol_HandleLine(char *line);

/* 遥测输出：USART1 立即发送，USB CDC 由主循环发送 */
void Telemetry_Send(const uint8_t *data, uint16_t len);
void Telemetry_Service(void);

/* 由 usbd_cdc_if.c 的 CDC_TransmitCplt_FS 调用 */
void Protocol_UsbTxComplete(void);

#endif /* __PROTOCOL_H */
