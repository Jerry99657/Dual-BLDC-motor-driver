#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define PROTO_SRC_USART 0U
#define PROTO_SRC_USB   1U

/* 命令行输入：USART1 和 USB CDC 共用 */
void Protocol_Feed(uint8_t source, const uint8_t *data, uint32_t len);
/* Direct legacy calls are UART-owned. Feed retains each transport's identity.
 * Dual motion requires M1:/M2:; bare Stop remains global, Follow is a group action. */
void Protocol_HandleLine(char *line);
/* Last HSO command: 1=accepted, -1=unknown, -2=invalid parameter, -3=not ready. */
int8_t Protocol_GetHsoResult(void);
/* Configuration executes in main context; motor commands remain bounded ISR
 * operations. Result: 1 accepted, 0 queued, -1 rejected, -2 syntax, -3 busy. */
void Protocol_Service(void);
int8_t Protocol_GetControlResult(void);

/* Main-loop only: USART1 nonblocking TX; USB latest-frame mailbox/service. */
void Telemetry_Send(const uint8_t *data, uint16_t len);
void Telemetry_Service(void);
/* R86 binary v1, shared by USB CDC and USART1. Stream selection is volatile. */
bool Telemetry_Select(const char *name);

#endif /* __PROTOCOL_H */
