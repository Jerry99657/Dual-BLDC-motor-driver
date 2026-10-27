#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define PROTO_SRC_USART 0U
#define PROTO_SRC_USB   1U
/* Main-loop remote action gate; call inside the action's critical section. */
bool Protocol_RequestCurrent(void);

/* 命令行输入：USART1 和 USB CDC 共用 */
void Protocol_Feed(uint8_t source, const uint8_t *data, uint32_t len);
/* Transport fault: discard the partial line through the next clean newline.
 * Called from that transport's RX context, never cancels the other source. */
void Protocol_DiscardLine(uint8_t source);
/* Direct legacy calls are UART-owned. Feed retains each transport's identity.
 * Dual motion requires M1:/M2:; bare Stop remains global, Follow is a group action. */
void Protocol_HandleLine(char *line);
/* Last HSO command: 1=accepted, -1=unknown, -2=invalid parameter, -3=not ready. */
int8_t Protocol_GetHsoResult(void);
/* Complete commands execute in main context; Stop bypasses and clears the
 * pending FIFO. Result: 1 accepted, 0 queued, -1 rejected, -2 syntax, -3 busy,
 * -4 cancelled/expired. Numbered @ID:command replies use binary kind 32.
 * Get reads current numeric settings. Ordinary legacy text remains accepted. */
void Protocol_Service(void);
int8_t Protocol_GetControlResult(void);

/* Main-loop only: USART1 nonblocking TX; USB latest-frame mailbox/service. */
void Telemetry_Send(const uint8_t *data, uint16_t len);
void Telemetry_Service(void);
/* Both ports share this RAM-only output format. Reset defaults to JustFloat.
 * Main-loop changes finish active frames and discard unsent old-format data. */
bool Telemetry_IsNative(void);
void Telemetry_SetNative(bool native);
const char *Telemetry_FormatText(void);
/* Stream selection is volatile; Native retains the R86 diagnostic groups. */
bool Telemetry_Select(const char *name);

#endif /* __PROTOCOL_H */
