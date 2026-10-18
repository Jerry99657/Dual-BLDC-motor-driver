#include "protocol.h"
#include "main.h"
#include "usart.h"
#include "JerryFOC.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <stdlib.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
extern volatile uint8_t g_usb_cdc_ready;

#define PROTO_LINE_MAX 64

typedef struct {
    char buf[PROTO_LINE_MAX];
    uint8_t len;
} ProtoCtx;

/* USART 和 USB 各用一份上下文，避免两个中断互相踩缓冲区 */
static ProtoCtx ctx[2];

/* 遥测发送缓冲：USART1 和 USB CDC 各用双缓冲 */
#define USB_TX_BUF_SIZE 128U

static uint8_t usart_tx_buf[2][USB_TX_BUF_SIZE];
static volatile uint8_t usart_tx_idx = 0U;

static uint8_t usb_tx_buf[2][USB_TX_BUF_SIZE];
static volatile uint16_t usb_tx_len[2];
static volatile uint8_t usb_tx_write = 0U;
static volatile uint8_t usb_tx_ready = 0U;  /* 0=无, 1=buf0, 2=buf1 */
static volatile uint8_t usb_tx_busy = 0U;

void Protocol_Feed(uint8_t source, const uint8_t *data, uint32_t len)
{
    if (source > PROTO_SRC_USB) {
        return;
    }

    ProtoCtx *c = &ctx[source];
    for (uint32_t i = 0U; i < len; i++) {
        char ch = (char)data[i];
        if (ch == '\n') {
            c->buf[c->len] = '\0';
            Protocol_HandleLine(c->buf);
            c->len = 0U;
        } else if (ch != '\r' && c->len < PROTO_LINE_MAX - 1U) {
            c->buf[c->len++] = ch;
        }
    }
}

void Protocol_HandleLine(char *line)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (strncmp(line, "Speed:", 6) == 0) {
        g_openloop_debug = 0U;
        JerryFOC_setMode(JERRYFOC_MODE_VELOCITY);
        JerryFOC_setVelocity(atof(line + 6));
    } else if (strncmp(line, "Angle:", 6) == 0) {
        g_openloop_debug = 0U;
        JerryFOC_setMode(JERRYFOC_MODE_POSITION);
        JerryFOC_setPosition(atof(line + 6), 0.0f);
    } else if (strncmp(line, "Torque:", 7) == 0) {
        g_openloop_debug = 0U;
        JerryFOC_setMode(JERRYFOC_MODE_TORQUE);
        JerryFOC_setCurrent(atof(line + 7));
    } else if (strncmp(line, "OpenLoop:", 9) == 0) {
        /* 纯开环诊断：OpenLoop:速度,电压，例如 OpenLoop:30,6 */
        float speed = 10.0f;
        float voltage = 4.0f;
        char *comma = strchr(line + 9, ',');

        if (comma) {
            *comma = '\0';
            speed = atof(line + 9);
            voltage = atof(comma + 1);
        } else {
            speed = atof(line + 9);
        }

        if (speed < 2.0f) {
            speed = 10.0f;
        }
        if (voltage < 0.5f) {
            voltage = 0.5f;
        }
        if (voltage > 11.0f) {
            voltage = 11.0f;
        }

        JerryFOC_useCurrentLoop(0U);
        JerryFOC_setMode(JERRYFOC_MODE_TORQUE);
        JerryFOC_setVelocity(speed);
        JerryFOC_setCurrent(voltage);
        g_openloop_debug = 1U;
        g_openloop_angle = 0.0f;
    } else if (strcmp(line, "Stop") == 0) {
        g_openloop_debug = 0U;
        JerryFOC_setVelocity(0.0f);
        JerryFOC_setCurrent(0.0f);
        JerryFOC_setMode(JERRYFOC_MODE_TORQUE);
    }

    if (!primask) {
        __enable_irq();
    }
}

void Telemetry_Send(const uint8_t *data, uint16_t len)
{
    if (len > USB_TX_BUF_SIZE) {
        return;
    }

    /* USART1：复制到静态缓冲后非阻塞发送，忙时跳过本帧。
     * 不能直接把调用者栈上的帧交给 HAL_UART_Transmit_IT。 */
    if (huart1.gState == HAL_UART_STATE_READY) {
        uint8_t idx = usart_tx_idx;
        memcpy(usart_tx_buf[idx], data, len);
        if (HAL_UART_Transmit_IT(&huart1, usart_tx_buf[idx], len) == HAL_OK) {
            usart_tx_idx ^= 1U;
        }
    }

    /* USB CDC：复制到双缓冲，由主循环发送 */
    {
        uint8_t idx = usb_tx_write;
        memcpy(usb_tx_buf[idx], data, len);
        usb_tx_len[idx] = len;
        usb_tx_write ^= 1U;
        usb_tx_ready = (uint8_t)(idx + 1U);
    }
}

void Telemetry_Service(void)
{
    if (usb_tx_ready == 0U || usb_tx_busy) {
        return;
    }
    if (!g_usb_cdc_ready || hUsbDeviceFS.pClassData == NULL) {
        return;
    }

    uint8_t idx = (uint8_t)(usb_tx_ready - 1U);
    if (CDC_Transmit_FS(usb_tx_buf[idx], usb_tx_len[idx]) == USBD_OK) {
        usb_tx_ready = 0U;
        usb_tx_busy = 1U;
    }
}

void Protocol_UsbTxComplete(void)
{
    usb_tx_busy = 0U;
}
