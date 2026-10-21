#include "protocol.h"
#include "main.h"
#include "usart.h"
#include "JerryFOC.h"
#include "st_hso.h"
#include "usbd_cdc_if.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define PROTO_LINE_MAX 64

typedef struct {
    char buf[PROTO_LINE_MAX];
    uint8_t len;
    bool overflow;
} ProtoCtx;

/* USART 和 USB 各用一份上下文，避免两个中断互相踩缓冲区 */
static ProtoCtx ctx[2];
static volatile int8_t hso_result;
int8_t Protocol_GetHsoResult(void) { return hso_result; }

static bool parse_hso_values(const char *text, float *values, unsigned count)
{
    for (unsigned i = 0U; i < count; ++i) {
        char *end;
        values[i] = strtof(text, &end);
        if (end == text || !isfinite(values[i])) return false;
        if (i + 1U < count) {
            if (*end != ',') return false;
            text = end + 1;
        } else if (*end != '\0') return false;
    }
    return true;
}

static void handle_hso(const char *command)
{
    hso_result = 1;
    if (strcmp(command, "On") == 0) {
        if (JerryFOC_isHSOActive()) { hso_result = -3; return; }
        StHso_Enable(true);
    } else if (strcmp(command, "Off") == 0) {
        if (JerryFOC_isHSOActive() || JerryFOC_hasHSOFault()) (void)JerryFOC_useHSO(0U);
        StHso_Enable(false);
    } else if (strcmp(command, "Encoder") == 0) {
        (void)JerryFOC_useHSO(0U);
    } else if (strcmp(command, "Use") == 0) {
        if (!JerryFOC_useHSO(1U)) hso_result = -3;
    } else if (strcmp(command, "Learn") == 0) {
        const MotorParams *motor = JerryFOC_getMotorParams();
        if (JerryFOC_isHSOActive() || JerryFOC_hasHSOFault() ||
            !JerryFOC_isDriveReady() || g_openloop_debug ||
            fabsf(JerryFOC_getVelocity()) * motor->pole_pairs < 31.4159265f ||
            !StHso_LearnFlux()) hso_result = -3;
    } else if (strncmp(command, "Motor:", 6) == 0 || strncmp(command, "Filter:", 7) == 0) {
        if (JerryFOC_isHSOActive() || JerryFOC_hasHSOFault()) { hso_result = -3; return; }
        StHsoConfig config;
        StHso_GetConfig(&config);
        float values[3];
        bool motor = command[0] == 'M';
        if (!parse_hso_values(command + (motor ? 6 : 7), values, motor ? 3U : 2U)) {
            hso_result = -2;
            return;
        }
        if (motor) { config.rs_ohm = values[0]; config.ls_h = values[1]; config.flux_wb = values[2]; }
        else { config.voltage_pole_rps = values[0]; config.sample_delay_cycles = values[1]; }
        if (!StHso_SetConfig(&config)) hso_result = -2;
    } else hso_result = -1;
}

/* UART double buffer; USB latest-frame mailbox plus immutable active buffer. */
#define USB_TX_BUF_SIZE 128U

static uint8_t usart_tx_buf[2][USB_TX_BUF_SIZE];
static volatile uint8_t usart_tx_idx = 0U;

/* Single ISR producer, main-loop consumer. Pending data may be replaced by a
 * newer sample, but the active buffer belongs to USB until TxState is idle. */
static uint8_t usb_tx_pending[USB_TX_BUF_SIZE];
static uint8_t usb_tx_active[USB_TX_BUF_SIZE];
static volatile uint16_t usb_tx_pending_len;

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
            if (!c->overflow) Protocol_HandleLine(c->buf);
            c->len = 0U;
            c->overflow = false;
        } else if (ch != '\r') {
            if (c->len < PROTO_LINE_MAX - 1U) c->buf[c->len++] = ch;
            else c->overflow = true;
        }
    }
}

void Protocol_HandleLine(char *line)
{
    if (strncmp(line, "HSO:", 4) == 0) {
        /* Parse outside the existing control critical section. The adapter
         * queues configuration for ADC1; never call ST library code here. */
        handle_hso(line + 4);
        return;
    }
    /* Invalid input must not change mode or release an HSO recovery stop. */
    float command_values[2] = {0.0f, 4.0f};
    bool speed_cmd = strncmp(line, "Speed:", 6) == 0;
    bool angle_cmd = strncmp(line, "Angle:", 6) == 0;
    bool torque_cmd = strncmp(line, "Torque:", 7) == 0;
    bool open_cmd = strncmp(line, "OpenLoop:", 9) == 0;
    if ((speed_cmd || angle_cmd || torque_cmd) &&
        !parse_hso_values(line + (torque_cmd ? 7 : 6), command_values, 1U)) return;
    if (open_cmd && !parse_hso_values(line + 9, command_values,
                                     strchr(line + 9, ',') ? 2U : 1U)) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (strncmp(line, "Speed:", 6) == 0) {
        g_openloop_debug = 0U;
        JerryFOC_setMode(JERRYFOC_MODE_VELOCITY);
        JerryFOC_setVelocity(command_values[0]);
    } else if (strncmp(line, "Angle:", 6) == 0) {
        g_openloop_debug = 0U;
        JerryFOC_setMode(JERRYFOC_MODE_POSITION);
        JerryFOC_setPosition(command_values[0], 0.0f);
    } else if (strncmp(line, "Torque:", 7) == 0) {
        g_openloop_debug = 0U;
        JerryFOC_setMode(JERRYFOC_MODE_TORQUE);
        JerryFOC_setCurrent(command_values[0]);
    } else if (strncmp(line, "OpenLoop:", 9) == 0) {
        if (JerryFOC_isHSOActive() || JerryFOC_hasHSOFault()) (void)JerryFOC_useHSO(0U);
        /* 纯开环诊断：OpenLoop:速度,电压，例如 OpenLoop:30,6 */
        float speed = command_values[0];
        float voltage = command_values[1];

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
        if (JerryFOC_isHSOActive() || JerryFOC_hasHSOFault()) (void)JerryFOC_useHSO(0U);
        JerryFOC_stop();
    }
    if (speed_cmd || angle_cmd || torque_cmd || open_cmd) (void)JerryFOC_startOutput();

    if (!primask) {
        __enable_irq();
    }
}

void Telemetry_Send(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U || len > USB_TX_BUF_SIZE) {
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

    /* Keep only the latest unsent telemetry frame. Never touch active data. */
    memcpy(usb_tx_pending, data, len);
    usb_tx_pending_len = len;
}

void Telemetry_Service(void)
{
    if (usb_tx_pending_len == 0U || !CDC_TxReady_FS()) {
        return;
    }

    /* Exclude the TIM2 producer only while taking its mailbox (<=128 bytes).
     * Claim/clear BEFORE submitting: completion or a new sample may interrupt
     * CDC_Transmit_FS before it returns. No application busy flag is needed. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint16_t len = usb_tx_pending_len;
    memcpy(usb_tx_active, usb_tx_pending, len);
    usb_tx_pending_len = 0U;
    __set_PRIMASK(primask);
    /* A reset/suspend between the readiness check and submission may reject
     * this sample. Drop it; the next TIM2 frame replaces it within 10 ms. */
    (void)CDC_Transmit_FS(usb_tx_active, len);
}
