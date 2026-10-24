#include "protocol.h"
#include "main.h"
#include "usart.h"
#include "JerryFOC.h"
#include "st_hso.h"
#include "motor_control.h"
#include "motor_sampling.h"
#include "app_storage.h"
#include "usbd_cdc_if.h"
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

static int digit_value(char c,bool hex)
{
    if (c>='0' && c<='9') return c-'0';
    if (hex && c>='a' && c<='f') return c-'a'+10;
    if (hex && c>='A' && c<='F') return c-'A'+10;
    return -1;
}

/* The wire format is at most 63 bytes and needs finite binary32 numbers,
 * not newlib's heap-based arbitrary-precision strtod machinery. Retain signs,
 * leading ASCII whitespace, decimal/scientific and C hexadecimal notation.
 * Accumulate/scale in double so tiny inductance/flux values do not underflow
 * before their exponent is applied; the control interface remains float. */
static float parse_number(const char *text,const char **end)
{
    const char *p=text;
    while (*p==' ' || (*p>='\t' && *p<='\r')) ++p;
    bool negative=*p=='-';
    if (*p=='-' || *p=='+') ++p;
    bool hex=p[0]=='0' && (p[1]=='x' || p[1]=='X');
    if (hex) p+=2;
    double value=0.0;
    unsigned digits=0;
    int exponent=0,base=hex ? 16 : 10;
    bool dot=false;
    for (;;) {
        int d=digit_value(*p,hex);
        if (d>=0) {
            value=value*base+d; ++digits; ++p;
            if (dot) exponent-=hex ? 4 : 1;
        } else if (*p=='.' && !dot) { dot=true; ++p; }
        else break;
    }
    if (!digits) { *end=text; return 0.0f; }
    if (*p==(hex ? 'p' : 'e') || *p==(hex ? 'P' : 'E')) {
        const char *mark=p++;
        bool minus=*p=='-';
        if (*p=='-' || *p=='+') ++p;
        const char *first=p;
        int extra=0;
        while (*p>='0' && *p<='9') {
            if (extra<10000) extra=extra*10+(*p-'0');
            ++p;
        }
        if (p==first) p=mark; /* Caller rejects the unconsumed suffix. */
        else exponent+=minus ? -extra : extra;
    }
    *end=p;
    if (value!=0.0) {
        double factor=hex ? 2.0 : 10.0;
        if (exponent<0) { factor=1.0/factor; exponent=-exponent; }
        while (exponent) {
            if (exponent&1) value*=factor;
            exponent>>=1;
            if (exponent) factor*=factor;
        }
    }
    return (float)(negative ? -value : value);
}

static bool parse_hso_values(const char *text, float *values, unsigned count)
{
    for (unsigned i = 0U; i < count; ++i) {
        const char *end;
        values[i] = parse_number(text, &end);
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
    if (!MotorControl_HsoAllowed()) {
        if (!strcmp(command,"Off")) { StHso_Enable(false); return; }
        if (strcmp(command,"Encoder") || MotorControl_GetWork()!=MOTOR_WORK_SINGLE_M1) {
            hso_result=-3; return;
        }
    }
    if (strcmp(command, "On") == 0) {
        if (JerryFOC_isHSOActive()) { hso_result = -3; return; }
        StHso_Enable(true);
    } else if (strcmp(command, "Off") == 0) {
        if (JerryFOC_isHSOActive() || JerryFOC_hasHSOFault()) (void)JerryFOC_useHSO(0U);
        StHso_Enable(false);
    } else if (strcmp(command, "Encoder") == 0) {
        if (!MotorControl_SelectFeedback(false)) hso_result=-3;
    } else if (strcmp(command, "Use") == 0) {
        if (!JerryFOC_useHSO(1U)) hso_result = -3;
    } else if (strcmp(command, "Learn") == 0) {
        if (JerryFOC_isHSOActive() || JerryFOC_hasHSOFault() ||
            !JerryFOC_isDriveReady() || g_openloop_debug ||
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

static char config_queue[4][PROTO_LINE_MAX];
static volatile uint8_t config_head,config_count;
static volatile bool config_applying;
static volatile int8_t control_result;
static bool config_save;
int8_t Protocol_GetControlResult(void) { return control_result; }
static bool is_configuration(const char *s)
{
    return !strncmp(s,"Work:",5) || !strncmp(s,"Motor1:",7) || !strncmp(s,"Motor2:",7) ||
           !strncmp(s,"Feedback:",9) || !strncmp(s,"Gear:",5) || !strncmp(s,"Align:",6) || !strncmp(s,"Focus:",6);
}
static bool apply_configuration(char *s)
{
    if (!strncmp(s,"Work:",5)) {
        static const char *const names[]={"SingleM1","SingleM2","Independent","SyncSpeed","SyncPosition","MasterM1","MasterM2"};
        for (unsigned i=0;i<MOTOR_WORK_COUNT;++i)
            if (!strcmp(s+5,names[i])) return MotorControl_SelectWork((MotorWorkMode)i);
    } else if (!strncmp(s,"Motor1:",7) || !strncmp(s,"Motor2:",7)) {
        MotorAxis a=s[5]=='1' ? MOTOR_AXIS_1 : MOTOR_AXIS_2;
        if (!strcmp(s+7,"2804")) return MotorControl_SelectProfile(a,MOTOR_PROFILE_2804);
        if (!strcmp(s+7,"4015")) return MotorControl_SelectProfile(a,MOTOR_PROFILE_4015);
    } else if (!strncmp(s,"Feedback:",9)) {
        if (!strcmp(s+9,"Encoder")) return MotorControl_SelectFeedback(false);
        if (!strcmp(s+9,"HSO")) return MotorControl_SelectFeedback(true);
    } else if (!strncmp(s,"Gear:",5)) {
        float v[2];
        if (parse_hso_values(s+5,v,2) && fabsf(v[0])>=0.01f && fabsf(v[0])<=10.0f && fabsf(v[1])<=62.83f)
            return MotorControl_SetGear((int32_t)roundf(v[0]*100),(int32_t)roundf(v[1]*100));
    } else if (!strncmp(s,"Align:",6) || !strncmp(s,"Focus:",6)) {
        bool view=s[0]=='F';
        if (!strcmp(s+6,"M1") || !strcmp(s+6,"M2")) {
            MotorAxis a=s[7]=='1' ? MOTOR_AXIS_1 : MOTOR_AXIS_2;
            if (view) { MotorControl_SetFocus(a); return true; }
            return MotorControl_RequestAlign(a);
        }
    }
    return false;
}
void Protocol_Service(void)
{
    /* Drain the bounded batch before main services automatic alignment. */
    for (unsigned i=0;i<4;++i) {
        char line[PROTO_LINE_MAX];
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        if (!config_count) { __set_PRIMASK(mask); break; }
        config_applying=true;
        memcpy(line,config_queue[config_head],sizeof(line));
        config_head=(config_head+1U)%4U; --config_count;
        __set_PRIMASK(mask);
        bool ok=apply_configuration(line);
        control_result=ok ? 1 : -1;
        if (ok && strncmp(line,"Focus:",6) && strncmp(line,"Align:",6)) config_save=true;
    }
    config_applying=false;
    if (config_save && AppStorage_RequestSave()) config_save=false;
}

void Protocol_HandleLine(char *line)
{
    if (is_configuration(line)) {
        if (strlen(line)>=PROTO_LINE_MAX) { control_result=-2; return; }
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        if (config_count>=4U) control_result=-3;
        else {
            strcpy(config_queue[(config_head+config_count)%4U],line);
            ++config_count; control_result=0;
        }
        __set_PRIMASK(mask); return;
    }
    MotorWorkMode work=MotorControl_GetWork();
    MotorAxis axis=work==MOTOR_WORK_SINGLE_M1 ? MOTOR_AXIS_1 :
                   work==MOTOR_WORK_SINGLE_M2 ? MOTOR_AXIS_2 : MotorControl_GetFocus();
    bool explicit_axis=false;
    if (!strncmp(line,"M1:",3) || !strncmp(line,"M2:",3)) {
        axis=line[1]=='1' ? MOTOR_AXIS_1 : MOTOR_AXIS_2; line+=3; explicit_axis=true;
    }
    if ((config_count || config_applying) && strcmp(line,"Stop")) { control_result=-3; return; }
    if (strncmp(line, "HSO:", 4) == 0) {
        /* Parse outside the existing control critical section. The adapter
         * queues configuration for ADC1; never call ST library code here. */
        if (axis!=MOTOR_AXIS_1) { hso_result=-3; return; }
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
        !parse_hso_values(line + (torque_cmd ? 7 : 6), command_values, 1U)) { control_result=-2; return; }
    if (open_cmd && !parse_hso_values(line + 9, command_values,
                                     strchr(line + 9, ',') ? 2U : 1U)) { control_result=-2; return; }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (!strcmp(line,"Follow")) {
        control_result=MotorControl_StartFollow() ? 1 : -1;
    } else if (speed_cmd || angle_cmd || torque_cmd) {
        JerryFOC_ControlMode mode=speed_cmd ? JERRYFOC_MODE_VELOCITY :
                                  angle_cmd ? JERRYFOC_MODE_POSITION : JERRYFOC_MODE_TORQUE;
        uint32_t flags=axis==MOTOR_AXIS_1 ? AppStorage_Get()->flags : AppStorage_Get()->axis2_flags;
        float value=command_values[0];
        if (angle_cmd && !MotorControl_IsCoupled()) {
            MotorAxisSnapshot state; MotorControl_GetAxis(axis,&state);
            value+=floorf(state.position/(2.0f*PI))*(2.0f*PI);
        }
        control_result=MotorControl_Command(axis,mode,value,0,(flags & APP_FLAG_CURRENT_LOOP)!=0U) ? 1 : -1;
    } else if (strncmp(line, "OpenLoop:", 9) == 0) {
        if (MotorControl_GetWork()!=MOTOR_WORK_SINGLE_M1 || MotorControl_HsoAllowed() || axis!=MOTOR_AXIS_1 ||
            !JerryFOC_isDriveReady() || !MotorSampling_ResumeGlobalStop()) {
            control_result=-1; __set_PRIMASK(primask); return;
        }
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
        if (explicit_axis) MotorControl_Stop(axis); else MotorControl_StopAll();
        control_result=1;
    }
    if (open_cmd) control_result=JerryFOC_startOutput() ? 1 : -1;

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
