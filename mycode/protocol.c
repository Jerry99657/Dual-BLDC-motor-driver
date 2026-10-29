#if defined(__GNUC__) && !defined(_WIN32)
/* User protocol handlers stay shared; official compilation is unchanged. */
#pragma GCC optimize ("Os,no-ipa-cp-clone")
#endif
#include "protocol.h"
#include "number_parse.h"
#include "main.h"
#include "uart_port.h"
#include "JerryFOC.h"
#include "st_hso.h"
#include "motor_control.h"
#include "motor_detect.h"
#include "motor_sampling.h"
#include "app_storage.h"
#include "app_control.h"
#include "MT6701.h"
#include "motor_sixstep.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <math.h>

#define PROTO_LINE_MAX 64
_Static_assert(PROTO_LINE_MAX==64,"Review Number_Parse integer capacity before increasing the wire limit");

typedef struct {
    char buf[PROTO_LINE_MAX];
    uint8_t len;
    bool overflow;
} ProtoCtx;

/* USART 和 USB 各用一份上下文，避免两个中断互相踩缓冲区 */
static ProtoCtx ctx[2];
static volatile int8_t hso_result;
int8_t Protocol_GetHsoResult(void) { return hso_result; }

__attribute__((noinline,noclone)) static bool parse_hso_values(const char *text, float *values, unsigned count)
{
    for (unsigned i = 0U; i < count; ++i) {
        const char *end;
        values[i] = Number_Parse(text, &end);
        if (end == text || !isfinite(values[i])) return false;
        if (i + 1U < count) {
            if (*end != ',') return false;
            text = end + 1;
        } else if (*end != '\0') return false;
    }
    return true;
}

/* Exact enum tokens in packed NUL strings. Retain every legacy spelling;
 * one cold matcher serves configuration and transport-format commands. */
__attribute__((noinline,noclone)) static unsigned enum_value(const char *names,const char *value,unsigned count)
{
    for(unsigned i=0;i<count;++i) {
        if(!strcmp(value,names)) return i;
        while(*names++);
    }
    return count;
}
static void handle_hso(const char *command,MotorInput input)
{
    unsigned kind=enum_value("On\0" "Off\0" "Encoder\0" "Use\0" "Learn",command,5);
    hso_result = 1;
    if (!MotorControl_HsoAllowed()) {
        if (kind==1) { StHso_Enable(false); return; }
        if (kind!=2 || MotorControl_GetWork()!=MOTOR_WORK_SINGLE_M1) {
            hso_result=-3; return;
        }
    }
    if (kind==0) {
        if (JerryFOC_isHSOActive()) { hso_result = -3; return; }
        StHso_Enable(true);
    } else if (kind==1) {
        if (JerryFOC_isHSOActive() || JerryFOC_hasHSOFault()) (void)JerryFOC_useHSO(0U);
        StHso_Enable(false);
    } else if (kind==2) {
        if (!MotorControl_SelectFeedback(false)) hso_result=-3;
    } else if (kind==3) {
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        if (!Protocol_RequestCurrent() || !MotorControl_ClaimInput(MOTOR_AXIS_1,input) || !JerryFOC_useHSO(1U)) hso_result = -3;
        __set_PRIMASK(mask);
    } else if (kind==4) {
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

/* USART owns its copied active frame through TC. USB has a separate
 * latest-frame mailbox and immutable active buffer. Main-loop producer only. */
#define USB_TX_BUF_SIZE 128U

/* Main-loop mailbox. Pending data may be replaced by a newer sample, but
 * the active buffer belongs to USB until TxState is idle. */
static uint8_t usb_tx_pending[USB_TX_BUF_SIZE];
static uint8_t usb_tx_active[USB_TX_BUF_SIZE];
static volatile uint16_t usb_tx_pending_len;

static int8_t handle_line(char *line,MotorInput input);
static void receive_line(char *line,MotorInput input);
void Protocol_DiscardLine(uint8_t source)
{
    if(source<=PROTO_SRC_USB) { ctx[source].len=0; ctx[source].overflow=true; }
}
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
            if (!c->overflow) receive_line(c->buf,source==PROTO_SRC_USB ? MOTOR_INPUT_USB : MOTOR_INPUT_UART);
            c->len = 0U;
            c->overflow = false;
        } else if (ch != '\r') {
            if (c->len < PROTO_LINE_MAX - 1U) c->buf[c->len++] = ch;
            else c->overflow = true;
        }
    }
}

/* Both transports publish complete bounded requests. Main owns parsing and
 * configuration; only Stop bypasses the queue and cancels pending commands. */
typedef struct { uint32_t at; uint16_t id; uint8_t input; bool stopped; char text[PROTO_LINE_MAX]; } CommandLine;
static CommandLine config_queue[4];
static volatile uint8_t config_head,config_count;
static volatile uint32_t stop_generation;
static uint32_t command_generation;
static uint32_t command_at;
static bool command_was_stopped;
static volatile int8_t control_result;
static bool reply_pending,reply_built;
static uint8_t reply_row[3],reply_kind;
static uint16_t reply_id;
static uint8_t reply_source;
static int8_t reply_result;
static uint32_t reply_at;
int8_t Protocol_GetControlResult(void) { return control_result; }
__attribute__((noinline,noclone)) bool Protocol_RequestCurrent(void)
{
    return command_generation==stop_generation &&
        (command_was_stopped || !MotorSampling_IsGlobalStopped()) &&
        (!command_was_stopped || HAL_GetTick()-command_at<=10U);
}

enum { CONFIG_WORK,CONFIG_MOTOR1,CONFIG_MOTOR2,CONFIG_FEEDBACK,
       CONFIG_GEAR,CONFIG_ALIGN,CONFIG_FOCUS,CONFIG_COUNT,
       CMD_SPEED=CONFIG_COUNT,CMD_ANGLE,CMD_TORQUE,CMD_THROTTLE,CMD_OPEN,CMD_HSO,CMD_STREAM,CMD_APP,CMD_ROW,CMD_FORMAT,CMD_COUNT };
_Static_assert(CMD_THROTTLE==CMD_SPEED+3,"Contiguous scalar motion commands");
/* One source for names and lengths; no per-name NUL/padding in Flash. */
#define COMMAND_PREFIXES(X) \
    X("Work:",5) X("Motor1:",7) X("Motor2:",7) X("Feedback:",9) \
    X("Gear:",5) X("Align:",6) X("Focus:",6) X("Speed:",6) \
    X("Angle:",6) X("Torque:",7) X("Throttle:",9) X("OpenLoop:",9) \
    X("HSO:",4) X("Stream:",7) X("App:",4) X("GetRow:",7) X("Format:",7)
#define PREFIX_NAME(name,length) name
static const char command_names[]=COMMAND_PREFIXES(PREFIX_NAME);
#undef PREFIX_NAME
#define PREFIX_LENGTH(name,length) length,
static const uint8_t command_lengths[]= { COMMAND_PREFIXES(PREFIX_LENGTH) };
#undef PREFIX_LENGTH
#define PREFIX_CHECK(name,length) _Static_assert(sizeof(name)-1==length,"Command prefix length");
COMMAND_PREFIXES(PREFIX_CHECK)
#undef PREFIX_CHECK
#undef COMMAND_PREFIXES
_Static_assert(sizeof(command_lengths)==CMD_COUNT,"Command order/count");
static unsigned command_kind(const char *s,const char **argument)
{
    const char *name=command_names;
    for(unsigned i=0;i<CMD_COUNT;++i) {
        unsigned length=command_lengths[i];
        if(!strncmp(s,name,length)) { *argument=s+length; return i; }
        name+=length;
    }
    return CMD_COUNT;
}
static bool apply_configuration(uint8_t kind,const char *value)
{
    switch(kind) {
    case CONFIG_WORK: {
        static const char names[]=
        "SingleM1\0"
        "SingleM2\0"
        "Independent\0"
        "SyncSpeed\0"
        "SyncPosition\0"
        "MasterM1\0"
        "MasterM2";
        unsigned i=enum_value(names,value,MOTOR_WORK_COUNT);
        return i<MOTOR_WORK_COUNT && MotorControl_SelectWork((MotorWorkMode)i);
    }
    case CONFIG_MOTOR1: case CONFIG_MOTOR2: {
        MotorAxis a=kind==CONFIG_MOTOR1 ? MOTOR_AXIS_1 : MOTOR_AXIS_2;
        static const char names[]="2804\0" "4015\0" "ESC\0" "Custom1\0" "Custom2\0" "Custom3";
        _Static_assert(MOTOR_PROFILE_2804==0 && MOTOR_PROFILE_4015==1 && MOTOR_PROFILE_ESC==2 &&
            MOTOR_PROFILE_CUSTOM1==3 && MOTOR_PROFILE_CUSTOM3==5 && MOTOR_PROFILE_COUNT==6,"Motor command order");
        unsigned i=enum_value(names,value,MOTOR_PROFILE_COUNT);
        return i<MOTOR_PROFILE_COUNT && (i!=MOTOR_PROFILE_ESC || a==MOTOR_AXIS_1) && App_SelectMotor(a,(MotorProfile)i);
    }
    case CONFIG_FEEDBACK: {
        unsigned i=enum_value("Encoder\0" "HSO\0" "SixStep",value,3);
        return i<2 ? MotorControl_SelectFeedback(i!=0) : i==2 && MotorControl_SelectSixStep(true);
    }
    case CONFIG_GEAR: {
        float v[2];
        if (parse_hso_values(value,v,2) && fabsf(v[0])>=0.01f && fabsf(v[0])<=10.0f && fabsf(v[1])<=62.83f)
            return MotorControl_SetGear((int32_t)roundf(v[0]*100),(int32_t)roundf(v[1]*100));
        break;
    }
    case CONFIG_ALIGN: case CONFIG_FOCUS:
        if (value[0]=='M' && (value[1]=='1' || value[1]=='2') && value[2]=='\0') {
            MotorAxis a=value[1]=='1' ? MOTOR_AXIS_1 : MOTOR_AXIS_2;
            if (kind==CONFIG_FOCUS) { MotorControl_SetFocus(a); return true; }
            uint32_t mask=__get_PRIMASK(); __disable_irq();
            bool ok=Protocol_RequestCurrent() && MotorControl_RequestAlign(a);
            __set_PRIMASK(mask); return ok;
        }
        break;
    default: break;
    }
    return false;
}
/* Inspect only characters known to be present; a truncated prefix is data,
 * never an axis selector. Both immediate Stop and queued dispatch use this. */
__attribute__((noinline,noclone)) static bool axis_prefix(const char *s)
{ return s[0]=='M' && (s[1]=='1' || s[1]=='2') && s[2]==':'; }
static void receive_line(char *line,MotorInput input)
{
    uint32_t id=0;
    if (*line=='@') {
        ++line;
        const char *first=line;
        while (*line>='0' && *line<='9') {
            id=id*10U+(unsigned)(*line++-'0');
            if(id>65535U) return;
        }
        if(line==first || !id || *line++!=':') return;
    }
    if(strlen(line)>=PROTO_LINE_MAX) { control_result=-2; return; }
    const char *action=line;
    if(axis_prefix(action)) action+=3;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if(!strcmp(action,"Stop")) {
        ++stop_generation; config_count=0;
        control_result=handle_line(line,input); /* Immediate power stop, even if TX is stalled. */
        if(!id) { __set_PRIMASK(mask); return; }
    }
    if(config_count>=4U) control_result=-3;
    else {
        CommandLine *request=&config_queue[(config_head+config_count)%4U];
        request->at=HAL_GetTick();
        request->stopped=MotorSampling_IsGlobalStopped();
        request->id=(uint16_t)id; request->input=(uint8_t)input;
        strcpy(request->text,line); ++config_count; control_result=0;
    }
    __set_PRIMASK(mask);
}
__attribute__((noinline,noclone)) void Protocol_Service(void)
{
    App_SaveService(HAL_GetTick());
    if(reply_pending) return;
    for(unsigned i=0;i<4;++i) {
        CommandLine request;
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        if(!config_count) { __set_PRIMASK(mask); break; }
        request=config_queue[config_head];
        config_head=(config_head+1U)%4U; --config_count;
        command_generation=stop_generation;
        command_at=request.at;
        command_was_stopped=request.stopped;
        __set_PRIMASK(mask);
        int8_t result=1; reply_kind=32;
        if(HAL_GetTick()-request.at>500U || (!request.stopped && MotorSampling_IsGlobalStopped())) result=-4;
        else if(strcmp(request.text,"Get")) result=handle_line(request.text,(MotorInput)request.input);
        control_result=result;
        if(request.id && Telemetry_IsNative()) {
            reply_id=request.id; reply_source=request.input==MOTOR_INPUT_USB ? PROTO_SRC_USB : PROTO_SRC_USART;
            reply_result=result;
            reply_at=HAL_GetTick(); reply_built=false; reply_pending=true;
            break;
        }
    }
}
void Protocol_HandleLine(char *line)
{
    receive_line(line,MOTOR_INPUT_UART);
}
__attribute__((noinline,noclone)) static int8_t handle_line(char *line,MotorInput input)
{
    int8_t result=1;
    MotorWorkMode work=MotorControl_GetWork();
    MotorAxis axis=work==MOTOR_WORK_SINGLE_M2 ? MOTOR_AXIS_2 : MOTOR_AXIS_1;
    bool explicit_axis=false;
    if (axis_prefix(line)) {
        axis=line[1]=='1' ? MOTOR_AXIS_1 : MOTOR_AXIS_2; line+=3; explicit_axis=true;
    }
    const char *argument=line;
    unsigned kind=command_kind(line,&argument);
    if(kind==CMD_FORMAT) {
        if(explicit_axis) return -2;
        unsigned i=enum_value("Native\0" "JustFloat",argument,2);
        if(i==2) return -2;
        Telemetry_SetNative(i==0); return 1;
    }
    if (kind==CMD_APP || kind==CMD_ROW) {
        float v[3];
        if (explicit_axis || !parse_hso_values(argument,v,3)) return -2;
        for (unsigned i=0;i<3;++i) if (v[i]<-100000 || v[i]>100000 || v[i]!=(float)(int32_t)v[i]) return -2;
        if (kind==CMD_APP) {
            if(!Protocol_RequestCurrent()) return -4;
            return App_Control((unsigned)(int32_t)v[0],(unsigned)(int32_t)v[1],(int32_t)v[2]);
        }
        if (v[0]<1 || v[0]>2 || v[1]<0 || v[1]>7 || v[2]<0 || v[2]>255) return -2;
        for (unsigned i=0;i<3;++i) reply_row[i]=(uint8_t)v[i];
        reply_kind=33; return 1;
    }
    if (kind<CONFIG_COUNT) {
        if (!App_CanConfigure()) return -3;
        if(kind==CONFIG_ALIGN && command_was_stopped && HAL_GetTick()-command_at>10U) return -4;
        result=!explicit_axis && apply_configuration(kind,argument) ? 1 : -1;
        if(result==1 && kind!=CONFIG_ALIGN) App_ConfigChanged();
        return result;
    }
    if (kind==CMD_STREAM) {
        result=!explicit_axis && Telemetry_Select(argument) ? 1 : -2;
        return result;
    }
    /* Stop is transport-independent and remains available during configuration. */
    if (!strcmp(line,"Stop")) {
        App_CancelNavigation();
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        if ((!explicit_axis || axis==MOTOR_AXIS_1) && (JerryFOC_isHSOActive() || JerryFOC_hasHSOFault())) (void)JerryFOC_useHSO(0U);
        if (explicit_axis) MotorControl_Stop(axis); else MotorControl_StopAll();
        result=1; __set_PRIMASK(mask); return result;
    }
    if (kind==CMD_HSO) {
        /* Parse outside the existing control critical section. The adapter
         * queues configuration for ADC1; never call ST library code here. */
        if (axis!=MOTOR_AXIS_1) { hso_result=-3; return -3; }
        handle_hso(argument,input);
        result=hso_result;
        if (result==1 && line[4]=='U') App_RemoteMotion(axis,false);
        return result;
    }
    /* Invalid input must not change mode or release an HSO recovery stop. */
    float command_values[2] = {0.0f, 4.0f};
    bool motion_cmd=kind-CMD_SPEED<4U;
    bool angle_cmd=kind==CMD_ANGLE;
    bool torque_cmd=kind==CMD_TORQUE;
    bool throttle_cmd=kind==CMD_THROTTLE;
    bool open_cmd=kind==CMD_OPEN;
    if (work>=MOTOR_WORK_INDEPENDENT && !explicit_axis &&
        (motion_cmd || open_cmd)) { result=-2; return result; }
    if (motion_cmd &&
        !parse_hso_values(argument,command_values,1U)) { result=-2; return result; }
    if ((throttle_cmd && (axis!=MOTOR_AXIS_1 || !MotorControl_SixStepAllowed())) ||
        (torque_cmd && MotorControl_SixStepAllowed())) { result=-1; return result; }
    if (open_cmd && !parse_hso_values(argument, command_values,
                                     strchr(argument, ',') ? 2U : 1U)) { result=-2; return result; }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* A command received with the stop latch set must not wait through the
     * physical key's 30 ms release guard and then start without a new click. */
    if(!Protocol_RequestCurrent()) { result=-4; __set_PRIMASK(primask); return result; }

    if (!strcmp(line,"Follow")) {
        result=MotorControl_StartFollow() ? 1 : -1;
    } else if (motion_cmd) {
        JerryFOC_ControlMode mode=kind==CMD_SPEED ? JERRYFOC_MODE_VELOCITY :
                                  angle_cmd ? JERRYFOC_MODE_POSITION : JERRYFOC_MODE_TORQUE;
        uint32_t flags=MotorControl_GetOptions(axis);
        float value=command_values[0];
        if (angle_cmd && !MotorControl_IsCoupled()) {
            MotorAxisSnapshot state; MotorControl_GetAxis(axis,&state);
            value+=floorf(state.position/(2.0f*PI))*(2.0f*PI);
        }
        result=MotorControl_CommandFrom(axis,mode,value,0,(flags & APP_FLAG_CURRENT_LOOP)!=0U,input) ? 1 : -1;
    } else if (open_cmd) {
        if (MotorDetect_Active() || MotorControl_GetWork()!=MOTOR_WORK_SINGLE_M1 || MotorControl_HsoAllowed() || MotorControl_SixStepAllowed() || axis!=MOTOR_AXIS_1 ||
            !JerryFOC_isDriveReady() || !MotorSampling_ResumeGlobalStop() || !MotorControl_ClaimInput(axis,input)) {
            result=-1; __set_PRIMASK(primask); return result;
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
    } else {
        result=-2;
    }
    if (open_cmd) result=JerryFOC_startOutput() ? 1 : -1;

    if (!primask) {
        __enable_irq();
    }
    if (result==1 && !open_cmd) App_RemoteMotion(axis,false);
    return result;
}

void Telemetry_Send(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U || len > USB_TX_BUF_SIZE) {
        return;
    }

    /* The USART transport copies before publishing; busy skips this curve. */
    (void)Uart1_TrySend(data,len);

    /* Keep only the latest unsent telemetry frame. Never touch active data. */
    memcpy(usb_tx_pending, data, len);
    usb_tx_pending_len = len;
}

static void telemetry_produce(void);
static void telemetry_reply(void);
__attribute__((noinline,noclone)) void Telemetry_Service(void)
{
    if(reply_pending) { telemetry_reply(); return; }
    telemetry_produce();
    if (usb_tx_pending_len == 0U || !CDC_TxReady_FS()) {
        return;
    }

    /* Main owns both producer and mailbox; no IRQ masking is needed here.
     * Claim/clear BEFORE submitting: a completion/new publication during the
     * submit must not clear the next pending frame. */
    uint16_t len = usb_tx_pending_len;
    memcpy(usb_tx_active, usb_tx_pending, len);
    usb_tx_pending_len = 0U;
    /* A reset/suspend between the readiness check and submission may reject
     * this sample. Drop it; the next TIM2 frame replaces it within 10 ms. */
    (void)CDC_Transmit_FS(usb_tx_active, len);
}

/* No control ISR formatting: bounded main-loop packets, same bytes on both
 * transports. All wire integers/floats are little endian on this target. */
typedef struct { uint16_t sync; uint8_t version,kind; uint16_t sequence,length; uint32_t ms; } TelemetryHeader;
typedef struct { uint8_t meta[16]; uint32_t values[4]; char text[4][16]; } TelemetryStatus;
typedef struct { uint32_t counts[8]; uint16_t values[7]; uint8_t status,error; } TelemetryEncoder;
typedef struct { float data[7]; uint32_t counts[3]; uint8_t flags[4]; } TelemetryHso;
typedef struct { float startup_emf; uint32_t crossings; uint16_t values[13]; uint8_t flags[4]; } TelemetrySix;
/* Schema 2 retains schema 1's numeric layout. Only the
 * contiguous numeric settings fields are copied; no metadata/CRC/pointers. */
typedef struct { uint16_t id; int8_t result; uint8_t source,schema,firmware,save,focus; uint32_t settings[21]; } TelemetryReply;
typedef struct { uint16_t id; int8_t result; uint8_t source,axis,group,row,count; char label[24],value[32]; } TelemetryRow;
_Static_assert(sizeof(TelemetryRow)==64,"RPC diagnostic row layout");
typedef struct {
    TelemetryHeader header;
    union { TelemetryStatus status; TelemetryReply reply; TelemetryRow row; TelemetryEncoder encoder; TelemetryHso hso;
        TelemetrySix six; float curves[12]; uint8_t bytes[114]; } payload;
} TelemetryPacket;
/* Main-loop producer only. Transports copy into their own immutable active
 * buffers before this workspace is reused; no DMA references this object. */
static TelemetryPacket telemetry_packet;
_Static_assert(sizeof(TelemetryHeader)==12 && sizeof(TelemetryStatus)==96,"Telemetry wire layout");
_Static_assert(sizeof(TelemetryHeader)+sizeof(TelemetryStatus)+2<=USB_TX_BUF_SIZE,"Largest telemetry packet");
_Static_assert(sizeof(TelemetryPacket)==USB_TX_BUF_SIZE && sizeof(TelemetryEncoder)==48 &&
    sizeof(TelemetryHso)==44 && offsetof(TelemetrySix,flags)==34,"Typed packet workspace");
enum { STREAM_AUTO,STREAM_M1,STREAM_M2,STREAM_FOC1,STREAM_FOC2,STREAM_ENC1,
       STREAM_ENC2,STREAM_HSO,STREAM_SIX,STREAM_OFF,STREAM_COUNT };
static volatile uint8_t telemetry_stream;
static bool telemetry_native; /* BSS/reset: standard JustFloat, never persisted. */
static uint16_t telemetry_sequence;
static uint32_t telemetry_tick,telemetry_status_tick;
static uint32_t telemetry_diagnostic_tick;
static bool telemetry_have_status;
static TelemetryStatus telemetry_previous;
bool Telemetry_IsNative(void) { return telemetry_native; }
const char *Telemetry_FormatText(void) { return telemetry_native ? "Native" : "JustFloat"; }
void Telemetry_SetNative(bool native)
{
    if(telemetry_native==native) return;
    telemetry_native=native;
    /* Active USB/UART buffers own complete copies through completion. */
    usb_tx_pending_len=0;reply_pending=false;telemetry_have_status=false;
    if(!native && (telemetry_stream==STREAM_ENC1 || telemetry_stream==STREAM_ENC2)) telemetry_stream=STREAM_AUTO;
}
bool Telemetry_Select(const char *name)
{
    static const char names[][5]={"Auto","M1","M2","FOC1","FOC2","ENC1","ENC2","HSO","Six","Off"};
    for(unsigned i=0;i<STREAM_COUNT;++i) if(!strcmp(name,names[i])) {
        if(!telemetry_native && (i==STREAM_ENC1 || i==STREAM_ENC2)) return false;
        telemetry_stream=(uint8_t)i; return true;
    }
    return false;
}
static unsigned telemetry_frame(uint8_t kind,unsigned length,uint32_t now)
{
    uint8_t *bytes=(uint8_t *)&telemetry_packet;
    telemetry_packet.header=(TelemetryHeader){0x5AA5,1,kind,telemetry_sequence++,(uint16_t)length,now};
    unsigned total=sizeof(TelemetryHeader)+length;
    uint16_t crc=0xFFFF;
    for(unsigned i=0;i<total;++i) {
        /* Algebraic byte update of CRC16/CCITT-FALSE (poly 0x1021),
         * cross-checked against the independent bitwise C and Python oracles. */
        uint16_t x=(crc>>8)^bytes[i]; x^=x>>4;
        crc=(uint16_t)((crc<<8)^(x<<12)^(x<<5)^x);
    }
    bytes[total++]=(uint8_t)crc; bytes[total++]=(uint8_t)(crc>>8);
    return total;
}
static void telemetry_emit(uint8_t kind,unsigned length,uint32_t now)
{
    unsigned total=telemetry_frame(kind,length,now);
    Telemetry_Send((const uint8_t *)&telemetry_packet,(uint16_t)total);
}
static void telemetry_reply(void)
{
    /* A disconnected host cannot freeze command service. Pending commands
     * also expire before execution; the PC never retries motion on timeout. */
    if(HAL_GetTick()-reply_at>1000U) { reply_pending=false; return; }
    if(!reply_built) {
        if (reply_kind==33) {
            TelemetryRow *r=&telemetry_packet.payload.row;
            memset(r,0,sizeof(*r)); r->id=reply_id; r->source=reply_source;
            r->axis=reply_row[0]; r->group=reply_row[1]; r->row=reply_row[2];
            int count=App_ReadRow(r->axis,r->group,r->row,r->label,r->value);
            r->result=count>0 ? 1 : (int8_t)count; r->count=count>0 ? (uint8_t)count : 0;
            control_result=r->result;
        } else {
            TelemetryReply *r=&telemetry_packet.payload.reply;
            _Static_assert(sizeof(TelemetryReply)==92 && offsetof(AppSettings,calibration)-offsetof(AppSettings,speed_x10)==84,"RPC settings layout");
            r->id=reply_id; r->result=reply_result; r->source=reply_source;
            r->schema=2; r->firmware=114; r->save=(uint8_t)App_SaveState(); r->focus=(uint8_t)MotorControl_GetFocus();
            memcpy(r->settings,&AppStorage_Get()->speed_x10,sizeof(r->settings));
            r->settings[(offsetof(AppSettings,control_config)-offsetof(AppSettings,speed_x10))/4U]=
                (AppStorage_Get()->control_config & ~(7UL<<28)) | (MotorParams_CustomMask()<<28);
        }
        (void)telemetry_frame(reply_kind,reply_kind==33 ? sizeof(TelemetryRow) : sizeof(TelemetryReply),HAL_GetTick());
        usb_tx_pending_len=0; reply_built=true;
    }
    const unsigned len=sizeof(TelemetryHeader)+(reply_kind==33 ? sizeof(TelemetryRow) : sizeof(TelemetryReply))+2U;
    if(reply_source==PROTO_SRC_USB) {
        if(!CDC_TxReady_FS()) return;
        memcpy(usb_tx_active,&telemetry_packet,len);
        if(CDC_Transmit_FS(usb_tx_active,len)==USBD_OK) reply_pending=false;
    } else if(Uart1_TrySend((const uint8_t *)&telemetry_packet,len)) reply_pending=false;
}
static uint8_t axis_flags(MotorAxis axis,const MotorAxisSnapshot *s)
{
    bool loop=s->current_loop;
    if(axis==MOTOR_AXIS_1) {
        if(MotorControl_HsoAllowed()) loop=true;
        if(MotorControl_SixStepAllowed()) loop=false;
    }
    return s->ready | (s->healthy<<1) | (s->aligned<<2) | (s->active<<3) |
        (loop<<4) | (s->aligning<<5) | (MotorSampling_IsAxisOutputEnabled(axis)<<6) |
        (MotorControl_AxisEnabled(axis)<<7);
}
static void telemetry_status(TelemetryStatus *s,const MotorAxisSnapshot axis[2])
{
    StHsoSnapshot h; StHso_GetSnapshot(&h);
    SixStepSnapshot six; SixStep_GetSnapshot(&six);
    s->meta[0]=MotorControl_GetWork(); s->meta[1]=MotorControl_GetDrive(MOTOR_AXIS_1);
    for(unsigned i=0;i<2;++i) {
        s->meta[2+i]=axis[i].mode; s->meta[4+i]=MotorControl_GetInput((MotorAxis)i);
        s->meta[6+i]=axis_flags((MotorAxis)i,&axis[i]);
    }
    s->meta[8]=h.state; s->meta[9]=h.reason; s->meta[10]=six.state; s->meta[11]=six.reject;
    s->meta[12]=(uint8_t)((telemetry_stream<<1)|SixStep_IsGeneric()|(six.speed_control<<5)|
        (MotorSampling_IsBusFresh()<<6)|(h.position_valid<<7));
    /* M2's snapshot already carries the actual encoder health. M1's feedback
     * health can instead be HSO, so E1 must still query its encoder cache. */
    s->meta[13]=(uint8_t)((MT6701_IsHealthy()!=0)|(axis[1].healthy<<1));
    s->meta[14]=(uint8_t)control_result; s->meta[15]=(uint8_t)hso_result;
    s->values[0]=MotorControl_MaxCycles()/170U;
    s->values[1]=MotorDetect_Result(); s->values[2]=MotorDetect_Inductance(); s->values[3]=MotorDetect_Flux();
    const char *text[]={axis[0].status,axis[1].status,MotorControl_CommandText(),MotorDetect_Status()};
    for(unsigned i=0;i<4;++i) {
        memset(s->text[i],0,16);
        unsigned n=0; while(n<15 && text[i][n]) { s->text[i][n]=text[i][n]; ++n; }
    }
}
static void motion_values(float *out,MotorAxis axis,const MotorAxisSnapshot *s)
{
    bool on=MotorControl_AxisEnabled(axis) && !(axis==MOTOR_AXIS_1 && MotorControl_SixStepAllowed());
    out[0]=on && s->mode!=JERRYFOC_MODE_TORQUE ? s->target_velocity : NAN;
    out[1]=on && s->healthy ? s->velocity : NAN;
    out[2]=on && s->mode==JERRYFOC_MODE_POSITION ? s->target_position : NAN;
    out[3]=on && s->healthy ? s->position : NAN;
    out[4]=on && s->ready && s->active && !MotorDetect_Active() ? s->iq : NAN;
}
static void telemetry_curves(uint8_t stream,const MotorAxisSnapshot axis[2],uint32_t now)
{
    float *data=telemetry_packet.payload.curves; unsigned count=6; uint8_t kind=1; uint16_t format_flags=0;
    float bus=MotorSampling_IsBusFresh() ? MotorSampling_GetBusVoltage() : NAN;
    if((stream==STREAM_AUTO && telemetry_native && MotorControl_HsoAllowed()) || (!telemetry_native && stream==STREAM_HSO)) {
        StHsoSnapshot h; StHso_GetSnapshot(&h); kind=4; count=9;
        bool moving=StHso_IsRunning();
        data[0]=axis[0].target_velocity; data[1]=StHso_ControlVelocity(); data[2]=moving ? axis[0].velocity : NAN;
        data[3]=moving && h.reference_flux_wb>0 ? h.velocity_rad_s : NAN;
        data[4]=axis[0].ready && moving ? axis[0].iq : NAN; data[5]=bus;
        data[6]=h.angle_error_rad; data[7]=h.flux_wb; data[8]=h.position_valid ? h.mechanical_position : NAN;
    } else if((stream==STREAM_AUTO && telemetry_native && MotorControl_SixStepAllowed()) || (!telemetry_native && stream==STREAM_SIX)) {
        SixStepSnapshot s; SixStep_GetSnapshot(&s); kind=5;
        format_flags=(s.speed_control ? 0x8000U : 0U)|(SixStep_IsGeneric() ? 0x4000U : 0U);
        data[0]=s.target; data[1]=s.state==SIX_RUN ? s.velocity : NAN; data[2]=s.reference;
        data[3]=s.duty; data[4]=s.bemf; data[5]=bus;
    } else if(stream==STREAM_FOC1 || stream==STREAM_FOC2) {
        unsigned a=stream-STREAM_FOC1; const MotorAxisSnapshot *s=&axis[a]; kind=(uint8_t)(6+a); count=7;
        bool valid=MotorControl_AxisEnabled((MotorAxis)a) && s->ready && s->healthy && s->active &&
            !MotorDetect_Active() && !(a==0 && (MotorControl_SixStepAllowed() || (MotorControl_HsoAllowed() && !StHso_IsRunning())));
        format_flags=(s->current_loop || (a==0 && MotorControl_HsoAllowed())) ? 0x8000U : 0;
        data[0]=valid ? s->target_iq : NAN; data[1]=valid ? s->iq : NAN;
        MotorSampleFrame sample; MotorSampling_CopyFrame(&sample);
        const float *phase=a==0 ? sample.m1_current : sample.m2_current;
        bool raw=MotorControl_AxisEnabled((MotorAxis)a) && s->ready;
        data[2]=valid ? s->id : NAN; data[3]=raw ? phase[0] : NAN; data[4]=raw ? phase[1] : NAN;
        data[5]=valid ? s->velocity : NAN; data[6]=bus;
    } else {
        /* JustFloat Auto has a fixed 12-channel layout even in Single/HSO/Six.
         * Select HSO/Six explicitly for their dedicated float-only curves. */
        bool dual=stream==STREAM_AUTO && (!telemetry_native || MotorControl_GetWork()>=MOTOR_WORK_INDEPENDENT);
        unsigned a=!dual && (stream==STREAM_M2 || (stream==STREAM_AUTO && MotorControl_GetWork()==MOTOR_WORK_SINGLE_M2));
        kind=dual ? 3 : (uint8_t)(1+a);
        motion_values(data,(MotorAxis)a,&axis[a]);
        if(dual) { motion_values(data+5,MOTOR_AXIS_2,&axis[1]); data[10]=bus;
            data[11]=MotorControl_IsCoupled() ? MotorControl_GroupError() : NAN; count=12;
            if(MotorControl_GetWork()==MOTOR_WORK_SYNC_SPEED) format_flags=0x8000U; }
        else data[5]=bus;
    }
    if(!telemetry_native) {
        /* +Inf is the JustFloat delimiter. Canonical NaN preserves invalid
         * samples without creating a false tail or claiming a zero reading. */
        for(unsigned i=0;i<count;++i) if(!isfinite(data[i])) data[i]=NAN;
        const uint32_t tail=0x7F800000U;
        memcpy(telemetry_packet.payload.bytes+count*4U,&tail,4);
        Telemetry_Send((const uint8_t *)data,(uint16_t)(count*4U+4U));return;
    }
    uint16_t valid=format_flags;
    for(unsigned i=0;i<count;++i) if(isfinite(data[i])) valid|=(uint16_t)(1U<<i);
    unsigned length=count*4U;
    memcpy(telemetry_packet.payload.bytes+length,&valid,2);
    telemetry_emit(kind,length+2U,now);
}
static void telemetry_produce(void)
{
    uint32_t now=HAL_GetTick();
    if(now-telemetry_tick<10U) return;
    telemetry_tick=now;
    MotorAxisSnapshot axis[2];
    MotorControl_GetAxis(MOTOR_AXIS_1,&axis[0]); MotorControl_GetAxis(MOTOR_AXIS_2,&axis[1]);
    uint8_t stream=telemetry_stream;
    if(!telemetry_native) {
        if(stream!=STREAM_OFF) telemetry_curves(stream,axis,now);
        return; /* Never interleave Native heartbeats, diagnostics or RPC. */
    }
    TelemetryStatus *status=&telemetry_packet.payload.status; telemetry_status(status,axis);
    /* Coalesce rapid state chatter so it cannot starve all curve slots. */
    if(!telemetry_have_status || now-telemetry_status_tick>=100U ||
        (now-telemetry_status_tick>=20U && memcmp(status,&telemetry_previous,sizeof(*status)))) {
        telemetry_previous=*status; telemetry_have_status=true; telemetry_status_tick=now;
        telemetry_emit(0x10,sizeof(*status),now); return;
    }
    if(stream==STREAM_OFF) return;
    if(stream>=STREAM_ENC1) {
        if(now-telemetry_diagnostic_tick<100U) return;
        telemetry_diagnostic_tick=now;
    }
    if(stream==STREAM_ENC1 || stream==STREAM_ENC2) {
        MT6701Diagnostics e; unsigned a=stream-STREAM_ENC1;
        if(a==0) MT6701_GetDiagnostics(&e); else MT6701_2_GetDiagnostics(&e);
        MT6701_2_Snapshot e2; MT6701_2_GetSnapshot(&e2);
        TelemetryEncoder *wire=&telemetry_packet.payload.encoder;
        _Static_assert(offsetof(MT6701Diagnostics,age_cycles)==24 && offsetof(MT6701Diagnostics,zero_started_at)==36 &&
            offsetof(MT6701Diagnostics,status)==44,"Review encoder diagnostic prefixes");
        memcpy(wire->counts,&e,24); memcpy(wire->counts+6,&e.zero_started_at,8);
        memcpy(wire->values,&e.age_cycles,12);
        wire->values[6]=a==0 ? MT6701_GetRawAngle() : e2.raw_angle;
        wire->status=e.status; wire->error=e.last_error;
        telemetry_emit((uint8_t)(8+a),sizeof(*wire),now);
    } else if(stream==STREAM_HSO) {
        StHsoSnapshot h; StHso_GetSnapshot(&h);
        TelemetryHso *wire=&telemetry_packet.payload.hso;
        _Static_assert(offsetof(StHsoSnapshot,emf_offset_limit_v)==offsetof(StHsoSnapshot,capture_angle_rad)+20 &&
            offsetof(StHsoSnapshot,capture_resets)==offsetof(StHsoSnapshot,qualified_samples)+8,"Review HSO diagnostic prefixes");
        wire->data[0]=h.reference_flux_wb; memcpy(wire->data+1,&h.capture_angle_rad,24);
        memcpy(wire->counts,&h.qualified_samples,12);
        wire->flags[0]=h.entry_reason; wire->flags[1]=h.capture_reason; wire->flags[2]=h.position_valid; wire->flags[3]=h.usable;
        telemetry_emit(10,sizeof(*wire),now);
    } else if(stream==STREAM_SIX) {
        SixStepSnapshot s; SixStep_GetSnapshot(&s);
        /* Explicit fields, never pointers/padding from the native snapshot. */
        TelemetrySix *wire=&telemetry_packet.payload.six;
        wire->startup_emf=s.startup_emf; wire->crossings=s.crossings;
        uint16_t values[]={s.phase_pct,s.period_pct,s.period_fraction,s.startup_ms,s.sector,s.quality,s.gate};
        memcpy(wire->values,values,sizeof(values)); memcpy(wire->values+7,s.period_10us,sizeof(s.period_10us));
        wire->flags[0]=s.state; wire->flags[1]=s.sampling_off; wire->flags[2]=s.speed_control; wire->flags[3]=s.reject;
        telemetry_emit(11,38U,now);
    } else telemetry_curves(stream,axis,now);
}
