#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "protocol.h"
#include "JerryFOC.h"
#include "st_hso.h"
#include "usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

uint32_t host_primask;
UART_HandleTypeDef huart1;
USBD_HandleTypeDef hUsbDeviceFS;
volatile uint8_t g_usb_cdc_ready;
volatile uint8_t g_openloop_debug;
volatile float g_openloop_angle;
static bool ready=true, active, fault, usable;
static float velocity=10, target;
static JerryFOC_ControlMode mode;
static StHsoConfig config={2.3f,0.00086f,0,25000,0.019f};
static unsigned configurations, learns, disables;
static bool observer_enabled=true;
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h,uint8_t *p,uint16_t n) {
    (void)h; (void)p; (void)n; return HAL_OK;
}
static bool usb_connected, usb_busy, complete_early, publish_during_submit;
static uint8_t submit_result;
static unsigned submissions;
static uint8_t *inflight;
static uint16_t inflight_len;
static uint8_t next_frame[104];
uint8_t CDC_TxReady_FS(void) { return usb_connected && !usb_busy; }
uint8_t CDC_Transmit_FS(uint8_t *p,uint16_t n) {
    assert(host_primask==0); /* Mailbox critical section ended before submission. */
    ++submissions;
    if(submit_result!=USBD_OK) return submit_result;
    assert(usb_connected && !usb_busy);
    inflight=p; inflight_len=n; usb_busy=true;
    if(publish_during_submit) {
        publish_during_submit=false;
        Telemetry_Send(next_frame,sizeof(next_frame));
    }
    /* Model the completion interrupt before CDC_Transmit_FS returns. */
    if(complete_early) usb_busy=false;
    return USBD_OK;
}
static void check_usb_telemetry(void) {
    uint8_t frame[104];
    memset(frame,0x11,sizeof(frame));
    Telemetry_Send(frame,sizeof(frame)); Telemetry_Service();
    assert(submissions==0); /* Disconnected. */
    usb_connected=true;
    Telemetry_Service();
    assert(submissions==1 && usb_busy && inflight_len==sizeof(frame));
    for(unsigned i=0;i<1000;++i) {
        memset(frame,(uint8_t)i,sizeof(frame));
        Telemetry_Send(frame,sizeof(frame)); Telemetry_Service();
        assert(submissions==1);
        for(unsigned j=0;j<inflight_len;++j) assert(inflight[j]==0x11);
    }
    /* Unread host data stays immutable, pending queue remains bounded/latest. */
    usb_busy=false; Telemetry_Service();
    assert(submissions==2 && memcmp(inflight,frame,sizeof(frame))==0);
    usb_busy=false; complete_early=true;
    memset(next_frame,0x77,sizeof(next_frame));
    publish_during_submit=true;
    Telemetry_Send(frame,sizeof(frame)); Telemetry_Service();
    assert(submissions==3 && !usb_busy);
    Telemetry_Service(); /* Must retain the new frame published inside submit. */
    assert(submissions==4 && memcmp(inflight,next_frame,sizeof(next_frame))==0);
    for(unsigned i=0;i<1000;++i) {
        Telemetry_Send(frame,sizeof(frame)); Telemetry_Service();
    }
    assert(submissions==1004); /* No stuck flag after early completions. */
    submit_result=USBD_BUSY;
    Telemetry_Send(frame,sizeof(frame)); Telemetry_Service();
    submit_result=USBD_FAIL;
    Telemetry_Send(frame,sizeof(frame)); Telemetry_Service();
    submit_result=USBD_OK;
    Telemetry_Send(frame,sizeof(frame)); Telemetry_Service();
    assert(submissions==1007 && !usb_busy);
    usb_connected=false;
    Telemetry_Send(next_frame,sizeof(next_frame)); Telemetry_Service();
    assert(submissions==1007);
    usb_connected=true; Telemetry_Service();
    assert(submissions==1008 && memcmp(inflight,next_frame,sizeof(next_frame))==0);
    Telemetry_Send(NULL,4); Telemetry_Send(frame,0); Telemetry_Send(frame,129);
    Telemetry_Service(); assert(submissions==1008 && host_primask==0);
    puts("PASS: USB TX early completion, immutable active frame, latest mailbox, interrupt publication, BUSY/failure/reconnect");
}
uint8_t JerryFOC_isHSOActive(void) { return active; }
uint8_t JerryFOC_hasHSOFault(void) { return fault; }
uint8_t JerryFOC_isDriveReady(void) { return ready; }
float JerryFOC_getVelocity(void) { return velocity; }
const MotorParams *JerryFOC_getMotorParams(void) { return &MOTOR_2804; }
uint8_t JerryFOC_useHSO(uint8_t enable) {
    if(enable) { if(!usable) return 0; active=true; }
    else { ++disables; active=fault=false; target=0; }
    return 1;
}
void JerryFOC_useCurrentLoop(uint8_t en) { (void)en; }
void JerryFOC_setMode(JerryFOC_ControlMode m) { mode=m; }
void JerryFOC_setVelocity(float v) { target=v; }
void JerryFOC_setPosition(float p,float l) { (void)l; target=p; }
void JerryFOC_setCurrent(float v) { target=v; }
void StHso_Enable(bool en) { observer_enabled=en; }
bool StHso_LearnFlux(void) { ++learns; return observer_enabled; }
void StHso_GetConfig(StHsoConfig *c) { *c=config; }
bool StHso_SetConfig(const StHsoConfig *c) {
    assert(host_primask==0); /* No broad interrupt mask during float parsing/config. */
    if(c->rs_ohm<=0 || c->ls_h<=0 || c->flux_wb<0) return false;
    config=*c; ++configurations; return true;
}
static void command(const char *s) {
    Protocol_Feed(PROTO_SRC_USB,(const uint8_t *)s,(uint32_t)strlen(s));
    Protocol_Feed(PROTO_SRC_USB,(const uint8_t *)"\r\n",2);
}
int main(void) {
    check_usb_telemetry();
    command("HSO:Motor:2.3,0.00086,0.003");
    assert(Protocol_GetHsoResult()==1 && configurations==1 && config.flux_wb==0.003f);
    const char *invalid[]={"HSO:Motor:nan,1,1","HSO:Motor:2,inf,1","HSO:Motor:2,1,",
        "HSO:Motor:2,1,0.003junk","HSO:Motor:2,1,0.003,4","HSO:Motor:2,1,-1"};
    for(unsigned i=0;i<sizeof(invalid)/sizeof(*invalid);++i) {
        command(invalid[i]); assert(Protocol_GetHsoResult()==-2 && configurations==1);
    }
    command("HSO:Filter:25000,0.019"); assert(configurations==2);
    command("HSO:Use"); assert(Protocol_GetHsoResult()==-3 && !active);
    usable=true; command("HSO:Use"); assert(Protocol_GetHsoResult()==1 && active);
    command("HSO:Motor:1,1,1"); assert(Protocol_GetHsoResult()==-3 && configurations==2);
    command("HSO:Learn"); assert(Protocol_GetHsoResult()==-3 && learns==0);
    command("Stop"); assert(!active && disables==1 && target==0);
    velocity=0; command("HSO:Learn"); assert(Protocol_GetHsoResult()==-3 && learns==0);
    velocity=10; command("HSO:Learn"); assert(Protocol_GetHsoResult()==1 && learns==1);
    command("HSO:Off"); assert(!observer_enabled);
    command("HSO:On"); assert(observer_enabled);
    command("HSO:Unsupported"); assert(Protocol_GetHsoResult()==-1);
    /* A valid prefix must never execute if the rest of a line overflowed. */
    char long_line[100]; memset(long_line,'0',sizeof(long_line));
    memcpy(long_line,"HSO:Motor:2.3,0.00086,0.",sizeof("HSO:Motor:2.3,0.00086,0.")-1U);
    Protocol_Feed(PROTO_SRC_USB,(uint8_t *)long_line,sizeof(long_line));
    Protocol_Feed(PROTO_SRC_USB,(uint8_t *)"\n",1);
    assert(configurations==2);
    command("HSO:Motor:2.3,0.00086,0.004"); assert(configurations==3);
    /* USART and USB fragmented lines must remain independent. */
    Protocol_Feed(PROTO_SRC_USART,(uint8_t *)"HSO:Fil",7);
    command("Speed:20"); assert(target==20 && mode==JERRYFOC_MODE_VELOCITY);
    Protocol_Feed(PROTO_SRC_USART,(uint8_t *)"ter:20000,0.01\n",sizeof("ter:20000,0.01\n")-1U);
    assert(configurations==4 && config.voltage_pole_rps==20000);
    assert(host_primask==0);
    puts("PASS: real HSO protocol parsing, strict numbers, gates, stop, overflow discard, independent fragmented transports");
    return 0;
}
