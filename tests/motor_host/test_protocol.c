#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <float.h>
#include <stdint.h>
#include "protocol.h"
#include "JerryFOC.h"
#include "motor_control.h"
#include "app_storage.h"
#include "st_hso.h"
#include "usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

uint32_t host_primask;
UART_HandleTypeDef huart1;
USBD_HandleTypeDef hUsbDeviceFS;
volatile uint8_t g_usb_cdc_ready;
volatile float JerryFOC_Target_Iq, JerryFOC_Target_Position;
volatile uint8_t g_openloop_debug;
volatile float g_openloop_angle;
static bool ready=true, active, fault, usable;
bool MotorSampling_ResumeGlobalStop(void) { return true; }
static float velocity=10, target;
static JerryFOC_ControlMode mode;
static StHsoConfig config={2.3f,0.00086f,0,25000,0.019f};
static unsigned configurations, learns, disables;
static bool observer_enabled=true;
static uint8_t uart_capture[128];
static uint16_t uart_capture_length;
static uint8_t *uart_inflight;
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h,uint8_t *p,uint16_t n) {
    assert(n<=sizeof(uart_capture)); memcpy(uart_capture,p,n); uart_capture_length=n;
    uart_inflight=p;
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
void JerryFOC_stop(void) { active=false; target=0; mode=JERRYFOC_MODE_TORQUE; g_openloop_debug=0; }
uint8_t JerryFOC_startOutput(void) { return ready; }
void StHso_Enable(bool en) { observer_enabled=en; }
bool StHso_LearnFlux(void) { ++learns; return observer_enabled; }
void StHso_GetConfig(StHsoConfig *c) { *c=config; }
bool StHso_SetConfig(const StHsoConfig *c) {
    assert(host_primask==0); /* No broad interrupt mask during float parsing/config. */
    if(c->rs_ohm<=0 || c->ls_h<=0 || c->flux_wb<0) return false;
    config=*c; ++configurations; return true;
}
static void enqueue(const char *s) {
    Protocol_Feed(PROTO_SRC_USB,(const uint8_t *)s,(uint32_t)strlen(s));
    Protocol_Feed(PROTO_SRC_USB,(const uint8_t *)"\r\n",2);
}

static void command(const char *s) { enqueue(s); Protocol_Service(); }

static uint32_t number_digest=2166136261U;
static FILE *number_trace;
static void digest_target(void) {
    uint32_t bits;memcpy(&bits,&target,sizeof(bits));
    number_digest=(number_digest^bits)*16777619U;
    if(number_trace) assert(fwrite(&bits,sizeof(bits),1,number_trace)==1);
}
static void check_configuration_queue(void) {
    MotorControl_StopAll(); MotorControl_SelectWork(MOTOR_WORK_SINGLE_M1);
    MotorControl_SelectFeedback(false);
    MotorParams_SelectAxis(MOTOR_AXIS_1,MOTOR_PROFILE_2804);
    MotorParams_SelectAxis(MOTOR_AXIS_2,MOTOR_PROFILE_2804);
    enqueue("Work:Independent"); enqueue("Motor1:4015"); enqueue("Motor2:4015"); enqueue("Focus:M2");
    assert(Protocol_GetControlResult()==0 && MotorControl_GetWork()==MOTOR_WORK_SINGLE_M1);
    enqueue("Speed:123"); assert(Protocol_GetControlResult()==-3 && target==0);
    enqueue("Stop"); assert(Protocol_GetControlResult()==1 && target==0);
    Protocol_Service(); assert(MotorControl_GetWork()==MOTOR_WORK_SINGLE_M1); /* Stop cancelled the batch. */
    enqueue("Work:Independent"); enqueue("Motor1:4015"); enqueue("Motor2:4015"); enqueue("Focus:M2");
    Protocol_Service();
    assert(MotorControl_GetWork()==MOTOR_WORK_INDEPENDENT && MotorControl_GetFocus()==MOTOR_AXIS_2);
    assert(MotorParams_GetAxisProfile(MOTOR_AXIS_1)==MOTOR_PROFILE_4015 && MotorParams_GetAxisProfile(MOTOR_AXIS_2)==MOTOR_PROFILE_4015);
    const char *work_names[]={"SingleM1","SingleM2","Independent","SyncSpeed","SyncPosition","MasterM1","MasterM2"};
    for(unsigned i=0;i<MOTOR_WORK_COUNT;++i) {
        char line[64];snprintf(line,sizeof(line),"Work:%s",work_names[i]);
        command(line);assert(Protocol_GetControlResult()==1 && MotorControl_GetWork()==(MotorWorkMode)i);
    }
    command("Work:SingleM1");command("Motor1:2804");command("Motor2:2804");
    command("Feedback:HSO");assert(MotorControl_HsoAllowed());
    command("Feedback:SixStep");assert(MotorControl_SixStepAllowed() && !MotorControl_HsoAllowed());
    command("Feedback:Encoder");assert(!MotorControl_SixStepAllowed() && !MotorControl_HsoAllowed());
    command("Focus:M1");command("Align:M2");assert(Protocol_GetControlResult()==1);
    const char *gears[]={"Gear:-2.5,0.1","Gear:1e-2,-6.283e1","Gear:0x1p1,0x.8p1"};
    for(unsigned i=0;i<sizeof(gears)/sizeof(*gears);++i) { command(gears[i]);assert(Protocol_GetControlResult()==1); }
    const char *invalid[]={"Work:bad","Motor1:40150","Motor2:","Feedback:HSOx","Feedback:","Gear:0,0","Gear:1","Gear:1,0junk","Align:M3","Focus:M0"};
    for(unsigned i=0;i<sizeof(invalid)/sizeof(*invalid);++i) {
        uint32_t before=AppStorage_Get()->control_config;command(invalid[i]);
        assert(Protocol_GetControlResult()==-1 && AppStorage_Get()->control_config==before);
    }
    command("work:SingleM1");assert(Protocol_GetControlResult()==-2);
    char line[65];memset(line,'x',sizeof(line));memcpy(line,"Work:",5);line[63]=0;
    Protocol_HandleLine(line);assert(Protocol_GetControlResult()==0);Protocol_Service();assert(Protocol_GetControlResult()==-1);
    line[63]='x';line[64]=0;Protocol_HandleLine(line);assert(Protocol_GetControlResult()==-2);
    Protocol_Feed(PROTO_SRC_USART,(const uint8_t *)"Mo",2);enqueue("Work:SingleM1");
    Protocol_Feed(PROTO_SRC_USART,(const uint8_t *)"tor1:4015\n",sizeof("tor1:4015\n")-1);Protocol_Service();
    assert(MotorParams_GetActiveProfile()==MOTOR_PROFILE_4015);
    char overflow[100];memset(overflow,'x',sizeof(overflow));memcpy(overflow,"Work:",5);
    Protocol_Feed(PROTO_SRC_USB,(const uint8_t *)overflow,sizeof(overflow));Protocol_Feed(PROTO_SRC_USB,(const uint8_t *)"\n",1);Protocol_Service();
    command("Motor1:2804");assert(host_primask==0 && target==0);
    puts("PASS: main-loop FIFO, all work/model/feedback/axis commands, queue-full, immediate Stop cancels pending commands, bounds and fragmented dual-source lines");
}
static void check_numbers(void) {
    const char *valid[]={"0","-0","+.5","-.25"," 1.25","\t1e1","1E-3","1e+2",
        "0x1.8p+2","-0X1.Ap-2","0x1e","0x.8p1","0x1.","1e-45","1e-1000",
        "0e9999","-0e-9999","3.4028235e38","1.17549435e-38",
        "123456789012345678901234567890123456789012345678901e-49"};
    for(unsigned i=0;i<sizeof(valid)/sizeof(*valid);++i) {
        char line[64]; snprintf(line,sizeof(line),"Speed:%s",valid[i]);
        float expected=strtof(valid[i],NULL); command(line);
        assert(Protocol_GetControlResult()==1 && target==expected);
        assert((signbit(target)!=0)==(signbit(expected)!=0));
        digest_target();
    }
    const char *invalid[]={"","+",".","1e","1e+","1e-","1.2.3","0x","0x.p1",
        "0x1p","nan","NAN(0)","inf","-Infinity","1e1000","0x1p9999","1 ","1,2","1junk"};
    for(unsigned i=0;i<sizeof(invalid)/sizeof(*invalid);++i) {
        char line[64]; snprintf(line,sizeof(line),"Speed:%s",invalid[i]); float before=target;
        command(line); assert(Protocol_GetControlResult()==-2 && target==before);
    }
    /* Deterministic binary32 samples round-trip through the actual USB parser.
     * Permit one float ULP for decimal->double->float rounding at a tie. */
    uint32_t rng=0x12345678;
    for(unsigned i=0;i<20000;++i) {
        rng=rng*1664525U+1013904223U;
        float value; memcpy(&value,&rng,sizeof(value)); if(!isfinite(value)) continue;
        char line[64]; snprintf(line,sizeof(line),"Speed:%.9g",(double)value);
        command(line); assert(Protocol_GetControlResult()==1);
        assert(fabsf(target-value)<=fmaxf(FLT_TRUE_MIN,fabsf(value)*FLT_EPSILON));
        digest_target();
        snprintf(line,sizeof(line),"Speed:%a",(double)value);
        command(line);assert(Protocol_GetControlResult()==1 && target==value);digest_target();
    }
    command("Speed:20");
    printf("PASS: compact finite-number parser: decimal/exponent/hex/sign/whitespace, malformed/overflow rejection, subnormals and 20000 decimal + hex round trips; digest %08lx\n",(unsigned long)number_digest);
}
static void check_rpc(void) {
    extern uint32_t telemetry_host_tick;
    extern bool telemetry_host_stopped;
    FILE *trace=fopen("build/console-r87/rpc-fixture.bin","wb");assert(trace);
    MotorControl_StopAll();MotorControl_SelectWork(MOTOR_WORK_SINGLE_M1);MotorControl_SelectFeedback(false);
    AppStorage_Get()->speed_x10=123;AppStorage_Get()->led_brightness=37;
    AppSettings *settings=AppStorage_Get();
    settings->speed_step_x10=settings->torque_step_x100=settings->position_step_x100=5;
    settings->axis2_speed_step_x10=settings->axis2_torque_step_x100=settings->axis2_position_step_x100=5;
    settings->gear_ratio_x100=100;
    usb_connected=true;usb_busy=false;complete_early=true;submit_result=USBD_OK;
    telemetry_host_tick=100;assert(Telemetry_Select("Off"));
    command("@1:Get");Telemetry_Service();
    assert(inflight_len==106 && inflight[3]==32 && inflight[12]==1 && inflight[14]==1 && inflight[15]==PROTO_SRC_USB);
    assert(inflight[16]==2 && inflight[17]==89);
    assert(inflight[18]==APP_SAVE_PENDING); /* Dirty config must not claim an older save completed. */
    int32_t saved;memcpy(&saved,inflight+20,4);assert(saved==123);
    assert(fwrite(inflight,1,inflight_len,trace)==inflight_len);
    command("@2:Speed:19");assert(target==19);Telemetry_Service();
    assert(inflight[12]==2 && inflight[14]==1);
    command("@3:Speed:bad");Telemetry_Service();assert(inflight[12]==3 && (int8_t)inflight[14]==-2 && target==19);
    command("@4:Work:bad");Telemetry_Service();assert((int8_t)inflight[14]==-1);
    unsigned before=submissions;
    Protocol_Feed(PROTO_SRC_USART,(const uint8_t *)"@5:Get\n",7);Protocol_Service();
    huart1.gState=HAL_UART_STATE_READY^1U;Telemetry_Service();assert(submissions==before);
    huart1.gState=HAL_UART_STATE_READY;Telemetry_Service();
    assert(submissions==before && uart_capture_length==106 && uart_capture[12]==5 && uart_capture[15]==PROTO_SRC_USART);
    assert(fwrite(uart_capture,1,uart_capture_length,trace)==uart_capture_length);
    /* Failed USB submission retains the complete reply, not the next curve. */
    submit_result=USBD_BUSY;command("@6:Get");Telemetry_Service();
    unsigned blocked=submissions;for(unsigned i=0;i<20;++i) Telemetry_Service();assert(submissions==blocked+20);
    submit_result=USBD_OK;Telemetry_Service();assert(inflight[12]==6 && inflight[14]==1);
    /* TX stall never delays Stop and queued motion cannot run after Stop. */
    usb_busy=true;command("@7:Get");Telemetry_Service();enqueue("@8:Speed:30");enqueue("@9:Stop");
    assert(target==0);Protocol_Service();assert(target==0);
    usb_busy=false;Telemetry_Service();Protocol_Service();Telemetry_Service();
    assert(inflight[12]==9 && target==0);
    /* Drop an undeliverable reply; do not replay an old movement afterward. */
    usb_busy=true;command("@10:Get");Telemetry_Service();enqueue("@11:Speed:33");
    telemetry_host_tick+=1001;Telemetry_Service();Protocol_Service();usb_busy=false;Telemetry_Service();
    assert(inflight[12]==11 && (int8_t)inflight[14]==-4 && target==0);
    enqueue("@12:Speed:20");telemetry_host_stopped=true;Protocol_Service();Telemetry_Service();
    assert(inflight[12]==12 && (int8_t)inflight[14]==-4 && target==0);telemetry_host_stopped=false;
    telemetry_host_stopped=true;enqueue("@13:Speed:20");telemetry_host_tick+=11;Protocol_Service();Telemetry_Service();
    assert(inflight[12]==13 && (int8_t)inflight[14]==-4 && target==0);telemetry_host_stopped=false;
    unsigned calls=submissions;command("@0:Speed:12");command("@65536:Speed:12");command("@x:Speed:12");
    assert(target==0 && submissions==calls);
    extern unsigned app_host_axis,app_host_key;
    extern int32_t app_host_value;
    extern bool app_host_config_allowed;
    command("App:2,8,17");assert(Protocol_GetControlResult()==1 && app_host_axis==2 && app_host_key==8 && app_host_value==17);
    const char *invalid_app[]={"App:1,2,2.5","App:1,2,NaN","App:1,2,1e10","App:1,2","App:1,2,3junk","GetRow:0,1,0","GetRow:1,8,0","GetRow:1,0,-1"};
    for(unsigned i=0;i<sizeof(invalid_app)/sizeof(*invalid_app);++i) {
        command(invalid_app[i]);assert(Protocol_GetControlResult()==-2);
    }
    app_host_config_allowed=false;command("Focus:M2");assert(Protocol_GetControlResult()==-3);app_host_config_allowed=true;
    command("@21:GetRow:2,4,0");Telemetry_Service();
    assert(inflight_len==78 && inflight[3]==33 && inflight[12]==21 && inflight[14]==1 && inflight[15]==PROTO_SRC_USB);
    assert(inflight[16]==2 && inflight[17]==4 && inflight[18]==0 && inflight[19]==2);
    assert(!strcmp((char *)inflight+20,"Test") && !strcmp((char *)inflight+44,"24.0 V"));
    FILE *row_trace=fopen("build/console-r88/rpc-row-fixture.bin","wb");assert(row_trace);
    assert(fwrite(inflight,1,inflight_len,row_trace)==inflight_len);assert(fclose(row_trace)==0);
    command("@22:GetRow:1,0,255");Telemetry_Service();assert(inflight[3]==33 && (int8_t)inflight[14]==-2);
    assert(fclose(trace)==0);assert(host_primask==0);
    puts("PASS: real RPC schema/settings bytes, IDs, exact results, USB/UART reply routing, BUSY retry, immediate Stop, queue cancellation, disconnected reply timeout, stale-motion and KEY1 cancellation");
}
int main(void) {
    const char *trace=getenv("PROTO_NUMBER_TRACE");
    if(trace) { number_trace=fopen(trace,"wb");assert(number_trace); }
    check_usb_telemetry();
    check_configuration_queue();
    MotorControl_SelectFeedback(true);
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
    unsigned before_m2_stop=disables;
    command("M2:Stop"); assert(active && disables==before_m2_stop);
    command("HSO:Motor:1,1,1"); assert(Protocol_GetHsoResult()==-3 && configurations==2);
    command("HSO:Learn"); assert(Protocol_GetHsoResult()==-3 && learns==0);
    command("Stop"); assert(!active && disables==1 && target==0);
    velocity=0; command("HSO:Learn"); assert(Protocol_GetHsoResult()==1 && learns==1);
    velocity=10; command("HSO:Learn"); assert(Protocol_GetHsoResult()==1 && learns==2);
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
    Protocol_Service();
    assert(configurations==4 && config.voltage_pole_rps==20000);
    check_numbers();
    check_rpc();
    if(number_trace) { assert(fclose(number_trace)==0);number_trace=NULL; }
    assert(host_primask==0);
    puts("PASS: real HSO protocol parsing, strict numbers, gates, stop, overflow discard, independent fragmented transports");
    return 0;
}
