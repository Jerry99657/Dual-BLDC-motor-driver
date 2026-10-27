#define TELEMETRY_CAPTURE
#define main protocol_fixture_main
#include "test_protocol.c"
#undef main
extern uint32_t telemetry_host_tick;
extern bool control_stub_detect_active;
static FILE *trace;
static uint8_t observed[17];
static unsigned curve_frames;
static void packet(void) {
    unsigned old=submissions;
    usb_busy=false; telemetry_host_tick+=10U; Telemetry_Service();
    if(old==submissions) return;
    assert(inflight_len==uart_capture_length && !memcmp(inflight,uart_capture,inflight_len));
    assert(inflight[0]==0xA5 && inflight[1]==0x5A && inflight[2]==1);
    uint16_t length;memcpy(&length,inflight+6,2);assert(inflight_len==length+14U);
    uint16_t crc=0xFFFF;
    for(unsigned i=0;i<inflight_len-2U;++i) {
        crc^=(uint16_t)inflight[i]<<8;
        for(unsigned b=0;b<8;++b) crc=(uint16_t)((crc<<1)^((crc&0x8000) ? 0x1021 : 0));
    }
    uint16_t actual;memcpy(&actual,inflight+inflight_len-2,2);assert(crc==actual);
    assert(inflight[3]<=16);observed[inflight[3]]=1;
    if(inflight[3]==1) ++curve_frames;
    assert(fwrite(inflight,1,inflight_len,trace)==inflight_len);
    assert(host_primask==0);
}
static void pump(void) { for(unsigned i=0;i<25;++i) packet(); }
int main(void) {
    command("Format:Native");assert(Telemetry_IsNative());
    uart_busy=false;usb_connected=true;complete_early=true;
    trace=fopen("build/telemetry-r86/wire-fixture.bin","wb");assert(trace);
    assert(MotorControl_SelectWork(MOTOR_WORK_SINGLE_M1));pump();assert(observed[1] && observed[16]);
    uint8_t saved[128];unsigned saved_length=uart_capture_length;memcpy(saved,uart_inflight,saved_length);
    uart_busy=true;
    uint8_t replacement[12]={0};Telemetry_Send(replacement,sizeof(replacement));
    assert(!memcmp(saved,uart_inflight,saved_length));uart_busy=false;
    unsigned curves_before=curve_frames;
    for(unsigned i=0;i<40;++i) { ready=!ready;packet(); }
    assert(curve_frames>curves_before);ready=true; /* Chattering status cannot starve every curve. */
    control_stub_detect_active=true;unsigned detecting=submissions;pump();assert(submissions>detecting);
    control_stub_detect_active=false;
    assert(MotorControl_SelectWork(MOTOR_WORK_SINGLE_M2));pump();assert(observed[2]);
    assert(MotorControl_SelectWork(MOTOR_WORK_INDEPENDENT));pump();assert(observed[3]);
    assert(MotorControl_SelectWork(MOTOR_WORK_SINGLE_M1));assert(MotorControl_SelectFeedback(true));pump();assert(observed[4]);
    assert(MotorControl_SelectFeedback(false));assert(MotorControl_SelectSixStep(true));pump();assert(observed[5]);
    const char *streams[]={"Stream:FOC1","Stream:FOC2","Stream:ENC1","Stream:ENC2","Stream:HSO","Stream:Six"};
    for(unsigned i=0;i<6;++i) { command(streams[i]);assert(Protocol_GetControlResult()==1);pump();assert(observed[6+i]); }
    command("Stream:wrong");assert(Protocol_GetControlResult()==-2);
    command("M1:Stream:Auto");assert(Protocol_GetControlResult()==-2);
    command("Stream:Off");pump();unsigned before=submissions;pump();assert(submissions>before); /* Heartbeat survives Off. */
    telemetry_host_tick=UINT32_MAX-12U;pump(); /* Unsigned millisecond wrap. */
    assert(fclose(trace)==0);
    puts("PASS: R86 real producer: USB/UART identical framing/CRC, all 12 packet types, stream admission, integer diagnostics, Off heartbeat and timestamp wrap");
    return 0;
}
