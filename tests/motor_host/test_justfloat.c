/* Actual producer/parser/transports, with the existing board snapshot model. */
#define TELEMETRY_CAPTURE
#define main protocol_fixture_main
#include "test_protocol.c"
#undef main
extern uint32_t telemetry_host_tick;
static FILE *jf_trace;
static void jf(unsigned count)
{
    usb_busy=false;unsigned before=submissions;
    telemetry_host_tick+=10U;Telemetry_Service();
    if(!count) { assert(submissions==before);return; }
    assert(submissions==before+1 && inflight_len==count*4U+4U);
    assert(inflight_len==uart_capture_length && !memcmp(inflight,uart_capture,inflight_len));
    uint32_t bits;
    for(unsigned i=0;i<=count;++i) {
        memcpy(&bits,inflight+4U*i,4);
        if(i==count) assert(bits==0x7F800000U);
        else { float value;memcpy(&value,&bits,4);assert(!isinf(value));if(isnan(value)) assert(bits==0x7FC00000U); }
    }
    if(jf_trace) assert(fwrite(inflight,1,inflight_len,jf_trace)==inflight_len);
    assert(host_primask==0);
}
static float channel(unsigned i) { float value;memcpy(&value,inflight+i*4U,4);return value; }
int main(void)
{
    assert(!Telemetry_IsNative() && !strcmp(Telemetry_FormatText(),"JustFloat"));
    usb_connected=true;complete_early=true;
    jf_trace=fopen("build/telemetry-r86/justfloat-fixture.bin","wb");assert(jf_trace);
    jf(12);assert(channel(10)==12 && isnan(channel(5)));
    command("@90:Get");jf(12); /* No RPC/status bytes in a pure JustFloat stream. */
    command("Work:SingleM2");jf(12);assert(isnan(channel(0)) && isnan(channel(1)));
    command("Work:Independent");jf(12); /* Auto count and channel slots never change. */
    assert(fclose(jf_trace)==0);jf_trace=NULL;
    command("Stream:M1");jf(6);assert(channel(5)==12);
    velocity=INFINITY;jf(6);assert(isnan(channel(1)));velocity=10;
    command("Stream:M2");jf(6);assert(channel(5)==12);
    command("Stream:FOC1");jf(7);command("Stream:FOC2");jf(7);
    command("Stream:HSO");jf(9);command("Stream:Six");jf(6);
    command("Stream:ENC1");assert(Protocol_GetControlResult()==-2);jf(6);
    command("Stream:Off");jf(0);jf(0); /* No hidden heartbeat. */
    command("M1:Format:Native");assert(Protocol_GetControlResult()==-2 && !Telemetry_IsNative());
    command("Format:wrong");assert(Protocol_GetControlResult()==-2 && !Telemetry_IsNative());
    /* Native handshake is ordinary ASCII, succeeds through either RX source. */
    Protocol_Feed(PROTO_SRC_USART,(const uint8_t *)"Format:Nat",10);
    Protocol_Feed(PROTO_SRC_USART,(const uint8_t *)"ive\n",4);
    Protocol_Service();assert(Telemetry_IsNative());
    command("@91:Get");Telemetry_Service();assert(inflight_len==106 && inflight[3]==32 && inflight[12]==91);
    assert(inflight[17]==103 && inflight[14]==1);
    command("Stream:ENC2");assert(Protocol_GetControlResult()==1);
    command("Format:JustFloat");jf(12); /* Native-only diagnostics reset safely to Auto. */
    command("@92:Format:Native");Telemetry_Service();assert(inflight[3]==32 && inflight[12]==92);
    /* Changing from the UI while an RPC is blocked cancels that old-format
     * pending reply/mailbox; active buffers must remain immutable to TC. */
    complete_early=false;usb_busy=false;command("@93:Get");Telemetry_Service();
    uint8_t active_copy[128];uint16_t active_len=inflight_len;memcpy(active_copy,inflight,active_len);
    usb_busy=true;command("@94:Get");Telemetry_Service();
    uint8_t stale[28];memset(stale,0xCC,sizeof(stale));Telemetry_Send(stale,sizeof(stale));
    Telemetry_SetNative(false);assert(!memcmp(active_copy,inflight,active_len));
    jf(12);assert(inflight[0]!=0xCC); /* Stale bytes/replies were discarded. */
    command("M1:Speed:12");assert(target==12);command("@95:Stop");assert(target==0);jf(12);
    command("Format:Native");usb_busy=false;command("@96:Get");Telemetry_Service();
    assert(inflight[3]==32 && inflight[12]==96 && inflight[14]==1);
    command("Stream:M1");usb_busy=false;telemetry_host_tick+=10;Telemetry_Service();
    uint8_t uart_copy[128];uint16_t uart_len=uart_capture_length;memcpy(uart_copy,uart_inflight,uart_len);
    uart_busy=true;command("Format:JustFloat");usb_busy=false;telemetry_host_tick+=10;Telemetry_Service();
    assert(inflight_len==28 && !memcmp(uart_copy,uart_inflight,uart_len));
    uart_busy=false;jf(6); /* A busy UART never gets a partially overwritten frame. */
    telemetry_host_tick=UINT32_MAX-5U;jf(6);jf(6); /* Producer cadence survives tick wrap. */
    puts("PASS: default pure JustFloat, fixed Auto slots, canonical invalids, all curve groups, Off silence, shared USB/UART format, fragmented Native handshake, RPC exclusion, immutable active/cancelled pending buffers and Stop");
    return 0;
}
