/* Exercise the real implementation; white-box access models retained SRAM
 * corruption and startup clearing without adding test APIs to firmware. */
#define __DMB() ((void)0)
#ifndef BOARD_DIAG_TEST_SOURCE
#define BOARD_DIAG_TEST_SOURCE "../../mycode/board_diag.c"
#endif
#include BOARD_DIAG_TEST_SOURCE
#include <assert.h>
#include <math.h>
#include <stdio.h>
uint32_t host_primask;
HostDwt host_dwt;
static uint32_t now,recorded_cycles;
static bool requested[2],recurse;
static bool guard_allowed=true;
static unsigned timing_faults;
static const char *timing_reason;
bool MotorControl_OuterRequired(void) { return guard_allowed && (requested[0] || requested[1]); }
void MotorControl_TimingFault(const char *reason) {
 if(!requested[0] && !requested[1]) return;
 requested[0]=requested[1]=false;++timing_faults;timing_reason=reason;
 BoardDiag_Fault(0,DIAG_CONTROL,reason);
}
static MotorSampleFrame sample;
static MotorGlobalStopRecord stop;
void MotorSampling_GetGlobalStop(MotorGlobalStopRecord *out) { *out=stop; }
static MotorDriveMode drive;
static SixStepSnapshot six;static bool generic;
bool SixStep_IsGeneric(void) { return generic; }
uint32_t HAL_GetTick(void) { return now; }
bool MotorSampling_IsOutputRequested(void) { return requested[0]; }
bool MotorSampling_IsAxisOutputRequested(MotorAxis a) { return requested[a]; }
MotorWorkMode MotorControl_GetWork(void) { return MOTOR_WORK_MASTER_M2; }
MotorDriveMode MotorControl_GetDrive(MotorAxis a) { return a ? MOTOR_DRIVE_ENCODER : drive; }
MotorProfile MotorParams_GetAxisProfile(MotorAxis a) { (void)a; return MOTOR_PROFILE_4015; }
const MotorSampleFrame *MotorSampling_GetFrame(void) { return &sample; }
float MotorSampling_GetBusVoltage(void) { return 20.1f; }
void MotorSampling_GetAxisDiagnostics(MotorAxis a,MotorSamplingDiagnostics *out) {
 (void)a; *out=(MotorSamplingDiagnostics){.reference_voltage=3.26f};
 if(recurse) { recurse=false;BoardDiag_Fault(2,DIAG_SAMPLE,"SECOND"); }
}
void MT6701_GetDiagnostics(MT6701Diagnostics *out) { *out=(MT6701Diagnostics){.errors=51,.age_cycles=4,.status=3,.last_error=MT6701_ERROR_FIELD,.health=MT6701_HEALTH_AGE,.wall_ms=1}; }
void MT6701_2_GetDiagnostics(MT6701Diagnostics *out) { *out=(MT6701Diagnostics){.errors=26,.age_cycles=5,.status=1,.last_error=MT6701_ERROR_TIMEOUT,.health=MT6701_HEALTH_RECOVER,.wall_ms=6,.failures=2}; }
const char *MT6701_ErrorText(MT6701Error e) { return e==MT6701_ERROR_FIELD ? "FIELD" : "TIMEOUT"; }
void SixStep_GetSnapshot(SixStepSnapshot *out) { *out=six; }
void StHso_RecordControlCycles(uint32_t n) { recorded_cycles=n; }
void MotorControl_RecordCycles(uint32_t n) { assert(n==recorded_cycles); }
static void retention(void) {
 memset(&retained,0xA5,sizeof(retained));BoardDiag_Init();assert(!retained.fault.source && !retained.crash.kind);
 now=0xfffffff0U;host_primask=1;
 BoardDiag_Motion(0,1,23,4,.3f,22,3.9f);BoardDiag_Motion(1,2,24,5,.4f,21,4.9f);
 assert(host_primask==1);now+=20;
 sample=(MotorSampleFrame){.sequence=250,.m1_current={.1f,.2f,-.3f},.m2_current={.4f,-.1f},
  .m1_current_raw={2001,2020,2060},.m2_current_raw={2200,1800}};
 recurse=true;BoardDiag_Fault(1,DIAG_ENCODER,"ENCODER1 LOST");
 BoardDiagFault saved=*BoardDiag_LastFault();
 assert(!recurse && saved.source==DIAG_ENCODER && saved.axis==1 && saved.at==4);
 assert(saved.sequence==250 && saved.motion[0].target_speed==23 && saved.at-saved.motion[0].at==20);
 assert(saved.work==MOTOR_WORK_MASTER_M2 && saved.raw[1][2]==0 && fabsf(saved.current[1][2]+.3f)<.00001f);
 assert(saved.enc_age[0]==4 && saved.enc_age[1]==5 && saved.enc_errors[1]==26);
 assert(saved.enc_health[0]==0x108 && saved.enc_health[1]==0x20602);
 char gate[32];BoardDiag_Row(1,29,gate);assert(!strcmp(gate,"E2 H02 T6 F2"));
 assert(saved.reference==3.26f && !strcmp(saved.reason,"ENCODER1 LOST"));
 now=900;sample.sequence=999;BoardDiag_Fault(2,DIAG_SAMPLE,"REPLACEMENT");assert(!memcmp(&saved,&retained.fault,sizeof(saved)));
 memset(motion,0,sizeof(motion));memset(&timing,0,sizeof(timing));BoardDiag_Init();assert(!memcmp(&saved,&retained.fault,sizeof(saved)));
 requested[1]=true;assert(!BoardDiag_Clear() && !memcmp(&saved,&retained.fault,sizeof(saved)) && host_primask==1);
 requested[1]=false;requested[0]=true;assert(!BoardDiag_Clear());requested[0]=false;
 assert(BoardDiag_Clear() && host_primask==1 && !retained.fault.source);host_primask=0;
 BoardDiag_Fault(2,DIAG_SAMPLE,"0123456789012345678901234567890123456789");
 assert(strlen(retained.fault.reason)==23 && host_primask==0);
 retained.crash.kind=3;commit();BoardDiag_Init();assert(retained.crash.kind==3 && retained.fault.source);
 retained.check^=1;BoardDiag_Init();assert(!retained.crash.kind && !retained.fault.source);
 drive=MOTOR_DRIVE_SIXSTEP;six=(SixStepSnapshot){.target=30,.velocity=11,.duty=.25f};
 BoardDiag_Fault(1,DIAG_SIXSTEP,"PHASE");assert(retained.fault.motion[0].mode==3 && retained.fault.motion[0].speed==11 && retained.fault.motion[0].target_speed==30);
 char generic_value[32];BoardDiag_Row(0,10,generic_value);assert(strstr(generic_value,"mrad/s"));
 assert(BoardDiag_Clear());generic=true;BoardDiag_Fault(1,DIAG_SIXSTEP,"NO ZC");
 BoardDiag_Row(0,10,generic_value);assert(strstr(generic_value,"mHz") && retained.fault.model[0]==MOTOR_PROFILE_ESC);
 BoardDiag_Row(0,11,generic_value);assert(!strcmp(generic_value,"--"));
 assert(BoardDiag_Clear());drive=MOTOR_DRIVE_HSO;generic=false;
 BoardDiag_Motion(0,1,30,2,.2f,29,1.9f);BoardDiag_Fault(1,DIAG_HSO,"LOCK");assert(retained.fault.motion[0].position==1.9f);
 char value[32];
 for(unsigned axis=0;axis<2;++axis) for(unsigned row=0;row<BOARD_DIAG_ROWS;++row) { memset(value,0x55,32);assert(BoardDiag_Row(axis,row,value));assert(memchr(value,0,32)); }
 assert(!BoardDiag_Row(2,0,value) && !BoardDiag_Row(0,BOARD_DIAG_ROWS,value));
 retained.fault.current[0][0]=NAN;BoardDiag_Row(0,12,value);assert(!strcmp(value,"INVALID"));
 assert(BoardDiag_Clear());BoardDiag_Row(0,6,value);assert(!strcmp(value,"--"));
 puts("PASS: first fault immutability/re-entrant getters, retained SRAM checksum/torn writes, unsigned tick wrap, both-axis stop gate, PRIMASK, reason bounds, six-step/HSO units and row bounds");
}
static void stop_rows(void) {
 char value[32];
 for(unsigned row=26;row<30U;++row) {
  assert(BoardDiag_Row(0,row,value) && !strcmp(value,"--"));
 }
 const char *names[]={"CPU BOOT","KEY1 EDGE","KEY1 LOW","KEY1 PEND","COMMAND"};
 for(unsigned source=1;source<=MOTOR_STOP_COMMAND;++source) {
  stop=(MotorGlobalStopRecord){.source=(uint8_t)source,.at=4,.stage=3,.key=0,.now=1};
  assert(!strcmp(BoardDiag_Row(0,26,value),"Stop Src") && !strcmp(value,names[source-1]));
  assert(!strcmp(BoardDiag_Row(1,27,value),"Stop At") && !strcmp(value,"4 ms S3"));
  assert(!strcmp(BoardDiag_Row(0,28,value),"Stop K/P") && !strcmp(value,"0/0 Now 1/0"));
  assert(BoardDiag_Row(0,17,value) && !strcmp(value,"--"));
  assert(BoardDiag_Clear());
  assert(BoardDiag_Row(1,26,value) && !strcmp(value,names[source-1]));
 }
 stop=(MotorGlobalStopRecord){0};
 puts("PASS: current-boot stop rows independent of empty/cleared CPU crash and retained first-fault slots; both axes, sources, stage/time and captured/live key states");
}
static void loader_debug_retention(void) {
 /* Observed after ST-Link F5 download: a RAM algorithm returns to its debug
  * breakpoint before application startup has initialized g_boot_stage. */
 const BoardDiagCrash loader={.kind=3,.stage=1430396978U,.pc=0x20000000,
  .lr=0x200007c1,.sp=0x20000bd8,.xpsr=0x21000200,.exc_return=0xfffffff9,
  .cfsr=0,.hfsr=0x80000000};
 assert(BoardDiag_Clear());now=123;BoardDiag_Fault(1,DIAG_ENCODER,"ENCODER1 LOST");
 BoardDiagFault fault=retained.fault;
 retained.crash=loader;commit();BoardDiag_Init();
 assert(!retained.crash.kind && !memcmp(&fault,&retained.fault,sizeof(fault)));
 BoardDiag_Init();assert(!retained.crash.kind && retained.check==checksum());
 /* Default to retaining suspicious records: never filter a real application
  * stage, fault bits, Flash/RAM application PC, handler/PSP/FP return, or stack
  * outside the low-RAM programming workspace just because DEBUGEVT is set. */
 const BoardDiagCrash keep[]={
  {.kind=2},{.kind=4},{.kind=5},{.kind=6},
  {.stage=1},{.stage=2},{.stage=3},{.stage=4},{.stage=5},{.stage=6},{.stage=7},
  {.cfsr=1},{.cfsr=0x20000},{.hfsr=0x40000000},{.hfsr=0xc0000000},
  {.pc=0x08000100},{.pc=0x20000100},
  {.sp=0x20001000},{.sp=0x20007f00},{.sp=0x1ffffff8},
  {.lr=0x08000101},{.lr=0x20001001},{.lr=0x200007c0},
  {.xpsr=0x21000203},{.xpsr=0x20000200},
  {.exc_return=0xfffffffd},{.exc_return=0xfffffff1},{.exc_return=0xffffffe9}
 };
 for(unsigned i=0;i<sizeof(keep)/sizeof(keep[0]);++i) {
  retained.crash=loader;
  const uint32_t *source=(const uint32_t *)&keep[i];uint32_t *dest=(uint32_t *)&retained.crash;
  for(unsigned j=0;j<sizeof(loader)/sizeof(uint32_t);++j) if(source[j]) dest[j]=source[j];
  BoardDiagCrash saved=retained.crash;commit();BoardDiag_Init();
  assert(!memcmp(&saved,&retained.crash,sizeof(saved)) && !memcmp(&fault,&retained.fault,sizeof(fault)));
 }
 retained.crash=loader;retained.crash.hfsr=0;commit();BoardDiag_Init();assert(retained.crash.kind==3);
 retained.crash=loader;retained.crash.exc_return=0;commit();BoardDiag_Init();assert(retained.crash.kind==3);
 retained.crash=loader;retained.crash.stage=0;commit();BoardDiag_Init();assert(!retained.crash.kind);
 assert(BoardDiag_Clear());
 puts("PASS: exact ST-Link RAM debug-stop signature discarded on initialization; application faults, unknown contexts, first motor fault and retained checksum preserved");
}
static void timing_test(void) {
 BoardDiag_Clear();
 uint32_t start=0xfffffff0U;host_dwt.CYCCNT=start+7650;BoardDiag_Timing(0,start,880);
 assert(timing.adc_max==7650 && !timing.adc_jitter && timing.center_age==880 && recorded_cycles==7650);
 start+=17020;host_dwt.CYCCNT=start+15300;BoardDiag_Timing(0,start,990);
 assert(timing.adc_max==15300 && timing.adc_jitter==20 && timing.center_age==990 && !timing.overruns);
 start+=16970;host_dwt.CYCCNT=start+17000;BoardDiag_Timing(0,start,100);
 assert(timing.adc_jitter==30 && timing.overruns==1);
 host_dwt.CYCCNT=4100;BoardDiag_Timing(1,100,0);
 host_dwt.CYCCNT=175200;BoardDiag_Timing(1,170120,0);
 assert(timing.outer_max==5080 && timing.outer_jitter==20);
 assert(BoardDiag_Clear());assert(!timing.adc_max && !timing.overruns);
 host_dwt.CYCCNT=19;BoardDiag_Timing(0,10,20);assert(!timing.adc_jitter && timing.adc_max==9);
 puts("PASS: 10 kHz ADC/1 kHz outer-loop duration, start jitter, PWM age, deadline counter, DWT wrap, existing protection forwarding and clear/rebaseline");
}
static void report(unsigned outer,uint32_t entered,uint32_t duration) {
 host_dwt.CYCCNT=entered+duration;BoardDiag_Timing(outer,entered,0);
}
static void monitor_start(unsigned axis,uint32_t origin) {
 requested[0]=requested[1]=false;assert(BoardDiag_Clear());
 timing_faults=0;timing_reason=NULL;guard_allowed=true;
 requested[axis]=true;report(1,origin,4000);
}
static void outer_trip(void) {
 monitor_start(0,0);requested[1]=true;
 report(1,170000,475830); /* Reported 2799 us, not a single ADC overrun. */
 assert(timing_faults==1 && !requested[0] && !requested[1]);
 assert(!strcmp(timing_reason,"OUTER OVERRUN"));
 assert(!strcmp(BoardDiag_LastFault()->reason,"OUTER OVERRUN"));
}
#if BOARD_DIAG_ROWS>=31U
static void outer_monitor(void) {
 outer_trip();
 for(unsigned axis=0;axis<2;++axis) {
  uint32_t t=0xffff0000U;monitor_start(axis,t);
  for(unsigned i=0;i<1000;++i) { t+=170000U;report(1,t,66810); }
  assert(!timing_faults && !timing.outer_overruns && !timing.outer_gaps);
  /* Isolated execution overruns count; a healthy cycle clears the streak. */
  t+=170000;report(1,t,170000);t+=170000;report(1,t,4000);
  t+=170000;report(1,t,170000);t+=170000;report(1,t,170000);
  assert(!timing_faults && timing.outer_overruns==3);
  assert(!BoardDiag_Clear()); /* Cannot erase a running fault streak. */
  t+=170000;report(1,t,170000);
  assert(timing_faults==1 && timing.outer_overruns==4 && !requested[axis]);
  BoardDiagFault first=*BoardDiag_LastFault();
  BoardDiag_Fault(1,DIAG_ENCODER,"REPLACE");assert(!memcmp(&first,BoardDiag_LastFault(),sizeof(first)));
  report(1,t+170000,600000);assert(timing_faults==1);
  for(unsigned short_gap=0;short_gap<2;++short_gap) {
   t=0xffff0000U;monitor_start(axis,t);
   for(unsigned i=0;i<2;++i) { t+=short_gap ? 84999U : 255001U;report(1,t,4000); }
   assert(!timing_faults && timing.outer_gaps==2);
   t+=short_gap ? 84999U : 255001U;report(1,t,4000);
   assert(timing_faults==1 && !strcmp(timing_reason,"OUTER GAP") && timing.outer_gaps==3);
  }
  /* ADC observes a completely missing completion, including DWT wrap. */
  monitor_start(axis,0xffff0000U);t=0xffff0000U+4000U;
  report(0,t+339999U-1U,1);assert(!timing_faults);
  report(0,t+340000U-1U,1);
  assert(timing_faults==1 && timing.outer_stalls==1 && !strcmp(timing_reason,"OUTER STALE"));
  /* Also arm from ADC when TIM2 has NEVER completed after start. */
  requested[0]=requested[1]=false;assert(BoardDiag_Clear());
  timing_faults=0;requested[axis]=true;
  report(0,0xfffffff0U,1);
  report(0,0xfffffff0U+339999U,1);assert(!timing_faults);
  report(0,0xfffffff0U+340000U,1);
  assert(timing_faults==1 && timing.outer_stalls==1 && !strcmp(timing_reason,"OUTER STALE"));
  monitor_start(axis,0);guard_allowed=false;
  report(0,10000000,1);report(1,11000000,500000);
  assert(!timing_faults && !timing.outer_overruns && !timing.outer_stalls);
  guard_allowed=true;requested[0]=requested[1]=false;BoardDiag_OuterReset();
  requested[axis]=true;report(0,20000000,1);report(1,20010000,4000);
  assert(!timing_faults && !timing.outer_gaps);
  /* Exact gap bounds are admitted; 1000 us execution is a miss, not 999. */
  t=20010000;host_primask=1;
  t+=85000;report(1,t,169999);assert(host_primask==1 && !timing.outer_overruns);
  t+=255000;report(1,t,4000);assert(host_primask==1 && !timing.outer_gaps);
  host_primask=0;
  requested[0]=requested[1]=false;assert(BoardDiag_Clear());
  char value[32];for(unsigned row=30;row<31;++row) {
   assert(BoardDiag_Row(axis,row,value) && !strcmp(value,"0/0/0"));
  }
 }
 puts("PASS: outer execution/gap/stall monitoring: 393us normal, 2799us immediate trip, three consecutive misses, isolated recovery, no-callback ADC watchdog, both axes, wrap, idle/align exclusion, restart, Clear gate and PRIMASK");
}
#endif
int main(int argc,char **argv) {
 if(argc>1 && !strcmp(argv[1],"outer-trip")) { outer_trip();return 0; }
 retention();loader_debug_retention();stop_rows();timing_test();
#if BOARD_DIAG_ROWS>=31U
 outer_monitor();
#endif
 return 0;
}
