#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "ui_app.h"
#include "ui_input.h"
#include "kk_ui_internal.h"
#include "kk_oled_internal.h"
#include "JerryFOC.h"
#include "MT6701.h"
#include "motor_sampling.h"
#include "st_hso.h"
#include "app_storage.h"

GPIO_TypeDef host_gpio = {0xFFFFU};
static uint32_t tick, dma_done;
static bool dma_busy, healthy = true, aligned = true, drive_ready = true;
static bool hso_active, hso_fault;
static bool bus_data = true;
static bool reference_data = true, reference_seen = true;
static float bus_voltage = 24.0f;
static const char *unready_status = "FRAME TIMEOUT";
static unsigned submissions, frame_changes;
static uint8_t displayed[OLED_BUFFER_SIZE];
static FILE *record_file;
static uint32_t last_hash;
static unsigned transfer_delay = 30;
static bool fail_next_frame;
static unsigned recoveries;
static JerryFOC_ControlMode mode;
static float velocity, angle, iq;
static uint8_t current_loop;
static bool output_requested=true;
static bool aligning;
static MT6701Diagnostics enc_diag;
volatile float JerryFOC_Target_Velocity, JerryFOC_Target_Position, JerryFOC_Target_Iq;
volatile uint8_t g_openloop_debug;
static AppSettings settings;
uint32_t HAL_GetTick(void) { return tick; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init) { (void)port; (void)init; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) {
    return (port->pins & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
uint8_t MT6701_IsHealthy(void) { return healthy; }
void MT6701_2_GetSnapshot(MT6701_2_Snapshot *out) {
    *out=(MT6701_2_Snapshot){.healthy=1,.raw_angle=456};
}
uint8_t JerryFOC_isDriveReady(void) { return drive_ready && !hso_fault; }
uint8_t JerryFOC_isHSOActive(void) { return hso_active; }
uint8_t JerryFOC_hasHSOFault(void) { return hso_fault; }
uint8_t JerryFOC_isHSOReturning(void) { return 0; }
const char *JerryFOC_getFeedbackText(void) { return hso_active ? "HSO" : "ENCODER"; }
const char *JerryFOC_getHSOFaultText(void) { return hso_fault ? "ANGLE" : "--"; }
uint8_t JerryFOC_useHSO(uint8_t en) {
    hso_active=en;
    if(!en) { hso_fault=false; JerryFOC_Target_Iq=JerryFOC_Target_Velocity=0; }
    return 1;
}
void StHso_GetSnapshot(StHsoSnapshot *out) { *out=(StHsoSnapshot){.state=ST_HSO_WAIT_PARAM}; }
const char *StHso_StateText(StHsoState state) { (void)state; return "WAIT FLUX"; }
const char *StHso_ReasonText(StHsoReason reason) { (void)reason; return "OK"; }
float JerryFOC_getBusVoltage(void) { return bus_voltage; }
const char *MotorSampling_GetStatusText(void) { return drive_ready ? "READY" : unready_status; }
void MotorSampling_GetDiagnostics(MotorSamplingDiagnostics *out) {
    *out=(MotorSamplingDiagnostics){.bus_fresh=bus_data,.bus_voltage=bus_voltage,.bus_raw=2707,.bus_samples=tick*2,.frame_samples=tick*10,
        .current_raw={2047,2048,2049},.current_zero={2046,2047,2048},
        .fault_current_raw={2047,0,2049},.fault_phase=2,.fault_during_zero=true,.silicon_revision=0x2002,
        .reference_voltage=3.26f,.reference_raw=1518,.reference_cal=1650,
        .reference_updates=reference_seen?1+tick/32:0,.reference_fresh=reference_data};
}
uint8_t AngleSrc_Encoder_IsAligned(void) { return aligned; }
uint8_t JerryFOC_isAligning(void) { return aligning; }
uint16_t MT6701_GetRawAngle(void) { return 123; }
uint32_t MT6701_GetErrorCounter(void) { return enc_diag.errors; }
void MT6701_GetDiagnostics(MT6701Diagnostics *out) { *out=enc_diag; }
const char *MT6701_ErrorText(MT6701Error error) {
    const char *names[]={"--","NO FRAME","CRC","FIELD","TRACK","SPI"};
    return names[error];
}
void JerryFOC_useCurrentLoop(uint8_t enable) { current_loop = enable; }
void JerryFOC_setMode(JerryFOC_ControlMode m) { mode = m; }
JerryFOC_ControlMode JerryFOC_getMode(void) { return mode; }
void JerryFOC_setVelocity(float v) { JerryFOC_Target_Velocity = v; }
void JerryFOC_setCurrent(float v) { JerryFOC_Target_Iq = v; }
void JerryFOC_setAbsolutePosition(float v, float limit) { (void)limit; JerryFOC_Target_Position = v; }
void JerryFOC_stop(void) {
    output_requested=false; hso_active=false; current_loop=0; g_openloop_debug=0;
    mode=JERRYFOC_MODE_TORQUE;
    JerryFOC_Target_Iq=JerryFOC_Target_Velocity=JerryFOC_Target_Position=0;
}
uint8_t JerryFOC_startOutput(void) { output_requested=drive_ready && healthy && aligned; return output_requested; }
bool MotorSampling_IsOutputRequested(void) { return output_requested; }
bool MotorSampling_IsOutputEnabled(void) { return output_requested && drive_ready; }
float JerryFOC_getVelocity(void) { return velocity; }
float JerryFOC_getAngle(void) { return angle; }
float JerryFOC_getIq(void) { return iq; }
float JerryFOC_getId(void) { return 0; }
float JerryFOC_getPhaseCurrent_A(void) { return 0; }
float JerryFOC_getPhaseCurrent_B(void) { return 0; }
float JerryFOC_getElectricalAngle(void) { return 0; }
float JerryFOC_getTargetVelocity(void) { return JerryFOC_Target_Velocity; }
const MotorParams *JerryFOC_getMotorParams(void) { return MotorParams_GetActive(); }
uint8_t JerryFOC_selectMotor(MotorProfile profile) {
    if(!healthy || fabsf(velocity)>1) return 0;
    MotorParams_Select(profile); aligned=false; current_loop=0;
    mode=JERRYFOC_MODE_TORQUE;
    JerryFOC_Target_Iq=JerryFOC_Target_Velocity=JerryFOC_Target_Position=0;
    return 1;
}
bool AppStorage_IsReady(void) { return true; }
AppSettings *AppStorage_Get(void) { return &settings; }
void AppStorage_Load(void) {}
bool AppStorage_Save(void) { return true; }
bool AppStorage_RequestSave(void) { return true; }
bool AppStorage_PollSaveResult(bool *success) { (void)success; return false; }
void AppStorage_ResetDefaults(void) { memset(&settings, 0, sizeof(settings)); settings.led_brightness = 100; }
void AppStorage_Log(uint16_t event, uint32_t value) { (void)event; (void)value; }

OLED_Status OLED_DriverInit(void) { dma_busy = false; return OLED_OK; }
static void count_frame(void) {
    const uint8_t *frame = OLED_InternalGetTransferBuffer();
    memcpy(displayed, frame, sizeof(displayed));
    uint32_t hash = 2166136261U;
    for (unsigned i=0; i<OLED_BUFFER_SIZE; ++i) hash = (hash ^ frame[i])*16777619U;
    if (hash != last_hash) { ++frame_changes; last_hash=hash; }
    ++submissions;
}
OLED_Status OLED_DriverWriteBlocking(void) { count_frame(); return OLED_OK; }
OLED_Status OLED_DriverWriteIT(void) { return OLED_UNSUPPORTED; }
OLED_Status OLED_DriverWriteDMA(void) {
    count_frame(); dma_busy=true; dma_done=tick+transfer_delay; return OLED_OK;
}
bool OLED_DriverIsBusy(void) { return dma_busy; }
void OLED_DriverAbort(void) { dma_busy=false; OLED_InternalTransferFinished(OLED_ERROR); }
OLED_Status OLED_DriverSetContrast(uint8_t value) { (void)value; return OLED_UNSUPPORTED; }
OLED_Status OLED_DriverSetPowerSave(bool enable) { (void)enable; return OLED_OK; }
static void step(uint8_t keys, unsigned ms) {
    for (unsigned i=0; i<ms; ++i) {
        ++tick;
        if (dma_busy && (int32_t)(tick-dma_done)>=0) {
            dma_busy=false;
            OLED_InternalTransferFinished(fail_next_frame?OLED_ERROR:OLED_OK);
            fail_next_frame=false;
        }
        ui_app_update(tick);
        static const uint16_t pins[]={KEY_Up_Pin,KEY_Down_Pin,KEY_OK_Pin,KEY_Left_Pin,KEY_Right_Pin,KEY2_Pin};
        host_gpio.pins=0xFFFFU;
        for(unsigned key=0;key<6;++key) if(keys&(1U<<key)) host_gpio.pins&=~pins[key];
        KK_UI_Status status=KK_UI_Update(tick,(KK_UI_Input){.keys=ui_input_poll()});
        if(status==KK_UI_DISPLAY_ERROR || status==KK_UI_DISPLAY_TIMEOUT) {
            ++recoveries;
            ui_app_note_display_error();
            assert(KK_UI_RecoverDisplay()==KK_UI_OK);
        } else assert(status==KK_UI_OK);
        ui_app_process_events();
        if (dma_busy) assert(memcmp(displayed, OLED_InternalGetTransferBuffer(), OLED_BUFFER_SIZE)==0);
        if (record_file && tick%20U==0U) assert(fwrite(displayed,1,sizeof(displayed),record_file)==sizeof(displayed));
    }
}
static void press(uint8_t key) { step(key,30); step(0,30); }
static void settle(void) { step(0,600); }
static void back(void) { press(KK_UI_KEY_BACK); settle(); }
static void reset(void) {
    MotorParams_Select(MOTOR_PROFILE_2804); velocity=0;
    output_requested=true;
    aligning=false;
    memset(&enc_diag,0,sizeof(enc_diag));
    memset(&settings,0,sizeof(settings)); settings.led_brightness=100;
    healthy=true; aligned=true; drive_ready=true; bus_data=true; bus_voltage=24.0f; mode=JERRYFOC_MODE_TORQUE;
    unready_status="FRAME TIMEOUT";
    hso_active=hso_fault=false;
    JerryFOC_Target_Iq=JerryFOC_Target_Velocity=JerryFOC_Target_Position=0; angle=0; current_loop=0;
    OLED_Abort(); assert(OLED_Init()==OLED_OK); OLED_SetRotation(OLED_ROTATION_270);
    ui_app_reset(tick); assert(KK_UI_Init(ui_app_get())==KK_UI_OK); settle();
}
static void snapshot(const char *name) {
    char path[160]; snprintf(path,sizeof(path),"build/ui-validation/%s.pgm",name);
    FILE *file=fopen(path,"wb"); assert(file);
    fprintf(file,"P5\n160 80\n255\n");
    const uint8_t *fb=displayed;
    /* 270-degree logical coordinates -> native 80x160 framebuffer. */
    for (int y=0;y<80;++y) for (int x=0;x<160;++x) {
        unsigned px=y, py=159-x;
        fputc((fb[(py/8)*80+px] & (1U<<(py%8))) ? 255 : 0,file);
    }
    fclose(file);
}
static void assert_home_continuity(uint8_t direction) {
    uint8_t before[OLED_BUFFER_SIZE];
    kk_ui.last_update=tick;
    KK_UI_DrawScene(tick);
    memcpy(before,OLED_InternalTestGetDrawBuffer(),sizeof(before));
    KK_UI_DispatchInput((KK_UI_InputEvent){direction==KK_UI_KEY_RIGHT ?
        KK_UI_INPUT_RIGHT:KK_UI_INPUT_LEFT,KK_UI_INPUT_PRESS,1},tick);
    KK_UI_DrawScene(tick);
    unsigned changed=0;
    for(unsigned i=0;i<sizeof(before);++i)
        if(before[i]!=OLED_InternalTestGetDrawBuffer()[i]) ++changed;
    if(changed) fprintf(stderr,"Home discontinuity: selected=%u direction=%u changed_bytes=%u\n",
                        kk_ui.current.selected,direction,changed);
    assert(changed==0); /* A new target must not teleport pixels at time zero. */
}
static float motor_target(unsigned run_mode) {
    return run_mode==JERRYFOC_MODE_VELOCITY ? JerryFOC_Target_Velocity :
           run_mode==JERRYFOC_MODE_POSITION ? JerryFOC_Target_Position : JerryFOC_Target_Iq;
}
static void select_setting(const char *name) {
    assert(KK_UI_GetCurrentPage()==UI_PAGE_SETTINGS);
    const KK_UI_MenuPage *menu=&ui_app_get()->menu_pages[0];
    unsigned index;
    for(index=0;index<menu->item_count;++index)
        if(strcmp(menu->items[index].label,name)==0) break;
    assert(index<menu->item_count);
    while(kk_ui.current.selected!=index) press(KK_UI_KEY_DOWN);
}
static void adjust_editor(int16_t delta) {
    assert(kk_ui.overlay.type==KK_UI_OVERLAY_INT && kk_ui.overlay.editing);
    assert(KK_UI_Update(tick,(KK_UI_Input){.encoder_delta=delta})==KK_UI_OK);
}
static const char *status_value(const char *name) {
    const KK_UI_InfoPage *page=&ui_app_get()->info_pages[0];
    for(unsigned i=0;i<page->row_count;++i)
        if(strcmp(page->rows[i].name,name)==0) return page->rows[i].value;
    assert(0); return "";
}
int main(void) {
    ui_input_init(); assert(ui_input_poll()==0);
    host_gpio.pins &= ~KEY2_Pin; assert(ui_input_poll()==KK_UI_KEY_BACK);
    host_gpio.pins=0xFFFFU;
    /* Regression: every wrap edge, plus rapid presses before animation settles. */
    for(unsigned i=0;i<4;++i) {
        reset(); kk_ui.current.selected=i;
        assert_home_continuity(KK_UI_KEY_RIGHT);
        settle(); assert_home_continuity(KK_UI_KEY_LEFT);
    }
    reset();
    assert(strcmp(status_value("Bus"),"24.0 V")==0);
    assert(strcmp(status_value("Drive"),"READY")==0);
    assert(strcmp(status_value("IU R/Z"),"2047 / 2046")==0);
    assert(strcmp(status_value("IV R/Z"),"2048 / 2047")==0);
    assert(strcmp(status_value("IW R/Z"),"2049 / 2048")==0);
    assert(strcmp(status_value("I Trip"),"ZERO V 0")==0);
    assert(strcmp(status_value("ChipRev"),"0x2002")==0);
    assert(strcmp(status_value("Enc2"),"OK 456")==0);
    assert(strcmp(status_value("Enc2Err"),"0")==0);
    bus_voltage=0; drive_ready=false; unready_status="WAIT BUS"; settle();
    assert(strcmp(status_value("Bus"),"0.0 V")==0 && strcmp(status_value("Drive"),"WAIT BUS")==0);
    press(KK_UI_KEY_OK); settle(); snapshot("run-wait-bus");
    press(KK_UI_KEY_UP); press(KK_UI_KEY_OK); assert(JerryFOC_Target_Iq==0);
    bus_voltage=11.9f; unready_status="FRAME TIMEOUT"; settle();
    assert(strcmp(status_value("Bus"),"11.9 V")==0 && strcmp(status_value("Drive"),"FRAME TIMEOUT")==0);
    bus_data=false; settle(); assert(strcmp(status_value("Bus"),"NO DATA")==0);
    reset();
    for(unsigned i=0;i<12;++i) {
        assert_home_continuity(i<7 ? KK_UI_KEY_RIGHT : KK_UI_KEY_LEFT);
        step(0,45);
    }
    reset();
    press(KK_UI_KEY_DOWN); assert(kk_ui.current.selected==0); /* home horizontal */
    step(KK_UI_KEY_RIGHT,5); step(0,30); assert(kk_ui.current.selected==0); /* bounce */
    press(KK_UI_KEY_RIGHT); assert(kk_ui.current.selected==1);
    step(KK_UI_KEY_LEFT,500); assert(kk_ui.current.selected==2); step(0,30);
    reset(); press(KK_UI_KEY_OK); settle(); assert(KK_UI_GetCurrentPage()==UI_PAGE_RUN);
    press(KK_UI_KEY_UP); assert(JerryFOC_Target_Iq==0); /* draft does not energize */
    press(KK_UI_KEY_OK); assert(fabsf(JerryFOC_Target_Iq-0.05f)<0.001f);
    press(KK_UI_KEY_RIGHT); assert(mode==JERRYFOC_MODE_TORQUE && JerryFOC_Target_Iq==0);
    press(KK_UI_KEY_UP); press(KK_UI_KEY_OK);
    assert(mode==JERRYFOC_MODE_VELOCITY && JerryFOC_Target_Velocity==0.5f);
    press(KK_UI_KEY_DOWN); assert(JerryFOC_Target_Velocity==0.5f);
    press(KK_UI_KEY_DOWN); assert(JerryFOC_Target_Velocity==0.5f);
    press(KK_UI_KEY_OK);
    assert(mode==JERRYFOC_MODE_VELOCITY && JerryFOC_Target_Velocity==-0.5f);
    snapshot("run-speed");
    press(KK_UI_KEY_RIGHT); angle=12; press(KK_UI_KEY_UP); press(KK_UI_KEY_OK);
    assert(mode==JERRYFOC_MODE_POSITION && fabsf(JerryFOC_Target_Position-12.05f)<0.001f);
    angle=25; press(KK_UI_KEY_UP);
    assert(fabsf(JerryFOC_Target_Position-12.05f)<0.001f);
    press(KK_UI_KEY_OK);
    assert(fabsf(JerryFOC_Target_Position-12.10f)<0.001f); /* origin stays fixed */
    healthy=false; step(0,1); assert(mode==JERRYFOC_MODE_TORQUE && JerryFOC_Target_Iq==0);
    healthy=true; press(KK_UI_KEY_OK); assert(mode==JERRYFOC_MODE_POSITION);
    press(KK_UI_KEY_BACK); /* motor stops before transition finishes */
    assert(mode==JERRYFOC_MODE_TORQUE && JerryFOC_Target_Iq==0 && JerryFOC_Target_Velocity==0);
    settle(); assert(KK_UI_GetCurrentPage()==UI_PAGE_HOME);
    /* Status and About must render multiple distinct intermediate frames. */
    for (unsigned home_index=2;home_index<=3;++home_index) {
        reset(); for(unsigned i=0;i<home_index;++i) press(KK_UI_KEY_RIGHT); settle();
        unsigned before=frame_changes;
        press(KK_UI_KEY_OK); step(0,100); assert(kk_ui.transition.active);
        snapshot(home_index==2 ? "status-transition" : "about-transition");
        settle(); assert(frame_changes-before>=6);
        assert(KK_UI_GetCurrentPage()==(home_index==2?UI_PAGE_STATUS:UI_PAGE_ABOUT));
        press(KK_UI_KEY_RIGHT); assert(kk_ui.current.scroll_q8==0);
        press(KK_UI_KEY_DOWN); assert(kk_ui.current.scroll_q8>0);
        snapshot(home_index==2 ? "status" : "about");
        back(); assert(KK_UI_GetCurrentPage()==UI_PAGE_HOME);
    }
    reset(); press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK); settle();
    assert(KK_UI_GetCurrentPage()==UI_PAGE_SETTINGS);
    press(KK_UI_KEY_RIGHT); assert(kk_ui.current.selected==0);
    press(KK_UI_KEY_OK); settle(); /* bool: only horizontal */
    press(KK_UI_KEY_UP); assert(kk_ui.overlay.draft==0);
    press(KK_UI_KEY_RIGHT); assert(kk_ui.overlay.draft==1);
    back(); assert(settings.flags==0); /* Back cancels draft */
    press(KK_UI_KEY_DOWN); press(KK_UI_KEY_DOWN); press(KK_UI_KEY_OK); settle();
    assert(kk_ui.overlay.type==KK_UI_OVERLAY_INT);
    press(KK_UI_KEY_DOWN); assert(kk_ui.overlay.draft==95);
    press(KK_UI_KEY_LEFT); assert(kk_ui.overlay.draft==95);
    back(); assert(settings.led_brightness==100);
    /* Confirm Back cancels, never resets. */
    select_setting("Reset"); press(KK_UI_KEY_OK); settle();
    assert(kk_ui.overlay.type==KK_UI_OVERLAY_CONFIRM); back();
    assert(settings.led_brightness==100); back();
    /* Boot-held, opposite keys, transition-held and timer wrap. */
    assert(KK_UI_Init(ui_app_get())==KK_UI_OK);
    step(KK_UI_KEY_RIGHT,1000); assert(kk_ui.current.selected==0); step(0,30);
    step(KK_UI_KEY_LEFT|KK_UI_KEY_RIGHT,100); assert(kk_ui.current.selected==0); step(0,30);
    press(KK_UI_KEY_OK); step(KK_UI_KEY_UP,500);
    assert(JerryFOC_Target_Iq==0); step(0,30); back();
    press(KK_UI_KEY_OK);
    step(0,270); /* key begins less than one debounce period before transition end */
    step(KK_UI_KEY_UP,150);
    assert(kk_ui.blocked_keys & KK_UI_KEY_UP);
    step(KK_UI_KEY_UP,350);
    assert((kk_ui.blocked_keys & KK_UI_KEY_UP)==0); /* deliberate hold resumes */
    assert(JerryFOC_Target_Iq==0); /* resumed adjustment cannot auto-start the motor */
    step(0,30); back();
    tick=0xFFFFFF00U; assert(KK_UI_Init(ui_app_get())==KK_UI_OK); settle();
    press(KK_UI_KEY_RIGHT); assert(kk_ui.current.selected==1);
    /* Slower-than-frame DMA still submits intermediate transition frames. */
    transfer_delay=60; reset(); unsigned before=frame_changes;
    press(KK_UI_KEY_OK); settle(); assert(frame_changes-before>=4);
    /* Current-loop torque has a separate draft and a rated-current clamp. */
    reset(); settings.flags=APP_FLAG_CURRENT_LOOP;
    ui_app_reset(tick); assert(KK_UI_Init(ui_app_get())==KK_UI_OK); settle();
    press(KK_UI_KEY_OK); settle();
    KK_UI_CustomOnInput(UI_PAGE_RUN, (KK_UI_InputEvent){KK_UI_INPUT_UP, KK_UI_INPUT_ENCODER, 1000});
    assert(JerryFOC_Target_Iq==0); press(KK_UI_KEY_OK);
    assert(current_loop==1 && JerryFOC_Target_Iq==MOTOR_2804.rated_current);
    settle(); snapshot("run-current");
    step(KK_UI_KEY_BACK,800); assert(current_loop==0 && JerryFOC_Target_Iq==0);
    assert(KK_UI_GetCurrentPage()==UI_PAGE_HOME); step(0,30);
    /* Integer commit and persistent position speed bits. */
    reset(); press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK); settle();
    select_setting("Pos Speed");
    press(KK_UI_KEY_OK); settle(); assert(kk_ui.overlay.draft==30);
    press(KK_UI_KEY_UP); assert(kk_ui.overlay.draft==31);
    press(KK_UI_KEY_OK); press(KK_UI_KEY_OK); settle();
    assert(((settings.flags & APP_POSITION_SPEED_MASK)>>APP_POSITION_SPEED_SHIFT)==31);
    /* Speed Limit: cancelled changes, actual held keys, and targets beyond 60 rad/s. */
    reset(); press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK); settle();
    select_setting("Speed Limit"); press(KK_UI_KEY_OK); settle();
    assert(kk_ui.overlay.draft==100);
    press(KK_UI_KEY_DOWN); assert(kk_ui.overlay.draft==99); back();
    press(KK_UI_KEY_OK); settle(); assert(kk_ui.overlay.draft==100);
    step(KK_UI_KEY_UP,1000); step(0,30); assert(kk_ui.overlay.draft==106);
    step(0,100); snapshot("speed-limit");
    press(KK_UI_KEY_OK); press(KK_UI_KEY_OK); settle();
    assert(((settings.flags & APP_SPEED_LIMIT_MASK)>>APP_SPEED_LIMIT_SHIFT)==106);
    assert(((settings.flags & APP_POSITION_SPEED_MASK)>>APP_POSITION_SPEED_SHIFT)==30);
    back(); press(KK_UI_KEY_LEFT); press(KK_UI_KEY_OK); settle(); press(KK_UI_KEY_RIGHT);
    KK_UI_CustomOnInput(UI_PAGE_RUN,(KK_UI_InputEvent){KK_UI_INPUT_UP,KK_UI_INPUT_ENCODER,10000});
    assert(JerryFOC_Target_Velocity==0); press(KK_UI_KEY_OK);
    assert(JerryFOC_Target_Velocity==106.0f);
    step(0,100); snapshot("run-speed-limit");
    KK_UI_CustomOnInput(UI_PAGE_RUN,(KK_UI_InputEvent){KK_UI_INPUT_DOWN,KK_UI_INPUT_ENCODER,10000});
    assert(JerryFOC_Target_Velocity==106.0f); press(KK_UI_KEY_OK);
    assert(JerryFOC_Target_Velocity==-106.0f);
    /* Lowering the range clamps the saved negative target without starting the motor. */
    back(); press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK); settle();
    select_setting("Speed Limit"); press(KK_UI_KEY_OK); settle();
    adjust_editor(101); assert(kk_ui.overlay.draft==5);
    press(KK_UI_KEY_OK); press(KK_UI_KEY_OK); settle();
    assert(settings.speed_x10==-50 && JerryFOC_Target_Velocity==0);
    step(0,2500); /* Reload committed RAM settings as on a restart. */
    ui_app_reset(tick); assert(KK_UI_Init(ui_app_get())==KK_UI_OK); settle();
    press(KK_UI_KEY_OK); settle(); press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK);
    assert(mode==JERRYFOC_MODE_VELOCITY && JerryFOC_Target_Velocity==-5.0f);
    /* Editor limits and their effect on both signed targets. */
    for(unsigned bound=0;bound<2;++bound) {
        back(); press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK); settle();
        select_setting("Speed Limit"); press(KK_UI_KEY_OK); settle();
        adjust_editor(bound ? -1000 : 1000);
        int32_t expected=bound ? 600 : 1;
        assert(kk_ui.overlay.draft==expected);
        press(KK_UI_KEY_OK); press(KK_UI_KEY_OK); settle();
        assert(((settings.flags & APP_SPEED_LIMIT_MASK)>>APP_SPEED_LIMIT_SHIFT)==(uint32_t)expected);
        back(); press(KK_UI_KEY_LEFT); press(KK_UI_KEY_OK); settle(); press(KK_UI_KEY_RIGHT);
        KK_UI_CustomOnInput(UI_PAGE_RUN,(KK_UI_InputEvent){KK_UI_INPUT_UP,KK_UI_INPUT_ENCODER,10000});
        press(KK_UI_KEY_OK); assert(JerryFOC_Target_Velocity==(float)expected);
        KK_UI_CustomOnInput(UI_PAGE_RUN,(KK_UI_InputEvent){KK_UI_INPUT_DOWN,KK_UI_INPUT_ENCODER,10000});
        press(KK_UI_KEY_OK); assert(JerryFOC_Target_Velocity==-(float)expected);
    }
    /* Reset restores the default range as well as the target. */
    back(); press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK); settle();
    select_setting("Reset"); press(KK_UI_KEY_OK); settle();
    press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK); settle();
    select_setting("Speed Limit"); press(KK_UI_KEY_OK); settle();
    assert(kk_ui.overlay.draft==100); back();
    /* Old records with no limit bits, and invalid encoded limits, use the default. */
    const unsigned invalid_limits[]={0,601,1023};
    for(unsigned i=0;i<3;++i) {
        reset(); settings.flags=invalid_limits[i]<<APP_SPEED_LIMIT_SHIFT;
        settings.speed_x10=i==1 ? -99999 : 99999;
        ui_app_reset(tick); assert(KK_UI_Init(ui_app_get())==KK_UI_OK); settle();
        press(KK_UI_KEY_OK); settle(); press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK);
        assert(JerryFOC_Target_Velocity==(i==1 ? -100.0f : 100.0f));
    }
    /* Adjustable steps: cancel, submit, and apply independently to all modes. */
    reset(); press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK); settle();
    select_setting("Run Steps");
    press(KK_UI_KEY_OK); settle(); assert(KK_UI_GetCurrentPage()==UI_PAGE_RUN_STEPS);
    KK_UI_CustomOnInput(UI_PAGE_RUN_STEPS,(KK_UI_InputEvent){KK_UI_INPUT_RIGHT,KK_UI_INPUT_ENCODER,1000});
    press(KK_UI_KEY_OK); settle(); assert(settings.torque_step_x100==100);
    press(KK_UI_KEY_OK); settle();
    KK_UI_CustomOnInput(UI_PAGE_RUN_STEPS,(KK_UI_InputEvent){KK_UI_INPUT_LEFT,KK_UI_INPUT_ENCODER,1000});
    press(KK_UI_KEY_OK); settle(); assert(settings.torque_step_x100==1);
    /* Restore defaults before testing a cancelled draft. */
    settings.torque_step_x100=0; ui_app_reset(tick);
    press(KK_UI_KEY_OK); settle();
    press(KK_UI_KEY_RIGHT); back(); assert(settings.torque_step_x100==0);
    press(KK_UI_KEY_OK); settle();
    press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_RIGHT); /* Torque 0.07 */
    press(KK_UI_KEY_DOWN); press(KK_UI_KEY_RIGHT); /* Speed 0.6 */
    press(KK_UI_KEY_DOWN); press(KK_UI_KEY_LEFT); /* Position 0.04 */
    settle(); snapshot("run-steps");
    press(KK_UI_KEY_OK); settle(); assert(KK_UI_GetCurrentPage()==UI_PAGE_SETTINGS);
    assert(settings.torque_step_x100==7 && settings.speed_step_x10==6 && settings.position_step_x100==4);
    back(); press(KK_UI_KEY_LEFT); press(KK_UI_KEY_OK); settle();
    press(KK_UI_KEY_UP); press(KK_UI_KEY_UP); press(KK_UI_KEY_OK);
    assert(fabsf(JerryFOC_Target_Iq-0.14f)<0.001f);
    press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_UP); press(KK_UI_KEY_OK);
    assert(fabsf(JerryFOC_Target_Velocity-0.6f)<0.001f);
    press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_UP); press(KK_UI_KEY_OK);
    assert(fabsf(JerryFOC_Target_Position-0.04f)<0.001f);
    /* Ten quick taps each count once; a hold repeats with the configured step. */
    reset(); press(KK_UI_KEY_OK); settle();
    for(unsigned i=0;i<10;++i) press(KK_UI_KEY_UP);
    assert(JerryFOC_Target_Iq==0); press(KK_UI_KEY_OK);
    assert(fabsf(JerryFOC_Target_Iq-0.50f)<0.001f);
    reset(); press(KK_UI_KEY_OK); settle();
    step(KK_UI_KEY_UP,1000); step(0,30); press(KK_UI_KEY_OK);
    assert(fabsf(JerryFOC_Target_Iq-0.30f)<0.001f);
    /* Run: both held directions in all modes, before and after starting.
     * A changed value always confirms; an unchanged value toggles stop/start. */
    for(unsigned run_mode=0;run_mode<3;++run_mode) {
        reset(); press(KK_UI_KEY_OK); settle();
        for(unsigned i=0;i<run_mode;++i) press(KK_UI_KEY_RIGHT);
        float held_target=run_mode==JERRYFOC_MODE_VELOCITY ? 3.0f : 0.30f;
        step(KK_UI_KEY_UP,1000); step(0,30);
        assert(fabsf(motor_target(run_mode))<0.001f);
        if(run_mode==1) { step(0,80); snapshot("run-edit-stopped"); }
        press(KK_UI_KEY_OK);
        assert(mode==(JerryFOC_ControlMode)run_mode);
        assert(fabsf(motor_target(run_mode)-held_target)<0.001f);
        step(KK_UI_KEY_DOWN,1000); step(0,30);
        assert(fabsf(motor_target(run_mode)-held_target)<0.001f);
        if(run_mode==1) { step(0,80); snapshot("run-edit-running"); }
        press(KK_UI_KEY_OK); /* Apply zero without stopping the selected mode. */
        assert(mode==(JerryFOC_ControlMode)run_mode && fabsf(motor_target(run_mode))<0.001f);
        press(KK_UI_KEY_OK); /* No edit: cancel/stop. */
        assert(mode==JERRYFOC_MODE_TORQUE && JerryFOC_Target_Iq==0);
        press(KK_UI_KEY_OK); /* Still no edit: confirm/start. */
        assert(mode==(JerryFOC_ControlMode)run_mode);
        step(KK_UI_KEY_UP,1000); step(0,30); press(KK_UI_KEY_OK);
        assert(mode==(JerryFOC_ControlMode)run_mode);
        assert(fabsf(motor_target(run_mode)-held_target)<0.001f);
        press(KK_UI_KEY_UP); press(KK_UI_KEY_DOWN); /* Revert draft to confirmed value. */
        press(KK_UI_KEY_OK);
        assert(mode==JERRYFOC_MODE_TORQUE && JerryFOC_Target_Iq==0);
    }
    /* Discard unconfirmed edits on Back; preserve only confirmed targets. */
    reset(); press(KK_UI_KEY_OK); settle();
    press(KK_UI_KEY_UP); back(); step(0,2500);
    assert(settings.voltage_x100==0);
    press(KK_UI_KEY_OK); settle(); press(KK_UI_KEY_OK);
    assert(JerryFOC_Target_Iq==0);
    press(KK_UI_KEY_UP); press(KK_UI_KEY_OK);
    assert(fabsf(JerryFOC_Target_Iq-0.05f)<0.001f);
    press(KK_UI_KEY_UP); back(); step(0,2500);
    assert(settings.voltage_x100==5);
    press(KK_UI_KEY_OK); settle(); press(KK_UI_KEY_OK);
    assert(fabsf(JerryFOC_Target_Iq-0.05f)<0.001f);
    /* Failed confirmation leaves the changed value available for retry. */
    press(KK_UI_KEY_UP); healthy=false; press(KK_UI_KEY_OK);
    assert(JerryFOC_Target_Iq==0);
    healthy=true; press(KK_UI_KEY_OK);
    assert(fabsf(JerryFOC_Target_Iq-0.10f)<0.001f);
    /* A lost ADC frame / VBUS stops Run, and a changed draft cannot restart it. */
    drive_ready=false; step(0,1);
    assert(mode==JERRYFOC_MODE_TORQUE && JerryFOC_Target_Iq==0);
    press(KK_UI_KEY_UP); press(KK_UI_KEY_OK);
    assert(JerryFOC_Target_Iq==0);
    drive_ready=true; press(KK_UI_KEY_OK);
    assert(fabsf(JerryFOC_Target_Iq-0.15f)<0.001f);
    /* Run Steps: repeat after settling and when the hold starts during entry. */
    for(unsigned early=0;early<2;++early) {
        reset(); press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK); settle();
        select_setting("Run Steps");
        press(KK_UI_KEY_OK);
        assert(KK_UI_GetCurrentPage()==UI_PAGE_RUN_STEPS);
        if(!early) settle();
        step(KK_UI_KEY_RIGHT,1000); step(0,30);
        press(KK_UI_KEY_OK); settle();
        if(settings.torque_step_x100!=(early?8:11))
            fprintf(stderr,"Run Steps hold: early=%u saved=%ld\n",early,(long)settings.torque_step_x100);
        assert(settings.torque_step_x100==(early?8:11));
        press(KK_UI_KEY_OK); settle();
        step(KK_UI_KEY_LEFT,1000); step(0,30); press(KK_UI_KEY_OK); settle();
        assert(settings.torque_step_x100==(early?2:5));
        step(KK_UI_KEY_OK,1000); step(0,30);
        assert(KK_UI_GetCurrentPage()==UI_PAGE_RUN_STEPS); /* held OK cannot save */
        step(early?KK_UI_KEY_UP:KK_UI_KEY_DOWN,650); step(0,30);
        press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK); settle();
        assert((early?settings.position_step_x100:settings.speed_step_x10)==6);
        step(KK_UI_KEY_OK,1000); step(0,30);
        step(KK_UI_KEY_BACK,1000); step(0,30);
        assert(KK_UI_GetCurrentPage()==UI_PAGE_SETTINGS); /* held Back returns once */
    }
    /* Match main.c: a refresh failure/recovery must not kill a held adjustment. */
    reset(); press(KK_UI_KEY_OK); settle();
    unsigned before_recovery=recoveries;
    fail_next_frame=true;
    step(KK_UI_KEY_UP,1000); step(0,30); press(KK_UI_KEY_OK);
    assert(recoveries==before_recovery+1);
    if(fabsf(JerryFOC_Target_Iq-0.30f)>=0.001f)
        fprintf(stderr,"Repeat after display recovery: target=%f\n",(double)JerryFOC_Target_Iq);
    assert(fabsf(JerryFOC_Target_Iq-0.30f)<0.001f);
    fail_next_frame=true;
    step(KK_UI_KEY_DOWN,1000); step(0,30);
    assert(fabsf(JerryFOC_Target_Iq-0.30f)<0.001f);
    press(KK_UI_KEY_OK);
    assert(fabsf(JerryFOC_Target_Iq)<0.001f && recoveries==before_recovery+2);
    reset(); press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK); settle();
    select_setting("Run Steps");
    press(KK_UI_KEY_OK); settle();
    fail_next_frame=true;
    step(KK_UI_KEY_RIGHT,1000); step(0,30); press(KK_UI_KEY_OK); settle();
    assert(settings.torque_step_x100==11);
    press(KK_UI_KEY_OK); settle(); fail_next_frame=true;
    step(KK_UI_KEY_LEFT,1000); step(0,30); press(KK_UI_KEY_OK); settle();
    assert(settings.torque_step_x100==5);
    const KK_UI_InfoPage *status=&ui_app_get()->info_pages[0];
    assert(strcmp(status_value("HSO"),"WAIT FLUX")==0);
    assert(strcmp(status_value("H Flux"),"0 uWb")==0);
    assert(strcmp(status_value("H Time"),"0 us / 0")==0);
    assert(strcmp(status_value("H Gate"),"OK")==0);
    assert(strcmp(status_value("Feedback"),"ENCODER")==0);
    assert(strcmp(status_value("H Qual"),"0/200 ms 0/20")==0);
    assert(strcmp(status_value("H Jitter"),"0/50 0/10")==0);
    assert(strcmp(status_value("H Reset"),"0.0 deg")==0);
    assert(strcmp(status_value("H Lost"),"--")==0);
    assert(strcmp(status_value("VREF"),"3.260 V")==0);
    assert(strcmp(status_value("RefState"),"OK")==0);
    assert(strcmp(status_value("Ref R/C"),"1518 / 1650")==0);
    reference_data=false; step(0,501);
    assert(strcmp(status_value("RefState"),"STALE")==0);
    assert(strcmp(status_value("VREF"),"3.260 V")==0); /* Mark retained estimate stale. */
    reference_seen=false; step(0,501);
    assert(strcmp(status_value("RefState"),"WAIT")==0);
    assert(strcmp(status_value("VREF"),"NO DATA")==0); /* Never show a guessed value as measured. */
    reference_data=reference_seen=true; step(0,501);
    for(unsigned row=0;row<status->row_count;++row) {
        if(strcmp(status->rows[row].name,"DispErr")==0) assert(strcmp(status->rows[row].value,"2")==0);
        if(strcmp(status->rows[row].name,"KeyRpt")==0) assert(strcmp(status->rows[row].value,"0")!=0);
    }
    /* HSO fault acknowledgement is explicit and never restarts the motor. */
    reset(); press(KK_UI_KEY_OK); settle();
    press(KK_UI_KEY_UP); press(KK_UI_KEY_OK); settle();
    hso_active=hso_fault=true; settle();
    assert(hso_fault && JerryFOC_Target_Iq==0);
    assert(strcmp(status_value("H Lost"),"ANGLE")==0);
    drive_ready=false; unready_status="M1 OVERLOAD"; settle();
    assert(strcmp(status_value("Drive"),"M1 OVERLOAD")==0); /* Do not mask the root fault. */
    drive_ready=true; settle();
    press(KK_UI_KEY_OK); settle();
    assert(!hso_fault && !hso_active && JerryFOC_Target_Iq==0);
    press(KK_UI_KEY_OK); settle();
    hso_active=true; back(); assert(!hso_active && JerryFOC_Target_Iq==0);
    /* A transient encoder outage can recover to READY/OK; preserve exactly
     * why Run stopped, including age/error count before stop_run changes mode. */
    reset(); press(KK_UI_KEY_OK); settle(); press(KK_UI_KEY_RIGHT);
    press(KK_UI_KEY_UP); press(KK_UI_KEY_OK);
    assert(mode==JERRYFOC_MODE_VELOCITY && JerryFOC_Target_Velocity>0);
    enc_diag=(MT6701Diagnostics){.errors=443,.crc_errors=443,.age_cycles=4,.max_age_cycles=4,.last_error=MT6701_ERROR_CRC};
    healthy=false; step(0,1);
    assert(strcmp(status_value("Run Stop"),"ENCODER LOST")==0);
    assert(strcmp(status_value("Stop Age"),"4")==0 && strcmp(status_value("Stop Err"),"443")==0);
    assert(JerryFOC_Target_Velocity==0 && JerryFOC_Target_Iq==0);
    healthy=true; enc_diag.age_cycles=1; enc_diag.errors=444; settle();
    assert(strcmp(status_value("Drive"),"READY")==0 && strcmp(status_value("Encoder"),"OK")==0);
    assert(strcmp(status_value("Run Stop"),"ENCODER LOST")==0);
    assert(strcmp(status_value("Stop Err"),"443")==0);
    assert(strcmp(status_value("Enc CRC"),"443")==0 && strcmp(status_value("Enc Age"),"1 / 4")==0);
    assert(strcmp(status_value("Enc Last"),"CRC")==0);
    snapshot("run-encoder-stop");
    press(KK_UI_KEY_OK); assert(JerryFOC_Target_Velocity>0 && strcmp(status_value("Run Stop"),"--")==0);
    const char *stop_causes[]={"M1 OVERLOAD","NOT ALIGNED","MODE CHANGED","OPEN LOOP"};
    for(unsigned cause=0;cause<4;++cause) {
        reset(); press(KK_UI_KEY_OK); settle(); press(KK_UI_KEY_UP); press(KK_UI_KEY_OK);
        if(cause==0) { drive_ready=false; healthy=false; unready_status="M1 OVERLOAD"; }
        if(cause==1) aligned=false;
        if(cause==2) mode=JERRYFOC_MODE_VELOCITY;
        if(cause==3) g_openloop_debug=1;
        step(0,1);
        assert(strcmp(status_value("Run Stop"),stop_causes[cause])==0);
        assert(JerryFOC_Target_Iq==0 && !g_openloop_debug);
        drive_ready=healthy=aligned=true; settle();
        assert(strcmp(status_value("Run Stop"),stop_causes[cause])==0);
    }
    /* Run zero-speed confirmation keeps PWM; the next unchanged OK and KEY2
     * request an actual coast. The 1 kHz UI must not auto-restart afterwards. */
    reset(); press(KK_UI_KEY_OK); settle(); press(KK_UI_KEY_RIGHT);
    press(KK_UI_KEY_UP); press(KK_UI_KEY_OK); assert(output_requested);
    press(KK_UI_KEY_DOWN); press(KK_UI_KEY_OK);
    assert(output_requested && JerryFOC_Target_Velocity==0 && mode==JERRYFOC_MODE_VELOCITY);
    press(KK_UI_KEY_OK); assert(!output_requested && JerryFOC_Target_Iq==0);
    settle(); assert(!output_requested && strcmp(status_value("Output"),"COAST")==0);
    velocity=10; press(KK_UI_KEY_OK); assert(!output_requested);
    velocity=0;
    press(KK_UI_KEY_OK); assert(output_requested); back(); assert(!output_requested);
    /* Idle Run navigation preserves an independent startup alignment. */
    reset(); aligning=true; aligned=false; output_requested=true;
    press(KK_UI_KEY_OK); settle(); assert(output_requested);
    back(); assert(output_requested); aligning=false; aligned=true;
    /* Actual Settings -> Motor menus and modal confirmation: drafts/cancel
     * never select a motor, while a successful change clears all old targets. */
    reset(); settings.speed_x10=1000; settings.voltage_x100=300;
    settings.position_x100=100; settings.flags=APP_FLAG_CURRENT_LOOP;
    ui_app_reset(tick); assert(KK_UI_Init(ui_app_get())==KK_UI_OK); settle();
    press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK); settle();
    select_setting("Motor"); press(KK_UI_KEY_OK); settle();
    assert(KK_UI_GetCurrentPage()==UI_PAGE_MOTOR);
    press(KK_UI_KEY_DOWN); press(KK_UI_KEY_OK); settle();
    assert(kk_ui.overlay.type==KK_UI_OVERLAY_CONFIRM); snapshot("motor-confirm");
    back(); assert(MotorParams_GetActiveProfile()==MOTOR_PROFILE_2804 && settings.speed_x10==1000);
    press(KK_UI_KEY_OK); settle(); press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK); settle();
    assert(MotorParams_GetActiveProfile()==MOTOR_PROFILE_4015 && !aligned);
    assert((settings.flags & APP_FLAG_MOTOR_4015) && !(settings.flags & APP_FLAG_CURRENT_LOOP));
    assert(settings.speed_x10==0 && settings.voltage_x100==0 && settings.position_x100==0);
    assert(JerryFOC_Target_Iq==0 && JerryFOC_Target_Velocity==0 && JerryFOC_Target_Position==0);
    assert(strcmp(status_value("Motor"),"4015")==0);
    assert(strcmp(ui_app_get()->info_pages[1].rows[2].value,"4015")==0);
    snapshot("motor-4015"); back();
    select_setting("Speed Limit"); press(KK_UI_KEY_OK); settle();
    adjust_editor(-1000); assert(kk_ui.overlay.draft==120); back();
    back(); press(KK_UI_KEY_LEFT); press(KK_UI_KEY_OK); settle();
    press(KK_UI_KEY_UP); press(KK_UI_KEY_OK); assert(JerryFOC_Target_Iq==0); /* No alignment yet. */
    aligned=true; press(KK_UI_KEY_OK); assert(JerryFOC_Target_Iq>0);
    back(); press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK); settle();
    select_setting("Motor"); press(KK_UI_KEY_OK); settle();
    /* A moving rotor rejects the change and preserves the selected profile. */
    velocity=10;
    press(KK_UI_KEY_OK); settle(); press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK); settle();
    assert(MotorParams_GetActiveProfile()==MOTOR_PROFILE_4015 && (settings.flags & APP_FLAG_MOTOR_4015));
    velocity=0;
    press(KK_UI_KEY_OK); settle(); press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK); settle();
    assert(MotorParams_GetActiveProfile()==MOTOR_PROFILE_2804 && !(settings.flags & APP_FLAG_MOTOR_4015));
    assert(!aligned && JerryFOC_Target_Iq==0);
    /* Reloading UI preserves the active model and clamps old range records. */
    reset(); MotorParams_Select(MOTOR_PROFILE_4015);
    settings.flags=APP_FLAG_MOTOR_4015 | (600UL<<APP_SPEED_LIMIT_SHIFT);
    settings.speed_x10=5000;
    ui_app_reset(tick); assert(KK_UI_Init(ui_app_get())==KK_UI_OK); settle();
    press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK); settle();
    select_setting("Speed Limit"); press(KK_UI_KEY_OK); settle();
    assert(kk_ui.overlay.draft==120); back();
    /* The same Run control adopts the selected motor's dq and voltage ranges. */
    back(); press(KK_UI_KEY_LEFT); press(KK_UI_KEY_OK); settle();
    KK_UI_CustomOnInput(UI_PAGE_RUN,(KK_UI_InputEvent){KK_UI_INPUT_UP,KK_UI_INPUT_ENCODER,10000});
    press(KK_UI_KEY_OK); assert(fabsf(JerryFOC_Target_Iq-13.86f)<0.001f); back();
    settings.flags |= APP_FLAG_CURRENT_LOOP;
    ui_app_reset(tick); assert(KK_UI_Init(ui_app_get())==KK_UI_OK); settle();
    press(KK_UI_KEY_OK); settle();
    KK_UI_CustomOnInput(UI_PAGE_RUN,(KK_UI_InputEvent){KK_UI_INPUT_UP,KK_UI_INPUT_ENCODER,10000});
    press(KK_UI_KEY_OK); assert(fabsf(JerryFOC_Target_Iq-1.2f)<0.001f); back();
    press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK); settle();
    select_setting("Reset"); press(KK_UI_KEY_OK); settle();
    press(KK_UI_KEY_RIGHT); press(KK_UI_KEY_OK); settle();
    assert(MotorParams_GetActiveProfile()==MOTOR_PROFILE_2804 && !(settings.flags & APP_FLAG_MOTOR_4015));
    /* Export the user's two exact home sequences and rapid reversals. */
    const char *record_paths[]={"home-forward.frames","home-reverse.frames","home-rapid.frames"};
    for(unsigned sequence=0;sequence<3;++sequence) {
        reset();
        if(sequence==0) { press(KK_UI_KEY_LEFT); settle(); }
        char path[160]; snprintf(path,sizeof(path),"build/ui-validation/%s",record_paths[sequence]);
        record_file=fopen(path,"wb"); assert(record_file); step(0,200);
        if(sequence<2) {
            for(unsigned i=0;i<2;++i) { press(sequence==0?KK_UI_KEY_RIGHT:KK_UI_KEY_LEFT); step(0,350); }
        } else {
            for(unsigned i=0;i<8;++i) press(i<5?KK_UI_KEY_RIGHT:KK_UI_KEY_LEFT);
            step(0,400);
        }
        step(0,200); fclose(record_file); record_file=NULL;
        assert(kk_ui.current.selected==(sequence==0?1U:2U));
    }
    printf("PASS: GPIO, axes, bounce, repeat, Back/cancel, run modes, speed range, stop, fault, position origin, wrap, animations, async DMA (%u submissions)\n",submissions);
    return 0;
}
