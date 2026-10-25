#include "ui_app.h"

#include <math.h>
#include "ui_text.h"
#include <stdlib.h>
#include <string.h>

#include "app_storage.h"
#include "angle_src_encoder.h"
#include "JerryFOC.h"
#include "kk_oled.h"
#include "kk_ui_draw.h"
#include "MT6701.h"
#include "motor_sampling.h"
#include "st_hso.h"
#include "ui_fonts.h"
#include "ui_icons.h"
#include "motor_control.h"
#include "ui_input.h"
#include "protocol.h"
#include "motor_sixstep.h"
#include "motor_detect.h"

/* Application-owned text shares a base address. Descriptor offsets refer to
 * these permanent buffers; font resources and UI core ownership are unchanged. */
typedef struct {
    char ibus_state[24],ibus_limit[24],ibus_voltage[24];
    char ibus_code[16],ibus_trips[16],ibus_pin[8];
    char mode[16];
    char command[8];
    char input[8];
    char work[20],axis[4],m1[8],m2[8];
    char sync[20],control_time[20],save[16];
    char motor[8], motor_menu[20];
    char settings[24],steps[20],loop_title[24];
    char system[24],work_menu[24],save_state[24];
    char status[20],groups[5][20];
    char rpm[24],target_aux[24],elec_deg[24],mech_deg[24];
    char target_unit[12];
    char hso_min[24],hso_max[24];
    char target[24];
    char speed[24];
    char iq[24];
    char id[24];
    char ia[24];
    char ib[24];
    char enc[16];
    char enc2[16];
    char enc2_err[16];
    char drive[16];
    char output[8];
    char run_stop[24];
    char stop_age[16], stop_errors[16];
    char enc_crc[16], enc_field[16], enc_track[16];
    char enc_spi[16], enc_frame[16], enc_age[24], enc_last[16];
    char enc_zero[16], enc_zpre[16], enc_zpost[16], enc_ztime[16];
    char current_adc[3][24];
    char current_trip[24];
    char adc_revision[16];
    char align[16];
    char raw[16];
    char elec[24];
    char mech[24];
    char err[16];
    char bus[24];
    char bus_raw[16];
    char bus_count[16];
    char frame_count[16];
    char adc1_error[16];
    char adc2_error[16];
    char reference[16], ref_state[16], ref_raw[16];
    char ref_count[16], ref_errors[16];
    char repeat[16];
    char display_errors[16];
    char hso[16], hso_speed[24], hso_error[24];
    char hso_flux[24], hso_time[24];
    char hso_gate[16], hso_qual[24], hso_lost[16];
    char hso_flux_actual[24];
    char hso_reject[16],hso_resets[16];
    char hso_rangle[16],hso_rfreq[16],hso_remf[16],hso_rdrift[16];
    char hso_offset[16],hso_olimit[16];
    char save_result[16];
    char flash_error[16];
    char spi2_fault[16],spi2_resets[8];
    char feedback[16];
    char six_quality[24],six_duty[24],six_bemf[24],six_speed[24];
    char six_start[24],six_start_emf[24];
    char six_reject[16],six_sample[4];
} UiTexts;
static UiTexts texts;
static char s_txt_axes[2][10]={"Axis M1  ","Axis M2  "};
static char s_txt_work_items[MOTOR_WORK_COUNT][21]={"  Single M1","  Single M2","  Dual Independent",
    "  Dual Sync Speed","  Dual Sync Position","  Dual Master M1","  Dual Master M2"};

enum {
    EVT_CURRENT_LOOP_CHANGED = 1,
    EVT_LED_CHANGED,
    EVT_RESET_CONFIRMED,
    EVT_RESET_CANCELLED,
    EVT_POSITION_SPEED_CHANGED,
    EVT_RUN_STEPS_CHANGED,
    EVT_SPEED_LIMIT_CHANGED,
    EVT_MOTOR_2804_CONFIRMED,
    EVT_MOTOR_4015_CONFIRMED,
    EVT_FOCUS_M1, EVT_FOCUS_M2, EVT_FEEDBACK, EVT_GEAR, EVT_ALIGN, EVT_SAVE_STATUS,
    EVT_IBUS_LIMIT, EVT_IBUS_CLEAR, EVT_IBUS_STATUS,
    EVT_SIXSTEP, EVT_MOTOR_ESC,
    EVT_WORK_BASE=32,
};

/* ---------- 设置菜单下标 ---------- */
enum { INT_LED_BRIGHT, INT_POSITION_SPEED, INT_SPEED_LIMIT, INT_GEAR_RATIO, INT_GEAR_OFFSET, INT_IBUS_LIMIT };
enum { BOOL_CURRENT_LOOP, BOOL_LED, BOOL_HSO, BOOL_SIXSTEP };
enum { CONFIRM_RESET, CONFIRM_MOTOR_2804, CONFIRM_MOTOR_4015, CONFIRM_IBUS_CLEAR, CONFIRM_MOTOR_ESC };

/* ---------- 设置变量 ---------- */
static int32_t s_led_brightness;
static bool s_current_loop_display;
static bool s_led_enable;
static bool s_hso_choice;
static bool s_six_choice;
static int32_t s_ratio_x100=100,s_offset_x100;
static uint8_t sensorless_states[1];
static uint8_t motor_states[2];
static int32_t s_detect_ma=300;
static uint32_t s_detect_result;
static uint32_t s_detect_l,s_detect_flux;
static uint8_t s_detect_page;
static uint8_t settings_states[3]={[2]=KK_UI_ITEM_DISABLED<<2U};
static uint8_t protection_states[1]={[0]=KK_UI_ITEM_DISABLED<<4U};
static int32_t s_ibus_limit_ma;
/* 下标与 JerryFOC_ControlMode 对应：Torque、Speed、Position。 */
static int32_t s_step_drafts[3];
static uint8_t s_step_selected;

/* Run 页当前目标 */
static uint32_t s_last_input_at;
static uint32_t s_last_live_at;
/* Independent Run only: a short L/R action waits for release; holding never
 * performs the short action first. The press itself is debounced by KK_UI. */
static uint8_t s_run_lr_key;
static bool s_run_lr_releasing;
static bool s_global_stop_seen;
static uint32_t s_run_lr_pressed_at,s_run_lr_released_at;

/* ---------- 动态文本 ---------- */
static MotorProfile s_profiles[2];
static const char *const group_names[]={"Drive","Motion","Encoder","ADC"};
/* ui_app_reset always seeds these before KK_UI_Init; no Flash zero padding. */
/* Stable application-owned labels; update only their axis digit. */
static char s_enc_labels[][9]={"E1 State","E1 CRC","E1 Field","E1 Track",
    "E1 SPI","E1 Frame","E1 Age","E1 Last","E1 Raw","E1 Err",
    "E1 Zero","E1 ZPre","E1 ZPost","E1 ZTime"};
static uint32_t s_input_repeats;
static uint32_t s_display_errors;

static uint32_t s_info_at;
static bool s_settings_dirty;
static uint32_t s_settings_save_at;
static const char *s_pending_toast;

static MotorAxis s_view_axis;
__attribute__((noinline)) static MotorAxisSnapshot ui_state(void)
{
    MotorAxisSnapshot state; MotorControl_GetAxis(s_view_axis,&state); return state;
}
static void ui_encoder_diag(MT6701Diagnostics *out)
{
    if (s_view_axis==MOTOR_AXIS_1) MT6701_GetDiagnostics(out);
    else MT6701_2_GetDiagnostics(out);
}
static uint16_t ui_encoder_raw(void)
{
    if (s_view_axis==MOTOR_AXIS_1) return MT6701_GetRawAngle();
    MT6701_2_Snapshot e; MT6701_2_GetSnapshot(&e); return e.raw_angle;
}
static uint32_t ui_encoder_errors(void)
{
    MT6701Diagnostics e; ui_encoder_diag(&e); return e.errors;
}
static bool ui_store_current(void);
static void sync_from_storage(void);
static void focus_axis(MotorAxis axis);
static void update_position_speed_limit(void);
typedef struct {
    int32_t speed_x10;
    int32_t voltage_x100;
    int32_t position_x100;
    int32_t position_speed;
    int32_t speed_limit;
    bool current_loop;
    JerryFOC_ControlMode run_mode;
    int32_t run_speed_x10;
    int32_t run_voltage_x100;
    int32_t run_position_x100;
    int32_t run_current_x100;
    int32_t confirmed_current_x100;
    float run_position_origin;
    bool run_active;
    int32_t steps[3];
} AxisView;
static AxisView views[2];
static AxisView current_view;
/* Six-step throttle must never overwrite encoder Uq/Iq settings. */
static int32_t s_throttle_pct,s_run_throttle_pct;
#define s_speed_x10 (current_view.speed_x10)
#define s_voltage_x100 (current_view.voltage_x100)
#define s_position_x100 (current_view.position_x100)
#define s_position_speed (current_view.position_speed)
#define s_speed_limit (current_view.speed_limit)
#define s_current_loop (current_view.current_loop)
#define s_run_mode (current_view.run_mode)
#define s_run_speed_x10 (current_view.run_speed_x10)
#define s_run_voltage_x100 (current_view.run_voltage_x100)
#define s_run_position_x100 (current_view.run_position_x100)
#define s_run_current_x100 (current_view.run_current_x100)
#define s_confirmed_current_x100 (current_view.confirmed_current_x100)
#define s_run_position_origin (current_view.run_position_origin)
#define s_run_active (current_view.run_active)
#define s_run_steps (current_view.steps)
static void save_view(void) { views[s_view_axis]=current_view; }
static void load_view(MotorAxis axis)
{
    s_view_axis=axis;
    current_view=views[axis];
    update_position_speed_limit();
}

/* ---------- 工具 ---------- */
static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* The displayed 0.1-rad/s grid must lie inside the observer's strict
 * 5 <= electrical Hz < 400 operating range, independently of Speed Limit. */
__attribute__((noinline)) static bool sensorless(void) { return MotorControl_HsoAllowed() || MotorControl_SixStepAllowed(); }
__attribute__((noinline)) static bool esc_view(void) { return s_view_axis==MOTOR_AXIS_1 && SixStep_IsGeneric(); }
static JerryFOC_ControlMode fallback_mode(void) {
    return esc_view() ? JERRYFOC_MODE_TORQUE : MotorControl_ModeAllowed(JERRYFOC_MODE_VELOCITY) ?
        JERRYFOC_MODE_VELOCITY : JERRYFOC_MODE_POSITION;
}
__attribute__((noinline)) static bool throttle_mode(JerryFOC_ControlMode mode) { return mode==JERRYFOC_MODE_TORQUE && s_view_axis==MOTOR_AXIS_1 && MotorControl_SixStepAllowed(); }
static int run_scale(bool oil) { return oil ? 1 : s_run_mode==JERRYFOC_MODE_VELOCITY ? 10 : 100; }
static int32_t hso_min_x10(void) { return (int32_t)ceilf((MotorControl_SixStepAllowed() ? SixStep_MinSpeed() : StHso_MinSpeed())*10.0f); }
static int32_t hso_max_x10(void)
{
    int32_t limit=(int32_t)(400.0f*6.283185307f*10.0f/MotorParams_GetAxis(MOTOR_AXIS_1)->pole_pairs);
    if (MotorControl_SixStepAllowed()) limit=(int32_t)floorf(SixStep_MaxSpeed()*10.0f);
    int32_t configured=(s_view_axis==MOTOR_AXIS_1 ? s_speed_limit : views[0].speed_limit)*10;
    return configured < limit ? configured : limit;
}

static int32_t hso_draft(int32_t value)
{
    int32_t min=hso_min_x10(),max=hso_max_x10();
    if (max<min) return 0;
    if (value==0) return 0;
    return value>0 ? clamp_i32(value,min,max) : clamp_i32(value,-max,-min);
}

__attribute__((noinline)) static int32_t to_x10(float v)
{
    return (int32_t)(v * 10.0f + copysignf(0.5f,v));
}

__attribute__((noinline)) static int32_t to_x100(float v)
{
    return (int32_t)(v * 100.0f + copysignf(0.5f,v));
}

__attribute__((noinline)) static void fmt_fixed(char *buf, size_t len, int32_t v, int scale)
{
    UiText_Fixed(buf,len,v,scale==1 ? 0U : scale==10 ? 1U : 2U,NULL);
}

/* Keep repeated Status conversions in main-loop helpers, rather than
 * expanding rounding and two snprintf call frames for every diagnostic row. */
__attribute__((noinline)) static void fmt_unit(char *buf,int32_t value,int scale,const char *unit)
{
    UiText_Fixed(buf,24U,value,scale==1 ? 0U : scale==10 ? 1U : 2U,unit);
}

/* Share the repeated 16-byte diagnostic-counter formatting in the 128KiB
 * image. A bounded call avoids expanding the same snprintf setup per row. */
__attribute__((noinline)) static void fmt_counter(char *buf,uint32_t count)
{
    UiText_Format(buf,16U,"%lu",(unsigned long)count);
}

__attribute__((noinline)) static void fmt_pair(char *buf,unsigned first,unsigned second)
{
    UiText_Format(buf,16U,"%u / %u",first,second);
}

/* Fixed application descriptors, not UI widgets/callbacks. Each row occupies
 * four bytes; memcpy permits both 16- and 32-bit source fields. */
enum { VALUE_COUNT, VALUE_RAD, VALUE_AMP, VALUE_TENTH, VALUE_DEG,
       VALUE_PERCENT, VALUE_MV, VALUE_MWB, VALUE_COUNT16 };
typedef struct { uint16_t text_offset; uint8_t offset,format; } ValueRow;
_Static_assert(sizeof(ValueRow)==4,"Compact display descriptor");
_Static_assert(sizeof(UiTexts)<=UINT16_MAX,"Display buffer offsets");
_Static_assert(sizeof(MotorAxisSnapshot)<=UINT8_MAX && sizeof(StHsoSnapshot)<=UINT8_MAX &&
    sizeof(MT6701Diagnostics)<=UINT8_MAX && sizeof(MotorSamplingDiagnostics)<=UINT8_MAX,
    "Review source offsets before extending display snapshots");
#define VALUE_ROW(type,member,field,format) {offsetof(UiTexts,field),offsetof(type,member),format}
__attribute__((noinline)) static void refresh_values(const void *snapshot,const ValueRow *rows,unsigned count)
{
    for(unsigned i=0;i<count;++i) {
        const ValueRow *row=&rows[i];
        const char *source=(const char *)snapshot+row->offset;
        char *destination=(char *)&texts+row->text_offset;
        if(row->format==VALUE_COUNT || row->format==VALUE_COUNT16) {
            uint32_t value=0;
            memcpy(&value,source,row->format==VALUE_COUNT ? sizeof(value) : sizeof(uint16_t));
            fmt_counter(destination,value);
        } else {
            float value; memcpy(&value,source,sizeof(value));
            const char *unit=NULL;
            unsigned places=1;
            switch(row->format) {
            case VALUE_RAD: unit="rad"; places=2; break;
            case VALUE_AMP: unit="A"; places=2; break;
            case VALUE_DEG: value*=57.2957795f; break;
            case VALUE_PERCENT: value*=100.0f; break;
            case VALUE_MV: value*=1000.0f; break;
            case VALUE_MWB: value*=1000.0f; unit="mWb"; break;
            default: break;
            }
            UiText_Fixed(destination,unit ? 24U : 16U,
                places==2 ? to_x100(value) : to_x10(value),places,unit);
        }
    }
}

/* Angle auxiliaries in Status describe the within-turn angle. The original
 * multi-turn rad value remains available; reduce before integer conversion. */
static void fmt_degrees(char *buf,float radians)
{
    if (!isfinite(radians)) { strcpy(buf,"--"); return; }
    float turn=fmodf(radians,6.283185307f);
    if (turn<0.0f) turn+=6.283185307f;
    int32_t deg=to_x10(turn*57.2957795f);
    fmt_unit(buf,deg==3600 ? 0 : deg,10,"deg");
}

static void fmt_run_aux(char *buf,int32_t value)
{
    if(esc_view()) { strcpy(buf,"Hz"); return; }
    bool position=s_run_mode==JERRYFOC_MODE_POSITION;
    UiText_Format(buf,16U,"%ld%s",(long)to_x10((float)value*(position ? 0.05729578f : 0.095492966f)),
        position ? "deg" : "rpm");
}

static const char *mode_name(JerryFOC_ControlMode mode)
{
    switch (mode) {
    case JERRYFOC_MODE_VELOCITY: return "Speed";
    case JERRYFOC_MODE_POSITION: return "Position";
    default: return "Torque";
    }
}

static const char *drive_status_text(void) { return ui_state().status; }

static void refresh_save_state(void)
{
    AppSaveState state=AppStorage_GetSaveState();
    if (s_settings_dirty && AppStorage_IsReady() && state!=APP_SAVE_WRITING)
        state=AppStorage_IsDeferred() ? APP_SAVE_DEFERRED : APP_SAVE_PENDING;
    static const char *const names[]={"Defaults","Pending","Saving","Wait bus","Saved","Failed"};
    UiText_Format(texts.save_state,sizeof(texts.save_state),"Save: %s",names[state]);
}

__attribute__((noinline)) static void axis_title(char *text,const char *suffix)
{
    UiText_Format(text,20U,"M%u %s %s",(unsigned)s_view_axis+1U,texts.motor,suffix);
}

static void refresh_texts(void)
{
    const MotorAxisSnapshot state=ui_state();
    strcpy(texts.work,MotorControl_WorkText());
    UiText_Format(texts.command,sizeof(texts.command),"%d",(int)Protocol_GetControlResult());
    strcpy(texts.input,MotorControl_InputText(s_view_axis));
    strcpy(texts.axis,s_view_axis==MOTOR_AXIS_1 ? "M1" : "M2");
    strcpy(texts.m1,SixStep_IsGeneric() ? "ESC" : MotorParams_GetAxis(MOTOR_AXIS_1)->name);
    strcpy(texts.m2,MotorParams_GetAxis(MOTOR_AXIS_2)->name);
    fmt_fixed(texts.sync,sizeof(texts.sync),to_x100(MotorControl_GroupError()),100);
    UiText_Format(texts.control_time,sizeof(texts.control_time),"%lu us",(unsigned long)(MotorControl_MaxCycles()/170U));
    strcpy(texts.save,AppStorage_IsDeferred() ? "DEFERRED" : "AVAILABLE");
    strcpy(texts.save_result,AppStorage_SaveErrorText());
    uint32_t flash_error=AppStorage_FlashError();
    MT6701_2_BusDiagnostics bus_diag;
    MT6701_2_GetBusDiagnostics(&bus_diag);
    UiText_Format(texts.spi2_fault,sizeof(texts.spi2_fault),"%04X/%04X/%04X",
             (unsigned)bus_diag.cr1,(unsigned)bus_diag.cr2,(unsigned)bus_diag.sr);
    UiText_Format(texts.spi2_resets,sizeof(texts.spi2_resets),"%u",(unsigned)bus_diag.resets);
    UiText_Format(texts.flash_error,sizeof(texts.flash_error),"%lu/%lu/%02lX/%02lX",
        (unsigned long)(flash_error>>24),(unsigned long)((flash_error>>16)&255U),
        (unsigned long)((flash_error>>8)&255U),(unsigned long)(flash_error&255U));
    s_hso_choice=MotorControl_HsoAllowed();
    s_six_choice=MotorControl_SixStepAllowed();
    strcpy(texts.motor,esc_view() ? "ESC" : MotorParams_GetAxis(s_view_axis)->name);
    axis_title(texts.motor_menu,"Motor");
    axis_title(texts.settings,"Settings");
    axis_title(texts.steps,"Steps");
    axis_title(texts.status,"Status");
    for (unsigned i=0;i<4;++i) axis_title(texts.groups[i],group_names[i]);
    UiText_Format(texts.groups[4],sizeof(texts.groups[4]),"M1 %s HSO",MotorParams_GetAxis(MOTOR_AXIS_1)->name);
    fmt_unit(texts.hso_min,hso_min_x10(),10,"rad/s");
    fmt_unit(texts.hso_max,hso_max_x10(),10,"rad/s");
    if (hso_max_x10()<hso_min_x10()) strcpy(texts.hso_max,"No run range");
    axis_title(texts.loop_title,"Curr Loop");
    UiText_Format(texts.system,sizeof(texts.system),"System: %s",MotorControl_WorkText());
    UiText_Format(texts.work_menu,sizeof(texts.work_menu),"Mode: %s",MotorControl_WorkText());
    for (unsigned i=0;i<MOTOR_WORK_COUNT;++i)
        s_txt_work_items[i][0]=i==(unsigned)MotorControl_GetWork() ? '*' : ' ';
    for (unsigned i=0;i<2;++i)
        s_txt_axes[i][8]=i==(unsigned)s_view_axis ? '*' : ' ';
    s_current_loop_display=s_current_loop || (s_view_axis==MOTOR_AXIS_1 && MotorControl_HsoAllowed());
    if (s_view_axis==MOTOR_AXIS_1 && MotorControl_SixStepAllowed()) s_current_loop_display=false;
    refresh_save_state();
    strcpy(texts.output,
             !MotorSampling_IsAxisOutputRequested(s_view_axis) ? "COAST" :
             MotorSampling_IsAxisOutputEnabled(s_view_axis) ? "PWM" : "OFF");
    fmt_counter(texts.repeat,s_input_repeats);
    fmt_counter(texts.display_errors,s_display_errors);

    bool oil=throttle_mode(state.mode);
    strcpy(texts.mode,oil ? "Throttle" : mode_name(state.mode));

    switch (state.mode) {
    case JERRYFOC_MODE_VELOCITY:
        fmt_unit(texts.target, to_x10(state.target_velocity), 10, "rad/s");
        break;
    case JERRYFOC_MODE_POSITION:
        fmt_unit(texts.target, to_x100(state.target_position), 100, "rad");
        break;
    default:
        fmt_unit(texts.target,to_x100(state.target_iq),100,oil ? "%" : s_current_loop ? "A" : "V");
        break;
    }

    fmt_unit(texts.speed, to_x10(state.velocity), 10, esc_view() ? "Hz" : "rad/s");
    fmt_unit(texts.rpm,to_x10(state.velocity*9.5492966f),10,"rpm");
    if(esc_view()) strcpy(texts.rpm,"--");
    if (state.mode==JERRYFOC_MODE_VELOCITY) {
        strcpy(texts.target_unit,"Target rpm");
        fmt_unit(texts.target_aux,to_x10(state.target_velocity*9.5492966f),10,"rpm");
    } else if (state.mode==JERRYFOC_MODE_POSITION) {
        strcpy(texts.target_unit,"Tgt 0-360");
        fmt_degrees(texts.target_aux,state.target_position);
    } else {
        strcpy(texts.target_unit,"Target aux"); strcpy(texts.target_aux,"--");
    }
    fmt_degrees(texts.elec_deg,state.electrical_angle);
    fmt_degrees(texts.mech_deg,state.position);

    static const ValueRow motion_values[]={
        VALUE_ROW(MotorAxisSnapshot,iq,iq,VALUE_AMP),
        VALUE_ROW(MotorAxisSnapshot,id,id,VALUE_AMP),
        VALUE_ROW(MotorAxisSnapshot,ia,ia,VALUE_AMP),
        VALUE_ROW(MotorAxisSnapshot,ib,ib,VALUE_AMP),
        VALUE_ROW(MotorAxisSnapshot,electrical_angle,elec,VALUE_RAD),
        VALUE_ROW(MotorAxisSnapshot,position,mech,VALUE_RAD),
    };
    refresh_values(&state,motion_values,sizeof(motion_values)/sizeof(motion_values[0]));

    strcpy(texts.enc,
             (s_view_axis==MOTOR_AXIS_1 ? MT6701_IsHealthy()!=0U : state.healthy) ? "OK" : "ERROR");
    MT6701_2_Snapshot encoder2;
    MT6701_2_GetSnapshot(&encoder2);
    UiText_Format(texts.enc2, sizeof(texts.enc2), "%s %u",
             encoder2.healthy ? "OK" : encoder2.stopped ? "OFF" : "ERROR", (unsigned)encoder2.raw_angle);
    fmt_counter(texts.enc2_err,encoder2.errors);
    UiText_Format(texts.drive, sizeof(texts.drive), "%s",
             drive_status_text());
    StHsoSnapshot hso;
    StHso_GetSnapshot(&hso);
    strcpy(texts.feedback,(s_view_axis==MOTOR_AXIS_1 ? JerryFOC_getFeedbackText() : "ENCODER"));
    SixStepSnapshot six; SixStep_GetSnapshot(&six);
    UiText_Format(texts.six_quality,sizeof(texts.six_quality),"%u/12 %c %u/%u.%c",six.quality,six.gate ? six.gate : '-',six.phase_pct,six.period_pct,'0'+six.period_fraction%10U);
    static const char *const reject_text[]={"-","PHASE EARLY","PHASE LATE","PERIOD LOW","PERIOD HIGH"};
    strcpy(texts.six_reject,reject_text[six.reject<=SIX_REJECT_LONG ? six.reject : 0]);
    strcpy(texts.six_sample,six.sampling_off ? "OFF" : "ON");
    fmt_unit(texts.six_duty,to_x10(six.duty*100.0f),10,"%");
    fmt_unit(texts.six_bemf,to_x100(six.bemf),100,"V");
    fmt_unit(texts.six_speed,to_x10(six.velocity),10,SixStep_IsGeneric() ? "Hz" : "rad/s");
    UiText_Format(texts.six_start,sizeof(texts.six_start),"%u/500 ms",six.startup_ms);
    fmt_unit(texts.six_start_emf,to_x100(six.startup_emf),100,"V");
    /* All reason literals fit these 16-byte application buffers. */
    strcpy(texts.hso_gate,StHso_ReasonText(JerryFOC_isHSOActive() ? hso.reason : hso.entry_reason));
    UiText_Format(texts.hso_qual, sizeof(texts.hso_qual), "%lu/%u ms %lu/20",
             (unsigned long)(hso.qualified_samples / 10U),
             hso.state==ST_HSO_LEARNING || hso.reason==ST_HSO_REASON_LEARN_TIMEOUT ? 1000U : 200U,
             (unsigned long)hso.bad_samples);
    strcpy(texts.hso_lost,JerryFOC_getHSOFaultText());
    strcpy(texts.hso_reject,StHso_ReasonText(hso.capture_reason));
    static const ValueRow hso_values[]={
        VALUE_ROW(StHsoSnapshot,capture_resets,hso_resets,VALUE_COUNT),
        VALUE_ROW(StHsoSnapshot,capture_angle_rad,hso_rangle,VALUE_DEG),
        VALUE_ROW(StHsoSnapshot,capture_frequency_error_hz,hso_rfreq,VALUE_TENTH),
        VALUE_ROW(StHsoSnapshot,capture_emf_ratio,hso_remf,VALUE_PERCENT),
        VALUE_ROW(StHsoSnapshot,capture_drift_rad,hso_rdrift,VALUE_DEG),
        VALUE_ROW(StHsoSnapshot,emf_offset_v,hso_offset,VALUE_MV),
        VALUE_ROW(StHsoSnapshot,emf_offset_limit_v,hso_olimit,VALUE_MV),
        VALUE_ROW(StHsoSnapshot,flux_wb,hso_flux_actual,VALUE_MWB),
    };
    refresh_values(&hso,hso_values,sizeof(hso_values)/sizeof(hso_values[0]));
    strcpy(texts.hso,
             JerryFOC_hasHSOFault() ? "LOST" : StHso_StateText(hso.state));
    fmt_unit(texts.hso_speed, to_x10(hso.velocity_rad_s), 10, "rad/s");
    if (isfinite(hso.angle_error_rad)) {
        fmt_unit(texts.hso_error, to_x10(hso.angle_error_rad * 57.2957795f), 10, "deg");
    } else strcpy(texts.hso_error,"--"); /* Do not convert the startup NaN to integer. */
    UiText_Format(texts.hso_flux, sizeof(texts.hso_flux), "%lu uWb", (unsigned long)(hso.reference_flux_wb * 1000000.0f));
    UiText_Format(texts.hso_time, sizeof(texts.hso_time), "%lu us / %lu", (unsigned long)(hso.max_control_cycles / 170U), (unsigned long)hso.overruns);
    strcpy(texts.align,state.aligning ? "ALIGNING" : state.aligned ? "YES" : "NO");
    fmt_counter(texts.raw,ui_encoder_raw());

    if (s_view_axis==MOTOR_AXIS_1 && MotorControl_SixStepAllowed()) {
        char *unavailable[]={texts.iq,texts.id,texts.elec,texts.mech,texts.elec_deg,texts.mech_deg};
        for (unsigned i=0;i<sizeof(unavailable)/sizeof(unavailable[0]);++i) strcpy(unavailable[i],"--");
    }

    fmt_counter(texts.err,ui_encoder_errors());
    MT6701Diagnostics enc;
    ui_encoder_diag(&enc);
    for (unsigned i=0;i<sizeof(s_enc_labels)/sizeof(s_enc_labels[0]);++i)
        s_enc_labels[i][1]=(char)('1'+s_view_axis);
    static const ValueRow encoder_values[]={
        VALUE_ROW(MT6701Diagnostics,crc_errors,enc_crc,VALUE_COUNT),
        VALUE_ROW(MT6701Diagnostics,field_errors,enc_field,VALUE_COUNT),
        VALUE_ROW(MT6701Diagnostics,track_errors,enc_track,VALUE_COUNT),
        VALUE_ROW(MT6701Diagnostics,spi_errors,enc_spi,VALUE_COUNT),
        VALUE_ROW(MT6701Diagnostics,frame_errors,enc_frame,VALUE_COUNT),
    };
    refresh_values(&enc,encoder_values,sizeof(encoder_values)/sizeof(encoder_values[0]));
    fmt_pair(texts.enc_age,enc.age_cycles,enc.max_age_cycles);
    strcpy(texts.enc_last,MT6701_ErrorText(enc.last_error));
    const char *zero=enc.zero_age>3U ? "GAP" :
        (enc.zero_before<=128U || enc.zero_before>=16384U-128U) ? "NEAR" : "FAR";
    if (enc.zero_count) {
        UiText_Format(texts.enc_zero,sizeof(texts.enc_zero),"%s %u",zero,enc.zero_count);
        if (enc.zero_age!=UINT16_MAX) fmt_counter(texts.enc_zpre,enc.zero_before);
        else strcpy(texts.enc_zpre,"--");
        if (enc.zero_after!=UINT16_MAX) fmt_counter(texts.enc_zpost,enc.zero_after);
        else strcpy(texts.enc_zpost,"--");
        fmt_counter(texts.enc_ztime,(enc.zero_after==UINT16_MAX ? HAL_GetTick() : enc.zero_ended_at)-enc.zero_started_at);
    } else {
        strcpy(texts.enc_zero,"--"); strcpy(texts.enc_zpre,"--"); strcpy(texts.enc_zpost,"--");
        strcpy(texts.enc_ztime,"--");
    }

    MotorSamplingDiagnostics adc;
    MotorSampling_GetAxisDiagnostics(s_view_axis,&adc);
    s_ibus_limit_ma=MotorSampling_GetIbusLimit();
    fmt_unit(texts.ibus_limit,s_ibus_limit_ma/100,10,"A");
    fmt_unit(texts.ibus_voltage,to_x100(adc.ibus_threshold_v),100,"V");
    static const ValueRow adc_values[]={
        VALUE_ROW(MotorSamplingDiagnostics,ibus_dac_code,ibus_code,VALUE_COUNT16),
        VALUE_ROW(MotorSamplingDiagnostics,ibus_trips,ibus_trips,VALUE_COUNT),
        VALUE_ROW(MotorSamplingDiagnostics,reference_updates,ref_count,VALUE_COUNT),
        VALUE_ROW(MotorSamplingDiagnostics,reference_errors,ref_errors,VALUE_COUNT),
        VALUE_ROW(MotorSamplingDiagnostics,bus_raw,bus_raw,VALUE_COUNT16),
        VALUE_ROW(MotorSamplingDiagnostics,bus_samples,bus_count,VALUE_COUNT),
        VALUE_ROW(MotorSamplingDiagnostics,frame_samples,frame_count,VALUE_COUNT),
    };
    refresh_values(&adc,adc_values,sizeof(adc_values)/sizeof(adc_values[0]));
    strcpy(texts.ibus_pin,adc.ibus_high ? "HIGH" : "LOW");
    strcpy(texts.ibus_state,adc.ibus_latched ? "IBUS: TRIPPED" : adc.ibus_ready ? "IBUS: READY" : "IBUS: NOT READY");
    unsigned reference_mv = (unsigned)(adc.reference_voltage * 1000.0f + 0.5f);
    if (adc.reference_updates != 0U)
        UiText_Format(texts.reference, sizeof(texts.reference), "%u.%03u V",
                 reference_mv / 1000U, reference_mv % 1000U);
    else
        strcpy(texts.reference,"NO DATA");
    strcpy(texts.ref_state,
             adc.reference_fresh ? "OK" : (adc.reference_updates ? "STALE" : "WAIT"));
    fmt_pair(texts.ref_raw,adc.reference_raw,adc.reference_cal);
    UiText_Format(texts.adc_revision, sizeof(texts.adc_revision), "0x%04X", (unsigned)adc.silicon_revision);
    for (unsigned i = 0U; i < 3U; ++i) {
        if (s_view_axis==MOTOR_AXIS_2 && i==2U) { strcpy(texts.current_adc[i],"derived"); continue; }
        fmt_pair(texts.current_adc[i],adc.current_raw[i],adc.current_zero[i]);
    }
    if (s_view_axis==MOTOR_AXIS_2 && adc.fault_phase==3U) {
        strcpy(texts.current_trip,"RUN W derived");
    } else if (adc.fault_phase != 0U) {
        unsigned i = adc.fault_phase - 1U;
        UiText_Format(texts.current_trip, sizeof(texts.current_trip), "%s %c %u",
                 adc.fault_during_zero ? "ZERO" : "RUN", "UVW"[i],
                 (unsigned)adc.fault_current_raw[i]);
    } else {
        strcpy(texts.current_trip,"--");
    }
    if (adc.bus_fresh) {
        fmt_unit(texts.bus, to_x10(adc.bus_voltage), 10, "V");
    } else {
        strcpy(texts.bus,"NO DATA");
    }
    UiText_Format(texts.adc1_error, sizeof(texts.adc1_error), "0x%lX", (unsigned long)adc.adc1_error);
    UiText_Format(texts.adc2_error, sizeof(texts.adc2_error), "0x%lX", (unsigned long)adc.adc2_error);
}

/* ---------- 页面描述 ---------- */
static const KK_UI_HomeItem home_items[] = {
    {"Run",      home_icons[2], UI_PAGE_RUN},
    {"Settings", home_icons[0], UI_PAGE_SETTINGS},
    {"Status",   home_icons[1], UI_PAGE_STATUS},
    {"About",    home_icons[3], UI_PAGE_ABOUT},
};

static const KK_UI_HomePage home_pages[] = {
    {home_items, sizeof(home_items) / sizeof(home_items[0])},
};

static const KK_UI_MenuItem settings_items[] = {
    {texts.system, KK_UI_MENU_PAGE, UI_PAGE_SYSTEM},
    {"Curr Loop",  KK_UI_MENU_BOOL,   BOOL_CURRENT_LOOP},
    {"Pos Speed",  KK_UI_MENU_INT,    INT_POSITION_SPEED},
    {"Speed Limit", KK_UI_MENU_INT,   INT_SPEED_LIMIT},
    {"Run Steps",  KK_UI_MENU_PAGE, UI_PAGE_RUN_STEPS},
    {"M1 Protection",KK_UI_MENU_PAGE,UI_PAGE_PROTECTION},
    {"Motor",      KK_UI_MENU_PAGE, UI_PAGE_MOTOR},
    {"Status LED", KK_UI_MENU_BOOL, BOOL_LED},
    {"LED Bright", KK_UI_MENU_INT, INT_LED_BRIGHT},
    {texts.save_state, KK_UI_MENU_ACTION, EVT_SAVE_STATUS},
    {"Restore Defaults", KK_UI_MENU_CONFIRM, CONFIRM_RESET},
};

static const KK_UI_MenuItem motor_items[] = {
    {"2804", KK_UI_MENU_CONFIRM, CONFIRM_MOTOR_2804},
    {"4015", KK_UI_MENU_CONFIRM, CONFIRM_MOTOR_4015},
    {"ESC Auto", KK_UI_MENU_CONFIRM, CONFIRM_MOTOR_ESC},
    {"M1 Detect", KK_UI_MENU_PAGE, UI_PAGE_DETECT},
};

static const KK_UI_MenuItem system_items[] = {
    {texts.work_menu, KK_UI_MENU_PAGE, UI_PAGE_WORK},
    {s_txt_axes[0], KK_UI_MENU_ACTION, EVT_FOCUS_M1},
    {s_txt_axes[1], KK_UI_MENU_ACTION, EVT_FOCUS_M2},
    {"Sensorless", KK_UI_MENU_PAGE, UI_PAGE_SENSORLESS},
    {"Gear Ratio", KK_UI_MENU_INT, INT_GEAR_RATIO},
    {"Gear Offset", KK_UI_MENU_INT, INT_GEAR_OFFSET},
    {"Align / Retry", KK_UI_MENU_ACTION, EVT_ALIGN},
};
static const KK_UI_MenuItem sensorless_items[] = {
    {"HSO Feedback", KK_UI_MENU_BOOL, BOOL_HSO},
    {"Six Step", KK_UI_MENU_BOOL, BOOL_SIXSTEP},
};
static const KK_UI_MenuItem work_items[] = {
    {s_txt_work_items[0], KK_UI_MENU_ACTION, EVT_WORK_BASE+MOTOR_WORK_SINGLE_M1},
    {s_txt_work_items[1], KK_UI_MENU_ACTION, EVT_WORK_BASE+MOTOR_WORK_SINGLE_M2},
    {s_txt_work_items[2], KK_UI_MENU_ACTION, EVT_WORK_BASE+MOTOR_WORK_INDEPENDENT},
    {s_txt_work_items[3], KK_UI_MENU_ACTION, EVT_WORK_BASE+MOTOR_WORK_SYNC_SPEED},
    {s_txt_work_items[4], KK_UI_MENU_ACTION, EVT_WORK_BASE+MOTOR_WORK_SYNC_POSITION},
    {s_txt_work_items[5], KK_UI_MENU_ACTION, EVT_WORK_BASE+MOTOR_WORK_MASTER_M1},
    {s_txt_work_items[6], KK_UI_MENU_ACTION, EVT_WORK_BASE+MOTOR_WORK_MASTER_M2},
};

static const KK_UI_MenuItem status_items[] = {
    {"Drive",KK_UI_MENU_PAGE,UI_PAGE_DRIVE},
    {"Motion",KK_UI_MENU_PAGE,UI_PAGE_MOTION},
    {"Encoder",KK_UI_MENU_PAGE,UI_PAGE_ENCODER},
    {"ADC",KK_UI_MENU_PAGE,UI_PAGE_ADC},
    {"HSO",KK_UI_MENU_PAGE,UI_PAGE_HSO},
    {"Six Step",KK_UI_MENU_PAGE,UI_PAGE_SIXSTEP},
    {"System",KK_UI_MENU_PAGE,UI_PAGE_DIAGNOSTICS},
    {s_txt_axes[0],KK_UI_MENU_ACTION,EVT_FOCUS_M1},
    {s_txt_axes[1],KK_UI_MENU_ACTION,EVT_FOCUS_M2},
};
static const KK_UI_MenuItem protection_items[] = {
    {"IBUS Trip",KK_UI_MENU_INT,INT_IBUS_LIMIT},
    {"Clear IBUS Trip",KK_UI_MENU_CONFIRM,CONFIRM_IBUS_CLEAR},
    {texts.ibus_state,KK_UI_MENU_ACTION,EVT_IBUS_STATUS},
};

static const KK_UI_MenuPage menu_pages[] = {
    {texts.settings, settings_items, settings_states,
     sizeof(settings_items) / sizeof(settings_items[0])},
    {texts.motor_menu, motor_items, motor_states,
     sizeof(motor_items) / sizeof(motor_items[0])},
    {"System", system_items, NULL, sizeof(system_items)/sizeof(system_items[0])},
    {"Work Mode", work_items, NULL, sizeof(work_items)/sizeof(work_items[0])},
    {texts.status,status_items,NULL,sizeof(status_items)/sizeof(status_items[0])},
    {"M1 Protection",protection_items,protection_states,sizeof(protection_items)/sizeof(protection_items[0])},
    {"M1 Sensorless",sensorless_items,sensorless_states,sizeof(sensorless_items)/sizeof(sensorless_items[0])},
};

static const KK_UI_InfoRow drive_rows[] = {
    {"Drive",   texts.drive},
    {"Output",  texts.output},
    {"Feedback",texts.feedback},
    {"Bus",     texts.bus},
    {"Motor",   texts.motor},
    {"Mode",    texts.mode},
    {"VREF",    texts.reference},
    {"RefState",texts.ref_state},
    {"Work",texts.work},
    {"Axis",texts.axis},
    {"Aligned", texts.align},
    {"Run Stop",texts.run_stop},
    {"Stop Age",texts.stop_age},
    {"Stop Err",texts.stop_errors},
    {"Sync Err",texts.sync},
};
static const KK_UI_InfoRow sixstep_rows[] = {
    {"6S ZC",texts.six_quality}, {"6Step Duty",texts.six_duty},
    {"6Step EMF",texts.six_bemf}, {"6Step Speed",texts.six_speed},
    {"6S Hold",texts.six_start}, {"6S Start EMF",texts.six_start_emf},
    {"6S Cause",texts.six_reject}, {"6S Sample",texts.six_sample},
};

static const KK_UI_InfoRow motion_rows[] = {
    {"Input",texts.input},
    {"Target",  texts.target},
    {texts.target_unit,texts.target_aux},
    {"Speed",   texts.speed},
    {"Speed rpm",texts.rpm},
    {"Iq",      texts.iq},
    {"Id",      texts.id},
    {"Phase A", texts.ia},
    {"Phase B", texts.ib},
    {"Elec",    texts.elec},
    {"Mech",    texts.mech},
    {"Elec 0-360",texts.elec_deg}, {"Mech 0-360",texts.mech_deg},
};

static const KK_UI_InfoRow encoder_rows[] = {
    {s_enc_labels[0], texts.enc},
    {s_enc_labels[1], texts.enc_crc},
    {s_enc_labels[2], texts.enc_field},
    {s_enc_labels[3], texts.enc_track},
    {s_enc_labels[4], texts.enc_spi},
    {s_enc_labels[5], texts.enc_frame},
    {s_enc_labels[6], texts.enc_age},
    {s_enc_labels[7], texts.enc_last},
    {s_enc_labels[8], texts.raw},
    {s_enc_labels[9], texts.err},
    {s_enc_labels[10], texts.enc_zero},
    {s_enc_labels[11], texts.enc_zpre},
    {s_enc_labels[12], texts.enc_zpost},
    {s_enc_labels[13], texts.enc_ztime},
    {"E2 Probe",texts.enc2},
    {"E2 Total",texts.enc2_err},
};

static const KK_UI_InfoRow adc_rows[] = {
    {"M1 IBUS",texts.ibus_state},
    {"IBUS Limit",texts.ibus_limit}, {"IBUS DAC",texts.ibus_code},
    {"IBUS Thresh",texts.ibus_voltage}, {"IBUS Trips",texts.ibus_trips}, {"COMP4",texts.ibus_pin},
    {"I Trip",  texts.current_trip},
    {"IU R/Z",  texts.current_adc[0]},
    {"IV R/Z",  texts.current_adc[1]},
    {"IW R/Z",  texts.current_adc[2]},
    {"ChipRev", texts.adc_revision},
    {"BusRaw",  texts.bus_raw},
    {"BusCnt",  texts.bus_count},
    {"FrameCnt",texts.frame_count},
    {"ADC1Err", texts.adc1_error},
    {"ADC2Err", texts.adc2_error},
    {"Ref R/C", texts.ref_raw},
    {"RefCnt",  texts.ref_count},
    {"RefErr",  texts.ref_errors},
};

static const KK_UI_InfoRow hso_rows[] = {
    {"HSO",     texts.hso},
    {"H Min",texts.hso_min}, {"H Max",texts.hso_max},
    {"H Zero","Stop / coast"}, {"H Reverse","Stop first"},
    {"H Gate",  texts.hso_gate},
    {"H Qual",  texts.hso_qual},
    {"H Reject",texts.hso_reject},
    {"H Reset",texts.hso_resets},
    {"H R Deg",texts.hso_rangle},
    {"H R Hz",texts.hso_rfreq},
    {"H R EMF%",texts.hso_remf},
    {"H R Drift",texts.hso_rdrift},
    {"H Off mV",texts.hso_offset},
    {"H Lim mV",texts.hso_olimit},
    {"H Lost",  texts.hso_lost},
    {"H Speed", texts.hso_speed},
    {"H Error", texts.hso_error},
    {"H Flux",  texts.hso_flux},
    {"H Actual",texts.hso_flux_actual},
    {"H Time",  texts.hso_time},
};

static const KK_UI_InfoRow diagnostics_rows[] = {
    {"Cmd",texts.command},
    {"M1 Model",texts.m1},
    {"M2 Model",texts.m2},
    {"Ctrl Max",texts.control_time},
    {"Save Bus",texts.save},
    {"Save Last",texts.save_result},
    {"Flash Err",texts.flash_error},
    {"SPI2 Fault",texts.spi2_fault},
    {"SPI2 Reset",texts.spi2_resets},
    {"KeyRpt",  texts.repeat},
    {"DispErr", texts.display_errors},
};

static const KK_UI_InfoRow about_rows[] = {
    {"FW",      "UI-R81-CF"},
    {"MCU",     "STM32G431"},
    {"Motor",   texts.motor},
    {"Control", "10kHz FOC"},
    {"Encoder", "MT6701"},
    {"LCD",     "ST7735S"},
    {"UI",      "KK_UI"},
    {"Keys",    "6 keys"},
    {"Flash",   "W25Q64"},
};

static const KK_UI_InfoPage info_pages[] = {
    {texts.groups[0], drive_rows, sizeof(drive_rows) / sizeof(drive_rows[0])},
    {"About",  about_rows,  sizeof(about_rows) / sizeof(about_rows[0])},
    {texts.groups[1],motion_rows,sizeof(motion_rows)/sizeof(motion_rows[0])},
    {texts.groups[2],encoder_rows,sizeof(encoder_rows)/sizeof(encoder_rows[0])},
    {texts.groups[3],adc_rows,sizeof(adc_rows)/sizeof(adc_rows[0])},
    {texts.groups[4],hso_rows,sizeof(hso_rows)/sizeof(hso_rows[0])},
    {"System",diagnostics_rows,sizeof(diagnostics_rows)/sizeof(diagnostics_rows[0])},
    {"M1 Six Step",sixstep_rows,sizeof(sixstep_rows)/sizeof(sixstep_rows[0])},
};

/* 下标 = 页面 ID - 1 */
static const KK_UI_PageRoute routes[] = {
    {KK_UI_PAGE_HOME,   0U},
    {KK_UI_PAGE_CUSTOM, 0U},
    {KK_UI_PAGE_MENU,   0U},
    {KK_UI_PAGE_MENU,   4U},
    {KK_UI_PAGE_INFO,   1U},
    {KK_UI_PAGE_CUSTOM, 1U},
    {KK_UI_PAGE_MENU,   1U},
    {KK_UI_PAGE_MENU,   2U},
    {KK_UI_PAGE_MENU,   3U},
    {KK_UI_PAGE_INFO,   0U},
    {KK_UI_PAGE_INFO,   2U},
    {KK_UI_PAGE_INFO,   3U},
    {KK_UI_PAGE_INFO,   4U},
    {KK_UI_PAGE_INFO,   5U},
    {KK_UI_PAGE_INFO,   6U},
    {KK_UI_PAGE_MENU,   5U},
    {KK_UI_PAGE_MENU,   6U},
    {KK_UI_PAGE_INFO,   7U},
    {KK_UI_PAGE_CUSTOM, 2U},
};

static KK_UI_IntBinding int_bindings[] = {
    [INT_IBUS_LIMIT] = {"M1 IBUS Trip",&s_ibus_limit_ma,APP_IBUS_MIN_MA,APP_IBUS_MAX_MA,100U,"mA",EVT_IBUS_LIMIT},
    [INT_GEAR_RATIO] = {"Gear Ratio", &s_ratio_x100,-1000,1000,1U,"%",EVT_GEAR},
    [INT_GEAR_OFFSET] = {"Gear Offset", &s_offset_x100,-6283,6283,5U,"0.01rad",EVT_GEAR},
    [INT_LED_BRIGHT] = {"LED Brightness", &s_led_brightness, 0, 100, 5U, "%", EVT_LED_CHANGED},
    [INT_POSITION_SPEED] = {"Position Speed", &s_position_speed, 1, APP_SPEED_LIMIT_DEFAULT, 1U, "rad/s", EVT_POSITION_SPEED_CHANGED},
    [INT_SPEED_LIMIT] = {"Speed Limit", &s_speed_limit, 1, APP_SPEED_LIMIT_MAX, 1U, "rad/s", EVT_SPEED_LIMIT_CHANGED},
};

static KK_UI_BoolBinding bool_bindings[] = {
    [BOOL_HSO] = {"M1 HSO Feedback", &s_hso_choice, EVT_FEEDBACK},
    [BOOL_SIXSTEP] = {"M1 6 Step", &s_six_choice, EVT_SIXSTEP},
    [BOOL_CURRENT_LOOP] = {texts.loop_title, &s_current_loop_display, EVT_CURRENT_LOOP_CHANGED},
    [BOOL_LED]          = {"Status LED",   &s_led_enable,   EVT_LED_CHANGED},
};

static const KK_UI_ConfirmDesc confirm_descs[] = {
    [CONFIRM_IBUS_CLEAR] = {"Clear M1 IBUS?\nOutput stays OFF",EVT_IBUS_CLEAR,0U},
    [CONFIRM_RESET] = {"Reset all\nsettings?", EVT_RESET_CONFIRMED, EVT_RESET_CANCELLED},
    [CONFIRM_MOTOR_2804] = {"Use 2804?\nStop and align", EVT_MOTOR_2804_CONFIRMED, 0U},
    [CONFIRM_MOTOR_4015] = {"Use 4015?\nStop and align", EVT_MOTOR_4015_CONFIRMED, 0U},
    [CONFIRM_MOTOR_ESC] = {"ESC Auto?\nThrottle only", EVT_MOTOR_ESC, 0U},
};

static const KK_UI_App s_app = {
    .root_page = UI_PAGE_HOME,
    .routes = routes,
    .route_count = sizeof(routes) / sizeof(routes[0]),
    .home_pages = home_pages,
    .home_page_count = sizeof(home_pages) / sizeof(home_pages[0]),
    .menu_pages = menu_pages,
    .menu_page_count = sizeof(menu_pages) / sizeof(menu_pages[0]),
    .info_pages = info_pages,
    .info_page_count = sizeof(info_pages) / sizeof(info_pages[0]),
    .custom_page_count = 3U,
    .int_bindings = int_bindings,
    .int_binding_count = sizeof(int_bindings) / sizeof(int_bindings[0]),
    .bool_bindings = bool_bindings,
    .bool_binding_count = sizeof(bool_bindings) / sizeof(bool_bindings[0]),
    .confirm_descs = confirm_descs,
    .confirm_desc_count = sizeof(confirm_descs) / sizeof(confirm_descs[0]),
    .fonts = {ui_font_title, ui_font_title, ui_font_body},
    .texts = {"Back", "Cancel", "OK", "ON", "OFF", "Notice"},
};

/* ---------- 设置与 FOC 绑定 ---------- */
static void clear_run_stop(void)
{
    strcpy(texts.run_stop, "--");
    strcpy(texts.stop_age, "--");
    strcpy(texts.stop_errors, "--");
}

/* 模式和目标一起提交，防止 1 kHz 外环在两次写入之间使用旧目标。 */
static void stop_run(void)
{
    s_run_lr_key=0U;
    /* Entering/backing out of an idle Run page must not interrupt the
     * independent startup alignment. An explicit serial Stop still can. */
    if (!s_run_active && ui_state().aligning) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* Entering/editing a view cannot cancel a USB/UART-owned run. Explicit
     * Back and KEY1 still use the unconditional global stop path. */
    if (MotorControl_GetInput(s_view_axis)<=MOTOR_INPUT_LOCAL) {
        MotorControl_Stop(s_view_axis);
        if (s_view_axis==MOTOR_AXIS_1) JerryFOC_useCurrentLoop(0U);
    }
    s_run_active = false;
    __set_PRIMASK(primask);
}

static int32_t *run_target(bool draft);

static bool apply_run_target(void)
{
    if (!s_run_active || !ui_store_current()) return false;
    float target=(float)*run_target(true)/(float)run_scale(throttle_mode(s_run_mode));
    if (s_run_mode==JERRYFOC_MODE_POSITION && !MotorControl_IsCoupled()) target+=s_run_position_origin;
    return MotorControl_Command(s_view_axis,s_run_mode,target,(float)s_position_speed,s_current_loop);
}

static void mark_settings_dirty(uint32_t now_ms)
{
    s_settings_dirty = true;
    s_settings_save_at = now_ms;
    refresh_save_state();
}

/* 草稿与已确认值分开：按住调参只修改草稿，OK 才提交本次目标。 */
static int32_t *run_target(bool draft)
{
    if (throttle_mode(s_run_mode)) return draft ? &s_run_throttle_pct : &s_throttle_pct;
    switch (s_run_mode) {
    case JERRYFOC_MODE_VELOCITY:
        return draft ? &s_run_speed_x10 : &s_speed_x10;
    case JERRYFOC_MODE_POSITION:
        return draft ? &s_run_position_x100 : &s_position_x100;
    default:
        if (s_current_loop)
            return draft ? &s_run_current_x100 : &s_confirmed_current_x100;
        return draft ? &s_run_voltage_x100 : &s_voltage_x100;
    }
}

static bool run_target_changed(void)
{
    return *run_target(true) != *run_target(false);
}

static void update_position_speed_limit(void)
{
    int_bindings[INT_POSITION_SPEED].maximum=s_speed_limit;
    s_position_speed=clamp_i32(s_position_speed,1,s_speed_limit);
}

static uint32_t settings_flags(void)
{
    return (s_current_loop ? APP_FLAG_CURRENT_LOOP : 0U) |
           (s_led_enable ? APP_FLAG_LED_ENABLE : 0U) |
           AppSettings_EncodePositionSpeed((uint32_t)s_position_speed) |
           ((uint32_t)s_speed_limit << APP_SPEED_LIMIT_SHIFT) |
           (MotorParams_GetAxisProfile(s_view_axis) == MOTOR_PROFILE_4015 ? APP_FLAG_MOTOR_4015 : 0U);
}

/* Three targets and three steps retain their v5 wire layout. Byte copies
 * avoid aliasing AppSettings as a different structure or walking struct members. */
#define UI_TRIPLE(type,a,b,c) _Static_assert(offsetof(type,b)==offsetof(type,a)+4 && \
    offsetof(type,c)==offsetof(type,a)+8,"Contiguous UI/storage triple")
UI_TRIPLE(AxisView,speed_x10,voltage_x100,position_x100);
UI_TRIPLE(AppSettings,speed_x10,voltage_x100,position_x100);
UI_TRIPLE(AppSettings,axis2_speed_x10,axis2_voltage_x100,axis2_position_x100);
UI_TRIPLE(AppSettings,speed_step_x10,torque_step_x100,position_step_x100);
UI_TRIPLE(AppSettings,axis2_speed_step_x10,axis2_torque_step_x100,axis2_position_step_x100);
#undef UI_TRIPLE
_Static_assert(sizeof(AppSettings)<256 && offsetof(AxisView,speed_x10)==0,"Compact UI/storage offsets");
static char *axis_data(AppSettings *settings,bool steps)
{
    static const uint8_t offsets[2][2]={
        {offsetof(AppSettings,speed_x10),offsetof(AppSettings,speed_step_x10)},
        {offsetof(AppSettings,axis2_speed_x10),offsetof(AppSettings,axis2_speed_step_x10)}};
    return (char *)settings+offsets[s_view_axis][steps];
}
static bool ui_store_current(void)
{
    AppSettings *st=AppStorage_Get();
    uint32_t flags=settings_flags();
    if (!MotorControl_SetOptions(s_view_axis,flags)) {
        sync_from_storage(); s_current_loop_display=s_current_loop;
        s_pending_toast=MotorControl_CommandText(); return false;
    }
    st->control_config=(st->control_config & ~APP_CONFIG_THROTTLE_MASK) |
        ((uint32_t)s_throttle_pct << APP_CONFIG_THROTTLE_SHIFT);
    if (s_view_axis==MOTOR_AXIS_1) st->flags=flags; else st->axis2_flags=flags;
    memcpy(axis_data(st,false),&current_view,12);
    int32_t steps[]={s_run_steps[1],s_run_steps[0],s_run_steps[2]};
    memcpy(axis_data(st,true),steps,sizeof(steps));
    st->flags=(st->flags & ~(APP_FLAG_LED_ENABLE|APP_FLAG_MOTOR_4015)) |
        (s_led_enable ? APP_FLAG_LED_ENABLE : 0U) |
        (MotorParams_GetAxisProfile(MOTOR_AXIS_1)==MOTOR_PROFILE_4015 ? APP_FLAG_MOTOR_4015 : 0U);
    st->led_brightness=(uint32_t)s_led_brightness;
    return true;
}

static bool request_settings_save(void)
{
    AppSettings *st = AppStorage_Get();
    if (!ui_store_current()) return false;
    st->speed_x10 = s_view_axis==MOTOR_AXIS_1 ? s_speed_x10 : st->speed_x10;
    if (!AppStorage_IsReady()) {
        s_pending_toast = AppStorage_SaveErrorText();
        return true;
    }
    return AppStorage_RequestSave();
}

static void sync_from_storage(void)
{
    AppSettings *st=AppStorage_Get();
    uint32_t flags=MotorControl_GetOptions(s_view_axis);
    s_throttle_pct=s_run_throttle_pct=clamp_i32((st->control_config & APP_CONFIG_THROTTLE_MASK)>>APP_CONFIG_THROTTLE_SHIFT,0,100);
    memcpy(&current_view,axis_data(st,false),12);

    /* 防止 W25Q 中的旧记录/损坏记录把绑定值带到编辑器范围之外 */
    s_speed_limit = (int32_t)((flags & APP_SPEED_LIMIT_MASK) >> APP_SPEED_LIMIT_SHIFT);
    if (s_speed_limit < 1 || s_speed_limit > APP_SPEED_LIMIT_MAX)
        s_speed_limit = APP_SPEED_LIMIT_DEFAULT;
    int32_t profile_max = (int32_t)MotorParams_GetAxis(s_view_axis)->max_speed_mech;
    int_bindings[INT_SPEED_LIMIT].maximum = profile_max;
    s_speed_limit = clamp_i32(s_speed_limit, 1, profile_max);
    s_speed_x10 = clamp_i32(s_speed_x10, -s_speed_limit * 10, s_speed_limit * 10);
    int32_t voltage_max = to_x100(MotorParams_GetAxis(s_view_axis)->vel_limit);
    s_voltage_x100 = clamp_i32(s_voltage_x100, -voltage_max, voltage_max);
    s_position_x100 = clamp_i32(s_position_x100, -314, 314);
    s_led_brightness = clamp_i32((int32_t)st->led_brightness, 0, 100);
    s_current_loop = (flags & APP_FLAG_CURRENT_LOOP) != 0U;
    s_led_enable = (st->flags & APP_FLAG_LED_ENABLE) != 0U;
    s_position_speed = (int32_t)AppSettings_GetPositionSpeed(flags);
    if (s_position_speed < 1 || s_position_speed > APP_SPEED_LIMIT_MAX) s_position_speed = 30;
    update_position_speed_limit();
    int32_t steps[3]; memcpy(steps,axis_data(st,true),sizeof(steps));
    s_run_steps[JERRYFOC_MODE_VELOCITY] = steps[0];
    s_run_steps[JERRYFOC_MODE_TORQUE] = steps[1];
    s_run_steps[JERRYFOC_MODE_POSITION] = steps[2];
    for (unsigned i = 0U; i < 3U; ++i)
        if (s_run_steps[i] < 1 || s_run_steps[i] > 100) s_run_steps[i] = 5;
}

/* Boot and Defaults publish the same accepted axis options. Saved targets
 * seed stopped drafts only, never an automatic output request. */
__attribute__((noinline,noclone)) static void reload_axis_views(void)
{
    for (unsigned a=0;a<2;++a) {
        s_view_axis=(MotorAxis)a;
        s_profiles[a]=MotorParams_GetAxisProfile((MotorAxis)a);
        AppSettings *saved=AppStorage_Get();
        (void)MotorControl_SetOptions((MotorAxis)a,a==0 ? saved->flags : saved->axis2_flags);
        sync_from_storage();
        s_run_mode=JERRYFOC_MODE_TORQUE; s_run_active=false;
        s_run_speed_x10=s_speed_x10; s_run_voltage_x100=s_voltage_x100;
        s_run_position_x100=s_position_x100;
        s_run_current_x100=s_confirmed_current_x100=0; s_run_position_origin=0;
        save_view();
    }
}

static void focus_axis(MotorAxis axis)
{
    s_run_lr_key=0U;
    bool changed=s_view_axis!=axis;
    ui_store_current(); save_view();
    MotorControl_SetFocus(axis); load_view(axis);
    if (changed && MotorControl_GetWork()==MOTOR_WORK_INDEPENDENT) mark_settings_dirty(HAL_GetTick());
    int_bindings[INT_SPEED_LIMIT].maximum=(int32_t)MotorParams_GetAxis(axis)->max_speed_mech;
    clear_run_stop(); refresh_texts(); KK_UI_Invalidate();
}

__attribute__((noinline)) static void reset_motor_view(void)
{
    /* Never carry a 2804 target/current-mode command into the other motor. */
    s_speed_x10 = s_run_speed_x10 = 0;
    if (s_view_axis==MOTOR_AXIS_1) s_throttle_pct=s_run_throttle_pct=0;
    s_voltage_x100 = s_run_voltage_x100 = 0;
    s_position_x100 = s_run_position_x100 = 0;
    s_run_current_x100 = s_confirmed_current_x100 = 0;
    s_run_position_origin = 0.0f;
    s_run_mode = JERRYFOC_MODE_TORQUE;
    s_run_active=false;
    s_current_loop = false;
    s_speed_limit = APP_SPEED_LIMIT_DEFAULT;
    int_bindings[INT_SPEED_LIMIT].maximum = (int32_t)MotorParams_GetAxis(s_view_axis)->max_speed_mech;
    s_position_speed = (int32_t)MotorParams_GetAxis(s_view_axis)->pos_limit;
    update_position_speed_limit();
}

static bool select_motor(MotorProfile profile)
{
    if (profile == MotorParams_GetAxisProfile(s_view_axis) && !esc_view()) return true;
    /* An idle sensorless Stop clears its 200ms electrical-rest counter.
     * Preserve that qualification before SelectProfile checks it; running
     * outputs still coast first and must qualify again before a retry. */
    bool independent_feedback=sensorless();
    if (!independent_feedback || ui_state().active) stop_run();
    bool ok=profile==MOTOR_PROFILE_COUNT ? s_view_axis==MOTOR_AXIS_1 && MotorControl_SelectGeneric() :
        MotorControl_SelectProfile(s_view_axis,profile);
    if (!ok) {
        s_pending_toast = independent_feedback ? "Wait for stop" : "Stop/check encoder";
        return false;
    }
    reset_motor_view(); clear_run_stop();
    ui_store_current();
    mark_settings_dirty(HAL_GetTick());
    refresh_texts();
    s_pending_toast = independent_feedback ? "Motor set" : "Motor set; align";
    KK_UI_Invalidate();
    return true;
}

static void run_mode_step(int delta)
{
    if(esc_view()) return; /* A throttle-only ESC has no alternate mode. */
    stop_run();
    int mode = (int)s_run_mode + delta;
    if (mode < 0) mode = 2;
    if (mode > 2) mode = 0;
    s_run_mode = (JerryFOC_ControlMode)mode;
    for (unsigned i=0;i<3 && !MotorControl_ModeAllowed(s_run_mode);++i)
        s_run_mode=(JerryFOC_ControlMode)(((unsigned)s_run_mode+1U)%3U);
    /* 各模式保留自己的草稿，切换后必须再次 OK 才能启动。 */
    KK_UI_Invalidate();
}

static void service_run_lr(uint32_t now)
{
    if (!s_run_lr_key) return;
    uint8_t raw=ui_input_raw_keys();
    if (KK_UI_GetCurrentPage()!=UI_PAGE_RUN || MotorControl_GetWork()!=MOTOR_WORK_INDEPENDENT ||
        (raw && raw!=s_run_lr_key)) {
        s_run_lr_key=0U; return; /* Back/OK, chords or a context change cancel it. */
    }
    bool held;
    if (raw) {
        s_run_lr_releasing=false;
        if (now-s_run_lr_pressed_at<600U) return;
        held=true;
    } else {
        if (!s_run_lr_releasing) { s_run_lr_releasing=true; s_run_lr_released_at=now; }
        if (now-s_run_lr_released_at<20U) return;
        /* If a foreground stall crossed the deadline, release still cannot
         * turn a long press into a short mode-change/stop operation. */
        held=s_run_lr_released_at-s_run_lr_pressed_at>=600U;
    }
    uint8_t key=s_run_lr_key;
    s_run_lr_key=0U;
    s_last_input_at=now;
    if (held) focus_axis(key==KK_UI_KEY_LEFT ? MOTOR_AXIS_1 : MOTOR_AXIS_2);
    else run_mode_step(key==KK_UI_KEY_LEFT ? -1 : 1);
}

/* ---------- 公共接口 ---------- */
const KK_UI_App *ui_app_get(void)
{
    return &s_app;
}

void ui_app_reset(uint32_t now_ms)
{
    s_global_stop_seen=false;
    clear_run_stop();
    AppStorage_Load();
    reload_axis_views();
    load_view(MotorControl_GetFocus());
    sync_from_storage();
    s_ratio_x100=AppStorage_Get()->gear_ratio_x100;
    if (!s_ratio_x100) s_ratio_x100=100;
    s_offset_x100=AppStorage_Get()->gear_offset_x100;

    if (s_view_axis==MOTOR_AXIS_1) JerryFOC_useCurrentLoop(s_current_loop ? 1U : 0U);

    s_run_mode = ui_state().mode;
    s_run_speed_x10 = s_speed_x10;
    if (sensorless()) s_run_speed_x10=hso_draft(s_run_speed_x10);
    s_run_voltage_x100 = s_voltage_x100;
    s_run_position_x100 = s_position_x100;

    s_run_active = false;
    s_run_lr_key=0U;
    s_input_repeats = s_display_errors = 0U;
    s_run_current_x100 = s_confirmed_current_x100 = 0;
    s_run_position_origin = ui_state().position;
    s_settings_dirty = false;
    s_settings_save_at = now_ms;
    s_info_at = now_ms;
    s_last_input_at = now_ms;
    s_last_live_at = now_ms;
    s_pending_toast = AppStorage_IsReady() ? NULL : "W25Q64 not found";

    refresh_texts();
}

void ui_app_note_display_error(void)
{
    ++s_display_errors;
}

void ui_app_update(uint32_t now_ms)
{
    static uint8_t s_last_healthy = 0xFFU;
    bool global_stop=MotorSampling_IsGlobalStopped();
    if (global_stop) {
        bool notify=!s_global_stop_seen || s_run_active || views[0].run_active || views[1].run_active;
        s_run_active=views[0].run_active=views[1].run_active=false;
        if (notify) {
            s_run_lr_key=0U;
            strcpy(texts.run_stop,"GLOBAL STOP");
            strcpy(texts.stop_age,"--"); strcpy(texts.stop_errors,"--");
            s_pending_toast="GLOBAL STOP";
            refresh_texts(); KK_UI_Invalidate();
        }
    }
    s_global_stop_seen=global_stop;
    if (MotorControl_GetFocus()!=s_view_axis) focus_axis(MotorControl_GetFocus());
    service_run_lr(now_ms);
    if (MotorControl_IsHandFollow() && KK_UI_GetCurrentPage()==UI_PAGE_RUN && MotorControl_IsPassive(s_view_axis))
        focus_axis((MotorAxis)(1U-s_view_axis));
    for (unsigned a=0;a<2;++a) {
        if (s_profiles[a]==MotorParams_GetAxisProfile((MotorAxis)a)) continue;
        MotorAxis old=s_view_axis; save_view(); load_view((MotorAxis)a);
        s_profiles[a]=MotorParams_GetAxisProfile((MotorAxis)a);
        reset_motor_view();
        ui_store_current(); save_view(); load_view(old);
        int_bindings[INT_SPEED_LIMIT].maximum=(int32_t)MotorParams_GetAxis(old)->max_speed_mech;
        mark_settings_dirty(now_ms); KK_UI_Invalidate();
    }
    if (!MotorControl_ModeAllowed(s_run_mode))
        s_run_mode=fallback_mode();
    KK_UI_ItemState hso_state=MotorControl_GetWork()==MOTOR_WORK_SINGLE_M1 ? KK_UI_ITEM_NORMAL : KK_UI_ITEM_DISABLED;
    KK_UI_ItemState esc_state=s_view_axis==MOTOR_AXIS_1 && MotorControl_SixStepAllowed() ? KK_UI_ITEM_NORMAL : KK_UI_ITEM_DISABLED;
    if(((motor_states[0]>>4U)&3U)!=(unsigned)esc_state)
        (void)KK_UI_SetMenuItemState(UI_PAGE_MOTOR,2U,esc_state);
    KK_UI_ItemState detect_state=hso_state==KK_UI_ITEM_NORMAL && s_view_axis==MOTOR_AXIS_1 && !sensorless() ? KK_UI_ITEM_NORMAL : KK_UI_ITEM_DISABLED;
    if(((motor_states[0]>>6U)&3U)!=(unsigned)detect_state)
        (void)KK_UI_SetMenuItemState(UI_PAGE_MOTOR,3U,detect_state);
    if ((sensorless_states[0] & 3U)!=(unsigned)hso_state)
        for(unsigned i=0;i<2;++i)
            (void)KK_UI_SetMenuItemState(UI_PAGE_SENSORLESS,(uint16_t)i,hso_state);
    KK_UI_ItemState loop_state=s_view_axis==MOTOR_AXIS_1 && sensorless() ? KK_UI_ITEM_DISABLED : KK_UI_ITEM_NORMAL;
    if (((settings_states[0]>>2U) & 3U)!=(unsigned)loop_state)
        (void)KK_UI_SetMenuItemState(UI_PAGE_SETTINGS,1U,loop_state);
    MotorSamplingDiagnostics ibus;
    MotorSampling_GetAxisDiagnostics(MOTOR_AXIS_1,&ibus);
    KK_UI_ItemState ibus_states[]={MotorSampling_IsAxisOutputRequested(MOTOR_AXIS_1) ? KK_UI_ITEM_DISABLED : KK_UI_ITEM_NORMAL,
        ibus.ibus_latched ? KK_UI_ITEM_NORMAL : KK_UI_ITEM_DISABLED};
    for (unsigned i=0;i<2;++i)
        if (((protection_states[0]>>(2U*i)) & 3U)!=(unsigned)ibus_states[i])
            (void)KK_UI_SetMenuItemState(UI_PAGE_PROTECTION,(uint16_t)i,ibus_states[i]);
    bool save_ok;
    if (AppStorage_PollSaveResult(&save_ok))
        s_pending_toast = save_ok ? "Saved" : AppStorage_SaveErrorText();

    uint8_t healthy = ui_state().healthy ? 1U : 0U;
    if (s_last_healthy != 0xFFU && s_last_healthy != 0U && healthy == 0U) {
        AppStorage_Log(APP_LOG_ENCODER_ERROR, ui_encoder_errors());
    }
    s_last_healthy = healthy;

    if (s_pending_toast != NULL) {
        if (KK_UI_ShowToast(s_pending_toast, 1200U) != KK_UI_BUSY) {
            s_pending_toast = NULL;
        }
    }

    if (s_run_active) {
        /* Capture one coherent gate before stop_run() changes mode/targets.
         * A recovered encoder must not erase why the UI stopped the motor. */
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        const MotorAxisSnapshot state=ui_state();
        const char *reason = NULL;
        if (!state.ready) reason = drive_status_text();
        else if (!MotorSampling_IsAxisOutputRequested(s_view_axis) && strcmp(state.status,"READY")) reason=state.status;
        else if (!state.healthy) reason = MotorControl_EncoderLostText(s_view_axis);
        else if (!state.aligned) reason = "NOT ALIGNED";
        else if (s_view_axis==MOTOR_AXIS_1 && g_openloop_debug) reason = "OPEN LOOP";
        else if (state.mode != s_run_mode) reason = "MODE CHANGED";
        else if (!MotorSampling_IsAxisOutputRequested(s_view_axis))
            reason=strcmp(state.status,"READY")==0 ? "OUTPUT OFF" : state.status;
        MT6701Diagnostics enc;
        MotorAxis stop_axis=s_view_axis;
        if (reason) {
            if (!strcmp(reason,MotorControl_EncoderLostText(MOTOR_AXIS_1))) stop_axis=MOTOR_AXIS_1;
            else if (!strcmp(reason,MotorControl_EncoderLostText(MOTOR_AXIS_2))) stop_axis=MOTOR_AXIS_2;
            if (stop_axis==MOTOR_AXIS_1) MT6701_GetDiagnostics(&enc);
            else MT6701_2_GetDiagnostics(&enc);
        }
        __set_PRIMASK(mask);
        if (reason) {
            UiText_Format(texts.run_stop, sizeof(texts.run_stop), "%s", reason);
            UiText_Format(texts.stop_age, sizeof(texts.stop_age), "E%u %u", (unsigned)stop_axis+1U, enc.age_cycles);
            UiText_Format(texts.stop_errors, sizeof(texts.stop_errors), "E%u %lu", (unsigned)stop_axis+1U, (unsigned long)enc.errors);
            stop_run();
            s_pending_toast = reason;
            refresh_texts();
            KK_UI_Invalidate();
        }
    }

    if ((now_ms - s_info_at) >= 500U) {
        s_info_at = now_ms;
        refresh_texts();
        KK_UI_Invalidate();
    }

    /* 连按/长按时攒一段时间再写 W25Q64，减少擦写次数 */
    if (s_settings_dirty && !s_run_active &&
        ((now_ms - s_settings_save_at) >= 2000U) &&
        ((now_ms - s_last_input_at) >= 2000U) && KK_UI_IsDisplayIdle()) {
        if (request_settings_save()) { s_settings_dirty = false; refresh_save_state(); KK_UI_Invalidate(); }
    }
}

void ui_app_process_events(void)
{
    KK_UI_EventId evt;

    while (KK_UI_PollEvent(&evt)) {
        if (evt>=EVT_WORK_BASE && evt<EVT_WORK_BASE+MOTOR_WORK_COUNT) {
            if (MotorControl_SelectWork((MotorWorkMode)(evt-EVT_WORK_BASE))) {
                focus_axis(MotorControl_GetFocus()); s_run_active=false;
                s_pending_toast=MotorControl_WorkText(); mark_settings_dirty(HAL_GetTick());
            } else s_pending_toast="Stop both first";
            continue;
        }
        switch (evt) {
        case EVT_IBUS_LIMIT:
            if (MotorSampling_SetIbusLimit((uint16_t)s_ibus_limit_ma)) {
                AppSettings *st=AppStorage_Get();
                st->control_config=(st->control_config & ~APP_CONFIG_IBUS_MASK) |
                    ((uint32_t)(s_ibus_limit_ma/100)<<APP_CONFIG_IBUS_SHIFT);
                mark_settings_dirty(HAL_GetTick()); s_pending_toast="IBUS limit applied";
            } else s_pending_toast="Stop M1 / check IBUS";
            refresh_texts(); KK_UI_Invalidate(); break;
        case EVT_IBUS_CLEAR:
            MotorControl_Stop(MOTOR_AXIS_1);
            if (MotorControl_IsCoupled()) s_run_active=views[0].run_active=views[1].run_active=false;
            else if (s_view_axis==MOTOR_AXIS_1) s_run_active=false;
            else views[0].run_active=false;
            if (MotorSampling_ClearIbusTrip()) {
                if (MotorControl_HsoAllowed() && JerryFOC_hasHSOFault()) (void)JerryFOC_useHSO(0U);
                if (s_view_axis==MOTOR_AXIS_1 || MotorControl_IsCoupled()) clear_run_stop();
                s_pending_toast="IBUS cleared";
            } else s_pending_toast="Wait / check IBUS";
            refresh_texts(); KK_UI_Invalidate(); break;
        case EVT_FOCUS_M1: case EVT_FOCUS_M2:
            focus_axis(evt==EVT_FOCUS_M1 ? MOTOR_AXIS_1 : MOTOR_AXIS_2);
            s_pending_toast=s_view_axis==MOTOR_AXIS_1 ? "Axis M1" : "Axis M2";
            break;
        case EVT_FEEDBACK:
            if (MotorControl_SelectFeedback(s_hso_choice)) mark_settings_dirty(HAL_GetTick());
            else s_pending_toast="Single M1 / stop";
            s_hso_choice=MotorControl_HsoAllowed();
            refresh_texts(); KK_UI_Invalidate();
            break;
        case EVT_SIXSTEP:
            if (MotorControl_SelectSixStep(s_six_choice)) mark_settings_dirty(HAL_GetTick());
            else s_pending_toast="Single M1 / stop";
            refresh_texts(); KK_UI_Invalidate();
            break;
        case EVT_GEAR:
            if (MotorControl_SetGear(s_ratio_x100,s_offset_x100)) mark_settings_dirty(HAL_GetTick());
            else {
                s_ratio_x100=AppStorage_Get()->gear_ratio_x100;
                s_offset_x100=AppStorage_Get()->gear_offset_x100;
                s_pending_toast="Stop / ratio nonzero";
            }
            break;
        case EVT_ALIGN:
            s_pending_toast=MotorControl_RequestAlign(s_view_axis) ? "Align requested" : "Stop / check axis";
            break;
        case EVT_MOTOR_2804_CONFIRMED:
        case EVT_MOTOR_4015_CONFIRMED:
            (void)select_motor(evt == EVT_MOTOR_4015_CONFIRMED ? MOTOR_PROFILE_4015 : MOTOR_PROFILE_2804);
            break;
        case EVT_MOTOR_ESC:
            (void)select_motor(MOTOR_PROFILE_COUNT); break;
        case EVT_CURRENT_LOOP_CHANGED:
            if (s_view_axis==MOTOR_AXIS_1 && sensorless()) {
                s_current_loop_display=MotorControl_HsoAllowed(); break;
            }
            s_current_loop=s_current_loop_display;
            s_run_current_x100 = s_confirmed_current_x100 = 0;
            if (ui_store_current()) mark_settings_dirty(HAL_GetTick());
            break;

        case EVT_POSITION_SPEED_CHANGED:
        case EVT_LED_CHANGED:
            if (ui_store_current()) mark_settings_dirty(HAL_GetTick());
            break;

        case EVT_SPEED_LIMIT_CHANGED: {
            update_position_speed_limit();
            int32_t limit = s_speed_limit * 10;
            s_speed_x10 = clamp_i32(s_speed_x10, -limit, limit);
            s_run_speed_x10 = clamp_i32(s_run_speed_x10, -limit, limit);
            if (ui_store_current()) mark_settings_dirty(HAL_GetTick());
            KK_UI_Invalidate();
            break;
        }

        case EVT_RUN_STEPS_CHANGED:
            for (unsigned i = 0U; i < 3U; ++i) s_run_steps[i] = s_step_drafts[i];
            if (ui_store_current()) mark_settings_dirty(HAL_GetTick());
            break;

        case EVT_RESET_CONFIRMED:
            MotorControl_StopAll();
            if (!MotorControl_SelectProfile(MOTOR_AXIS_1,MOTOR_PROFILE_2804) ||
                !MotorControl_SelectProfile(MOTOR_AXIS_2,MOTOR_PROFILE_2804) ||
                !MotorControl_SelectWork(MOTOR_WORK_SINGLE_M1) || !MotorControl_SelectFeedback(false)) {
                s_pending_toast="Stop/check encoder"; break;
            }
            clear_run_stop(); AppStorage_ResetDefaults();
            (void)MotorSampling_SetIbusLimit(APP_IBUS_DEFAULT_MA);
            (void)MotorControl_SetGear(100,0);
            reload_axis_views();
            load_view(MOTOR_AXIS_1); MotorControl_SetFocus(MOTOR_AXIS_1);
            s_ratio_x100=100; s_offset_x100=0;
            mark_settings_dirty(HAL_GetTick());
            s_pending_toast="Defaults restored"; KK_UI_Invalidate();
            break;

        default:
            break;
        }
    }
}

/* ---------- Run 页自定义绘制 ---------- */
void KK_UI_CustomOnEnter(KK_UI_PageId page)
{
    if (page == UI_PAGE_DETECT) {
        MotorDetect_Reset(); s_detect_result=AppStorage_Get()->motor_detect;
        s_detect_l=AppStorage_Get()->motor_detect_l_uh;
        s_detect_flux=AppStorage_Get()->motor_detect_flux_uwb; s_detect_page=0;
        return;
    }
    if (page == UI_PAGE_RUN_STEPS) {
        for (unsigned i = 0U; i < 3U; ++i) s_step_drafts[i] = s_run_steps[i];
        s_step_selected = 0U;
        return;
    }
    if (page != UI_PAGE_RUN) return;
    if (MotorControl_IsCoupled())
        focus_axis(MotorControl_GetWork()==MOTOR_WORK_MASTER_M1 ? MOTOR_AXIS_2 : MOTOR_AXIS_1);
    else if (MotorControl_GetWork()==MOTOR_WORK_SINGLE_M1) focus_axis(MOTOR_AXIS_1);
    else if (MotorControl_GetWork()==MOTOR_WORK_SINGLE_M2) focus_axis(MOTOR_AXIS_2);
    s_run_mode = ui_state().mode;
    if (!MotorControl_ModeAllowed(s_run_mode))
        s_run_mode=fallback_mode();
    stop_run();
    s_run_throttle_pct=s_throttle_pct;
    s_run_speed_x10 = s_speed_x10;
    if (sensorless()) s_run_speed_x10=hso_draft(s_run_speed_x10);
    s_run_voltage_x100 = s_voltage_x100;
    s_run_position_x100 = s_position_x100;
    s_run_current_x100 = s_confirmed_current_x100 = 0;
    s_run_position_origin = ui_state().position;
    s_last_live_at = HAL_GetTick();
}

void KK_UI_CustomOnLeave(KK_UI_PageId page)
{
    if (page == UI_PAGE_DETECT) MotorDetect_Stop();
    if (page == UI_PAGE_RUN) {
        s_run_lr_key=0U;
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        for (unsigned a=0;a<2;++a)
            if (MotorControl_GetInput((MotorAxis)a)==MOTOR_INPUT_LOCAL) MotorControl_Stop((MotorAxis)a);
        __set_PRIMASK(mask);
        s_run_active=views[0].run_active=views[1].run_active=false;
    }
}

void KK_UI_CustomOnInput(KK_UI_PageId page, KK_UI_InputEvent event)
{
    s_last_input_at = HAL_GetTick();
    if (page == UI_PAGE_DETECT) {
        if (event.action==KK_UI_INPUT_BACK) {
            MotorDetect_Stop(); (void)KK_UI_CustomRequestClose();
        } else if (event.action==KK_UI_INPUT_OK) {
            if (MotorDetect_Active()) MotorDetect_Stop();
            else if (MotorDetect_Result()) {
                AppStorage_Get()->motor_detect=MotorDetect_Result();
                AppStorage_Get()->motor_detect_l_uh=MotorDetect_Inductance();
                AppStorage_Get()->motor_detect_flux_uwb=MotorDetect_Flux();
                mark_settings_dirty(HAL_GetTick());
                (void)KK_UI_ShowToast("Save queued",900U);
            } else {
                s_detect_result=0; (void)MotorDetect_Start((unsigned)s_detect_ma);
            }
        } else if (!MotorDetect_Active()) {
            if(s_detect_result && (event.action==KK_UI_INPUT_LEFT || event.action==KK_UI_INPUT_RIGHT)) {
                s_detect_page=(s_detect_page+(event.action==KK_UI_INPUT_LEFT ? 3 : 1))%4;
                KK_UI_Invalidate(); return;
            }
            if (event.action==KK_UI_INPUT_UP || event.action==KK_UI_INPUT_DOWN)
                s_detect_ma=clamp_i32(s_detect_ma+(event.action==KK_UI_INPUT_UP ? 1 : -1)*(int32_t)event.steps*50,100,500);
            MotorDetect_Reset(); s_detect_result=0; s_detect_page=0;
        }
        KK_UI_Invalidate(); return;
    }
    if ((page == UI_PAGE_RUN || page == UI_PAGE_RUN_STEPS) &&
        event.source == KK_UI_INPUT_REPEAT) ++s_input_repeats;
    if (page == UI_PAGE_RUN_STEPS) {
        switch (event.action) {
        case KK_UI_INPUT_BACK:
            (void)KK_UI_CustomRequestClose();
            return;
        case KK_UI_INPUT_OK:
            (void)KK_UI_CustomFinish(EVT_RUN_STEPS_CHANGED);
            return;
        case KK_UI_INPUT_UP:
            s_step_selected = (uint8_t)((s_step_selected + 2U * (event.steps % 3U)) % 3U);
            break;
        case KK_UI_INPUT_DOWN:
            s_step_selected = (uint8_t)((s_step_selected + event.steps) % 3U);
            break;
        case KK_UI_INPUT_LEFT:
        case KK_UI_INPUT_RIGHT: {
            int32_t delta = (int32_t)event.steps * (event.action == KK_UI_INPUT_LEFT ? -1 : 1);
            s_step_drafts[s_step_selected] = clamp_i32(s_step_drafts[s_step_selected] + delta, 1, 100);
            break;
        }
        default: return;
        }
        KK_UI_Invalidate();
        return;
    }
    if (page != UI_PAGE_RUN) return;
    if (event.action == KK_UI_INPUT_BACK) {
        if ((s_view_axis==MOTOR_AXIS_1 && JerryFOC_hasHSOFault())) (void)JerryFOC_useHSO(0U);
        if (s_run_active || views[1U-s_view_axis].run_active || !ui_state().aligning)
            MotorControl_StopAll();
        s_run_active=views[0].run_active=views[1].run_active=false;
        stop_run(); /* 在退出动画开始前停止，不等待 OnLeave。 */
        (void)KK_UI_CustomRequestClose();
        return;
    }
    if (MotorControl_IsHandFollow()) {
        if (event.action==KK_UI_INPUT_OK) {
            if (MotorControl_IsGroupActive()) { MotorControl_StopAll(); s_run_active=false; }
            else {
                ui_store_current();
                s_run_active=MotorControl_StartFollow();
                if (s_run_active) clear_run_stop();
                else (void)KK_UI_ShowToast(MotorControl_CommandText(),1200U);
            }
            KK_UI_Invalidate();
        }
        return; /* Hand motion supplies the target; arrows never command a motor. */
    }
    if (event.action == KK_UI_INPUT_LEFT || event.action == KK_UI_INPUT_RIGHT) {
        if (MotorControl_GetWork()==MOTOR_WORK_INDEPENDENT && event.source==KK_UI_INPUT_PRESS) {
            s_run_lr_key=event.action==KK_UI_INPUT_LEFT ? KK_UI_KEY_LEFT : KK_UI_KEY_RIGHT;
            s_run_lr_pressed_at=HAL_GetTick(); s_run_lr_releasing=false;
        } else if (event.source != KK_UI_INPUT_REPEAT)
            run_mode_step(event.action == KK_UI_INPUT_LEFT ? -1 : 1);
        return;
    }
    if (event.action == KK_UI_INPUT_OK) {
        const MotorAxisSnapshot state=ui_state();
        bool changed = run_target_changed();
        if ((s_view_axis==MOTOR_AXIS_1 && JerryFOC_hasHSOFault())) {
            (void)JerryFOC_useHSO(0U);
            stop_run();
            (void)KK_UI_ShowToast("HSO reset", 900U);
        } else if (s_run_active && !changed) {
            stop_run();
        } else if (!state.ready) {
            (void)KK_UI_ShowToast(state.status, 900U);
        } else if (!state.healthy || !state.aligned) {
            (void)KK_UI_ShowToast(state.healthy ? "NOT ALIGNED" : MotorControl_EncoderLostText(s_view_axis), 900U);
        } else if (!s_run_active && !MotorSampling_IsAxisOutputRequested(s_view_axis) &&
                   (fabsf(state.velocity) > 1.0f ||
                    (MotorControl_HsoAllowed() && s_run_speed_x10!=0 && !StHso_AtRest()))) {
            (void)KK_UI_ShowToast("Wait for stop", 900U);
        } else if (sensorless() && s_run_mode==JERRYFOC_MODE_VELOCITY &&
                   (hso_draft(s_run_speed_x10)!=s_run_speed_x10 ||
                    (MotorSampling_IsAxisOutputRequested(s_view_axis) && state.target_velocity*(float)s_run_speed_x10<0.0f))) {
            (void)KK_UI_ShowToast("Stop / check range",1200U);
        } else {
            /* 运行中确认新位置时保留本次原点，停止后启动才重新捕获。 */
            if (!s_run_active) s_run_position_origin = state.position;
            s_run_active = true;
            clear_run_stop();
            if (!apply_run_target()) {
                stop_run();
                s_run_active=false;
                (void)KK_UI_ShowToast(MotorControl_CommandText(),1200U);
                KK_UI_Invalidate(); return;
            }
            *run_target(false) = *run_target(true);
            if (sensorless() && *run_target(true)==0)
                s_run_active=false; /* Sensorless zero requests coast, not zero-speed holding. */
            ui_store_current();
            if (changed) mark_settings_dirty(HAL_GetTick());
        }
        KK_UI_Invalidate();
        return;
    }
    if (event.action != KK_UI_INPUT_UP && event.action != KK_UI_INPUT_DOWN) return;
    /* PRESS / REPEAT 共用同一条加减路径，三种模式均按当前步长修改草稿。 */
    int32_t step = (throttle_mode(s_run_mode) ? 1 : s_run_steps[s_run_mode]) * (int32_t)event.steps;
    if (event.action == KK_UI_INPUT_DOWN) step = -step;
    int32_t limit = s_run_mode == JERRYFOC_MODE_VELOCITY ? s_speed_limit * 10 :
                    s_run_mode == JERRYFOC_MODE_POSITION ? 314 :
                    to_x100(MotorParams_GetAxis(s_view_axis)->vel_limit);
    if (s_run_mode == JERRYFOC_MODE_TORQUE && s_current_loop)
        limit = to_x100(MotorParams_GetAxis(s_view_axis)->rated_current);
    if (throttle_mode(s_run_mode)) limit=100;
    int32_t *draft = run_target(true);
    if (sensorless() && s_run_mode==JERRYFOC_MODE_VELOCITY) {
        int32_t next=*draft+step,min=hso_min_x10();
        if (hso_max_x10()<min) {
            *draft=0;
            (void)KK_UI_ShowToast("Raise Speed Limit",1200U);
        } else {
            if (*draft!=0 && ((*draft>0 && next<min) || (*draft<0 && next>-min))) next=0;
            else next=hso_draft(next);
            if (MotorSampling_IsAxisOutputRequested(s_view_axis) && ui_state().target_velocity*(float)next<0.0f)
                (void)KK_UI_ShowToast("Stop before reverse",1200U);
            else *draft=next;
        }
    } else *draft = clamp_i32(*draft + step, throttle_mode(s_run_mode) ? 0 : -limit, limit);
    KK_UI_Invalidate();
}

bool KK_UI_CustomOnTick(KK_UI_PageId page, uint32_t now_ms)
{
    if ((page != UI_PAGE_RUN && page != UI_PAGE_DETECT) || now_ms - s_last_live_at < 100U) return false;
    s_last_live_at = now_ms;
    return true;
}

void KK_UI_CustomOnDraw(KK_UI_PageId page, int16_t x_offset,
                        int16_t clip_x, uint16_t clip_width)
{
    if (page != UI_PAGE_RUN && page != UI_PAGE_RUN_STEPS && page != UI_PAGE_DETECT) return;
    const MotorAxisSnapshot motor_state=ui_state();
    const int16_t W = (int16_t)OLED_GetWidth();
    const int16_t H = (int16_t)OLED_GetHeight();
    char buf[40];
    char v[16];
    OLED_SetClipWindow(clip_x, 0, clip_width, (uint16_t)H);
    OLED_SetDrawMode(OLED_DRAW_SET);
    if (page == UI_PAGE_DETECT) {
        OLED_SetFont(ui_font_body);
        OLED_DrawUTF8((int16_t)(4+x_offset),1,"M1 Detect");
        OLED_DrawHLine(x_offset,14,(uint16_t)W);
        UiText_Format(buf,sizeof(buf),"I test %ld mA",(long)s_detect_ma);
        OLED_DrawUTF8((int16_t)(4+x_offset),17,buf);
        uint32_t detected=MotorDetect_Result();
        OLED_DrawUTF8((int16_t)(4+x_offset),33,!detected && s_detect_result ? "Saved" : MotorDetect_Status());
        if(detected) {
            s_detect_result=detected; s_detect_l=MotorDetect_Inductance(); s_detect_flux=MotorDetect_Flux();
        }
        int p=(int8_t)s_detect_result;
        fmt_fixed(v,sizeof(v),(int32_t)((s_detect_result>>8)&65535U)/10,100);
        UiText_Format(buf,sizeof(buf),"PP %u%s R %s ohm",(unsigned)abs(p),p<0 ? "-" : "+",v);
        if(s_detect_page==1) UiText_Format(buf,sizeof(buf),"Ls %lu uH",(unsigned long)s_detect_l);
        if(s_detect_page==2) UiText_Format(buf,sizeof(buf),"Flux %lu uWb",(unsigned long)s_detect_flux);
        if(s_detect_page==3) {
            fmt_fixed(v,sizeof(v),(int32_t)((float)s_detect_flux*(float)abs(p)*0.0128254983f+0.5f),100);
            UiText_Format(buf,sizeof(buf),"Ke %s V/krpm",v); /* Line-line RMS / mechanical krpm. */
        }
        if(s_detect_page && !(s_detect_page==1 ? s_detect_l : s_detect_flux)) strcpy(buf,"NO DATA");
        OLED_DrawUTF8((int16_t)(4+x_offset),49,s_detect_result ? buf : "NO DATA");
        OLED_DrawHLine(x_offset,65,(uint16_t)W);
        OLED_DrawUTF8((int16_t)(4+x_offset),67,MotorDetect_Active() ? "OK:Stop K2:Back" :
            detected ? "OK:Save LR:View" : "OK:Start K2:Back");
        OLED_ResetClipWindow(); return;
    }
    if (page == UI_PAGE_RUN_STEPS) {
        /* Compact header keeps its existing baseline; Home uses the restored
         * family line box, independently of which title glyphs were retained. */
        OLED_SetFontRefHeight(OLED_FONT_REF_TEXT);
        OLED_SetFont(ui_font_title);
        OLED_DrawUTF8((int16_t)(4 + x_offset), 1, texts.steps);
        const char *save = "OK:Save";
        OLED_DrawUTF8((int16_t)(W - 4 - OLED_GetUTF8Width(save) + x_offset), 1, save);
        OLED_DrawHLine(x_offset, 14, (uint16_t)W);
        OLED_SetFontRefHeight(OLED_FONT_REF_ALL);
        OLED_SetFont(ui_font_body);
        for (unsigned i = 0U; i < 3U; ++i) {
            int16_t y = (int16_t)(17 + 16 * i);
            if (i == s_step_selected) {
                OLED_DrawRBox((int16_t)(2 + x_offset), (int16_t)(y - 1), (uint16_t)(W - 4), 16U, 2U);
                OLED_SetDrawMode(OLED_DRAW_CLEAR);
            }
            OLED_DrawUTF8((int16_t)(5 + x_offset), y, mode_name((JerryFOC_ControlMode)i));
            const char *unit = i == JERRYFOC_MODE_VELOCITY ? "rad/s" :
                               i == JERRYFOC_MODE_POSITION ? "rad" : s_current_loop ? "A" : "V";
            fmt_fixed(v, sizeof(v), s_step_drafts[i], i == JERRYFOC_MODE_VELOCITY ? 10 : 100);
            UiText_Format(buf, sizeof(buf), "%s %s", v, unit);
            OLED_DrawUTF8((int16_t)(W - 5 - OLED_GetUTF8Width(buf) + x_offset), y, buf);
            OLED_SetDrawMode(OLED_DRAW_SET);
        }
        OLED_DrawHLine(x_offset, 65, (uint16_t)W);
        OLED_DrawUTF8((int16_t)(4 + x_offset), 67, "L/R:Step K2:Cancel");
        OLED_ResetClipWindow();
        return;
    }
    /* Use the live, validated bus measurement in every Run variant. */
    OLED_SetFont(ui_font_body);
    const char *bus=!strcmp(texts.bus,"NO DATA") ? "--V" : texts.bus;
    OLED_DrawUTF8((int16_t)(W-4-OLED_GetUTF8Width(bus)+x_offset),1,bus);
    if (MotorControl_IsHandFollow()) {
        unsigned master=MotorControl_GetWork()==MOTOR_WORK_MASTER_M2 ? 2U : 1U;
        bool active=MotorControl_IsGroupActive();
        UiText_Format(buf,sizeof(buf),"Hand M%u>M%u ENC",master,3U-master);
        OLED_DrawUTF8((int16_t)(4+x_offset),1,buf);
        OLED_DrawHLine(x_offset,14,(uint16_t)W);
        OLED_DrawUTF8((int16_t)(4+x_offset),19,"Turn master by hand");
        fmt_fixed(v,sizeof(v),to_x100(MotorControl_GroupError()),100);
        UiText_Format(buf,sizeof(buf),"Error %s rad",v);
        OLED_DrawUTF8((int16_t)(4+x_offset),35,buf);
        OLED_DrawUTF8((int16_t)(4+x_offset),51,active ? "RUN: Following" :
            strcmp(texts.run_stop,"--") ? texts.run_stop : "STOP: OK to set zero");
        OLED_DrawHLine(x_offset,65,(uint16_t)W);
        OLED_DrawUTF8((int16_t)(4+x_offset),67,active ? "OK:Stop K2:Back" : "OK:Start K2:Back");
        OLED_ResetClipWindow(); return;
    }
    bool hso=s_view_axis==MOTOR_AXIS_1 && MotorControl_HsoAllowed();
    bool six=s_view_axis==MOTOR_AXIS_1 && MotorControl_SixStepAllowed();
    bool oil=throttle_mode(s_run_mode);
    bool independent=MotorControl_GetWork()==MOTOR_WORK_INDEPENDENT;
    bool starting=hso && StHso_IsStarting();
    const char *mode=oil ? "Throttle" : s_run_mode==JERRYFOC_MODE_TORQUE ? (s_current_loop ? "Iq(A)" : "Uq(V)") : mode_name(s_run_mode);
    UiText_Format(buf,sizeof(buf),"M%u %s %s",(unsigned)s_view_axis+1U,mode,six ? (esc_view() ? "ESC" : "6STEP") : hso ? "HSO" : "ENC");
    OLED_DrawUTF8((int16_t)(4+x_offset),1,buf);
    OLED_DrawHLine(x_offset,14,(uint16_t)W);
    bool changed=run_target_changed();
    const char *state=changed ? (s_run_active ? "RUN*" : "EDIT") : (s_run_active ? "RUN" : "STOP");
    const char *unit=oil ? "%" : s_run_mode==JERRYFOC_MODE_VELOCITY ? "rad/s" :
        s_run_mode==JERRYFOC_MODE_POSITION ? "rad" : s_current_loop ? "A" : "V";
    int scale=run_scale(oil);
    fmt_fixed(v,sizeof(v),*run_target(true),scale);
    int16_t unit_width=(int16_t)OLED_GetUTF8Width(unit);
    OLED_SetFont(ui_font_big);
    int16_t value_width=(int16_t)OLED_GetUTF8Width(v);
    /* Center the complete value/unit block, including signed range limits.
     * Run/edit state lives below it so it cannot collide with -600.0. */
    int16_t value_x=(int16_t)((W-value_width-unit_width-4)/2+x_offset);
    OLED_DrawUTF8(value_x,19,v);
    OLED_SetFont(ui_font_body);
    OLED_DrawUTF8((int16_t)(value_x+value_width+4),24,unit);
    const char *fault=NULL;
    if (!motor_state.ready) fault=motor_state.status;
    else if (!s_run_active && strcmp(texts.run_stop,"--")) fault=texts.run_stop;
    else if (!motor_state.healthy || !motor_state.aligned)
        fault=motor_state.healthy ? "NOT ALIGNED" : MotorControl_EncoderLostText(s_view_axis);
    if (fault) UiText_Format(buf,sizeof(buf),"%s",fault);
    else {
        const char *speed_tag=starting ? "Ref" : six ? "Est" : "Act";
        int32_t actual=s_run_mode==JERRYFOC_MODE_POSITION ?
            to_x100(motor_state.position-s_run_position_origin) : to_x10(motor_state.velocity);
        fmt_fixed(v,sizeof(v),actual,s_run_mode==JERRYFOC_MODE_POSITION ? 100 : 10);
        if (changed) {
            char applied[16]; fmt_fixed(applied,sizeof(applied),*run_target(false),scale);
            UiText_Format(buf,sizeof(buf),"%s %s Set %s",speed_tag,v,applied);
        } else if (hso && *run_target(true)==0) UiText_Format(buf,sizeof(buf),"Act %s 0=Coast",v);
        else if (six || s_run_mode!=JERRYFOC_MODE_TORQUE) {
            char aux[16]; fmt_run_aux(aux,actual);
            UiText_Format(buf,sizeof(buf),"%s %s %s",speed_tag,v,aux);
        } else UiText_Format(buf,sizeof(buf),"Act %s Max %ld",v,(long)s_speed_limit);
    }
    OLED_DrawUTF8((int16_t)(4+x_offset),38,buf);
    if (independent) {
        /* Read both actual output requests; a failed/unviewed axis must not
         * remain labelled RUN merely because its UI view was cached. */
        for (unsigned a=0;a<2;++a) {
            UiText_Format(buf,sizeof(buf),"M%u:%s%s",a+1U,
                MotorSampling_IsAxisOutputRequested((MotorAxis)a) ? "RUN" : "STOP",
                a==(unsigned)s_view_axis && changed ? "*" : "");
            int16_t x=(int16_t)(4+78*a+x_offset);
            if (a==(unsigned)s_view_axis) {
                OLED_DrawRBox((int16_t)(x-1),50,74,14,2);
                OLED_SetDrawMode(OLED_DRAW_CLEAR);
            }
            OLED_DrawUTF8(x,51,buf); OLED_SetDrawMode(OLED_DRAW_SET);
        }
    } else if (six) {
        SixStepSnapshot ss; SixStep_GetSnapshot(&ss);
        OLED_DrawUTF8((int16_t)(4+x_offset),51,ss.status);
    } else if (hso) {
        StHsoSnapshot snapshot; StHso_GetSnapshot(&snapshot);
        UiText_Format(buf,sizeof(buf),"%s HSO %s",state,StHso_StateText(snapshot.state));
        OLED_DrawUTF8((int16_t)(4+x_offset),51,buf);
    } else if (!fault) {
        if (MotorControl_IsCoupled()) {
            MotorAxisSnapshot other; MotorControl_GetAxis((MotorAxis)(1U-s_view_axis),&other);
            fmt_fixed(v,sizeof(v),to_x10(other.velocity),10);
            UiText_Format(buf,sizeof(buf),"%s M%u %s",state,(unsigned)(2U-s_view_axis),v);
        } else {
            fmt_fixed(v,sizeof(v),to_x100(motor_state.iq),100);
            UiText_Format(buf,sizeof(buf),"%s Iq %s A",state,v);
        }
        if (!MotorControl_IsCoupled() && s_run_mode!=JERRYFOC_MODE_TORQUE) {
            fmt_run_aux(v,*run_target(true));
        } else {
            char step[10]; fmt_fixed(step,sizeof(step),s_run_steps[s_run_mode],scale);
            UiText_Format(v,sizeof(v),"Step %s",step);
        }
        int16_t right=(int16_t)(W-4-OLED_GetUTF8Width(v)+x_offset);
        if (4+x_offset+OLED_GetUTF8Width(buf)+4>right) strcpy(buf,state);
        OLED_DrawUTF8((int16_t)(4+x_offset),51,buf);
        OLED_DrawUTF8(right,51,v);
    } else OLED_DrawUTF8((int16_t)(4+x_offset),51,state);
    OLED_DrawHLine(x_offset,65,(uint16_t)W);
    const char *action=(hso || six) && *run_target(true)==0 ? "Stop" :
        s_run_active ? (changed ? "Apply" : "Stop") : "Start";
    UiText_Format(buf,sizeof(buf),independent ? "OK:%s LR hold K2:Back" : "OK:%s K2:Back",action);
    OLED_DrawUTF8((int16_t)(4+x_offset),67,buf);
    OLED_ResetClipWindow();
}
