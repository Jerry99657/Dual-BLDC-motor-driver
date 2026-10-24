#include "ui_app.h"

#include <math.h>
#include <stdio.h>
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

/* ---------- 事件 ---------- */
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
    EVT_WORK_BASE=32,
};

/* ---------- 设置菜单下标 ---------- */
enum { INT_LED_BRIGHT, INT_POSITION_SPEED, INT_SPEED_LIMIT, INT_GEAR_RATIO, INT_GEAR_OFFSET, INT_IBUS_LIMIT };
enum { BOOL_CURRENT_LOOP, BOOL_LED, BOOL_HSO };
enum { CONFIRM_RESET, CONFIRM_MOTOR_2804, CONFIRM_MOTOR_4015, CONFIRM_IBUS_CLEAR };

/* ---------- 设置变量 ---------- */
static int32_t s_led_brightness;
static bool s_current_loop_display;
static bool s_led_enable;
static bool s_hso_choice;
static int32_t s_ratio_x100=100,s_offset_x100;
static uint8_t system_states[2];
static uint8_t settings_states[3]={[2]=KK_UI_ITEM_DISABLED<<2U};
static uint8_t protection_states[1]={[0]=KK_UI_ITEM_DISABLED<<4U};
static int32_t s_ibus_limit_ma;
static char s_txt_ibus_state[24],s_txt_ibus_limit[24],s_txt_ibus_voltage[24];
static char s_txt_ibus_code[16],s_txt_ibus_trips[16],s_txt_ibus_pin[8];
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
static char s_txt_mode[16];
static char s_txt_command[8];
static MotorProfile s_profiles[2];
static char s_txt_work[20],s_txt_axis[4],s_txt_m1[8],s_txt_m2[8];
static char s_txt_sync[20],s_txt_control_time[20],s_txt_save[16];
static char s_txt_motor[8], s_txt_motor_menu[20];
static char s_txt_settings[24],s_txt_steps[20],s_txt_loop_title[24];
static char s_txt_system[24],s_txt_work_menu[24],s_txt_save_state[24];
static char s_txt_status[20],s_txt_groups[5][20];
static const char *const group_names[]={"Drive","Motion","Encoder","ADC","HSO"};
static char s_txt_rpm[24],s_txt_target_aux[24],s_txt_elec_deg[24],s_txt_mech_deg[24];
static char s_txt_target_unit[12];
static char s_txt_hso_min[24],s_txt_hso_max[24];
static char s_txt_work_items[MOTOR_WORK_COUNT][24]={"  Single M1","  Single M2","  Dual Independent",
    "  Dual Sync Speed","  Dual Sync Position","  Dual Master M1","  Dual Master M2"};
static char s_txt_axes[2][12]={"Axis M1  ","Axis M2  "};
static char s_txt_target[24];
static char s_txt_speed[24];
static char s_txt_iq[24];
static char s_txt_id[24];
static char s_txt_ia[24];
static char s_txt_ib[24];
static char s_txt_enc[16];
static char s_txt_enc2[16];
static char s_txt_enc2_err[16];
static char s_txt_drive[16];
static char s_txt_output[8];
static char s_txt_run_stop[24] = "--";
static char s_txt_stop_age[16] = "--", s_txt_stop_errors[16] = "--";
static char s_txt_enc_crc[16], s_txt_enc_field[16], s_txt_enc_track[16];
static char s_txt_enc_spi[16], s_txt_enc_frame[16], s_txt_enc_age[24], s_txt_enc_last[16];
static char s_txt_enc_zero[16], s_txt_enc_zpre[16], s_txt_enc_zpost[16], s_txt_enc_ztime[16];
/* Stable application-owned labels; update only their axis digit. */
static char s_enc_labels[][9]={"E1 State","E1 CRC","E1 Field","E1 Track",
    "E1 SPI","E1 Frame","E1 Age","E1 Last","E1 Raw","E1 Err",
    "E1 Zero","E1 ZPre","E1 ZPost","E1 ZTime"};
static char s_txt_current_adc[3][24];
static char s_txt_current_trip[24];
static char s_txt_adc_revision[16];
static char s_txt_align[16];
static char s_txt_raw[16];
static char s_txt_elec[24];
static char s_txt_mech[24];
static char s_txt_err[16];
static char s_txt_bus[24];
static char s_txt_bus_raw[16];
static char s_txt_bus_count[16];
static char s_txt_frame_count[16];
static char s_txt_adc1_error[16];
static char s_txt_adc2_error[16];
static char s_txt_reference[16], s_txt_ref_state[16], s_txt_ref_raw[16];
static char s_txt_ref_count[16], s_txt_ref_errors[16];
static char s_txt_repeat[16];
static char s_txt_display_errors[16];
static char s_txt_hso[16], s_txt_hso_speed[24], s_txt_hso_error[24];
static char s_txt_hso_flux[24], s_txt_hso_time[24];
static char s_txt_hso_gate[16], s_txt_hso_qual[24], s_txt_hso_lost[16];
static char s_txt_hso_flux_actual[24];
static char s_txt_hso_reject[16],s_txt_hso_resets[16];
static char s_txt_hso_rangle[16],s_txt_hso_rfreq[16],s_txt_hso_remf[16],s_txt_hso_rdrift[16];
static char s_txt_hso_offset[16],s_txt_hso_olimit[16];
static char s_txt_save_result[16];
static char s_txt_flash_error[16];
static char s_txt_spi2_fault[16],s_txt_spi2_resets[8];
static char s_txt_feedback[16];
static uint32_t s_input_repeats;
static uint32_t s_display_errors;

static uint32_t s_info_at;
static bool s_settings_dirty;
static uint32_t s_settings_save_at;
static const char *s_pending_toast;

static MotorAxis s_view_axis;
static MotorAxisSnapshot ui_state(void)
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
static void ui_store_current(void);
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
static int32_t hso_min_x10(void) { return (int32_t)ceilf(StHso_MinSpeed()*10.0f); }
static int32_t hso_max_x10(void)
{
    int32_t limit=(int32_t)(400.0f*6.283185307f*10.0f/MotorParams_GetAxis(MOTOR_AXIS_1)->pole_pairs);
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
    return (int32_t)(v * 10.0f + (v >= 0.0f ? 0.5f : -0.5f));
}

__attribute__((noinline)) static int32_t to_x100(float v)
{
    return (int32_t)(v * 100.0f + (v >= 0.0f ? 0.5f : -0.5f));
}

static void fmt_fixed(char *buf, size_t len, int32_t v, int scale)
{
    int32_t a = (v < 0) ? -v : v;
    const char *sign = (v < 0) ? "-" : "";

    if (scale == 10) {
        snprintf(buf, len, "%s%ld.%ld", sign,
                 (long)(a / 10), (long)(a % 10));
    } else {
        snprintf(buf, len, "%s%ld.%02ld", sign,
                 (long)(a / 100), (long)(a % 100));
    }
}

/* Keep repeated Status conversions in main-loop helpers, rather than
 * expanding rounding and two snprintf call frames for every diagnostic row. */
__attribute__((noinline)) static void fmt_unit(char *buf,int32_t value,int scale,const char *unit)
{
    char number[16];
    fmt_fixed(number,sizeof(number),value,scale);
    snprintf(buf,24U,"%s %s",number,unit);
}

/* Share the repeated 16-byte diagnostic-counter formatting in the 128KiB
 * image. A bounded call avoids expanding the same snprintf setup per row. */
__attribute__((noinline)) static void fmt_counter(char *buf,uint32_t count)
{
    snprintf(buf,16U,"%lu",(unsigned long)count);
}

__attribute__((noinline)) static void fmt_pair(char *buf,unsigned first,unsigned second)
{
    snprintf(buf,16U,"%u / %u",first,second);
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
    bool position=s_run_mode==JERRYFOC_MODE_POSITION;
    snprintf(buf,16U,"%ld%s",(long)to_x10((float)value*(position ? 0.05729578f : 0.095492966f)),
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
    snprintf(s_txt_save_state,sizeof(s_txt_save_state),"Save: %s",names[state]);
}

__attribute__((noinline)) static void axis_title(char *text,const char *suffix)
{
    snprintf(text,20U,"M%u %s %s",(unsigned)s_view_axis+1U,s_txt_motor,suffix);
}

static void refresh_texts(void)
{
    const MotorAxisSnapshot state=ui_state();
    strcpy(s_txt_work,MotorControl_WorkText());
    snprintf(s_txt_command,sizeof(s_txt_command),"%d",(int)Protocol_GetControlResult());
    strcpy(s_txt_axis,s_view_axis==MOTOR_AXIS_1 ? "M1" : "M2");
    strcpy(s_txt_m1,MotorParams_GetAxis(MOTOR_AXIS_1)->name);
    strcpy(s_txt_m2,MotorParams_GetAxis(MOTOR_AXIS_2)->name);
    fmt_fixed(s_txt_sync,sizeof(s_txt_sync),to_x100(MotorControl_GroupError()),100);
    snprintf(s_txt_control_time,sizeof(s_txt_control_time),"%lu us",(unsigned long)(MotorControl_MaxCycles()/170U));
    strcpy(s_txt_save,AppStorage_IsDeferred() ? "DEFERRED" : "AVAILABLE");
    strcpy(s_txt_save_result,AppStorage_SaveErrorText());
    uint32_t flash_error=AppStorage_FlashError();
    MT6701_2_BusDiagnostics bus_diag;
    MT6701_2_GetBusDiagnostics(&bus_diag);
    snprintf(s_txt_spi2_fault,sizeof(s_txt_spi2_fault),"%04X/%04X/%04X",
             (unsigned)bus_diag.cr1,(unsigned)bus_diag.cr2,(unsigned)bus_diag.sr);
    snprintf(s_txt_spi2_resets,sizeof(s_txt_spi2_resets),"%u",(unsigned)bus_diag.resets);
    snprintf(s_txt_flash_error,sizeof(s_txt_flash_error),"%lu/%lu/%02lX/%02lX",
        (unsigned long)(flash_error>>24),(unsigned long)((flash_error>>16)&255U),
        (unsigned long)((flash_error>>8)&255U),(unsigned long)(flash_error&255U));
    s_hso_choice=MotorControl_HsoAllowed();
    strcpy(s_txt_motor,MotorParams_GetAxis(s_view_axis)->name);
    axis_title(s_txt_motor_menu,"Motor");
    axis_title(s_txt_settings,"Settings");
    axis_title(s_txt_steps,"Steps");
    axis_title(s_txt_status,"Status");
    for (unsigned i=0;i<5;++i) axis_title(s_txt_groups[i],group_names[i]);
    snprintf(s_txt_groups[4],sizeof(s_txt_groups[4]),"M1 %s HSO",MotorParams_GetAxis(MOTOR_AXIS_1)->name);
    fmt_unit(s_txt_hso_min,hso_min_x10(),10,"rad/s");
    fmt_unit(s_txt_hso_max,hso_max_x10(),10,"rad/s");
    if (hso_max_x10()<hso_min_x10()) strcpy(s_txt_hso_max,"No run range");
    axis_title(s_txt_loop_title,"Curr Loop");
    snprintf(s_txt_system,sizeof(s_txt_system),"System: %s",MotorControl_WorkText());
    snprintf(s_txt_work_menu,sizeof(s_txt_work_menu),"Mode: %s",MotorControl_WorkText());
    for (unsigned i=0;i<MOTOR_WORK_COUNT;++i)
        s_txt_work_items[i][0]=i==(unsigned)MotorControl_GetWork() ? '*' : ' ';
    for (unsigned i=0;i<2;++i)
        s_txt_axes[i][8]=i==(unsigned)s_view_axis ? '*' : ' ';
    s_current_loop_display=s_current_loop || (s_view_axis==MOTOR_AXIS_1 && MotorControl_HsoAllowed());
    refresh_save_state();
    strcpy(s_txt_output,
             !MotorSampling_IsAxisOutputRequested(s_view_axis) ? "COAST" :
             MotorSampling_IsAxisOutputEnabled(s_view_axis) ? "PWM" : "OFF");
    fmt_counter(s_txt_repeat,s_input_repeats);
    fmt_counter(s_txt_display_errors,s_display_errors);

    strcpy(s_txt_mode,mode_name(state.mode));

    switch (state.mode) {
    case JERRYFOC_MODE_VELOCITY:
        fmt_unit(s_txt_target, to_x10(state.target_velocity), 10, "rad/s");
        break;
    case JERRYFOC_MODE_POSITION:
        fmt_unit(s_txt_target, to_x100(state.target_position), 100, "rad");
        break;
    default:
        fmt_unit(s_txt_target,to_x100(state.target_iq),100,s_current_loop ? "A" : "V");
        break;
    }

    fmt_unit(s_txt_speed, to_x10(state.velocity), 10, "rad/s");
    fmt_unit(s_txt_rpm,to_x10(state.velocity*9.5492966f),10,"rpm");
    if (state.mode==JERRYFOC_MODE_VELOCITY) {
        strcpy(s_txt_target_unit,"Target rpm");
        fmt_unit(s_txt_target_aux,to_x10(state.target_velocity*9.5492966f),10,"rpm");
    } else if (state.mode==JERRYFOC_MODE_POSITION) {
        strcpy(s_txt_target_unit,"Tgt 0-360");
        fmt_degrees(s_txt_target_aux,state.target_position);
    } else {
        strcpy(s_txt_target_unit,"Target aux"); strcpy(s_txt_target_aux,"--");
    }
    fmt_degrees(s_txt_elec_deg,state.electrical_angle);
    fmt_degrees(s_txt_mech_deg,state.position);

    fmt_unit(s_txt_iq, to_x100(state.iq), 100, "A");

    fmt_unit(s_txt_id, to_x100(state.id), 100, "A");

    fmt_unit(s_txt_ia, to_x100(state.ia), 100, "A");

    fmt_unit(s_txt_ib, to_x100(state.ib), 100, "A");

    strcpy(s_txt_enc,
             (s_view_axis==MOTOR_AXIS_1 ? MT6701_IsHealthy()!=0U : state.healthy) ? "OK" : "ERROR");
    MT6701_2_Snapshot encoder2;
    MT6701_2_GetSnapshot(&encoder2);
    snprintf(s_txt_enc2, sizeof(s_txt_enc2), "%s %u",
             encoder2.healthy ? "OK" : encoder2.stopped ? "OFF" : "ERROR", (unsigned)encoder2.raw_angle);
    fmt_counter(s_txt_enc2_err,encoder2.errors);
    snprintf(s_txt_drive, sizeof(s_txt_drive), "%s",
             drive_status_text());
    StHsoSnapshot hso;
    StHso_GetSnapshot(&hso);
    strcpy(s_txt_feedback,(s_view_axis==MOTOR_AXIS_1 ? JerryFOC_getFeedbackText() : "ENCODER"));
    /* All reason literals fit these 16-byte application buffers. */
    strcpy(s_txt_hso_gate,StHso_ReasonText(JerryFOC_isHSOActive() ? hso.reason : hso.entry_reason));
    snprintf(s_txt_hso_qual, sizeof(s_txt_hso_qual), "%lu/%u ms %lu/20",
             (unsigned long)(hso.qualified_samples / 10U),
             hso.state==ST_HSO_LEARNING || hso.reason==ST_HSO_REASON_LEARN_TIMEOUT ? 1000U : 200U,
             (unsigned long)hso.bad_samples);
    strcpy(s_txt_hso_lost,JerryFOC_getHSOFaultText());
    strcpy(s_txt_hso_reject,StHso_ReasonText(hso.capture_reason));
    fmt_counter(s_txt_hso_resets,hso.capture_resets);
    fmt_fixed(s_txt_hso_rangle,sizeof(s_txt_hso_rangle),to_x10(hso.capture_angle_rad*57.2957795f),10);
    fmt_fixed(s_txt_hso_rfreq,sizeof(s_txt_hso_rfreq),to_x10(hso.capture_frequency_error_hz),10);
    fmt_fixed(s_txt_hso_remf,sizeof(s_txt_hso_remf),to_x10(hso.capture_emf_ratio*100),10);
    fmt_fixed(s_txt_hso_rdrift,sizeof(s_txt_hso_rdrift),to_x10(hso.capture_drift_rad*57.2957795f),10);
    fmt_fixed(s_txt_hso_offset,sizeof(s_txt_hso_offset),to_x10(hso.emf_offset_v*1000),10);
    fmt_fixed(s_txt_hso_olimit,sizeof(s_txt_hso_olimit),to_x10(hso.emf_offset_limit_v*1000),10);
    fmt_unit(s_txt_hso_flux_actual, to_x10(hso.flux_wb * 1000.0f), 10, "mWb");
    strcpy(s_txt_hso,
             JerryFOC_hasHSOFault() ? "LOST" : StHso_StateText(hso.state));
    fmt_unit(s_txt_hso_speed, to_x10(hso.velocity_rad_s), 10, "rad/s");
    if (isfinite(hso.angle_error_rad)) {
        fmt_unit(s_txt_hso_error, to_x10(hso.angle_error_rad * 57.2957795f), 10, "deg");
    } else strcpy(s_txt_hso_error,"--"); /* Do not convert the startup NaN to integer. */
    snprintf(s_txt_hso_flux, sizeof(s_txt_hso_flux), "%lu uWb", (unsigned long)(hso.reference_flux_wb * 1000000.0f));
    snprintf(s_txt_hso_time, sizeof(s_txt_hso_time), "%lu us / %lu", (unsigned long)(hso.max_control_cycles / 170U), (unsigned long)hso.overruns);
    strcpy(s_txt_align,state.aligning ? "ALIGNING" : state.aligned ? "YES" : "NO");
    fmt_counter(s_txt_raw,ui_encoder_raw());

    fmt_unit(s_txt_elec, to_x100(state.electrical_angle), 100, "rad");

    fmt_unit(s_txt_mech, to_x100(state.position), 100, "rad");

    fmt_counter(s_txt_err,ui_encoder_errors());
    MT6701Diagnostics enc;
    ui_encoder_diag(&enc);
    for (unsigned i=0;i<sizeof(s_enc_labels)/sizeof(s_enc_labels[0]);++i)
        s_enc_labels[i][1]=(char)('1'+s_view_axis);
    fmt_counter(s_txt_enc_crc,enc.crc_errors);
    fmt_counter(s_txt_enc_field,enc.field_errors);
    fmt_counter(s_txt_enc_track,enc.track_errors);
    fmt_counter(s_txt_enc_spi,enc.spi_errors);
    fmt_counter(s_txt_enc_frame,enc.frame_errors);
    fmt_pair(s_txt_enc_age,enc.age_cycles,enc.max_age_cycles);
    strcpy(s_txt_enc_last,MT6701_ErrorText(enc.last_error));
    const char *zero=enc.zero_age>3U ? "GAP" :
        (enc.zero_before<=128U || enc.zero_before>=16384U-128U) ? "NEAR" : "FAR";
    if (enc.zero_count) {
        snprintf(s_txt_enc_zero,sizeof(s_txt_enc_zero),"%s %u",zero,enc.zero_count);
        if (enc.zero_age!=UINT16_MAX) fmt_counter(s_txt_enc_zpre,enc.zero_before);
        else strcpy(s_txt_enc_zpre,"--");
        if (enc.zero_after!=UINT16_MAX) fmt_counter(s_txt_enc_zpost,enc.zero_after);
        else strcpy(s_txt_enc_zpost,"--");
        fmt_counter(s_txt_enc_ztime,(enc.zero_after==UINT16_MAX ? HAL_GetTick() : enc.zero_ended_at)-enc.zero_started_at);
    } else {
        strcpy(s_txt_enc_zero,"--"); strcpy(s_txt_enc_zpre,"--"); strcpy(s_txt_enc_zpost,"--");
        strcpy(s_txt_enc_ztime,"--");
    }

    MotorSamplingDiagnostics adc;
    MotorSampling_GetAxisDiagnostics(s_view_axis,&adc);
    s_ibus_limit_ma=MotorSampling_GetIbusLimit();
    fmt_unit(s_txt_ibus_limit,s_ibus_limit_ma/100,10,"A");
    fmt_unit(s_txt_ibus_voltage,to_x100(adc.ibus_threshold_v),100,"V");
    fmt_counter(s_txt_ibus_code,adc.ibus_dac_code);
    fmt_counter(s_txt_ibus_trips,adc.ibus_trips);
    strcpy(s_txt_ibus_pin,adc.ibus_high ? "HIGH" : "LOW");
    strcpy(s_txt_ibus_state,adc.ibus_latched ? "IBUS: TRIPPED" : adc.ibus_ready ? "IBUS: READY" : "IBUS: NOT READY");
    unsigned reference_mv = (unsigned)(adc.reference_voltage * 1000.0f + 0.5f);
    if (adc.reference_updates != 0U)
        snprintf(s_txt_reference, sizeof(s_txt_reference), "%u.%03u V",
                 reference_mv / 1000U, reference_mv % 1000U);
    else
        strcpy(s_txt_reference,"NO DATA");
    strcpy(s_txt_ref_state,
             adc.reference_fresh ? "OK" : (adc.reference_updates ? "STALE" : "WAIT"));
    fmt_pair(s_txt_ref_raw,adc.reference_raw,adc.reference_cal);
    fmt_counter(s_txt_ref_count,adc.reference_updates);
    fmt_counter(s_txt_ref_errors,adc.reference_errors);
    snprintf(s_txt_adc_revision, sizeof(s_txt_adc_revision), "0x%04X", (unsigned)adc.silicon_revision);
    for (unsigned i = 0U; i < 3U; ++i) {
        if (s_view_axis==MOTOR_AXIS_2 && i==2U) { strcpy(s_txt_current_adc[i],"derived"); continue; }
        fmt_pair(s_txt_current_adc[i],adc.current_raw[i],adc.current_zero[i]);
    }
    if (s_view_axis==MOTOR_AXIS_2 && adc.fault_phase==3U) {
        strcpy(s_txt_current_trip,"RUN W derived");
    } else if (adc.fault_phase != 0U) {
        unsigned i = adc.fault_phase - 1U;
        snprintf(s_txt_current_trip, sizeof(s_txt_current_trip), "%s %c %u",
                 adc.fault_during_zero ? "ZERO" : "RUN", "UVW"[i],
                 (unsigned)adc.fault_current_raw[i]);
    } else {
        strcpy(s_txt_current_trip,"--");
    }
    if (adc.bus_fresh) {
        fmt_unit(s_txt_bus, to_x10(adc.bus_voltage), 10, "V");
    } else {
        strcpy(s_txt_bus,"NO DATA");
    }
    fmt_counter(s_txt_bus_raw,adc.bus_raw);
    fmt_counter(s_txt_bus_count,adc.bus_samples);
    fmt_counter(s_txt_frame_count,adc.frame_samples);
    snprintf(s_txt_adc1_error, sizeof(s_txt_adc1_error), "0x%lX", (unsigned long)adc.adc1_error);
    snprintf(s_txt_adc2_error, sizeof(s_txt_adc2_error), "0x%lX", (unsigned long)adc.adc2_error);
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
    {s_txt_system, KK_UI_MENU_PAGE, UI_PAGE_SYSTEM},
    {"Curr Loop",  KK_UI_MENU_BOOL,   BOOL_CURRENT_LOOP},
    {"Pos Speed",  KK_UI_MENU_INT,    INT_POSITION_SPEED},
    {"Speed Limit", KK_UI_MENU_INT,   INT_SPEED_LIMIT},
    {"Run Steps",  KK_UI_MENU_PAGE, UI_PAGE_RUN_STEPS},
    {"M1 Protection",KK_UI_MENU_PAGE,UI_PAGE_PROTECTION},
    {"Motor",      KK_UI_MENU_PAGE, UI_PAGE_MOTOR},
    {"Status LED", KK_UI_MENU_BOOL, BOOL_LED},
    {"LED Bright", KK_UI_MENU_INT, INT_LED_BRIGHT},
    {s_txt_save_state, KK_UI_MENU_ACTION, EVT_SAVE_STATUS},
    {"Restore Defaults", KK_UI_MENU_CONFIRM, CONFIRM_RESET},
};

static const KK_UI_MenuItem motor_items[] = {
    {"2804", KK_UI_MENU_CONFIRM, CONFIRM_MOTOR_2804},
    {"4015", KK_UI_MENU_CONFIRM, CONFIRM_MOTOR_4015},
};

static const KK_UI_MenuItem system_items[] = {
    {s_txt_work_menu, KK_UI_MENU_PAGE, UI_PAGE_WORK},
    {s_txt_axes[0], KK_UI_MENU_ACTION, EVT_FOCUS_M1},
    {s_txt_axes[1], KK_UI_MENU_ACTION, EVT_FOCUS_M2},
    {"HSO Feedback", KK_UI_MENU_BOOL, BOOL_HSO},
    {"Gear Ratio", KK_UI_MENU_INT, INT_GEAR_RATIO},
    {"Gear Offset", KK_UI_MENU_INT, INT_GEAR_OFFSET},
    {"Align / Retry", KK_UI_MENU_ACTION, EVT_ALIGN},
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
    {"System",KK_UI_MENU_PAGE,UI_PAGE_DIAGNOSTICS},
    {s_txt_axes[0],KK_UI_MENU_ACTION,EVT_FOCUS_M1},
    {s_txt_axes[1],KK_UI_MENU_ACTION,EVT_FOCUS_M2},
};
static const KK_UI_MenuItem protection_items[] = {
    {"IBUS Trip",KK_UI_MENU_INT,INT_IBUS_LIMIT},
    {"Clear IBUS Trip",KK_UI_MENU_CONFIRM,CONFIRM_IBUS_CLEAR},
    {s_txt_ibus_state,KK_UI_MENU_ACTION,EVT_IBUS_STATUS},
};

static const KK_UI_MenuPage menu_pages[] = {
    {s_txt_settings, settings_items, settings_states,
     sizeof(settings_items) / sizeof(settings_items[0])},
    {s_txt_motor_menu, motor_items, NULL,
     sizeof(motor_items) / sizeof(motor_items[0])},
    {"System", system_items, system_states, sizeof(system_items)/sizeof(system_items[0])},
    {"Work Mode", work_items, NULL, sizeof(work_items)/sizeof(work_items[0])},
    {s_txt_status,status_items,NULL,sizeof(status_items)/sizeof(status_items[0])},
    {"M1 Protection",protection_items,protection_states,sizeof(protection_items)/sizeof(protection_items[0])},
};

static const KK_UI_InfoRow drive_rows[] = {
    {"Drive",   s_txt_drive},
    {"Output",  s_txt_output},
    {"Feedback",s_txt_feedback},
    {"Bus",     s_txt_bus},
    {"Motor",   s_txt_motor},
    {"Mode",    s_txt_mode},
    {"VREF",    s_txt_reference},
    {"RefState",s_txt_ref_state},
    {"Work",s_txt_work},
    {"Axis",s_txt_axis},
    {"Aligned", s_txt_align},
    {"Run Stop",s_txt_run_stop},
    {"Stop Age",s_txt_stop_age},
    {"Stop Err",s_txt_stop_errors},
    {"Sync Err",s_txt_sync},
};

static const KK_UI_InfoRow motion_rows[] = {
    {"Target",  s_txt_target},
    {s_txt_target_unit,s_txt_target_aux},
    {"Speed",   s_txt_speed},
    {"Speed rpm",s_txt_rpm},
    {"Iq",      s_txt_iq},
    {"Id",      s_txt_id},
    {"Phase A", s_txt_ia},
    {"Phase B", s_txt_ib},
    {"Elec",    s_txt_elec},
    {"Mech",    s_txt_mech},
    {"Elec 0-360",s_txt_elec_deg}, {"Mech 0-360",s_txt_mech_deg},
};

static const KK_UI_InfoRow encoder_rows[] = {
    {s_enc_labels[0], s_txt_enc},
    {s_enc_labels[1], s_txt_enc_crc},
    {s_enc_labels[2], s_txt_enc_field},
    {s_enc_labels[3], s_txt_enc_track},
    {s_enc_labels[4], s_txt_enc_spi},
    {s_enc_labels[5], s_txt_enc_frame},
    {s_enc_labels[6], s_txt_enc_age},
    {s_enc_labels[7], s_txt_enc_last},
    {s_enc_labels[8], s_txt_raw},
    {s_enc_labels[9], s_txt_err},
    {s_enc_labels[10], s_txt_enc_zero},
    {s_enc_labels[11], s_txt_enc_zpre},
    {s_enc_labels[12], s_txt_enc_zpost},
    {s_enc_labels[13], s_txt_enc_ztime},
    {"E2 Probe",s_txt_enc2},
    {"E2 Total",s_txt_enc2_err},
};

static const KK_UI_InfoRow adc_rows[] = {
    {"M1 IBUS",s_txt_ibus_state},
    {"IBUS Limit",s_txt_ibus_limit}, {"IBUS DAC",s_txt_ibus_code},
    {"IBUS Thresh",s_txt_ibus_voltage}, {"IBUS Trips",s_txt_ibus_trips}, {"COMP4",s_txt_ibus_pin},
    {"I Trip",  s_txt_current_trip},
    {"IU R/Z",  s_txt_current_adc[0]},
    {"IV R/Z",  s_txt_current_adc[1]},
    {"IW R/Z",  s_txt_current_adc[2]},
    {"ChipRev", s_txt_adc_revision},
    {"BusRaw",  s_txt_bus_raw},
    {"BusCnt",  s_txt_bus_count},
    {"FrameCnt",s_txt_frame_count},
    {"ADC1Err", s_txt_adc1_error},
    {"ADC2Err", s_txt_adc2_error},
    {"Ref R/C", s_txt_ref_raw},
    {"RefCnt",  s_txt_ref_count},
    {"RefErr",  s_txt_ref_errors},
};

static const KK_UI_InfoRow hso_rows[] = {
    {"HSO",     s_txt_hso},
    {"H Min",s_txt_hso_min}, {"H Max",s_txt_hso_max},
    {"H Zero","Stop / coast"}, {"H Reverse","Stop first"},
    {"H Gate",  s_txt_hso_gate},
    {"H Qual",  s_txt_hso_qual},
    {"H Reject",s_txt_hso_reject},
    {"H Reset",s_txt_hso_resets},
    {"H R Deg",s_txt_hso_rangle},
    {"H R Hz",s_txt_hso_rfreq},
    {"H R EMF%",s_txt_hso_remf},
    {"H R Drift",s_txt_hso_rdrift},
    {"H Off mV",s_txt_hso_offset},
    {"H Lim mV",s_txt_hso_olimit},
    {"H Lost",  s_txt_hso_lost},
    {"H Speed", s_txt_hso_speed},
    {"H Error", s_txt_hso_error},
    {"H Flux",  s_txt_hso_flux},
    {"H Actual",s_txt_hso_flux_actual},
    {"H Time",  s_txt_hso_time},
};

static const KK_UI_InfoRow diagnostics_rows[] = {
    {"Cmd",s_txt_command},
    {"M1 Model",s_txt_m1},
    {"M2 Model",s_txt_m2},
    {"Ctrl Max",s_txt_control_time},
    {"Save Bus",s_txt_save},
    {"Save Last",s_txt_save_result},
    {"Flash Err",s_txt_flash_error},
    {"SPI2 Fault",s_txt_spi2_fault},
    {"SPI2 Reset",s_txt_spi2_resets},
    {"KeyRpt",  s_txt_repeat},
    {"DispErr", s_txt_display_errors},
};

static const KK_UI_InfoRow about_rows[] = {
    {"FW",      "UI-R59-STOP"},
    {"MCU",     "STM32G431"},
    {"Motor",   s_txt_motor},
    {"Control", "10kHz FOC"},
    {"Encoder", "MT6701"},
    {"LCD",     "ST7735S"},
    {"UI",      "KK_UI"},
    {"Keys",    "6 keys"},
    {"Flash",   "W25Q64"},
};

static const KK_UI_InfoPage info_pages[] = {
    {s_txt_groups[0], drive_rows, sizeof(drive_rows) / sizeof(drive_rows[0])},
    {"About",  about_rows,  sizeof(about_rows) / sizeof(about_rows[0])},
    {s_txt_groups[1],motion_rows,sizeof(motion_rows)/sizeof(motion_rows[0])},
    {s_txt_groups[2],encoder_rows,sizeof(encoder_rows)/sizeof(encoder_rows[0])},
    {s_txt_groups[3],adc_rows,sizeof(adc_rows)/sizeof(adc_rows[0])},
    {s_txt_groups[4],hso_rows,sizeof(hso_rows)/sizeof(hso_rows[0])},
    {"System",diagnostics_rows,sizeof(diagnostics_rows)/sizeof(diagnostics_rows[0])},
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
    [BOOL_CURRENT_LOOP] = {s_txt_loop_title, &s_current_loop_display, EVT_CURRENT_LOOP_CHANGED},
    [BOOL_LED]          = {"Status LED",   &s_led_enable,   EVT_LED_CHANGED},
};

static const KK_UI_ConfirmDesc confirm_descs[] = {
    [CONFIRM_IBUS_CLEAR] = {"Clear M1 IBUS?\nOutput stays OFF",EVT_IBUS_CLEAR,0U},
    [CONFIRM_RESET] = {"Reset all\nsettings?", EVT_RESET_CONFIRMED, EVT_RESET_CANCELLED},
    [CONFIRM_MOTOR_2804] = {"Use 2804?\nStop and align", EVT_MOTOR_2804_CONFIRMED, 0U},
    [CONFIRM_MOTOR_4015] = {"Use 4015?\nStop and align", EVT_MOTOR_4015_CONFIRMED, 0U},
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
    .custom_page_count = 2U,
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
    strcpy(s_txt_run_stop, "--");
    strcpy(s_txt_stop_age, "--");
    strcpy(s_txt_stop_errors, "--");
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
    MotorControl_Stop(s_view_axis);
    if (s_view_axis==MOTOR_AXIS_1) JerryFOC_useCurrentLoop(0U);
    s_run_active = false;
    __set_PRIMASK(primask);
}

static int32_t *run_target(bool draft);

static bool apply_run_target(void)
{
    if (!s_run_active) return false;
    ui_store_current();
    float target=(float)*run_target(true)/(s_run_mode==JERRYFOC_MODE_VELOCITY ? 10.0f : 100.0f);
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

static void ui_store_current(void)
{
    AppSettings *st=AppStorage_Get();
    uint32_t flags=settings_flags();
    if (s_view_axis==MOTOR_AXIS_1) {
        st->flags=flags; st->speed_x10=s_speed_x10;
        st->voltage_x100=s_voltage_x100; st->position_x100=s_position_x100;
        st->speed_step_x10=s_run_steps[1]; st->torque_step_x100=s_run_steps[0]; st->position_step_x100=s_run_steps[2];
    } else {
        st->axis2_flags=flags; st->axis2_speed_x10=s_speed_x10;
        st->axis2_voltage_x100=s_voltage_x100; st->axis2_position_x100=s_position_x100;
        st->axis2_speed_step_x10=s_run_steps[1]; st->axis2_torque_step_x100=s_run_steps[0]; st->axis2_position_step_x100=s_run_steps[2];
    }
    st->flags=(st->flags & ~(APP_FLAG_LED_ENABLE|APP_FLAG_MOTOR_4015)) |
        (s_led_enable ? APP_FLAG_LED_ENABLE : 0U) |
        (MotorParams_GetAxisProfile(MOTOR_AXIS_1)==MOTOR_PROFILE_4015 ? APP_FLAG_MOTOR_4015 : 0U);
    st->led_brightness=(uint32_t)s_led_brightness;
}

static bool request_settings_save(void)
{
    AppSettings *st = AppStorage_Get();
    ui_store_current();
    st->speed_x10 = s_view_axis==MOTOR_AXIS_1 ? s_speed_x10 : st->speed_x10;
    if (!AppStorage_IsReady()) {
        s_pending_toast = AppStorage_SaveErrorText();
        return true;
    }
    return AppStorage_RequestSave();
}

static void sync_from_storage(void)
{
    AppSettings copy=*AppStorage_Get();
    if (s_view_axis==MOTOR_AXIS_2) {
        copy.flags=copy.axis2_flags | (copy.flags & APP_FLAG_LED_ENABLE);
        copy.speed_x10=copy.axis2_speed_x10; copy.voltage_x100=copy.axis2_voltage_x100;
        copy.position_x100=copy.axis2_position_x100;
        copy.speed_step_x10=copy.axis2_speed_step_x10;
        copy.torque_step_x100=copy.axis2_torque_step_x100;
        copy.position_step_x100=copy.axis2_position_step_x100;
    }
    AppSettings *st=&copy;

    /* 防止 W25Q 中的旧记录/损坏记录把绑定值带到编辑器范围之外 */
    s_speed_limit = (int32_t)((st->flags & APP_SPEED_LIMIT_MASK) >> APP_SPEED_LIMIT_SHIFT);
    if (s_speed_limit < 1 || s_speed_limit > APP_SPEED_LIMIT_MAX)
        s_speed_limit = APP_SPEED_LIMIT_DEFAULT;
    int32_t profile_max = (int32_t)MotorParams_GetAxis(s_view_axis)->max_speed_mech;
    int_bindings[INT_SPEED_LIMIT].maximum = profile_max;
    s_speed_limit = clamp_i32(s_speed_limit, 1, profile_max);
    s_speed_x10 = clamp_i32(st->speed_x10, -s_speed_limit * 10, s_speed_limit * 10);
    int32_t voltage_max = to_x100(MotorParams_GetAxis(s_view_axis)->vel_limit);
    s_voltage_x100 = clamp_i32(st->voltage_x100, -voltage_max, voltage_max);
    s_position_x100 = clamp_i32(st->position_x100, -314, 314);
    s_led_brightness = clamp_i32((int32_t)st->led_brightness, 0, 100);
    s_current_loop = (st->flags & APP_FLAG_CURRENT_LOOP) != 0U;
    s_led_enable = (st->flags & APP_FLAG_LED_ENABLE) != 0U;
    s_position_speed = (int32_t)AppSettings_GetPositionSpeed(st->flags);
    if (s_position_speed < 1 || s_position_speed > APP_SPEED_LIMIT_MAX) s_position_speed = 30;
    update_position_speed_limit();
    s_run_steps[JERRYFOC_MODE_VELOCITY] = st->speed_step_x10;
    s_run_steps[JERRYFOC_MODE_TORQUE] = st->torque_step_x100;
    s_run_steps[JERRYFOC_MODE_POSITION] = st->position_step_x100;
    for (unsigned i = 0U; i < 3U; ++i)
        if (s_run_steps[i] < 1 || s_run_steps[i] > 100) s_run_steps[i] = 5;
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

static bool select_motor(MotorProfile profile)
{
    if (profile == MotorParams_GetAxisProfile(s_view_axis)) return true;
    stop_run();
    if (!MotorControl_SelectProfile(s_view_axis,profile)) {
        s_pending_toast = "Stop/check encoder";
        return false;
    }
    /* Never carry a 2804 target/current-mode command into the other motor. */
    s_speed_x10 = s_run_speed_x10 = 0;
    s_voltage_x100 = s_run_voltage_x100 = 0;
    s_position_x100 = s_run_position_x100 = 0;
    s_run_current_x100 = s_confirmed_current_x100 = 0;
    s_run_position_origin = 0.0f;
    s_run_mode = JERRYFOC_MODE_TORQUE;
    clear_run_stop();
    s_current_loop = false;
    s_speed_limit = APP_SPEED_LIMIT_DEFAULT;
    int_bindings[INT_SPEED_LIMIT].maximum = (int32_t)MotorParams_GetAxis(s_view_axis)->max_speed_mech;
    s_position_speed = (int32_t)MotorParams_GetAxis(s_view_axis)->pos_limit;
    update_position_speed_limit();
    ui_store_current();
    mark_settings_dirty(HAL_GetTick());
    refresh_texts();
    s_pending_toast = "Motor set; align";
    KK_UI_Invalidate();
    return true;
}

static void run_mode_step(int delta)
{
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
    for (unsigned a=0;a<2;++a) {
        s_view_axis=(MotorAxis)a; s_profiles[a]=MotorParams_GetAxisProfile((MotorAxis)a); sync_from_storage();
        s_run_mode=JERRYFOC_MODE_TORQUE; s_run_active=false;
        s_run_speed_x10=s_speed_x10; s_run_voltage_x100=s_voltage_x100;
        s_run_position_x100=s_position_x100;
        s_run_current_x100=s_confirmed_current_x100=0; s_run_position_origin=0;
        save_view();
    }
    load_view(MotorControl_GetFocus());
    sync_from_storage();
    s_ratio_x100=AppStorage_Get()->gear_ratio_x100;
    if (!s_ratio_x100) s_ratio_x100=100;
    s_offset_x100=AppStorage_Get()->gear_offset_x100;

    if (s_view_axis==MOTOR_AXIS_1) JerryFOC_useCurrentLoop(s_current_loop ? 1U : 0U);

    s_run_mode = ui_state().mode;
    s_run_speed_x10 = s_speed_x10;
    if (MotorControl_HsoAllowed()) s_run_speed_x10=hso_draft(s_run_speed_x10);
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
            strcpy(s_txt_run_stop,"GLOBAL STOP");
            strcpy(s_txt_stop_age,"--"); strcpy(s_txt_stop_errors,"--");
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
        s_speed_x10=s_run_speed_x10=s_voltage_x100=s_run_voltage_x100=0;
        s_position_x100=s_run_position_x100=s_run_current_x100=s_confirmed_current_x100=0;
        s_run_active=false; s_current_loop=false; s_run_mode=JERRYFOC_MODE_TORQUE;
        s_speed_limit=APP_SPEED_LIMIT_DEFAULT;
        s_position_speed=(int32_t)MotorParams_GetAxis((MotorAxis)a)->pos_limit;
        ui_store_current(); save_view(); load_view(old);
        int_bindings[INT_SPEED_LIMIT].maximum=(int32_t)MotorParams_GetAxis(old)->max_speed_mech;
        mark_settings_dirty(now_ms); KK_UI_Invalidate();
    }
    if (!MotorControl_ModeAllowed(s_run_mode))
        s_run_mode=MotorControl_ModeAllowed(JERRYFOC_MODE_VELOCITY) ? JERRYFOC_MODE_VELOCITY : JERRYFOC_MODE_POSITION;
    KK_UI_ItemState hso_state=MotorControl_GetWork()==MOTOR_WORK_SINGLE_M1 ? KK_UI_ITEM_NORMAL : KK_UI_ITEM_DISABLED;
    if (((system_states[0]>>6U) & 3U)!=(unsigned)hso_state)
        (void)KK_UI_SetMenuItemState(UI_PAGE_SYSTEM,3U,hso_state);
    KK_UI_ItemState loop_state=s_view_axis==MOTOR_AXIS_1 && MotorControl_HsoAllowed() ? KK_UI_ITEM_DISABLED : KK_UI_ITEM_NORMAL;
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
            snprintf(s_txt_run_stop, sizeof(s_txt_run_stop), "%s", reason);
            snprintf(s_txt_stop_age, sizeof(s_txt_stop_age), "E%u %u", (unsigned)stop_axis+1U, enc.age_cycles);
            snprintf(s_txt_stop_errors, sizeof(s_txt_stop_errors), "E%u %lu", (unsigned)stop_axis+1U, (unsigned long)enc.errors);
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
        case EVT_CURRENT_LOOP_CHANGED:
            if (s_view_axis==MOTOR_AXIS_1 && MotorControl_HsoAllowed()) {
                s_current_loop_display=true; break;
            }
            s_current_loop=s_current_loop_display;
            stop_run();
            s_run_current_x100 = s_confirmed_current_x100 = 0;
            ui_store_current();
            mark_settings_dirty(HAL_GetTick());
            break;

        case EVT_POSITION_SPEED_CHANGED:
        case EVT_LED_CHANGED:
            ui_store_current();
            AppStorage_Get()->led_brightness = (uint32_t)s_led_brightness;
            mark_settings_dirty(HAL_GetTick());
            break;

        case EVT_SPEED_LIMIT_CHANGED: {
            update_position_speed_limit();
            int32_t limit = s_speed_limit * 10;
            s_speed_x10 = clamp_i32(s_speed_x10, -limit, limit);
            s_run_speed_x10 = clamp_i32(s_run_speed_x10, -limit, limit);
            ui_store_current();
            mark_settings_dirty(HAL_GetTick());
            KK_UI_Invalidate();
            break;
        }

        case EVT_RUN_STEPS_CHANGED:
            for (unsigned i = 0U; i < 3U; ++i) s_run_steps[i] = s_step_drafts[i];
            ui_store_current();
            mark_settings_dirty(HAL_GetTick());
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
            for (unsigned a=0;a<2;++a) {
                s_view_axis=(MotorAxis)a; sync_from_storage();
                s_run_speed_x10=s_speed_x10; s_run_voltage_x100=s_voltage_x100;
                s_run_position_x100=s_position_x100; s_run_current_x100=s_confirmed_current_x100=0;
                s_run_active=false; s_run_mode=JERRYFOC_MODE_TORQUE; s_run_position_origin=0;
                save_view(); s_profiles[a]=MOTOR_PROFILE_2804;
            }
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
        s_run_mode=MotorControl_ModeAllowed(JERRYFOC_MODE_VELOCITY) ? JERRYFOC_MODE_VELOCITY : JERRYFOC_MODE_POSITION;
    stop_run();
    s_run_speed_x10 = s_speed_x10;
    if (MotorControl_HsoAllowed()) s_run_speed_x10=hso_draft(s_run_speed_x10);
    s_run_voltage_x100 = s_voltage_x100;
    s_run_position_x100 = s_position_x100;
    s_run_current_x100 = s_confirmed_current_x100 = 0;
    s_run_position_origin = ui_state().position;
    s_last_live_at = HAL_GetTick();
}

void KK_UI_CustomOnLeave(KK_UI_PageId page)
{
    if (page == UI_PAGE_RUN) {
        s_run_lr_key=0U;
        if (s_run_active || views[1U-s_view_axis].run_active) MotorControl_StopAll();
        s_run_active=views[0].run_active=views[1].run_active=false;
    }
}

void KK_UI_CustomOnInput(KK_UI_PageId page, KK_UI_InputEvent event)
{
    s_last_input_at = HAL_GetTick();
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
            if (MotorControl_IsGroupActive()) stop_run();
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
        } else if (MotorControl_HsoAllowed() &&
                   (hso_draft(s_run_speed_x10)!=s_run_speed_x10 ||
                    (MotorSampling_IsAxisOutputRequested(s_view_axis) && state.target_velocity*(float)s_run_speed_x10<0.0f))) {
            (void)KK_UI_ShowToast("Stop / check range",1200U);
        } else {
            /* 运行中确认新位置时保留本次原点，停止后启动才重新捕获。 */
            if (!s_run_active) s_run_position_origin = state.position;
            s_run_active = true;
            clear_run_stop();
            if (!apply_run_target()) {
                MotorControl_Stop(s_view_axis);
                s_run_active=false;
                (void)KK_UI_ShowToast(MotorControl_CommandText(),1200U);
                KK_UI_Invalidate(); return;
            }
            *run_target(false) = *run_target(true);
            if (MotorControl_HsoAllowed() && s_run_mode==JERRYFOC_MODE_VELOCITY && *run_target(true)==0)
                s_run_active=false; /* Sensorless zero requests coast, not zero-speed holding. */
            ui_store_current();
            if (changed) mark_settings_dirty(HAL_GetTick());
        }
        KK_UI_Invalidate();
        return;
    }
    if (event.action != KK_UI_INPUT_UP && event.action != KK_UI_INPUT_DOWN) return;
    /* PRESS / REPEAT 共用同一条加减路径，三种模式均按当前步长修改草稿。 */
    int32_t step = s_run_steps[s_run_mode] * (int32_t)event.steps;
    if (event.action == KK_UI_INPUT_DOWN) step = -step;
    int32_t limit = s_run_mode == JERRYFOC_MODE_VELOCITY ? s_speed_limit * 10 :
                    s_run_mode == JERRYFOC_MODE_POSITION ? 314 :
                    to_x100(MotorParams_GetAxis(s_view_axis)->vel_limit);
    if (s_run_mode == JERRYFOC_MODE_TORQUE && s_current_loop)
        limit = to_x100(MotorParams_GetAxis(s_view_axis)->rated_current);
    int32_t *draft = run_target(true);
    if (MotorControl_HsoAllowed() && s_run_mode==JERRYFOC_MODE_VELOCITY) {
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
    } else *draft = clamp_i32(*draft + step, -limit, limit);
    KK_UI_Invalidate();
}

bool KK_UI_CustomOnTick(KK_UI_PageId page, uint32_t now_ms)
{
    if (page != UI_PAGE_RUN || now_ms - s_last_live_at < 100U) return false;
    s_last_live_at = now_ms;
    return true;
}

void KK_UI_CustomOnDraw(KK_UI_PageId page, int16_t x_offset,
                        int16_t clip_x, uint16_t clip_width)
{
    if (page != UI_PAGE_RUN && page != UI_PAGE_RUN_STEPS) return;
    const MotorAxisSnapshot motor_state=ui_state();
    const int16_t W = (int16_t)OLED_GetWidth();
    const int16_t H = (int16_t)OLED_GetHeight();
    char buf[40];
    char v[16];
    OLED_SetClipWindow(clip_x, 0, clip_width, (uint16_t)H);
    OLED_SetDrawMode(OLED_DRAW_SET);
    if (page == UI_PAGE_RUN_STEPS) {
        /* Compact header keeps its existing baseline; Home uses the restored
         * family line box, independently of which title glyphs were retained. */
        OLED_SetFontRefHeight(OLED_FONT_REF_TEXT);
        OLED_SetFont(ui_font_title);
        OLED_DrawUTF8((int16_t)(4 + x_offset), 1, s_txt_steps);
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
            snprintf(buf, sizeof(buf), "%s %s", v, unit);
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
    const char *bus=!strcmp(s_txt_bus,"NO DATA") ? "--V" : s_txt_bus;
    OLED_DrawUTF8((int16_t)(W-4-OLED_GetUTF8Width(bus)+x_offset),1,bus);
    if (MotorControl_IsHandFollow()) {
        unsigned master=MotorControl_GetWork()==MOTOR_WORK_MASTER_M2 ? 2U : 1U;
        bool active=MotorControl_IsGroupActive();
        snprintf(buf,sizeof(buf),"Hand M%u>M%u ENC",master,3U-master);
        OLED_DrawUTF8((int16_t)(4+x_offset),1,buf);
        OLED_DrawHLine(x_offset,14,(uint16_t)W);
        OLED_DrawUTF8((int16_t)(4+x_offset),19,"Turn master by hand");
        fmt_fixed(v,sizeof(v),to_x100(MotorControl_GroupError()),100);
        snprintf(buf,sizeof(buf),"Error %s rad",v);
        OLED_DrawUTF8((int16_t)(4+x_offset),35,buf);
        OLED_DrawUTF8((int16_t)(4+x_offset),51,active ? "RUN: Following" :
            strcmp(s_txt_run_stop,"--") ? s_txt_run_stop : "STOP: OK to set zero");
        OLED_DrawHLine(x_offset,65,(uint16_t)W);
        OLED_DrawUTF8((int16_t)(4+x_offset),67,active ? "OK:Stop K2:Back" : "OK:Start K2:Back");
        OLED_ResetClipWindow(); return;
    }
    bool hso=s_view_axis==MOTOR_AXIS_1 && MotorControl_HsoAllowed();
    bool independent=MotorControl_GetWork()==MOTOR_WORK_INDEPENDENT;
    bool starting=hso && StHso_IsStarting();
    const char *mode=s_run_mode==JERRYFOC_MODE_TORQUE ? (s_current_loop ? "Iq(A)" : "Uq(V)") : mode_name(s_run_mode);
    snprintf(buf,sizeof(buf),"M%u %s %s",(unsigned)s_view_axis+1U,mode,hso ? "HSO" : "ENC");
    OLED_DrawUTF8((int16_t)(4+x_offset),1,buf);
    OLED_DrawHLine(x_offset,14,(uint16_t)W);
    bool changed=run_target_changed();
    const char *state=changed ? (s_run_active ? "RUN*" : "EDIT") : (s_run_active ? "RUN" : "STOP");
    const char *unit=s_run_mode==JERRYFOC_MODE_VELOCITY ? "rad/s" :
        s_run_mode==JERRYFOC_MODE_POSITION ? "rad" : s_current_loop ? "A" : "V";
    int scale=s_run_mode==JERRYFOC_MODE_VELOCITY ? 10 : 100;
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
    else if (!s_run_active && strcmp(s_txt_run_stop,"--")) fault=s_txt_run_stop;
    else if (!motor_state.healthy || !motor_state.aligned)
        fault=motor_state.healthy ? "NOT ALIGNED" : MotorControl_EncoderLostText(s_view_axis);
    if (fault) snprintf(buf,sizeof(buf),"%s",fault);
    else {
        int32_t actual=s_run_mode==JERRYFOC_MODE_POSITION ?
            to_x100(motor_state.position-s_run_position_origin) : to_x10(motor_state.velocity);
        fmt_fixed(v,sizeof(v),actual,s_run_mode==JERRYFOC_MODE_POSITION ? 100 : 10);
        if (changed) {
            char applied[16]; fmt_fixed(applied,sizeof(applied),*run_target(false),scale);
            snprintf(buf,sizeof(buf),"%s %s Set %s",starting ? "Ref" : "Act",v,applied);
        } else if (hso && *run_target(true)==0) snprintf(buf,sizeof(buf),"Act %s 0=Coast",v);
        else if (s_run_mode!=JERRYFOC_MODE_TORQUE) {
            char aux[16]; fmt_run_aux(aux,actual);
            snprintf(buf,sizeof(buf),"%s %s %s",starting ? "Ref" : "Act",v,aux);
        } else snprintf(buf,sizeof(buf),"Act %s Max %ld",v,(long)s_speed_limit);
    }
    OLED_DrawUTF8((int16_t)(4+x_offset),38,buf);
    if (independent) {
        /* Read both actual output requests; a failed/unviewed axis must not
         * remain labelled RUN merely because its UI view was cached. */
        for (unsigned a=0;a<2;++a) {
            snprintf(buf,sizeof(buf),"M%u:%s%s",a+1U,
                MotorSampling_IsAxisOutputRequested((MotorAxis)a) ? "RUN" : "STOP",
                a==(unsigned)s_view_axis && changed ? "*" : "");
            int16_t x=(int16_t)(4+78*a+x_offset);
            if (a==(unsigned)s_view_axis) {
                OLED_DrawRBox((int16_t)(x-1),50,74,14,2);
                OLED_SetDrawMode(OLED_DRAW_CLEAR);
            }
            OLED_DrawUTF8(x,51,buf); OLED_SetDrawMode(OLED_DRAW_SET);
        }
    } else if (hso) {
        StHsoSnapshot snapshot; StHso_GetSnapshot(&snapshot);
        snprintf(buf,sizeof(buf),"%s HSO %s",state,StHso_StateText(snapshot.state));
        OLED_DrawUTF8((int16_t)(4+x_offset),51,buf);
    } else if (!fault) {
        if (MotorControl_IsCoupled()) {
            MotorAxisSnapshot other; MotorControl_GetAxis((MotorAxis)(1U-s_view_axis),&other);
            fmt_fixed(v,sizeof(v),to_x10(other.velocity),10);
            snprintf(buf,sizeof(buf),"%s M%u %s",state,(unsigned)(2U-s_view_axis),v);
        } else {
            fmt_fixed(v,sizeof(v),to_x100(motor_state.iq),100);
            snprintf(buf,sizeof(buf),"%s Iq %s A",state,v);
        }
        if (!MotorControl_IsCoupled() && s_run_mode!=JERRYFOC_MODE_TORQUE) {
            fmt_run_aux(v,*run_target(true));
        } else {
            char step[10]; fmt_fixed(step,sizeof(step),s_run_steps[s_run_mode],scale);
            snprintf(v,sizeof(v),"Step %s",step);
        }
        int16_t right=(int16_t)(W-4-OLED_GetUTF8Width(v)+x_offset);
        if (4+x_offset+OLED_GetUTF8Width(buf)+4>right) strcpy(buf,state);
        OLED_DrawUTF8((int16_t)(4+x_offset),51,buf);
        OLED_DrawUTF8(right,51,v);
    } else OLED_DrawUTF8((int16_t)(4+x_offset),51,state);
    OLED_DrawHLine(x_offset,65,(uint16_t)W);
    const char *action=hso && s_run_speed_x10==0 ? "Stop" :
        s_run_active ? (changed ? "Apply" : "Stop") : "Start";
    snprintf(buf,sizeof(buf),independent ? "OK:%s LR hold K2:Back" : "OK:%s K2:Back",action);
    OLED_DrawUTF8((int16_t)(4+x_offset),67,buf);
    OLED_ResetClipWindow();
}
