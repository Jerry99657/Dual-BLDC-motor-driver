#include "ui_app.h"
#include "board_diag.h"
#include "app_control.h"

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
#include "ui_resources_packed.h"
#include "motor_control.h"
#include "ui_input.h"
#include "protocol.h"
#include "motor_sixstep.h"
#include "motor_detect.h"
#include "motor_math.h"

/* Shared application literals also serve menus, dialogs and custom pages. */
#define UI_STRINGS(X) \
    X(text_drive,"Drive") \
    X(text_output,"Output") \
    X(text_feedback,"Feedback") \
    X(text_bus,"Bus") \
    X(text_motor,"Motor") \
    X(text_mode,"Mode") \
    X(text_vref,"VREF") \
    X(text_refstate,"RefState") \
    X(text_work,"Work") \
    X(text_axis,"Axis") \
    X(text_aligned,"Aligned") \
    X(text_run_stop,"Run Stop") \
    X(text_stop_age,"Stop Age") \
    X(text_stop_err,"Stop Err") \
    X(text_sync_err,"Sync Err") \
    X(text_6s_zc,"6S ZC") \
    X(text_6step_duty,"6Step Duty") \
    X(text_6step_emf,"6Step EMF") \
    X(text_6step_speed,"6Step Speed") \
    X(text_6s_hold,"6S Hold") \
    X(text_6s_start_emf,"6S Start EMF") \
    X(text_6s_start_i,"6S Start I") \
    X(text_6s_ramp,"6S Ramp Hz/s") \
    X(text_6s_cause,"6S Cause") \
    X(text_6s_sample,"6S Sample") \
    X(text_input,"Input") \
    X(text_target,"Target") \
    X(text_speed,"Speed") \
    X(text_speed_rpm,"Speed rpm") \
    X(text_iq,"Iq") \
    X(text_id,"Id") \
    X(text_phase_a,"Phase A") \
    X(text_phase_b,"Phase B") \
    X(text_elec,"Elec") \
    X(text_mech,"Mech") \
    X(text_elec_0_360,"Elec 0-360") \
    X(text_mech_0_360,"Mech 0-360") \
    X(text_e2_probe,"E2 Probe") \
    X(text_e2_total,"E2 Total") \
    X(text_m1_ibus,"M1 IBUS") \
    X(text_ibus_limit,"IBUS Limit") \
    X(text_ibus_dac,"IBUS DAC") \
    X(text_ibus_thresh,"IBUS Thresh") \
    X(text_ibus_trips,"IBUS Trips") \
    X(text_comp4,"COMP4") \
    X(text_i_trip,"I Trip") \
    X(text_iu_r_z,"IU R/Z") \
    X(text_iv_r_z,"IV R/Z") \
    X(text_iw_r_z,"IW R/Z") \
    X(text_chiprev,"ChipRev") \
    X(text_busraw,"BusRaw") \
    X(text_buscnt,"BusCnt") \
    X(text_framecnt,"FrameCnt") \
    X(text_adc1err,"ADC1Err") \
    X(text_adc2err,"ADC2Err") \
    X(text_ref_r_c,"Ref R/C") \
    X(text_refcnt,"RefCnt") \
    X(text_referr,"RefErr") \
    X(text_hso,"HSO") \
    X(text_h_min,"H Min") \
    X(text_h_max,"H Max") \
    X(text_h_zero,"H Zero") \
    X(text_stop_coast,"Stop / coast") \
    X(text_h_reverse,"H Reverse") \
    X(text_stop_first,"Stop first") \
    X(text_h_gate,"H Gate") \
    X(text_h_qual,"H Qual") \
    X(text_h_reject,"H Reject") \
    X(text_h_reset,"H Reset") \
    X(text_h_r_deg,"H R Deg") \
    X(text_h_r_hz,"H R Hz") \
    X(text_h_r_emf,"H R EMF%") \
    X(text_h_r_drift,"H R Drift") \
    X(text_h_off_mv,"H Off mV") \
    X(text_h_lim_mv,"H Lim mV") \
    X(text_h_lost,"H Lost") \
    X(text_h_speed,"H Speed") \
    X(text_h_error,"H Error") \
    X(text_h_flux,"H Flux") \
    X(text_h_actual,"H Actual") \
    X(text_h_time,"H Time") \
    X(text_cmd,"Cmd") \
    X(text_m1_model,"M1 Model") \
    X(text_m2_model,"M2 Model") \
    X(text_ctrl_max,"Ctrl Max") \
    X(text_save_bus,"Save Bus") \
    X(text_save_last,"Save Last") \
    X(text_flash_err,"Flash Err") \
    X(text_spi2_fault,"SPI2 Fault") \
    X(text_spi2_reset,"SPI2 Reset") \
    X(text_keyrpt,"KeyRpt") \
    X(text_disperr,"DispErr") \
    X(text_fw,"FW") \
    X(text_firmware,"UI-R98-BOOT") \
    X(text_clear_diag,"Clear diag") \
    X(text_mcu,"MCU") \
    X(text_stm32g431,"STM32G431") \
    X(text_control,"Control") \
    X(text_10khz_foc,"10kHz FOC") \
    X(text_encoder,"Encoder") \
    X(text_mt6701,"MT6701") \
    X(text_lcd,"LCD") \
    X(text_st7735s,"ST7735S") \
    X(text_ui,"UI") \
    X(text_kk_ui,"KK_UI") \
    X(text_keys,"Keys") \
    X(text_6_keys,"6 keys") \
    X(text_flash,"Flash") \
    X(text_w25q64,"W25Q64") \
    X(text_curr_loop,"Curr Loop") \
    X(text_pos_speed,"Pos Speed") \
    X(text_speed_limit,"Speed Limit") \
    X(text_run_steps,"Run Steps") \
    X(text_m1_protection,"M1 Protection") \
    X(text_status_led,"Status LED") \
    X(text_led_bright,"LED Bright") \
    X(text_restore_defaults,"Restore Defaults") \
    X(text_2804,"2804") \
    X(text_4015,"4015") \
    X(text_esc_auto,"ESC Auto") \
    X(text_m1_detect,"M1 Detect") \
    X(text_sensorless,"Sensorless") \
    X(text_gear_ratio,"Gear Ratio") \
    X(text_gear_offset,"Gear Offset") \
    X(text_align_retry,"Align / Retry") \
    X(text_hso_feedback,"HSO Feedback") \
    X(text_six_step,"Six Step") \
    X(text_motion,"Motion") \
    X(text_adc,"ADC") \
    X(text_system,"System") \
    X(text_ibus_trip,"IBUS Trip") \
    X(text_clear_ibus_trip,"Clear IBUS Trip") \
    X(fmt_u_u,"%u / %u") \
    X(text_rad_s,"rad/s") \
    X(fmt_ld_s,"%ld%s") \
    X(text_position,"Position") \
    X(text_torque,"Torque") \
    X(text_defaults,"Defaults") \
    X(text_pending,"Pending") \
    X(text_saving,"Saving") \
    X(text_wait_bus,"Wait bus") \
    X(text_saved,"Saved") \
    X(text_failed,"Failed") \
    X(fmt_save_s,"Save: %s") \
    X(fmt_m_u_s_s,"M%u %s %s") \
    X(fmt_lu_us,"%lu us") \
    X(text_deferred,"DEFERRED") \
    X(text_available,"AVAILABLE") \
    X(fmt_04x_04x_04x,"%04X/%04X/%04X") \
    X(fmt_lu_lu_02lx_02lx,"%lu/%lu/%02lX/%02lX") \
    X(text_settings,"Settings") \
    X(text_steps,"Steps") \
    X(text_status,"Status") \
    X(fmt_m1_s_hso,"M1 %s HSO") \
    X(text_no_run_range,"No run range") \
    X(fmt_system_s,"System: %s") \
    X(fmt_mode_s,"Mode: %s") \
    X(text_coast,"COAST") \
    X(text_throttle,"Throttle") \
    X(text_target_rpm,"Target rpm") \
    X(text_tgt_0_360,"Tgt 0-360") \
    X(text_target_aux,"Target aux") \
    X(text_error,"ERROR") \
    X(fmt_s_u,"%s %u") \
    X(text_encoder_32,"ENCODER") \
    X(fmt_u_12_c_u_u_c,"%u/12 %c %u/%u.%c") \
    X(text_phase_early,"PHASE EARLY") \
    X(text_phase_late,"PHASE LATE") \
    X(text_period_low,"PERIOD LOW") \
    X(text_period_high,"PERIOD HIGH") \
    X(fmt_u_500_ms,"%u/500 ms") \
    X(fmt_lu_u_ms_lu_20,"%lu/%u ms %lu/20") \
    X(text_lost,"LOST") \
    X(fmt_lu_uwb,"%lu uWb") \
    X(fmt_lu_us_lu,"%lu us / %lu") \
    X(text_aligning,"ALIGNING") \
    X(text_align,"Align") \
    X(text_diagnostics,"Diagnostics") \
    X(text_near,"NEAR") \
    X(text_high,"HIGH") \
    X(text_ibus_tripped,"IBUS: TRIPPED") \
    X(text_ibus_ready,"IBUS: READY") \
    X(text_ibus_not_ready,"IBUS: NOT READY") \
    X(fmt_u_03u_v,"%u.%03u V") \
    X(text_no_data,"NO DATA") \
    X(text_stale,"STALE") \
    X(text_wait,"WAIT") \
    X(fmt_0x_04x,"0x%04X") \
    X(text_derived,"derived") \
    X(text_run_w_derived,"RUN W derived") \
    X(fmt_s_c_u,"%s %c %u") \
    X(text_zero,"ZERO") \
    X(fmt_0x_lx,"0x%lX") \
    X(text_about,"About") \
    X(text_m1_six_step,"M1 Six Step") \
    X(text_work_mode,"Work Mode") \
    X(text_m1_sensorless,"M1 Sensorless") \
    X(text_m1_ibus_trip,"M1 IBUS Trip") \
    X(text_0_01rad,"0.01rad") \
    X(text_led_brightness,"LED Brightness") \
    X(text_position_speed,"Position Speed") \
    X(text_m1_hso_feedback,"M1 HSO Feedback") \
    X(text_m1_6_step,"M1 6 Step") \
    X(text_clear_m1_ibus_output_stays_off,"Clear M1 IBUS?\nOutput stays OFF") \
    X(text_reset_all_settings,"Reset all\nsettings?") \
    X(text_use_2804_stop_and_align,"Use 2804?\nStop and align") \
    X(text_use_4015_stop_and_align,"Use 4015?\nStop and align") \
    X(text_esc_auto_throttle_only,"ESC Auto?\nThrottle only") \
    X(text_back,"Back") \
    X(text_cancel,"Cancel") \
    X(text_notice,"Notice") \
    X(text_wait_for_stop,"Wait for stop") \
    X(text_stop_check_encoder,"Stop/check encoder") \
    X(text_motor_set,"Motor set") \
    X(text_motor_set_align,"Motor set; align") \
    X(text_w25q64_not_found,"W25Q64 not found") \
    X(text_global_stop,"GLOBAL STOP") \
    X(text_ready,"READY") \
    X(text_not_aligned,"NOT ALIGNED") \
    X(text_open_loop,"OPEN LOOP") \
    X(text_mode_changed,"MODE CHANGED") \
    X(text_output_off,"OUTPUT OFF") \
    X(fmt_e_u_u,"E%u %u") \
    X(fmt_e_u_lu,"E%u %lu") \
    X(text_stop_both_first,"Stop both first") \
    X(text_ibus_limit_applied,"IBUS limit applied") \
    X(text_stop_m1_check_ibus,"Stop M1 / check IBUS") \
    X(text_ibus_cleared,"IBUS cleared") \
    X(text_wait_check_ibus,"Wait / check IBUS") \
    X(text_axis_m1,"Axis M1") \
    X(text_axis_m2,"Axis M2") \
    X(text_single_m1_stop,"Single M1 / stop") \
    X(text_stop_ratio_nonzero,"Stop / ratio nonzero") \
    X(text_align_requested,"Align requested") \
    X(text_stop_check_axis,"Stop / check axis") \
    X(text_defaults_restored,"Defaults restored") \
    X(text_remote,"Remote") \
    X(text_save_queued,"Save queued") \
    X(text_hso_reset,"HSO reset") \
    X(text_stop_check_range,"Stop / check range") \
    X(text_raise_speed_limit,"Raise Speed Limit") \
    X(text_stop_before_reverse,"Stop before reverse") \
    X(fmt_i_test_ld_ma,"I test %ld mA") \
    X(fmt_pp_u_s_r_s_ohm,"PP %u%s R %s ohm") \
    X(fmt_ls_lu_uh,"Ls %lu uH") \
    X(fmt_flux_lu_uwb,"Flux %lu uWb") \
    X(fmt_ke_s_v_krpm,"Ke %s V/krpm") \
    X(text_ok_stop_k2_back,"OK:Stop K2:Back") \
    X(text_ok_save_lr_view,"OK:Save LR:View") \
    X(text_ok_start_k2_back,"OK:Start K2:Back") \
    X(text_ok_save,"OK:Save") \
    X(text_l_r_step_k2_cancel,"L/R:Step K2:Cancel") \
    X(fmt_hand_m_u_m_u_enc,"Hand M%u>M%u ENC") \
    X(text_turn_master_by_hand,"Turn master by hand") \
    X(fmt_error_s_rad,"Error %s rad") \
    X(text_run_following,"RUN: Following") \
    X(text_stop_ok_to_set_zero,"STOP: OK to set zero") \
    X(text_iq_a,"Iq(A)") \
    X(text_uq_v,"Uq(V)") \
    X(text_6step,"6STEP") \
    X(text_run,"RUN*") \
    X(text_edit,"EDIT") \
    X(text_stop,"STOP") \
    X(fmt_s_s_set_s,"%s %s Set %s") \
    X(fmt_act_s_0_coast,"Act %s 0=Coast") \
    X(fmt_s_s_s,"%s %s %s") \
    X(fmt_act_s_max_ld,"Act %s Max %ld") \
    X(fmt_m_u_s_s_133,"M%u:%s%s") \
    X(fmt_s_hso_s,"%s HSO %s") \
    X(fmt_s_m_u_s,"%s M%u %s") \
    X(fmt_s_iq_s_a,"%s Iq %s A") \
    X(fmt_step_s,"Step %s") \
    X(text_stop_138,"Stop") \
    X(text_apply,"Apply") \
    X(text_start,"Start") \
    X(fmt_ok_s_lr_hold_k2_back,"OK:%s LR hold K2:Back") \
    X(fmt_ok_s_k2_back,"OK:%s K2:Back")

typedef struct {
#define INFO_STRING_FIELD(name,text) char name[sizeof(text)];
    UI_STRINGS(INFO_STRING_FIELD)
#undef INFO_STRING_FIELD
} UiStrings;
#undef UI_STRINGS
static struct { uint8_t icons[4][128]; UiStrings strings; char axes[2][10],work[7][21],encoder[14][9]; } ui_resources;
#define home_icons ui_resources.icons
#define ui_strings ui_resources.strings
#define s_txt_axes ui_resources.axes
#define s_txt_work_items ui_resources.work
#define s_enc_labels ui_resources.encoder
_Static_assert(sizeof(ui_resources)==UI_RESOURCE_BYTES,"Regenerate the packed UI resources");

/* Trusted, generated Flash data expands once per UI reset, before KK_UI_Init.
 * Back references may overlap; all storage and final pointers are static. */
__attribute__((noinline,noclone)) static void init_ui_resources(void)
{
    const uint8_t *source=ui_resources_packed;
    uint8_t *out=(uint8_t *)&ui_resources,*end=out+sizeof(ui_resources);
    unsigned flags=1U;
    while (out<end) {
        if (flags==1U) flags=*source++ | 256U;
        if (flags&1U) {
            unsigned token=source[0]|((unsigned)source[1]<<8U); source+=2;
            unsigned distance=(token&4095U)+1U;
            unsigned length=(token>>12U)+3U;
            const uint8_t *match=out-distance;
            do { *out++=*match++; } while (--length);
        } else *out++=*source++;
        flags>>=1U;
    }
}

/* Application-owned text shares a base address. Descriptor offsets refer to
 * these permanent buffers; font resources and UI core ownership are unchanged.
 * Frequently addressed short buffers precede colder diagnostic buffers to
 * keep measured Thumb address-setup costs down. This layout is never saved. */
typedef struct {
    char axis[4];
    char motor[8];
    char six_sample[4];
    char command[8];
    char run_stop[24];
    char spi2_resets[8];
    char target_unit[12];
    char stop_age[16];
    char stop_errors[16];
    char ibus_pin[8];
    char input[8];
    char m1[8];
    char m2[8];
    char output[8];
    char enc_zero[16];
    char enc_zpre[16];
    char enc_zpost[16];
    char reference[16];
    char current_trip[24];
    char bus[24];
    char enc2[16];
    char drive[16];
    char enc_ztime[16];
    char adc_revision[16];
    char adc1_error[16];
    char adc2_error[16];
    char flash_error[16];
    char spi2_fault[16];
    char target_aux[24];
    char target[24];
    char sync[20];
    char control_time[20];
    char ibus_state[24];
    char mode[16];
    char save[16];
    char system[24];
    char work_menu[24];
    char save_state[24];
    char rpm[24];
    char elec_deg[24];
    char mech_deg[24];
    char hso_max[24];
    char enc[16];
    char enc2_err[16];
    char enc_last[16];
    char align[16];
    char raw[16];
    char err[16];
    char ref_state[16];
    char ref_raw[16];
    char repeat[16];
    char display_errors[16];
    char hso[16];
    char hso_error[24];
    char hso_flux[24];
    char hso_time[24];
    char hso_gate[16];
    char hso_qual[24];
    char hso_lost[16];
    char hso_reject[16];
    char save_result[16];
    char feedback[16];
    char six_quality[24];
    char six_start[24];
    char six_reject[16];
    char work[20];
    char motor_menu[20];
    char steps[20];
    char status[20];
    char ibus_limit[24];
    char ibus_voltage[24];
    char settings[24];
    char loop_title[24];
    char hso_min[24];
    char speed[24];
    char iq[24];
    char id[24];
    char enc_age[24];
    char elec[24];
    char mech[24];
    char hso_speed[24];
    char six_duty[24];
    char six_bemf[24];
    char six_speed[24];
    char six_start_emf[24];
    char groups[5][20];
    char current_adc[3][24];
    char ibus_code[16];
    char ibus_trips[16];
    char enc_crc[16];
    char enc_field[16];
    char enc_track[16];
    char enc_spi[16];
    char enc_frame[16];
    char bus_raw[16];
    char bus_count[16];
    char frame_count[16];
    char ref_count[16];
    char ref_errors[16];
    char hso_resets[16];
    char hso_rangle[16];
    char hso_rfreq[16];
    char hso_remf[16];
    char hso_rdrift[16];
    char hso_offset[16];
    char hso_olimit[16];
    char ia[24];
    char ib[24];
    char hso_flux_actual[24];
    char six_start_i[24],six_ramp[24];
} UiTexts;
static UiTexts texts;



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
    EVT_IBUS_LIMIT, EVT_IBUS_CLEAR, EVT_IBUS_STATUS, EVT_DIAG_CLEAR,
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
static const char *const group_names[]={ui_strings.text_drive,ui_strings.text_motion,ui_strings.text_encoder,ui_strings.text_adc};
/* ui_app_reset always seeds these before KK_UI_Init; no Flash zero padding. */
/* Stable application-owned labels; update only their axis digit. */

static uint32_t s_input_repeats;
static uint32_t s_display_errors;

static uint32_t s_info_at;
static bool s_settings_dirty,s_model_ready,s_force_save;
static uint8_t s_remote_page;
static uint32_t s_remote_at;
static uint32_t s_settings_save_at;
static const char *s_pending_toast;

static MotorAxis s_view_axis;
/* Main-loop-only, synchronous row formatting. No heap or page-sized cache. */
typedef struct {
    MotorAxis axis;
    unsigned field;
    char *value,*name;
    char scratch[32];
} TextQuery;
static TextQuery *text_query;
static MotorAxis text_axis(void) { return text_query ? text_query->axis : s_view_axis; }
__attribute__((noinline)) static char *text_destination(unsigned field)
{
    if (!text_query) return (char *)&texts+field;
    if (field==text_query->field) return text_query->value;
    if (field==offsetof(UiTexts,target_unit) && text_query->name) return text_query->name;
    return text_query->scratch;
}
#define TEXT(field) (*(char (*)[sizeof(texts.field)])text_destination(offsetof(UiTexts,field)))
/* Topic order is the existing GetRow group order, independent of page IDs. */
enum { REF_DRIVE=1U, REF_ABOUT=2U, REF_MOTION=4U, REF_ENCODER=8U,
       REF_ADC=16U, REF_HSO=32U, REF_DIAGNOSTICS=64U, REF_SIXSTEP=128U };
/* UI reads run only in the main loop. Immediate field queries use one scratch
 * snapshot; callers needing a stable full value read into their own local.
 * Never retain this pointer across another ui_state() call. */
__attribute__((noinline)) static const MotorAxisSnapshot *ui_state(void)
{
    static MotorAxisSnapshot state; MotorControl_GetAxis(s_view_axis,&state); return &state;
}
static void ui_encoder_diag(MT6701Diagnostics *out)
{
    if (text_axis()==MOTOR_AXIS_1) MT6701_GetDiagnostics(out);
    else MT6701_2_GetDiagnostics(out);
}
static uint16_t ui_encoder_raw(void)
{
    if (text_axis()==MOTOR_AXIS_1) return MT6701_GetRawAngle();
    MT6701_2_Snapshot e; MT6701_2_GetSnapshot(&e); return e.raw_angle;
}
static uint32_t ui_encoder_errors(void)
{
    MT6701Diagnostics e; ui_encoder_diag(&e); return e.errors;
}
static bool ui_store_current(void);
static void sync_from_storage(void);
static void focus_axis(MotorAxis axis);
static void adopt_remote(void);
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
    int32_t configured=(text_query ? (int32_t)MotorControl_LimitSpeed(MOTOR_AXIS_1,1.0e6f) :
        s_view_axis==MOTOR_AXIS_1 ? s_speed_limit : views[0].speed_limit)*10;
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
    UiText_Format(buf,16U,ui_strings.fmt_u_u,first,second);
}

/* Fixed application descriptors: 12-bit text offset, 4-bit format, 8-bit
 * snapshot offset. Decode bytes explicitly, without unaligned loads. */
enum { VALUE_COUNT, VALUE_RAD, VALUE_AMP, VALUE_TENTH, VALUE_DEG,
       VALUE_PERCENT, VALUE_MV, VALUE_MWB, VALUE_COUNT16, VALUE_VOLT,
       VALUE_SPEED, VALUE_DUTY, VALUE_SIX_SPEED, VALUE_AXIS_SPEED, VALUE_RPM,
       VALUE_ERROR_DEG, VALUE_FORMAT_COUNT };
typedef struct { uint8_t text_low,text_high_format,offset; } ValueRow;
_Static_assert(sizeof(ValueRow)==3,"Compact display descriptor");
_Static_assert(sizeof(UiTexts)<=4096 && VALUE_FORMAT_COUNT<=16,"Display buffer offsets/formats");
_Static_assert(sizeof(MotorAxisSnapshot)<=UINT8_MAX && sizeof(StHsoSnapshot)<=UINT8_MAX &&
    sizeof(MT6701Diagnostics)<=UINT8_MAX && sizeof(MotorSamplingDiagnostics)<=UINT8_MAX &&
    sizeof(SixStepSnapshot)<=UINT8_MAX,
    "Review source offsets before extending display snapshots");
#define VALUE_ROW(type,member,field,format) {offsetof(UiTexts,field)&255U, \
    (offsetof(UiTexts,field)>>8U)|((format)<<4U),offsetof(type,member)}
__attribute__((noinline)) static void refresh_values(const void *snapshot,const ValueRow *rows,unsigned count)
{
    /* One byte per float format: unit:4, multiplier:3, hundredths:1. */
    enum { UNIT_NONE, UNIT_RAD, UNIT_AMP, UNIT_MWB, UNIT_VOLT, UNIT_SPEED,
           UNIT_PERCENT, UNIT_RPM, UNIT_DEG };
    static const char *const units[]={NULL,"rad","A","mWb","V",ui_strings.text_rad_s,"%","rpm","deg"};
    static const float multipliers[]={1.0f,57.2957795f,100.0f,1000.0f,9.5492966f};
#define VALUE_FORMAT(unit,multiplier,hundredths) ((unit)|((multiplier)<<4U)|((hundredths)<<7U))
    static const uint8_t formats[VALUE_FORMAT_COUNT]={
        [VALUE_RAD]=VALUE_FORMAT(UNIT_RAD,0,1),
        [VALUE_AMP]=VALUE_FORMAT(UNIT_AMP,0,1),
        [VALUE_TENTH]=VALUE_FORMAT(UNIT_NONE,0,0),
        [VALUE_DEG]=VALUE_FORMAT(UNIT_NONE,1,0),
        [VALUE_PERCENT]=VALUE_FORMAT(UNIT_NONE,2,0),
        [VALUE_MV]=VALUE_FORMAT(UNIT_NONE,3,0),
        [VALUE_MWB]=VALUE_FORMAT(UNIT_MWB,3,0),
        [VALUE_VOLT]=VALUE_FORMAT(UNIT_VOLT,0,1),
        [VALUE_SPEED]=VALUE_FORMAT(UNIT_SPEED,0,0),
        [VALUE_DUTY]=VALUE_FORMAT(UNIT_PERCENT,2,0),
        [VALUE_SIX_SPEED]=VALUE_FORMAT(UNIT_SPEED,0,0),
        [VALUE_AXIS_SPEED]=VALUE_FORMAT(UNIT_SPEED,0,0),
        [VALUE_RPM]=VALUE_FORMAT(UNIT_RPM,4,0),
        [VALUE_ERROR_DEG]=VALUE_FORMAT(UNIT_DEG,1,0),
    };
#undef VALUE_FORMAT
    for(unsigned i=0;i<count;++i) {
        const ValueRow *row=&rows[i];
        const char *source=(const char *)snapshot+row->offset;
        unsigned format=row->text_high_format>>4U;
        unsigned field=row->text_low+((row->text_high_format&15U)<<8U);
        if (text_query && field!=text_query->field) continue;
        char *destination=text_destination(field);
        if(format==VALUE_COUNT || format==VALUE_COUNT16) {
            uint32_t value=0;
            memcpy(&value,source,format==VALUE_COUNT ? sizeof(value) : sizeof(uint16_t));
            fmt_counter(destination,value);
        } else {
            float value; memcpy(&value,source,sizeof(value));
            if (!isfinite(value)) { strcpy(destination,"--"); continue; }
            unsigned spec=formats[format];
            const char *unit=units[spec&15U];
            unsigned places=spec>>7U ? 2U : 1U;
            value*=multipliers[(spec>>4U)&7U];
            if ((format==VALUE_SIX_SPEED && SixStep_IsGeneric()) ||
                (format==VALUE_AXIS_SPEED && text_axis()==MOTOR_AXIS_1 && SixStep_IsGeneric())) unit="Hz";
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
    float turn=MotorMath_Wrap(radians);
    int32_t deg=to_x10(turn*57.2957795f);
    fmt_unit(buf,deg==3600 ? 0 : deg,10,"deg");
}

static void fmt_run_aux(char *buf,int32_t value)
{
    if(esc_view()) { strcpy(buf,"Hz"); return; }
    bool position=s_run_mode==JERRYFOC_MODE_POSITION;
    UiText_Format(buf,16U,ui_strings.fmt_ld_s,(long)to_x10((float)value*(position ? 0.05729578f : 0.095492966f)),
        position ? "deg" : "rpm");
}

static const char *mode_name(JerryFOC_ControlMode mode)
{
    switch (mode) {
    case JERRYFOC_MODE_VELOCITY: return ui_strings.text_speed;
    case JERRYFOC_MODE_POSITION: return ui_strings.text_position;
    default: return ui_strings.text_torque;
    }
}

static void refresh_save_state(void)
{
    unsigned state=App_SaveState();
    static const char *const names[]={ui_strings.text_defaults,ui_strings.text_pending,ui_strings.text_saving,ui_strings.text_wait_bus,ui_strings.text_saved,ui_strings.text_failed};
    UiText_Format(texts.save_state,sizeof(texts.save_state),ui_strings.fmt_save_s,names[state]);
}

__attribute__((noinline)) static void axis_title(char *text,const char *suffix)
{
    UiText_Format(text,20U,ui_strings.fmt_m_u_s_s,(unsigned)s_view_axis+1U,texts.motor,suffix);
}

/* Shared LCD/query formatter. A query routes only its requested field into
 * the caller's buffer; other transient text stays in a 32-byte stack scratch.
 * Topic masks avoid unrelated snapshots, menu updates and view changes. */
static void refresh_info(unsigned mask)
{
    MotorAxis axis=text_axis();
    MotorAxisSnapshot state;
    bool oil=axis==MOTOR_AXIS_1 && MotorControl_SixStepAllowed();
    if (mask&(REF_DRIVE|REF_MOTION|REF_ENCODER)) {
        MotorControl_GetAxis(axis,&state); oil=oil && state.mode==JERRYFOC_MODE_TORQUE;
    }
    if (mask&(REF_DIAGNOSTICS)) {
        UiText_Format(TEXT(command),sizeof(TEXT(command)),"%d",(int)Protocol_GetControlResult());
        strcpy(TEXT(m1),SixStep_IsGeneric() ? "ESC" : MotorParams_GetAxis(MOTOR_AXIS_1)->name);
        strcpy(TEXT(m2),MotorParams_GetAxis(MOTOR_AXIS_2)->name);
        UiText_Format(TEXT(control_time),sizeof(TEXT(control_time)),ui_strings.fmt_lu_us,(unsigned long)(MotorControl_MaxCycles()/170U));
        strcpy(TEXT(save),AppStorage_IsDeferred() ? ui_strings.text_deferred : ui_strings.text_available);
        strcpy(TEXT(save_result),AppStorage_SaveErrorText());
        uint32_t flash_error=AppStorage_FlashError();
        MT6701_2_BusDiagnostics bus_diag;
        MT6701_2_GetBusDiagnostics(&bus_diag);
        UiText_Format(TEXT(spi2_fault),sizeof(TEXT(spi2_fault)),ui_strings.fmt_04x_04x_04x,
                 (unsigned)bus_diag.cr1,(unsigned)bus_diag.cr2,(unsigned)bus_diag.sr);
        UiText_Format(TEXT(spi2_resets),sizeof(TEXT(spi2_resets)),"%u",(unsigned)bus_diag.resets);
        UiText_Format(TEXT(flash_error),sizeof(TEXT(flash_error)),ui_strings.fmt_lu_lu_02lx_02lx,
            (unsigned long)(flash_error>>24),(unsigned long)((flash_error>>16)&255U),
            (unsigned long)((flash_error>>8)&255U),(unsigned long)(flash_error&255U));
        fmt_counter(TEXT(repeat),s_input_repeats);
        fmt_counter(TEXT(display_errors),s_display_errors);
    }
    if (!text_query || (mask&REF_DRIVE))
        strcpy(TEXT(motor),(axis==MOTOR_AXIS_1 && SixStep_IsGeneric()) ? "ESC" : MotorParams_GetAxis(axis)->name);
    if (mask&(REF_DRIVE)) {
        strcpy(TEXT(work),MotorControl_WorkText());
        strcpy(TEXT(axis),axis==MOTOR_AXIS_1 ? "M1" : "M2");
        fmt_fixed(TEXT(sync),sizeof(TEXT(sync)),to_x100(MotorControl_GroupError()),100);
        strcpy(TEXT(output),
                 !MotorSampling_IsAxisOutputRequested(axis) ? ui_strings.text_coast :
                 MotorSampling_IsAxisOutputEnabled(axis) ? "PWM" : "OFF");
        strcpy(TEXT(mode),oil ? ui_strings.text_throttle : mode_name(state.mode));

        UiText_Format(TEXT(drive),sizeof(TEXT(drive)),"%s",state.status);
        strcpy(TEXT(feedback),axis==MOTOR_AXIS_1 ? JerryFOC_getFeedbackText() : ui_strings.text_encoder_32);
        strcpy(TEXT(align),state.aligning ? ui_strings.text_aligning : state.aligned ? "YES" : "NO");
    }
    if (!text_query) {
        s_hso_choice=MotorControl_HsoAllowed();
        s_six_choice=MotorControl_SixStepAllowed();
        axis_title(TEXT(motor_menu),ui_strings.text_motor);
        axis_title(TEXT(settings),ui_strings.text_settings);
        axis_title(TEXT(steps),ui_strings.text_steps);
        axis_title(TEXT(status),ui_strings.text_status);
        for (unsigned i=0;i<4;++i) axis_title(TEXT(groups[i]),group_names[i]);
        UiText_Format(TEXT(groups[4]),sizeof(TEXT(groups[4])),ui_strings.fmt_m1_s_hso,MotorParams_GetAxis(MOTOR_AXIS_1)->name);
        axis_title(TEXT(loop_title),ui_strings.text_curr_loop);
        UiText_Format(TEXT(system),sizeof(TEXT(system)),ui_strings.fmt_system_s,MotorControl_WorkText());
        UiText_Format(TEXT(work_menu),sizeof(TEXT(work_menu)),ui_strings.fmt_mode_s,MotorControl_WorkText());
        for (unsigned i=0;i<MOTOR_WORK_COUNT;++i)
            s_txt_work_items[i][0]=i==(unsigned)MotorControl_GetWork() ? '*' : ' ';
        for (unsigned i=0;i<2;++i)
            s_txt_axes[i][8]=i==(unsigned)axis ? '*' : ' ';
        s_current_loop_display=s_current_loop || (axis==MOTOR_AXIS_1 && MotorControl_HsoAllowed());
        if (axis==MOTOR_AXIS_1 && MotorControl_SixStepAllowed()) s_current_loop_display=false;
        refresh_save_state();
    }
    if (mask&(REF_MOTION)) {
        strcpy(TEXT(input),MotorControl_InputText(axis));
        switch (state.mode) {
        case JERRYFOC_MODE_VELOCITY:
            fmt_unit(TEXT(target), to_x10(state.target_velocity), 10, ui_strings.text_rad_s);
            break;
        case JERRYFOC_MODE_POSITION:
            fmt_unit(TEXT(target), to_x100(state.target_position), 100, "rad");
            break;
        default:
            fmt_unit(TEXT(target),to_x100(state.target_iq),100,oil ? "%" : (text_query ? state.current_loop : s_current_loop) ? "A" : "V");
            break;
        }

        if (state.mode==JERRYFOC_MODE_VELOCITY) {
            strcpy(TEXT(target_unit),ui_strings.text_target_rpm);
            fmt_unit(TEXT(target_aux),to_x10(state.target_velocity*9.5492966f),10,"rpm");
        } else if (state.mode==JERRYFOC_MODE_POSITION) {
            strcpy(TEXT(target_unit),ui_strings.text_tgt_0_360);
            fmt_degrees(TEXT(target_aux),state.target_position);
        } else {
            strcpy(TEXT(target_unit),ui_strings.text_target_aux); strcpy(TEXT(target_aux),"--");
        }
        fmt_degrees(TEXT(elec_deg),state.electrical_angle);
        fmt_degrees(TEXT(mech_deg),state.position);

        static const ValueRow motion_values[]={
            VALUE_ROW(MotorAxisSnapshot,velocity,speed,VALUE_AXIS_SPEED),
            VALUE_ROW(MotorAxisSnapshot,velocity,rpm,VALUE_RPM),
            VALUE_ROW(MotorAxisSnapshot,iq,iq,VALUE_AMP),
            VALUE_ROW(MotorAxisSnapshot,id,id,VALUE_AMP),
            VALUE_ROW(MotorAxisSnapshot,ia,ia,VALUE_AMP),
            VALUE_ROW(MotorAxisSnapshot,ib,ib,VALUE_AMP),
            VALUE_ROW(MotorAxisSnapshot,electrical_angle,elec,VALUE_RAD),
            VALUE_ROW(MotorAxisSnapshot,position,mech,VALUE_RAD),
        };
        refresh_values(&state,motion_values,sizeof(motion_values)/sizeof(motion_values[0]));
        if(axis==MOTOR_AXIS_1 && SixStep_IsGeneric()) strcpy(TEXT(rpm),"--");

        if (axis==MOTOR_AXIS_1 && MotorControl_SixStepAllowed()) {
            char *unavailable[]={TEXT(iq),TEXT(id),TEXT(elec),TEXT(mech),TEXT(elec_deg),TEXT(mech_deg)};
            for (unsigned i=0;i<sizeof(unavailable)/sizeof(unavailable[0]);++i) strcpy(unavailable[i],"--");
        }
    }
    if (mask&(REF_ENCODER)) {
        strcpy(TEXT(enc),
                 (axis==MOTOR_AXIS_1 ? MT6701_IsHealthy()!=0U : state.healthy) ? "OK" : ui_strings.text_error);
        MT6701_2_Snapshot encoder2;
        MT6701_2_GetSnapshot(&encoder2);
        UiText_Format(TEXT(enc2), sizeof(TEXT(enc2)), ui_strings.fmt_s_u,
                 encoder2.healthy ? "OK" : encoder2.stopped ? "OFF" : ui_strings.text_error, (unsigned)encoder2.raw_angle);
        fmt_counter(TEXT(enc2_err),encoder2.errors);
        MT6701Diagnostics enc;
        ui_encoder_diag(&enc);
        fmt_counter(TEXT(err),enc.errors);
        fmt_counter(TEXT(raw),ui_encoder_raw());
        if (!text_query) for (unsigned i=0;i<sizeof(s_enc_labels)/sizeof(s_enc_labels[0]);++i)
            s_enc_labels[i][1]=(char)('1'+axis);
        static const ValueRow encoder_values[]={
            VALUE_ROW(MT6701Diagnostics,crc_errors,enc_crc,VALUE_COUNT),
            VALUE_ROW(MT6701Diagnostics,field_errors,enc_field,VALUE_COUNT),
            VALUE_ROW(MT6701Diagnostics,track_errors,enc_track,VALUE_COUNT),
            VALUE_ROW(MT6701Diagnostics,spi_errors,enc_spi,VALUE_COUNT),
            VALUE_ROW(MT6701Diagnostics,frame_errors,enc_frame,VALUE_COUNT),
        };
        refresh_values(&enc,encoder_values,sizeof(encoder_values)/sizeof(encoder_values[0]));
        fmt_pair(TEXT(enc_age),enc.age_cycles,enc.max_age_cycles);
        strcpy(TEXT(enc_last),MT6701_ErrorText(enc.last_error));
        const char *zero=enc.zero_age>3U ? "GAP" :
            (enc.zero_before<=128U || enc.zero_before>=16384U-128U) ? ui_strings.text_near : "FAR";
        if (enc.zero_count) {
            UiText_Format(TEXT(enc_zero),sizeof(TEXT(enc_zero)),ui_strings.fmt_s_u,zero,enc.zero_count);
            if (enc.zero_age!=UINT16_MAX) fmt_counter(TEXT(enc_zpre),enc.zero_before);
            else strcpy(TEXT(enc_zpre),"--");
            if (enc.zero_after!=UINT16_MAX) fmt_counter(TEXT(enc_zpost),enc.zero_after);
            else strcpy(TEXT(enc_zpost),"--");
            fmt_counter(TEXT(enc_ztime),(enc.zero_after==UINT16_MAX ? HAL_GetTick() : enc.zero_ended_at)-enc.zero_started_at);
        } else {
            strcpy(TEXT(enc_zero),"--"); strcpy(TEXT(enc_zpre),"--"); strcpy(TEXT(enc_zpost),"--");
            strcpy(TEXT(enc_ztime),"--");
        }
    }
    if (mask&(REF_SIXSTEP)) {
        SixStepSnapshot six; SixStep_GetSnapshot(&six);
        UiText_Format(TEXT(six_quality),sizeof(TEXT(six_quality)),ui_strings.fmt_u_12_c_u_u_c,six.quality,six.gate ? six.gate : '-',six.phase_pct,six.period_pct,'0'+six.period_fraction%10U);
        static const char *const reject_text[]={"-",ui_strings.text_phase_early,ui_strings.text_phase_late,ui_strings.text_period_low,ui_strings.text_period_high};
        strcpy(TEXT(six_reject),reject_text[six.reject<=SIX_REJECT_LONG ? six.reject : 0]);
        strcpy(TEXT(six_sample),six.sampling_off ? "OFF" : "ON");
        static const ValueRow six_values[]={
            VALUE_ROW(SixStepSnapshot,duty,six_duty,VALUE_DUTY),
            VALUE_ROW(SixStepSnapshot,bemf,six_bemf,VALUE_VOLT),
            VALUE_ROW(SixStepSnapshot,velocity,six_speed,VALUE_SIX_SPEED),
            VALUE_ROW(SixStepSnapshot,startup_emf,six_start_emf,VALUE_VOLT),
            VALUE_ROW(SixStepSnapshot,startup_current,six_start_i,VALUE_AMP),
            VALUE_ROW(SixStepSnapshot,ramp_hz_s,six_ramp,VALUE_TENTH),
        };
        refresh_values(&six,six_values,sizeof(six_values)/sizeof(six_values[0]));
        UiText_Format(TEXT(six_start),sizeof(TEXT(six_start)),ui_strings.fmt_u_500_ms,six.startup_ms);
    }
    if (mask&(REF_HSO)) {
        StHsoSnapshot hso; StHso_GetSnapshot(&hso);
        int32_t minimum=hso_min_x10(),maximum=hso_max_x10();
        fmt_unit(TEXT(hso_min),minimum,10,ui_strings.text_rad_s);
        fmt_unit(TEXT(hso_max),maximum,10,ui_strings.text_rad_s);
        if (maximum<minimum) strcpy(TEXT(hso_max),ui_strings.text_no_run_range);
        /* All reason literals fit these 16-byte application buffers. */
        strcpy(TEXT(hso_gate),StHso_ReasonText(JerryFOC_isHSOActive() ? hso.reason : hso.entry_reason));
        UiText_Format(TEXT(hso_qual), sizeof(TEXT(hso_qual)), ui_strings.fmt_lu_u_ms_lu_20,
                 (unsigned long)(hso.qualified_samples / 10U),
                 hso.state==ST_HSO_LEARNING || hso.reason==ST_HSO_REASON_LEARN_TIMEOUT ? 1000U : 200U,
                 (unsigned long)hso.bad_samples);
        strcpy(TEXT(hso_lost),JerryFOC_getHSOFaultText());
        strcpy(TEXT(hso_reject),StHso_ReasonText(hso.capture_reason));
        static const ValueRow hso_values[]={
            VALUE_ROW(StHsoSnapshot,velocity_rad_s,hso_speed,VALUE_SPEED),
            VALUE_ROW(StHsoSnapshot,angle_error_rad,hso_error,VALUE_ERROR_DEG),
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
        strcpy(TEXT(hso),
                 JerryFOC_hasHSOFault() ? ui_strings.text_lost : StHso_StateText(hso.state));
        UiText_Format(TEXT(hso_flux), sizeof(TEXT(hso_flux)), ui_strings.fmt_lu_uwb, (unsigned long)(hso.reference_flux_wb * 1000000.0f));
        UiText_Format(TEXT(hso_time), sizeof(TEXT(hso_time)), ui_strings.fmt_lu_us_lu, (unsigned long)(hso.max_control_cycles / 170U), (unsigned long)hso.overruns);
    }
    if (mask&(REF_ADC|REF_DRIVE)) {
        MotorSamplingDiagnostics adc; MotorSampling_GetAxisDiagnostics(axis,&adc);
        unsigned reference_mv = (unsigned)(adc.reference_voltage * 1000.0f + 0.5f);
        if (adc.reference_updates != 0U)
            UiText_Format(TEXT(reference), sizeof(TEXT(reference)), ui_strings.fmt_u_03u_v,
                     reference_mv / 1000U, reference_mv % 1000U);
        else
            strcpy(TEXT(reference),ui_strings.text_no_data);
        strcpy(TEXT(ref_state),
                 adc.reference_fresh ? "OK" : (adc.reference_updates ? ui_strings.text_stale : ui_strings.text_wait));
        if (adc.bus_fresh) {
            fmt_unit(TEXT(bus), to_x10(adc.bus_voltage), 10, "V");
        } else {
            strcpy(TEXT(bus),ui_strings.text_no_data);
        }
        if (mask&(REF_ADC)) {
            unsigned limit=MotorSampling_GetIbusLimit();
            if (!text_query) s_ibus_limit_ma=(int32_t)limit;
            fmt_unit(TEXT(ibus_limit),(int32_t)limit/100,10,"A");
            fmt_unit(TEXT(ibus_voltage),to_x100(adc.ibus_threshold_v),100,"V");
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
            strcpy(TEXT(ibus_pin),adc.ibus_high ? ui_strings.text_high : "LOW");
            strcpy(TEXT(ibus_state),adc.ibus_latched ? ui_strings.text_ibus_tripped : adc.ibus_ready ? ui_strings.text_ibus_ready : ui_strings.text_ibus_not_ready);
            fmt_pair(TEXT(ref_raw),adc.reference_raw,adc.reference_cal);
            UiText_Format(TEXT(adc_revision), sizeof(TEXT(adc_revision)), ui_strings.fmt_0x_04x, (unsigned)adc.silicon_revision);
            for (unsigned i = 0U; i < 3U; ++i) {
                if (axis==MOTOR_AXIS_2 && i==2U) { strcpy(TEXT(current_adc[i]),ui_strings.text_derived); continue; }
                fmt_pair(TEXT(current_adc[i]),adc.current_raw[i],adc.current_zero[i]);
            }
            if (axis==MOTOR_AXIS_2 && adc.fault_phase==3U) {
                strcpy(TEXT(current_trip),ui_strings.text_run_w_derived);
            } else if (adc.fault_phase != 0U) {
                unsigned i = adc.fault_phase - 1U;
                UiText_Format(TEXT(current_trip), sizeof(TEXT(current_trip)), ui_strings.fmt_s_c_u,
                         adc.fault_during_zero ? ui_strings.text_zero : "RUN", "UVW"[i],
                         (unsigned)adc.fault_current_raw[i]);
            } else {
                strcpy(TEXT(current_trip),"--");
            }
            UiText_Format(TEXT(adc1_error), sizeof(TEXT(adc1_error)), ui_strings.fmt_0x_lx, (unsigned long)adc.adc1_error);
            UiText_Format(TEXT(adc2_error), sizeof(TEXT(adc2_error)), ui_strings.fmt_0x_lx, (unsigned long)adc.adc2_error);
        }
    }
}
static void refresh_texts(void) { refresh_info(255U); }

/* ---------- 页面描述 ---------- */
static const KK_UI_HomeItem home_items[] = {
    {"Run",      home_icons[2], UI_PAGE_RUN},
    {ui_strings.text_settings, home_icons[0], UI_PAGE_SETTINGS},
    {ui_strings.text_status,   home_icons[1], UI_PAGE_STATUS},
    {ui_strings.text_about,    home_icons[3], UI_PAGE_ABOUT},
};

static const KK_UI_HomePage home_pages[] = {
    {home_items, sizeof(home_items) / sizeof(home_items[0])},
};

/* Fixed information/menu resources are immutable after reset. Compact Flash
 * references expand once into the public KK_UI types before KK_UI_Init.
 * Regions: 0 UiTexts, 1 literals, 2 encoder names, 3 axis labels, 4 work labels.
 * These are fixed application resources, with no heap or widget tree. */

typedef struct { uint8_t bytes[3]; } InfoRowRef;
_Static_assert(sizeof(InfoRowRef)==3,"Compact information row");
_Static_assert(sizeof(UiTexts)<4096 && sizeof(UiStrings)<4096 && sizeof(s_enc_labels)<4096 &&
    sizeof(s_txt_axes)<4096 && sizeof(s_txt_work_items)<4096,"UI resource reference offsets");
#define INFO_TEXT(field) offsetof(UiTexts,field)
#define INFO_LITERAL(field) (4096U|offsetof(UiStrings,field))
#define INFO_ENCODER(index) (8192U+sizeof(s_enc_labels[0])*(index))
#define INFO_AXIS(index) (12288U+sizeof(s_txt_axes[0])*(index))
#define INFO_WORK(index) (16384U+sizeof(s_txt_work_items[0])*(index))
enum {
    drive_rows_START=0, drive_rows_COUNT=15,
    sixstep_rows_START=drive_rows_START+drive_rows_COUNT, sixstep_rows_COUNT=10,
    motion_rows_START=sixstep_rows_START+sixstep_rows_COUNT, motion_rows_COUNT=13,
    encoder_rows_START=motion_rows_START+motion_rows_COUNT, encoder_rows_COUNT=16,
    adc_rows_START=encoder_rows_START+encoder_rows_COUNT, adc_rows_COUNT=19,
    hso_rows_START=adc_rows_START+adc_rows_COUNT, hso_rows_COUNT=21,
    diagnostics_rows_START=hso_rows_START+hso_rows_COUNT, diagnostics_rows_COUNT=11,
    about_rows_START=diagnostics_rows_START+diagnostics_rows_COUNT, about_rows_COUNT=9,
    INFO_ROW_COUNT=about_rows_START+about_rows_COUNT
};
/* Names use literal offsets or an encoder-label range. Value offsets are
 * four-byte aligned UiTexts buffers, or literals tagged above 1024. */
#define INFO_NAME(n) (((n)>>12U)==2U ? 1792U+((n)&4095U) : ((n)>>12U)==1U ? ((n)&4095U) : 1536U+(n)/4U)
#define INFO_VALUE(v) (((v)>>12U)==1U ? 1024U+((v)&4095U) : (v)/4U)
#define INFO_PACK(n,v) ((INFO_NAME(n)|(INFO_VALUE(v)<<11U))+0U*sizeof(char[((n)>>12U==1U ? (n&4095U)<1536U : (n)>>12U==2U ? (n&4095U)<256U : (n)%4U==0U && (n)/4U<256U) && ((v)>>12U==1U ? (v&4095U)<3072U : (v)%4U==0U && (v)/4U<1024U) ? 1 : -1]))
#define INFO_ROW(n,v) {{INFO_PACK(n,v)&255U,(INFO_PACK(n,v)>>8U)&255U,(INFO_PACK(n,v)>>16U)&255U}}
static const InfoRowRef info_refs[]={
    /* drive_rows */
    INFO_ROW(INFO_LITERAL(text_drive),INFO_TEXT(drive)),
    INFO_ROW(INFO_LITERAL(text_output),INFO_TEXT(output)),
    INFO_ROW(INFO_LITERAL(text_feedback),INFO_TEXT(feedback)),
    INFO_ROW(INFO_LITERAL(text_bus),INFO_TEXT(bus)),
    INFO_ROW(INFO_LITERAL(text_motor),INFO_TEXT(motor)),
    INFO_ROW(INFO_LITERAL(text_mode),INFO_TEXT(mode)),
    INFO_ROW(INFO_LITERAL(text_vref),INFO_TEXT(reference)),
    INFO_ROW(INFO_LITERAL(text_refstate),INFO_TEXT(ref_state)),
    INFO_ROW(INFO_LITERAL(text_work),INFO_TEXT(work)),
    INFO_ROW(INFO_LITERAL(text_axis),INFO_TEXT(axis)),
    INFO_ROW(INFO_LITERAL(text_aligned),INFO_TEXT(align)),
    INFO_ROW(INFO_LITERAL(text_run_stop),INFO_TEXT(run_stop)),
    INFO_ROW(INFO_LITERAL(text_stop_age),INFO_TEXT(stop_age)),
    INFO_ROW(INFO_LITERAL(text_stop_err),INFO_TEXT(stop_errors)),
    INFO_ROW(INFO_LITERAL(text_sync_err),INFO_TEXT(sync)),
    /* sixstep_rows */
    INFO_ROW(INFO_LITERAL(text_6s_zc),INFO_TEXT(six_quality)),
    INFO_ROW(INFO_LITERAL(text_6step_duty),INFO_TEXT(six_duty)),
    INFO_ROW(INFO_LITERAL(text_6step_emf),INFO_TEXT(six_bemf)),
    INFO_ROW(INFO_LITERAL(text_6step_speed),INFO_TEXT(six_speed)),
    INFO_ROW(INFO_LITERAL(text_6s_hold),INFO_TEXT(six_start)),
    INFO_ROW(INFO_LITERAL(text_6s_start_emf),INFO_TEXT(six_start_emf)),
    INFO_ROW(INFO_LITERAL(text_6s_start_i),INFO_TEXT(six_start_i)),
    INFO_ROW(INFO_LITERAL(text_6s_ramp),INFO_TEXT(six_ramp)),
    INFO_ROW(INFO_LITERAL(text_6s_cause),INFO_TEXT(six_reject)),
    INFO_ROW(INFO_LITERAL(text_6s_sample),INFO_TEXT(six_sample)),
    /* motion_rows */
    INFO_ROW(INFO_LITERAL(text_input),INFO_TEXT(input)),
    INFO_ROW(INFO_LITERAL(text_target),INFO_TEXT(target)),
    INFO_ROW(INFO_TEXT(target_unit),INFO_TEXT(target_aux)),
    INFO_ROW(INFO_LITERAL(text_speed),INFO_TEXT(speed)),
    INFO_ROW(INFO_LITERAL(text_speed_rpm),INFO_TEXT(rpm)),
    INFO_ROW(INFO_LITERAL(text_iq),INFO_TEXT(iq)),
    INFO_ROW(INFO_LITERAL(text_id),INFO_TEXT(id)),
    INFO_ROW(INFO_LITERAL(text_phase_a),INFO_TEXT(ia)),
    INFO_ROW(INFO_LITERAL(text_phase_b),INFO_TEXT(ib)),
    INFO_ROW(INFO_LITERAL(text_elec),INFO_TEXT(elec)),
    INFO_ROW(INFO_LITERAL(text_mech),INFO_TEXT(mech)),
    INFO_ROW(INFO_LITERAL(text_elec_0_360),INFO_TEXT(elec_deg)),
    INFO_ROW(INFO_LITERAL(text_mech_0_360),INFO_TEXT(mech_deg)),
    /* encoder_rows */
    INFO_ROW(INFO_ENCODER(0),INFO_TEXT(enc)),
    INFO_ROW(INFO_ENCODER(1),INFO_TEXT(enc_crc)),
    INFO_ROW(INFO_ENCODER(2),INFO_TEXT(enc_field)),
    INFO_ROW(INFO_ENCODER(3),INFO_TEXT(enc_track)),
    INFO_ROW(INFO_ENCODER(4),INFO_TEXT(enc_spi)),
    INFO_ROW(INFO_ENCODER(5),INFO_TEXT(enc_frame)),
    INFO_ROW(INFO_ENCODER(6),INFO_TEXT(enc_age)),
    INFO_ROW(INFO_ENCODER(7),INFO_TEXT(enc_last)),
    INFO_ROW(INFO_ENCODER(8),INFO_TEXT(raw)),
    INFO_ROW(INFO_ENCODER(9),INFO_TEXT(err)),
    INFO_ROW(INFO_ENCODER(10),INFO_TEXT(enc_zero)),
    INFO_ROW(INFO_ENCODER(11),INFO_TEXT(enc_zpre)),
    INFO_ROW(INFO_ENCODER(12),INFO_TEXT(enc_zpost)),
    INFO_ROW(INFO_ENCODER(13),INFO_TEXT(enc_ztime)),
    INFO_ROW(INFO_LITERAL(text_e2_probe),INFO_TEXT(enc2)),
    INFO_ROW(INFO_LITERAL(text_e2_total),INFO_TEXT(enc2_err)),
    /* adc_rows */
    INFO_ROW(INFO_LITERAL(text_m1_ibus),INFO_TEXT(ibus_state)),
    INFO_ROW(INFO_LITERAL(text_ibus_limit),INFO_TEXT(ibus_limit)),
    INFO_ROW(INFO_LITERAL(text_ibus_dac),INFO_TEXT(ibus_code)),
    INFO_ROW(INFO_LITERAL(text_ibus_thresh),INFO_TEXT(ibus_voltage)),
    INFO_ROW(INFO_LITERAL(text_ibus_trips),INFO_TEXT(ibus_trips)),
    INFO_ROW(INFO_LITERAL(text_comp4),INFO_TEXT(ibus_pin)),
    INFO_ROW(INFO_LITERAL(text_i_trip),INFO_TEXT(current_trip)),
    INFO_ROW(INFO_LITERAL(text_iu_r_z),INFO_TEXT(current_adc[0])),
    INFO_ROW(INFO_LITERAL(text_iv_r_z),INFO_TEXT(current_adc[1])),
    INFO_ROW(INFO_LITERAL(text_iw_r_z),INFO_TEXT(current_adc[2])),
    INFO_ROW(INFO_LITERAL(text_chiprev),INFO_TEXT(adc_revision)),
    INFO_ROW(INFO_LITERAL(text_busraw),INFO_TEXT(bus_raw)),
    INFO_ROW(INFO_LITERAL(text_buscnt),INFO_TEXT(bus_count)),
    INFO_ROW(INFO_LITERAL(text_framecnt),INFO_TEXT(frame_count)),
    INFO_ROW(INFO_LITERAL(text_adc1err),INFO_TEXT(adc1_error)),
    INFO_ROW(INFO_LITERAL(text_adc2err),INFO_TEXT(adc2_error)),
    INFO_ROW(INFO_LITERAL(text_ref_r_c),INFO_TEXT(ref_raw)),
    INFO_ROW(INFO_LITERAL(text_refcnt),INFO_TEXT(ref_count)),
    INFO_ROW(INFO_LITERAL(text_referr),INFO_TEXT(ref_errors)),
    /* hso_rows */
    INFO_ROW(INFO_LITERAL(text_hso),INFO_TEXT(hso)),
    INFO_ROW(INFO_LITERAL(text_h_min),INFO_TEXT(hso_min)),
    INFO_ROW(INFO_LITERAL(text_h_max),INFO_TEXT(hso_max)),
    INFO_ROW(INFO_LITERAL(text_h_zero),INFO_LITERAL(text_stop_coast)),
    INFO_ROW(INFO_LITERAL(text_h_reverse),INFO_LITERAL(text_stop_first)),
    INFO_ROW(INFO_LITERAL(text_h_gate),INFO_TEXT(hso_gate)),
    INFO_ROW(INFO_LITERAL(text_h_qual),INFO_TEXT(hso_qual)),
    INFO_ROW(INFO_LITERAL(text_h_reject),INFO_TEXT(hso_reject)),
    INFO_ROW(INFO_LITERAL(text_h_reset),INFO_TEXT(hso_resets)),
    INFO_ROW(INFO_LITERAL(text_h_r_deg),INFO_TEXT(hso_rangle)),
    INFO_ROW(INFO_LITERAL(text_h_r_hz),INFO_TEXT(hso_rfreq)),
    INFO_ROW(INFO_LITERAL(text_h_r_emf),INFO_TEXT(hso_remf)),
    INFO_ROW(INFO_LITERAL(text_h_r_drift),INFO_TEXT(hso_rdrift)),
    INFO_ROW(INFO_LITERAL(text_h_off_mv),INFO_TEXT(hso_offset)),
    INFO_ROW(INFO_LITERAL(text_h_lim_mv),INFO_TEXT(hso_olimit)),
    INFO_ROW(INFO_LITERAL(text_h_lost),INFO_TEXT(hso_lost)),
    INFO_ROW(INFO_LITERAL(text_h_speed),INFO_TEXT(hso_speed)),
    INFO_ROW(INFO_LITERAL(text_h_error),INFO_TEXT(hso_error)),
    INFO_ROW(INFO_LITERAL(text_h_flux),INFO_TEXT(hso_flux)),
    INFO_ROW(INFO_LITERAL(text_h_actual),INFO_TEXT(hso_flux_actual)),
    INFO_ROW(INFO_LITERAL(text_h_time),INFO_TEXT(hso_time)),
    /* diagnostics_rows */
    INFO_ROW(INFO_LITERAL(text_cmd),INFO_TEXT(command)),
    INFO_ROW(INFO_LITERAL(text_m1_model),INFO_TEXT(m1)),
    INFO_ROW(INFO_LITERAL(text_m2_model),INFO_TEXT(m2)),
    INFO_ROW(INFO_LITERAL(text_ctrl_max),INFO_TEXT(control_time)),
    INFO_ROW(INFO_LITERAL(text_save_bus),INFO_TEXT(save)),
    INFO_ROW(INFO_LITERAL(text_save_last),INFO_TEXT(save_result)),
    INFO_ROW(INFO_LITERAL(text_flash_err),INFO_TEXT(flash_error)),
    INFO_ROW(INFO_LITERAL(text_spi2_fault),INFO_TEXT(spi2_fault)),
    INFO_ROW(INFO_LITERAL(text_spi2_reset),INFO_TEXT(spi2_resets)),
    INFO_ROW(INFO_LITERAL(text_keyrpt),INFO_TEXT(repeat)),
    INFO_ROW(INFO_LITERAL(text_disperr),INFO_TEXT(display_errors)),
    /* about_rows */
    INFO_ROW(INFO_LITERAL(text_fw),INFO_LITERAL(text_firmware)),
    INFO_ROW(INFO_LITERAL(text_mcu),INFO_LITERAL(text_stm32g431)),
    INFO_ROW(INFO_LITERAL(text_motor),INFO_TEXT(motor)),
    INFO_ROW(INFO_LITERAL(text_control),INFO_LITERAL(text_10khz_foc)),
    INFO_ROW(INFO_LITERAL(text_encoder),INFO_LITERAL(text_mt6701)),
    INFO_ROW(INFO_LITERAL(text_lcd),INFO_LITERAL(text_st7735s)),
    INFO_ROW(INFO_LITERAL(text_ui),INFO_LITERAL(text_kk_ui)),
    INFO_ROW(INFO_LITERAL(text_keys),INFO_LITERAL(text_6_keys)),
    INFO_ROW(INFO_LITERAL(text_flash),INFO_LITERAL(text_w25q64)),
};
_Static_assert(sizeof(info_refs)/sizeof(info_refs[0])==INFO_ROW_COUNT,"Resource row counts");
#undef INFO_ENCODER
static KK_UI_InfoRow info_rows[INFO_ROW_COUNT];
static KK_UI_InfoRow diagnostic_rows[diagnostics_rows_COUNT+BOARD_DIAG_ROWS];
static char diagnostic_values[BOARD_DIAG_ROWS][32];
__attribute__((noinline,noclone)) static const char *info_pointer(uint16_t reference)
{
    static const char *const bases[]={(const char *)&texts,(const char *)&ui_strings,(const char *)s_enc_labels,(const char *)s_txt_axes,(const char *)s_txt_work_items};
    return bases[reference>>12U]+(reference&4095U);
}
static void init_menu_items(void);
__attribute__((noinline,noclone)) static void init_info_rows(void)
{
    for (unsigned i=0;i<INFO_ROW_COUNT;++i) {
        const uint8_t *b=info_refs[i].bytes;
        unsigned packed=b[0]|((unsigned)b[1]<<8U)|((unsigned)b[2]<<16U);
        unsigned name=packed&2047U,value=packed>>11U;
        info_rows[i].name=name>=1792U ? (const char *)s_enc_labels+name-1792U : name>=1536U ? (const char *)&texts+(name-1536U)*4U : (const char *)&ui_strings+name;
        info_rows[i].value=value>=1024U ? (const char *)&ui_strings+value-1024U : (const char *)&texts+value*4U;
    }
    memcpy(diagnostic_rows,&info_rows[diagnostics_rows_START],sizeof(KK_UI_InfoRow)*diagnostics_rows_COUNT);
    for (unsigned i=0;i<BOARD_DIAG_ROWS;++i) {
        diagnostic_rows[diagnostics_rows_COUNT+i].name=BoardDiag_Row(s_view_axis,i,diagnostic_values[i]);
        diagnostic_rows[diagnostics_rows_COUNT+i].value=diagnostic_values[i];
    }
    init_menu_items();
}

static const KK_UI_InfoPage info_pages[] = {
    {texts.groups[0],&info_rows[drive_rows_START],drive_rows_COUNT},
    {ui_strings.text_about,&info_rows[about_rows_START],about_rows_COUNT},
    {texts.groups[1],&info_rows[motion_rows_START],motion_rows_COUNT},
    {texts.groups[2],&info_rows[encoder_rows_START],encoder_rows_COUNT},
    {texts.groups[3],&info_rows[adc_rows_START],adc_rows_COUNT},
    {texts.groups[4],&info_rows[hso_rows_START],hso_rows_COUNT},
    {ui_strings.text_diagnostics,diagnostic_rows,diagnostics_rows_COUNT+BOARD_DIAG_ROWS},
    {ui_strings.text_m1_six_step,&info_rows[sixstep_rows_START],sixstep_rows_COUNT},
};

/* Three-byte application descriptors, expanded to the public menu type.
 * Every source entry checks label/type/ref capacity at compile time. */
typedef struct { uint8_t bytes[3]; } MenuItemRef;
_Static_assert(sizeof(MenuItemRef)==3,"Compact menu references");
#define MENU_PACK(n,t,r) ((uint32_t)(n)|((uint32_t)(t)<<15U)|((uint32_t)(r)<<18U)| \
    (0U*sizeof(char[((n)<32768U && (t)<8U && (r)<64U) ? 1 : -1])))
#define MENU_ROW(n,t,r) {{MENU_PACK(n,t,r)&255U,(MENU_PACK(n,t,r)>>8U)&255U,(MENU_PACK(n,t,r)>>16U)&255U}}

enum {
    settings_items_START=0, settings_items_COUNT=11,
    motor_items_START=settings_items_START+settings_items_COUNT, motor_items_COUNT=4,
    system_items_START=motor_items_START+motor_items_COUNT, system_items_COUNT=8,
    sensorless_items_START=system_items_START+system_items_COUNT, sensorless_items_COUNT=2,
    work_items_START=sensorless_items_START+sensorless_items_COUNT, work_items_COUNT=7,
    status_items_START=work_items_START+work_items_COUNT, status_items_COUNT=9,
    protection_items_START=status_items_START+status_items_COUNT, protection_items_COUNT=3,
    MENU_ITEM_COUNT=protection_items_START+protection_items_COUNT
};
static const MenuItemRef menu_refs[]={
    /* settings_items */
    MENU_ROW(INFO_TEXT(system),KK_UI_MENU_PAGE,UI_PAGE_SYSTEM),
    MENU_ROW(INFO_LITERAL(text_curr_loop),KK_UI_MENU_BOOL,BOOL_CURRENT_LOOP),
    MENU_ROW(INFO_LITERAL(text_pos_speed),KK_UI_MENU_INT,INT_POSITION_SPEED),
    MENU_ROW(INFO_LITERAL(text_speed_limit),KK_UI_MENU_INT,INT_SPEED_LIMIT),
    MENU_ROW(INFO_LITERAL(text_run_steps),KK_UI_MENU_PAGE,UI_PAGE_RUN_STEPS),
    MENU_ROW(INFO_LITERAL(text_m1_protection),KK_UI_MENU_PAGE,UI_PAGE_PROTECTION),
    MENU_ROW(INFO_LITERAL(text_motor),KK_UI_MENU_PAGE,UI_PAGE_MOTOR),
    MENU_ROW(INFO_LITERAL(text_status_led),KK_UI_MENU_BOOL,BOOL_LED),
    MENU_ROW(INFO_LITERAL(text_led_bright),KK_UI_MENU_INT,INT_LED_BRIGHT),
    MENU_ROW(INFO_TEXT(save_state),KK_UI_MENU_ACTION,EVT_SAVE_STATUS),
    MENU_ROW(INFO_LITERAL(text_restore_defaults),KK_UI_MENU_CONFIRM,CONFIRM_RESET),
    /* motor_items */
    MENU_ROW(INFO_LITERAL(text_2804),KK_UI_MENU_CONFIRM,CONFIRM_MOTOR_2804),
    MENU_ROW(INFO_LITERAL(text_4015),KK_UI_MENU_CONFIRM,CONFIRM_MOTOR_4015),
    MENU_ROW(INFO_LITERAL(text_esc_auto),KK_UI_MENU_CONFIRM,CONFIRM_MOTOR_ESC),
    MENU_ROW(INFO_LITERAL(text_m1_detect),KK_UI_MENU_PAGE,UI_PAGE_DETECT),
    /* system_items */
    MENU_ROW(INFO_TEXT(work_menu),KK_UI_MENU_PAGE,UI_PAGE_WORK),
    MENU_ROW(INFO_AXIS(0),KK_UI_MENU_ACTION,EVT_FOCUS_M1),
    MENU_ROW(INFO_AXIS(1),KK_UI_MENU_ACTION,EVT_FOCUS_M2),
    MENU_ROW(INFO_LITERAL(text_sensorless),KK_UI_MENU_PAGE,UI_PAGE_SENSORLESS),
    MENU_ROW(INFO_LITERAL(text_gear_ratio),KK_UI_MENU_INT,INT_GEAR_RATIO),
    MENU_ROW(INFO_LITERAL(text_gear_offset),KK_UI_MENU_INT,INT_GEAR_OFFSET),
    MENU_ROW(INFO_LITERAL(text_align_retry),KK_UI_MENU_ACTION,EVT_ALIGN),
    MENU_ROW(INFO_LITERAL(text_clear_diag),KK_UI_MENU_ACTION,EVT_DIAG_CLEAR),
    /* sensorless_items */
    MENU_ROW(INFO_LITERAL(text_hso_feedback),KK_UI_MENU_BOOL,BOOL_HSO),
    MENU_ROW(INFO_LITERAL(text_six_step),KK_UI_MENU_BOOL,BOOL_SIXSTEP),
    /* work_items */
    MENU_ROW(INFO_WORK(0),KK_UI_MENU_ACTION,EVT_WORK_BASE+MOTOR_WORK_SINGLE_M1),
    MENU_ROW(INFO_WORK(1),KK_UI_MENU_ACTION,EVT_WORK_BASE+MOTOR_WORK_SINGLE_M2),
    MENU_ROW(INFO_WORK(2),KK_UI_MENU_ACTION,EVT_WORK_BASE+MOTOR_WORK_INDEPENDENT),
    MENU_ROW(INFO_WORK(3),KK_UI_MENU_ACTION,EVT_WORK_BASE+MOTOR_WORK_SYNC_SPEED),
    MENU_ROW(INFO_WORK(4),KK_UI_MENU_ACTION,EVT_WORK_BASE+MOTOR_WORK_SYNC_POSITION),
    MENU_ROW(INFO_WORK(5),KK_UI_MENU_ACTION,EVT_WORK_BASE+MOTOR_WORK_MASTER_M1),
    MENU_ROW(INFO_WORK(6),KK_UI_MENU_ACTION,EVT_WORK_BASE+MOTOR_WORK_MASTER_M2),
    /* status_items */
    MENU_ROW(INFO_LITERAL(text_drive),KK_UI_MENU_PAGE,UI_PAGE_DRIVE),
    MENU_ROW(INFO_LITERAL(text_motion),KK_UI_MENU_PAGE,UI_PAGE_MOTION),
    MENU_ROW(INFO_LITERAL(text_encoder),KK_UI_MENU_PAGE,UI_PAGE_ENCODER),
    MENU_ROW(INFO_LITERAL(text_adc),KK_UI_MENU_PAGE,UI_PAGE_ADC),
    MENU_ROW(INFO_LITERAL(text_hso),KK_UI_MENU_PAGE,UI_PAGE_HSO),
    MENU_ROW(INFO_LITERAL(text_six_step),KK_UI_MENU_PAGE,UI_PAGE_SIXSTEP),
    MENU_ROW(INFO_LITERAL(text_diagnostics),KK_UI_MENU_PAGE,UI_PAGE_DIAGNOSTICS),
    MENU_ROW(INFO_AXIS(0),KK_UI_MENU_ACTION,EVT_FOCUS_M1),
    MENU_ROW(INFO_AXIS(1),KK_UI_MENU_ACTION,EVT_FOCUS_M2),
    /* protection_items */
    MENU_ROW(INFO_LITERAL(text_ibus_trip),KK_UI_MENU_INT,INT_IBUS_LIMIT),
    MENU_ROW(INFO_LITERAL(text_clear_ibus_trip),KK_UI_MENU_CONFIRM,CONFIRM_IBUS_CLEAR),
    MENU_ROW(INFO_TEXT(ibus_state),KK_UI_MENU_ACTION,EVT_IBUS_STATUS),
};
_Static_assert(sizeof(menu_refs)/sizeof(menu_refs[0])==MENU_ITEM_COUNT,"Resource row counts");
static KK_UI_MenuItem menu_items[MENU_ITEM_COUNT];
#undef INFO_TEXT
#undef INFO_LITERAL
#undef INFO_AXIS
#undef INFO_WORK
__attribute__((noinline,noclone)) static void init_menu_items(void)
{
    for (unsigned i=0;i<MENU_ITEM_COUNT;++i) {
        const uint8_t *b=menu_refs[i].bytes;
        unsigned packed=b[0]|((unsigned)b[1]<<8U)|((unsigned)b[2]<<16U);
        menu_items[i].label=info_pointer(packed&32767U);
        menu_items[i].type=(KK_UI_MenuItemType)((packed>>15U)&7U);
        menu_items[i].ref=(uint16_t)(packed>>18U);
    }
}
static const KK_UI_MenuPage menu_pages[] = {
    {texts.settings, &menu_items[settings_items_START], settings_states,
     settings_items_COUNT},
    {texts.motor_menu, &menu_items[motor_items_START], motor_states,
     motor_items_COUNT},
    {ui_strings.text_system, &menu_items[system_items_START], NULL, system_items_COUNT},
    {ui_strings.text_work_mode, &menu_items[work_items_START], NULL, work_items_COUNT},
    {texts.status,&menu_items[status_items_START],NULL,status_items_COUNT},
    {ui_strings.text_m1_protection,&menu_items[protection_items_START],protection_states,protection_items_COUNT},
    {ui_strings.text_m1_sensorless,&menu_items[sensorless_items_START],sensorless_states,sensorless_items_COUNT},
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
    [INT_IBUS_LIMIT] = {ui_strings.text_m1_ibus_trip,&s_ibus_limit_ma,APP_IBUS_MIN_MA,APP_IBUS_MAX_MA,100U,"mA",EVT_IBUS_LIMIT},
    [INT_GEAR_RATIO] = {ui_strings.text_gear_ratio, &s_ratio_x100,-1000,1000,1U,"%",EVT_GEAR},
    [INT_GEAR_OFFSET] = {ui_strings.text_gear_offset, &s_offset_x100,-6283,6283,5U,ui_strings.text_0_01rad,EVT_GEAR},
    [INT_LED_BRIGHT] = {ui_strings.text_led_brightness, &s_led_brightness, 0, 100, 5U, "%", EVT_LED_CHANGED},
    [INT_POSITION_SPEED] = {ui_strings.text_position_speed, &s_position_speed, 1, APP_SPEED_LIMIT_DEFAULT, 1U, ui_strings.text_rad_s, EVT_POSITION_SPEED_CHANGED},
    [INT_SPEED_LIMIT] = {ui_strings.text_speed_limit, &s_speed_limit, 1, APP_SPEED_LIMIT_MAX, 1U, ui_strings.text_rad_s, EVT_SPEED_LIMIT_CHANGED},
};

static KK_UI_BoolBinding bool_bindings[] = {
    [BOOL_HSO] = {ui_strings.text_m1_hso_feedback, &s_hso_choice, EVT_FEEDBACK},
    [BOOL_SIXSTEP] = {ui_strings.text_m1_6_step, &s_six_choice, EVT_SIXSTEP},
    [BOOL_CURRENT_LOOP] = {texts.loop_title, &s_current_loop_display, EVT_CURRENT_LOOP_CHANGED},
    [BOOL_LED]          = {ui_strings.text_status_led,   &s_led_enable,   EVT_LED_CHANGED},
};

static const KK_UI_ConfirmDesc confirm_descs[] = {
    [CONFIRM_IBUS_CLEAR] = {ui_strings.text_clear_m1_ibus_output_stays_off,EVT_IBUS_CLEAR,0U},
    [CONFIRM_RESET] = {ui_strings.text_reset_all_settings, EVT_RESET_CONFIRMED, EVT_RESET_CANCELLED},
    [CONFIRM_MOTOR_2804] = {ui_strings.text_use_2804_stop_and_align, EVT_MOTOR_2804_CONFIRMED, 0U},
    [CONFIRM_MOTOR_4015] = {ui_strings.text_use_4015_stop_and_align, EVT_MOTOR_4015_CONFIRMED, 0U},
    [CONFIRM_MOTOR_ESC] = {ui_strings.text_esc_auto_throttle_only, EVT_MOTOR_ESC, 0U},
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
    .texts = {ui_strings.text_back, ui_strings.text_cancel, "OK", "ON", "OFF", ui_strings.text_notice},
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
    if (!s_run_active && ui_state()->aligning) return;
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
    if (!ui_store_current()) return false;
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
    if (!independent_feedback || ui_state()->active) stop_run();
    bool ok=profile==MOTOR_PROFILE_COUNT ? s_view_axis==MOTOR_AXIS_1 && MotorControl_SelectGeneric() :
        MotorControl_SelectProfile(s_view_axis,profile);
    if (!ok) {
        s_pending_toast = independent_feedback ? ui_strings.text_wait_for_stop : ui_strings.text_stop_check_encoder;
        return false;
    }
    reset_motor_view(); clear_run_stop();
    ui_store_current();
    mark_settings_dirty(HAL_GetTick());
    refresh_texts();
    s_pending_toast = independent_feedback ? ui_strings.text_motor_set : ui_strings.text_motor_set_align;
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
/* Reset and Run entry seed the same stopped drafts. No command is applied. */
__attribute__((noinline,noclone)) static void seed_run_drafts(void)
{
    s_run_throttle_pct=s_throttle_pct;
    s_run_speed_x10=s_speed_x10;
    if (sensorless()) s_run_speed_x10=hso_draft(s_run_speed_x10);
    s_run_voltage_x100=s_voltage_x100;
    s_run_position_x100=s_position_x100;
    s_run_current_x100=s_confirmed_current_x100=0;
    s_run_position_origin=ui_state()->position;
}

const KK_UI_App *ui_app_get(void)
{
    return &s_app;
}

static void ensure_model(void)
{
    if (s_model_ready) return;
    init_ui_resources(); init_info_rows();
    reload_axis_views(); load_view(MotorControl_GetFocus());
    s_ratio_x100=AppStorage_Get()->gear_ratio_x100;
    s_offset_x100=AppStorage_Get()->gear_offset_x100;
    s_model_ready=true;
}

void ui_app_reset(uint32_t now_ms)
{
    s_remote_page=0; s_force_save=false;
    s_global_stop_seen=false;
    clear_run_stop();
    AppStorage_Load();
    s_model_ready=false; ensure_model();

    if (s_view_axis==MOTOR_AXIS_1) JerryFOC_useCurrentLoop(s_current_loop ? 1U : 0U);

    s_run_mode = ui_state()->mode;
    s_run_active = false;
    s_run_lr_key=0U;
    s_input_repeats = s_display_errors = 0U;
    seed_run_drafts();
    s_settings_dirty = false;
    s_settings_save_at = now_ms;
    s_info_at = now_ms;
    s_last_input_at = now_ms;
    s_last_live_at = now_ms;
    s_pending_toast = AppStorage_IsReady() ? NULL : ui_strings.text_w25q64_not_found;

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
            strcpy(texts.run_stop,ui_strings.text_global_stop);
            strcpy(texts.stop_age,"--"); strcpy(texts.stop_errors,"--");
            s_pending_toast=ui_strings.text_global_stop;
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
        s_pending_toast = save_ok ? ui_strings.text_saved : AppStorage_SaveErrorText();

    uint8_t healthy = ui_state()->healthy ? 1U : 0U;
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
        MotorAxisSnapshot state; MotorControl_GetAxis(s_view_axis,&state);
        const char *reason = NULL;
        if (!state.ready) reason = state.status;
        else if (!MotorSampling_IsAxisOutputRequested(s_view_axis) && strcmp(state.status,ui_strings.text_ready)) reason=state.status;
        else if (!state.healthy) reason = MotorControl_EncoderLostText(s_view_axis);
        else if (!state.aligned) reason = ui_strings.text_not_aligned;
        else if (s_view_axis==MOTOR_AXIS_1 && g_openloop_debug) reason = ui_strings.text_open_loop;
        else if (state.mode != s_run_mode) reason = ui_strings.text_mode_changed;
        else if (!MotorSampling_IsAxisOutputRequested(s_view_axis))
            reason=strcmp(state.status,ui_strings.text_ready)==0 ? ui_strings.text_output_off : state.status;
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
            UiText_Format(texts.stop_age, sizeof(texts.stop_age), ui_strings.fmt_e_u_u, (unsigned)stop_axis+1U, enc.age_cycles);
            UiText_Format(texts.stop_errors, sizeof(texts.stop_errors), ui_strings.fmt_e_u_lu, (unsigned)stop_axis+1U, (unsigned long)enc.errors);
            stop_run();
            s_pending_toast = reason;
            refresh_texts();
            KK_UI_Invalidate();
        }
    }

    if ((now_ms - s_info_at) >= 500U) {
        s_info_at = now_ms;
        refresh_texts();
        if (KK_UI_GetCurrentPage()==UI_PAGE_DIAGNOSTICS)
            for (unsigned i=0;i<BOARD_DIAG_ROWS;++i) (void)BoardDiag_Row(s_view_axis,i,diagnostic_values[i]);
        KK_UI_Invalidate();
    }

    /* Protocol changes may have a newer tick than this UI frame's timestamp. */
    App_SaveService(HAL_GetTick());
}

static bool apply_event(KK_UI_EventId evt)
{
    bool ok=true;
    if (evt>=EVT_WORK_BASE && evt<EVT_WORK_BASE+MOTOR_WORK_COUNT) {
        if (MotorControl_SelectWork((MotorWorkMode)(evt-EVT_WORK_BASE))) {
            focus_axis(MotorControl_GetFocus()); s_run_active=false;
            s_pending_toast=MotorControl_WorkText(); mark_settings_dirty(HAL_GetTick());
        } else { s_pending_toast=ui_strings.text_stop_both_first; ok=false; }
        return ok;
    }
    switch (evt) {
    case EVT_IBUS_LIMIT:
        if ((ok=MotorSampling_SetIbusLimit((uint16_t)s_ibus_limit_ma))) {
            AppSettings *st=AppStorage_Get();
            st->control_config=(st->control_config & ~APP_CONFIG_IBUS_MASK) |
                ((uint32_t)(s_ibus_limit_ma/100)<<APP_CONFIG_IBUS_SHIFT);
            mark_settings_dirty(HAL_GetTick()); s_pending_toast=ui_strings.text_ibus_limit_applied;
        } else s_pending_toast=ui_strings.text_stop_m1_check_ibus;
        refresh_texts(); KK_UI_Invalidate(); break;
    case EVT_IBUS_CLEAR:
        MotorControl_Stop(MOTOR_AXIS_1);
        if (MotorControl_IsCoupled()) s_run_active=views[0].run_active=views[1].run_active=false;
        else if (s_view_axis==MOTOR_AXIS_1) s_run_active=false;
        else views[0].run_active=false;
        if ((ok=MotorSampling_ClearIbusTrip())) {
            if (MotorControl_HsoAllowed() && JerryFOC_hasHSOFault()) (void)JerryFOC_useHSO(0U);
            if (s_view_axis==MOTOR_AXIS_1 || MotorControl_IsCoupled()) clear_run_stop();
            s_pending_toast=ui_strings.text_ibus_cleared;
        } else s_pending_toast=ui_strings.text_wait_check_ibus;
        refresh_texts(); KK_UI_Invalidate(); break;
    case EVT_FOCUS_M1: case EVT_FOCUS_M2:
        focus_axis(evt==EVT_FOCUS_M1 ? MOTOR_AXIS_1 : MOTOR_AXIS_2);
        s_pending_toast=s_view_axis==MOTOR_AXIS_1 ? ui_strings.text_axis_m1 : ui_strings.text_axis_m2;
        break;
    case EVT_FEEDBACK:
        if ((ok=MotorControl_SelectFeedback(s_hso_choice))) mark_settings_dirty(HAL_GetTick());
        else s_pending_toast=ui_strings.text_single_m1_stop;
        s_hso_choice=MotorControl_HsoAllowed();
        refresh_texts(); KK_UI_Invalidate();
        break;
    case EVT_SIXSTEP:
        if ((ok=MotorControl_SelectSixStep(s_six_choice))) mark_settings_dirty(HAL_GetTick());
        else s_pending_toast=ui_strings.text_single_m1_stop;
        refresh_texts(); KK_UI_Invalidate();
        break;
    case EVT_GEAR:
        if ((ok=MotorControl_SetGear(s_ratio_x100,s_offset_x100))) mark_settings_dirty(HAL_GetTick());
        else {
            s_ratio_x100=AppStorage_Get()->gear_ratio_x100;
            s_offset_x100=AppStorage_Get()->gear_offset_x100;
            s_pending_toast=ui_strings.text_stop_ratio_nonzero;
        }
        break;
    case EVT_ALIGN:
        s_pending_toast=(ok=MotorControl_RequestAlign(s_view_axis)) ? ui_strings.text_align_requested : ui_strings.text_stop_check_axis;
        break;
    case EVT_MOTOR_2804_CONFIRMED:
    case EVT_MOTOR_4015_CONFIRMED:
        ok=select_motor(evt == EVT_MOTOR_4015_CONFIRMED ? MOTOR_PROFILE_4015 : MOTOR_PROFILE_2804);
        break;
    case EVT_MOTOR_ESC:
        ok=select_motor(MOTOR_PROFILE_COUNT); break;
    case EVT_CURRENT_LOOP_CHANGED:
        if (s_view_axis==MOTOR_AXIS_1 && sensorless()) {
            s_current_loop_display=MotorControl_HsoAllowed(); ok=false; break;
        }
        s_current_loop=s_current_loop_display;
        s_run_current_x100 = s_confirmed_current_x100 = 0;
        if ((ok=ui_store_current())) mark_settings_dirty(HAL_GetTick());
        break;

    case EVT_POSITION_SPEED_CHANGED:
    case EVT_LED_CHANGED:
        if ((ok=ui_store_current())) mark_settings_dirty(HAL_GetTick());
        break;

    case EVT_SPEED_LIMIT_CHANGED: {
        update_position_speed_limit();
        int32_t limit = s_speed_limit * 10;
        s_speed_x10 = clamp_i32(s_speed_x10, -limit, limit);
        s_run_speed_x10 = clamp_i32(s_run_speed_x10, -limit, limit);
        if ((ok=ui_store_current())) mark_settings_dirty(HAL_GetTick());
        KK_UI_Invalidate();
        break;
    }

    case EVT_RUN_STEPS_CHANGED:
        for (unsigned i = 0U; i < 3U; ++i) s_run_steps[i] = s_step_drafts[i];
        if ((ok=ui_store_current())) mark_settings_dirty(HAL_GetTick());
        break;

    case EVT_RESET_CONFIRMED:
        MotorControl_StopAll();
        if (!MotorControl_SelectProfile(MOTOR_AXIS_1,MOTOR_PROFILE_2804) ||
            !MotorControl_SelectProfile(MOTOR_AXIS_2,MOTOR_PROFILE_2804) ||
            !MotorControl_SelectWork(MOTOR_WORK_SINGLE_M1) || !MotorControl_SelectFeedback(false)) {
            s_pending_toast=ui_strings.text_stop_check_encoder; ok=false; break;
        }
        clear_run_stop(); AppStorage_ResetDefaults();
        (void)MotorSampling_SetIbusLimit(APP_IBUS_DEFAULT_MA);
        (void)MotorControl_SetGear(100,0);
        reload_axis_views();
        load_view(MOTOR_AXIS_1); MotorControl_SetFocus(MOTOR_AXIS_1);
        s_ratio_x100=100; s_offset_x100=0;
        mark_settings_dirty(HAL_GetTick());
        s_pending_toast=ui_strings.text_defaults_restored; KK_UI_Invalidate();
        break;

    case EVT_SAVE_STATUS:
        s_force_save=true; mark_settings_dirty(HAL_GetTick()); break;
    case EVT_DIAG_CLEAR:
        ok=BoardDiag_Clear(); s_pending_toast=ok ? "Diag cleared" : "Stop first"; break;
    default:
        break;
    }
    return ok;
}

void ui_app_process_events(void)
{
    KK_UI_EventId evt;
    while (KK_UI_PollEvent(&evt)) (void)apply_event(evt);
}

/* Public application bridge: all calls run in main, never in a receive ISR. */
bool App_CanConfigure(void)
{
    ensure_model();
    return !KK_UI_GetCurrentPage() || (KK_UI_GetCurrentPage()!=UI_PAGE_RUN_STEPS &&
        KK_UI_ShowToast(ui_strings.text_remote,500U)!=KK_UI_BUSY);
}

void App_ConfigChanged(void)
{
    ensure_model(); save_view();
    for (unsigned a=0;a<2;++a) {
        load_view((MotorAxis)a); sync_from_storage();
        s_profiles[a]=MotorParams_GetAxisProfile((MotorAxis)a); save_view();
    }
    load_view(MotorControl_GetFocus());
    s_ratio_x100=AppStorage_Get()->gear_ratio_x100;
    s_offset_x100=AppStorage_Get()->gear_offset_x100;
    if (MotorControl_GetInput(s_view_axis)>=MOTOR_INPUT_USB && ui_state()->active) adopt_remote();
    mark_settings_dirty(HAL_GetTick()); KK_UI_Invalidate();
}

bool App_SelectMotor(MotorAxis axis,MotorProfile profile)
{
    if ((unsigned)axis>=MOTOR_AXIS_COUNT) return false;
    ensure_model(); MotorAxis old=s_view_axis;
    save_view(); load_view(axis);
    bool ok=select_motor(profile);
    save_view(); load_view(old); return ok;
}

unsigned App_SaveState(void)
{
    unsigned state=AppStorage_GetSaveState();
    if (s_settings_dirty && AppStorage_IsReady() && state!=APP_SAVE_WRITING)
        state=AppStorage_IsDeferred() ? APP_SAVE_DEFERRED : APP_SAVE_PENDING;
    return state;
}

__attribute__((noinline,noclone)) void App_SaveService(uint32_t now)
{
    if (s_settings_dirty && (s_force_save ||
        (!s_run_active && now-s_settings_save_at>=2000U && now-s_last_input_at>=2000U &&
        (!KK_UI_GetCurrentPage() || KK_UI_IsDisplayIdle())))) {
        if (request_settings_save()) {
            s_settings_dirty=s_force_save=false; refresh_save_state(); KK_UI_Invalidate();
        }
    }
}

static bool save_detection(void)
{
    if (MotorDetect_Active() || !MotorDetect_Result()) return false;
    AppStorage_Get()->motor_detect=MotorDetect_Result();
    AppStorage_Get()->motor_detect_l_uh=MotorDetect_Inductance();
    AppStorage_Get()->motor_detect_flux_uwb=MotorDetect_Flux();
    mark_settings_dirty(HAL_GetTick()); return true;
}

int8_t App_Control(unsigned axis,unsigned key,int32_t value)
{
    if (axis>2 || key>17) return -2;
    const unsigned axis_keys=(1U<<1)|(1U<<2)|(1U<<6)|(1U<<8)|(1U<<9)|(1U<<10);
    bool per_axis=((axis_keys>>key)&1U)!=0;
    if (per_axis != (axis!=0)) return -2;
    if (!App_CanConfigure()) return -3;
    MotorAxis old=s_view_axis;
    save_view(); if (axis) load_view((MotorAxis)(axis-1));
    bool ok=false;
    if (key<6) {
        KK_UI_IntBinding *binding=&int_bindings[key];
        if (value>=binding->minimum && value<=binding->maximum &&
            (key!=3 || value!=0) && (key!=5 || value%100==0)) {
            *binding->value=value; ok=apply_event(binding->changed_event);
        }
    } else if (key<8 && (value==0 || value==1)) {
        *bool_bindings[key-6].value=value!=0;
        ok=apply_event(bool_bindings[key-6].changed_event);
    } else if (key>=8 && key<=10 && value>=1 && value<=100) {
        for (unsigned i=0;i<3;++i) s_step_drafts[i]=s_run_steps[i];
        s_step_drafts[key==8 ? 1 : key==9 ? 0 : 2]=value;
        ok=apply_event(EVT_RUN_STEPS_CHANGED);
    } else if (key==13 && value>=100 && value<=500) {
        s_detect_ma=value;
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        ok=Protocol_RequestCurrent() && MotorDetect_Start((unsigned)value);
        __set_PRIMASK(mask);
        if (ok) App_RemoteMotion(MOTOR_AXIS_1,true);
    } else if (value==0) {
        switch (key) {
        case 11: ok=apply_event(EVT_RESET_CONFIRMED); break;
        case 12: ok=apply_event(EVT_IBUS_CLEAR); break;
        case 14: MotorSampling_GlobalStopIRQ(); MotorControl_StopAll(); ok=true; break;
        case 15: ok=save_detection(); break;
        case 16: ok=apply_event(EVT_SAVE_STATUS); break;
        case 17: ok=apply_event(EVT_DIAG_CLEAR); break;
        default: break;
        }
    }
    if (!ok) sync_from_storage();
    save_view(); load_view(old);
    KK_UI_Invalidate(); return ok ? 1 : -1;
}

static void adopt_remote(void)
{
    const MotorAxisSnapshot *state=ui_state();
    s_run_mode=state->mode; s_run_active=state->active;
    s_run_position_origin=MotorControl_IsCoupled() ? 0 : floorf(state->position/(2.0f*PI))*(2.0f*PI);
    MotorWorkMode work=MotorControl_GetWork();
    s_run_speed_x10=s_speed_x10=to_x10(work==MOTOR_WORK_SYNC_SPEED ? MotorControl_GroupTarget() : state->target_velocity);
    s_run_position_x100=s_position_x100=to_x100((work==MOTOR_WORK_SYNC_POSITION ? MotorControl_GroupTarget() : state->target_position)-s_run_position_origin);
    if (throttle_mode(s_run_mode))
        s_throttle_pct=s_run_throttle_pct=(int32_t)state->target_iq;
    else if (state->current_loop)
        s_confirmed_current_x100=s_run_current_x100=to_x100(state->target_iq);
    else s_run_voltage_x100=s_voltage_x100=to_x100(state->target_iq);
    clear_run_stop(); save_view(); KK_UI_Invalidate();
}

void App_RemoteMotion(MotorAxis axis,bool detect)
{
    ensure_model();
    if (MotorControl_IsHandFollow()) axis=MotorControl_IsPassive(MOTOR_AXIS_1) ? MOTOR_AXIS_2 : MOTOR_AXIS_1;
    save_view(); load_view(axis); MotorControl_SetFocus(axis);
    if (!detect) adopt_remote();
    s_remote_page=detect ? UI_PAGE_DETECT : UI_PAGE_RUN;
    s_remote_at=HAL_GetTick();
    if (!detect) mark_settings_dirty(s_remote_at);
}

void App_CancelNavigation(void) { s_remote_page=0; }

uint8_t App_NavigationKeys(uint32_t now)
{
    if (!s_remote_page) return 0;
    KK_UI_PageId page=KK_UI_GetCurrentPage();
    if (!page || MotorSampling_IsGlobalStopped()) {
        s_remote_page=0; return 0;
    }
    if (page==s_remote_page || KK_UI_GoTo(s_remote_page,now)==KK_UI_OK) {
        s_remote_page=0; return 0;
    }
    /* Cancel local editors / leave custom pages through public input. A
     * release between pulses is required by KK_UI's ordinary debounce. */
    return (now-s_remote_at)%80U<40U ? KK_UI_KEY_BACK : 0;
}

int App_ReadRow(unsigned axis,unsigned group,unsigned row,char *label,char *value)
{
    if (axis<1 || axis>2 || group>=sizeof(info_pages)/sizeof(info_pages[0])) return -2;
    const KK_UI_InfoPage *page=&info_pages[group];
    if (row>=page->row_count) return -2;
    ensure_model();
    if (group==6U && row>=diagnostics_rows_COUNT) {
        const char *name=BoardDiag_Row(axis-1U,row-diagnostics_rows_COUNT,value);
        UiText_Format(label,24,"%s",name); return page->row_count;
    }
    const KK_UI_InfoRow *entry=&page->rows[row];
    UiText_Format(label,24,"%s",entry->name);
    UiText_Format(value,32,"%s",entry->value);
    uintptr_t field=(uintptr_t)entry->value-(uintptr_t)&texts;
    if (field>=sizeof(texts)) return page->row_count; /* Immutable About/HSO literals. */
    if (group==3U && row<sizeof(s_enc_labels)/sizeof(s_enc_labels[0])) label[1]=(char)('0'+axis);
    TextQuery query={.axis=(MotorAxis)(axis-1U),.field=(unsigned)field,
        .value=value,.name=entry->name==texts.target_unit ? label : NULL};
    text_query=&query;
    refresh_info(1U<<group);
    text_query=NULL;
    return page->row_count;
}

/* ---------- Run 页自定义绘制 ---------- */
void KK_UI_CustomOnEnter(KK_UI_PageId page)
{
    if (page == UI_PAGE_DETECT) {
        if (!MotorDetect_Active()) MotorDetect_Reset();
        s_detect_result=AppStorage_Get()->motor_detect;
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
    s_run_mode = ui_state()->mode;
    if (!MotorControl_ModeAllowed(s_run_mode))
        s_run_mode=fallback_mode();
    if (MotorControl_GetInput(s_view_axis)>=MOTOR_INPUT_USB) adopt_remote();
    else { stop_run(); seed_run_drafts(); }
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
                (void)save_detection();
                (void)KK_UI_ShowToast(ui_strings.text_save_queued,900U);
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
        if (s_run_active || views[1U-s_view_axis].run_active || !ui_state()->aligning)
            MotorControl_StopAll();
        s_run_active=views[0].run_active=views[1].run_active=false;
        stop_run(); /* 在退出动画开始前停止，不等待 OnLeave。 */
        (void)KK_UI_CustomRequestClose();
        return;
    }
    /* A boot/global stop can leave the encoder unaligned. An explicit OK
     * requests alignment only; completing it must never apply the Run draft.
     * Keep held-key, hardware-trip and stationary gates in the controller. */
    if (event.action==KK_UI_INPUT_OK && !s_run_active) {
        MotorAxisSnapshot state;MotorControl_GetAxis(s_view_axis,&state);
        if (state.ready && state.healthy && (!state.aligned || state.aligning)) {
            const char *message=ui_strings.text_aligning;
            if (!state.aligning) {
                if (MotorControl_RequestAlign(s_view_axis)) {
                    clear_run_stop();message=ui_strings.text_align_requested;
                } else message=MotorSampling_IsGlobalStopped() ? ui_strings.text_global_stop : ui_strings.text_stop_check_axis;
            }
            (void)KK_UI_ShowToast(message,1200U);KK_UI_Invalidate();return;
        }
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
        MotorAxisSnapshot state; MotorControl_GetAxis(s_view_axis,&state);
        bool changed = run_target_changed();
        if ((s_view_axis==MOTOR_AXIS_1 && JerryFOC_hasHSOFault())) {
            (void)JerryFOC_useHSO(0U);
            stop_run();
            (void)KK_UI_ShowToast(ui_strings.text_hso_reset, 900U);
        } else if (s_run_active && !changed) {
            MotorControl_Stop(s_view_axis);
            stop_run();
        } else if (!state.ready) {
            (void)KK_UI_ShowToast(state.status, 900U);
        } else if (!state.healthy || !state.aligned) {
            (void)KK_UI_ShowToast(state.healthy ? ui_strings.text_not_aligned : MotorControl_EncoderLostText(s_view_axis), 900U);
        } else if (!s_run_active && !MotorSampling_IsAxisOutputRequested(s_view_axis) &&
                   (fabsf(state.velocity) > 1.0f ||
                    (MotorControl_HsoAllowed() && s_run_speed_x10!=0 && !StHso_AtRest()))) {
            (void)KK_UI_ShowToast(ui_strings.text_wait_for_stop, 900U);
        } else if (sensorless() && s_run_mode==JERRYFOC_MODE_VELOCITY &&
                   (hso_draft(s_run_speed_x10)!=s_run_speed_x10 ||
                    (MotorSampling_IsAxisOutputRequested(s_view_axis) && state.target_velocity*(float)s_run_speed_x10<0.0f))) {
            (void)KK_UI_ShowToast(ui_strings.text_stop_check_range,1200U);
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
            (void)KK_UI_ShowToast(ui_strings.text_raise_speed_limit,1200U);
        } else {
            if (*draft!=0 && ((*draft>0 && next<min) || (*draft<0 && next>-min))) next=0;
            else next=hso_draft(next);
            if (MotorSampling_IsAxisOutputRequested(s_view_axis) && ui_state()->target_velocity*(float)next<0.0f)
                (void)KK_UI_ShowToast(ui_strings.text_stop_before_reverse,1200U);
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

/* Custom pages share the same footer geometry and clip lifetime. */
__attribute__((noinline,noclone)) static void draw_footer(int16_t x,uint16_t width,const char *text)
{
    OLED_DrawHLine(x,65,width);
    OLED_DrawUTF8((int16_t)(4+x),67,text);
    OLED_ResetClipWindow();
}

void KK_UI_CustomOnDraw(KK_UI_PageId page, int16_t x_offset,
                        int16_t clip_x, uint16_t clip_width)
{
    if (page != UI_PAGE_RUN && page != UI_PAGE_RUN_STEPS && page != UI_PAGE_DETECT) return;
    const int16_t W = (int16_t)OLED_GetWidth();
    const int16_t H = (int16_t)OLED_GetHeight();
    char buf[40];
    char v[16];
    OLED_SetClipWindow(clip_x, 0, clip_width, (uint16_t)H);
    OLED_SetDrawMode(OLED_DRAW_SET);
    if (page == UI_PAGE_DETECT) {
        OLED_SetFont(ui_font_body);
        OLED_DrawUTF8((int16_t)(4+x_offset),1,ui_strings.text_m1_detect);
        OLED_DrawHLine(x_offset,14,(uint16_t)W);
        UiText_Format(buf,sizeof(buf),ui_strings.fmt_i_test_ld_ma,(long)s_detect_ma);
        OLED_DrawUTF8((int16_t)(4+x_offset),17,buf);
        uint32_t detected=MotorDetect_Result();
        OLED_DrawUTF8((int16_t)(4+x_offset),33,!detected && s_detect_result ? ui_strings.text_saved : MotorDetect_Status());
        if(detected) {
            s_detect_result=detected; s_detect_l=MotorDetect_Inductance(); s_detect_flux=MotorDetect_Flux();
        }
        int p=(int8_t)s_detect_result;
        fmt_fixed(v,sizeof(v),(int32_t)((s_detect_result>>8)&65535U)/10,100);
        UiText_Format(buf,sizeof(buf),ui_strings.fmt_pp_u_s_r_s_ohm,(unsigned)abs(p),p<0 ? "-" : "+",v);
        if(s_detect_page==1) UiText_Format(buf,sizeof(buf),ui_strings.fmt_ls_lu_uh,(unsigned long)s_detect_l);
        if(s_detect_page==2) UiText_Format(buf,sizeof(buf),ui_strings.fmt_flux_lu_uwb,(unsigned long)s_detect_flux);
        if(s_detect_page==3) {
            fmt_fixed(v,sizeof(v),(int32_t)((float)s_detect_flux*(float)abs(p)*0.0128254983f+0.5f),100);
            UiText_Format(buf,sizeof(buf),ui_strings.fmt_ke_s_v_krpm,v); /* Line-line RMS / mechanical krpm. */
        }
        if(s_detect_page && !(s_detect_page==1 ? s_detect_l : s_detect_flux)) strcpy(buf,ui_strings.text_no_data);
        OLED_DrawUTF8((int16_t)(4+x_offset),49,s_detect_result ? buf : ui_strings.text_no_data);
        draw_footer(x_offset,(uint16_t)W,MotorDetect_Active() ? ui_strings.text_ok_stop_k2_back :
            detected ? ui_strings.text_ok_save_lr_view : ui_strings.text_ok_start_k2_back);
        return;
    }
    if (page == UI_PAGE_RUN_STEPS) {
        /* Compact header keeps its existing baseline; Home uses the restored
         * family line box, independently of which title glyphs were retained. */
        OLED_SetFontRefHeight(OLED_FONT_REF_TEXT);
        OLED_SetFont(ui_font_title);
        OLED_DrawUTF8((int16_t)(4 + x_offset), 1, texts.steps);
        const char *save = ui_strings.text_ok_save;
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
            const char *unit = i == JERRYFOC_MODE_VELOCITY ? ui_strings.text_rad_s :
                               i == JERRYFOC_MODE_POSITION ? "rad" : s_current_loop ? "A" : "V";
            UiText_Fixed(buf, sizeof(buf), s_step_drafts[i], i == JERRYFOC_MODE_VELOCITY ? 1U : 2U, unit);
            OLED_DrawUTF8((int16_t)(W - 5 - OLED_GetUTF8Width(buf) + x_offset), y, buf);
            OLED_SetDrawMode(OLED_DRAW_SET);
        }
        draw_footer(x_offset,(uint16_t)W,ui_strings.text_l_r_step_k2_cancel);
        return;
    }
    /* Use the live, validated bus measurement in every Run variant. */
    MotorAxisSnapshot motor_state; MotorControl_GetAxis(s_view_axis,&motor_state);
    OLED_SetFont(ui_font_body);
    const char *bus=!strcmp(texts.bus,ui_strings.text_no_data) ? "--V" : texts.bus;
    OLED_DrawUTF8((int16_t)(W-4-OLED_GetUTF8Width(bus)+x_offset),1,bus);
    if (MotorControl_IsHandFollow()) {
        unsigned master=MotorControl_GetWork()==MOTOR_WORK_MASTER_M2 ? 2U : 1U;
        bool active=MotorControl_IsGroupActive();
        UiText_Format(buf,sizeof(buf),ui_strings.fmt_hand_m_u_m_u_enc,master,3U-master);
        OLED_DrawUTF8((int16_t)(4+x_offset),1,buf);
        OLED_DrawHLine(x_offset,14,(uint16_t)W);
        OLED_DrawUTF8((int16_t)(4+x_offset),19,ui_strings.text_turn_master_by_hand);
        fmt_fixed(v,sizeof(v),to_x100(MotorControl_GroupError()),100);
        UiText_Format(buf,sizeof(buf),ui_strings.fmt_error_s_rad,v);
        OLED_DrawUTF8((int16_t)(4+x_offset),35,buf);
        OLED_DrawUTF8((int16_t)(4+x_offset),51,active ? ui_strings.text_run_following :
            strcmp(texts.run_stop,"--") ? texts.run_stop : ui_strings.text_stop_ok_to_set_zero);
        draw_footer(x_offset,(uint16_t)W,active ? ui_strings.text_ok_stop_k2_back : ui_strings.text_ok_start_k2_back);
        return;
    }
    bool hso=s_view_axis==MOTOR_AXIS_1 && MotorControl_HsoAllowed();
    bool six=s_view_axis==MOTOR_AXIS_1 && MotorControl_SixStepAllowed();
    bool oil=throttle_mode(s_run_mode);
    bool independent=MotorControl_GetWork()==MOTOR_WORK_INDEPENDENT;
    bool starting=hso && StHso_IsStarting();
    const char *mode=oil ? ui_strings.text_throttle : s_run_mode==JERRYFOC_MODE_TORQUE ? (s_current_loop ? ui_strings.text_iq_a : ui_strings.text_uq_v) : mode_name(s_run_mode);
    UiText_Format(buf,sizeof(buf),ui_strings.fmt_m_u_s_s,(unsigned)s_view_axis+1U,mode,six ? (esc_view() ? "ESC" : ui_strings.text_6step) : hso ? ui_strings.text_hso : "ENC");
    OLED_DrawUTF8((int16_t)(4+x_offset),1,buf);
    OLED_DrawHLine(x_offset,14,(uint16_t)W);
    bool changed=run_target_changed();
    const char *state=changed ? (s_run_active ? ui_strings.text_run : ui_strings.text_edit) : (s_run_active ? "RUN" : ui_strings.text_stop);
    const char *unit=oil ? "%" : s_run_mode==JERRYFOC_MODE_VELOCITY ? ui_strings.text_rad_s :
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
    if (MotorSampling_IsGlobalStopped()) fault=ui_strings.text_global_stop;
    else if (!motor_state.ready) fault=motor_state.status;
    else if (!s_run_active && strcmp(texts.run_stop,"--")) fault=texts.run_stop;
    else if (!motor_state.healthy || !motor_state.aligned)
        fault=motor_state.healthy ? (motor_state.aligning ? ui_strings.text_aligning : ui_strings.text_not_aligned) : MotorControl_EncoderLostText(s_view_axis);
    if (fault) UiText_Format(buf,sizeof(buf),"%s",fault);
    else {
        const char *speed_tag=starting ? "Ref" : six ? "Est" : "Act";
        int32_t actual=s_run_mode==JERRYFOC_MODE_POSITION ?
            to_x100(motor_state.position-s_run_position_origin) : to_x10(motor_state.velocity);
        fmt_fixed(v,sizeof(v),actual,s_run_mode==JERRYFOC_MODE_POSITION ? 100 : 10);
        if (changed) {
            char applied[16]; fmt_fixed(applied,sizeof(applied),*run_target(false),scale);
            UiText_Format(buf,sizeof(buf),ui_strings.fmt_s_s_set_s,speed_tag,v,applied);
        } else if (hso && *run_target(true)==0) UiText_Format(buf,sizeof(buf),ui_strings.fmt_act_s_0_coast,v);
        else if (six || s_run_mode!=JERRYFOC_MODE_TORQUE) {
            char aux[16]; fmt_run_aux(aux,actual);
            UiText_Format(buf,sizeof(buf),ui_strings.fmt_s_s_s,speed_tag,v,aux);
        } else UiText_Format(buf,sizeof(buf),ui_strings.fmt_act_s_max_ld,v,(long)s_speed_limit);
    }
    OLED_DrawUTF8((int16_t)(4+x_offset),38,buf);
    if (independent) {
        /* Read both actual output requests; a failed/unviewed axis must not
         * remain labelled RUN merely because its UI view was cached. */
        for (unsigned a=0;a<2;++a) {
            UiText_Format(buf,sizeof(buf),ui_strings.fmt_m_u_s_s_133,a+1U,
                MotorSampling_IsAxisOutputRequested((MotorAxis)a) ? "RUN" : ui_strings.text_stop,
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
        UiText_Format(buf,sizeof(buf),ui_strings.fmt_s_hso_s,state,StHso_StateText(snapshot.state));
        OLED_DrawUTF8((int16_t)(4+x_offset),51,buf);
    } else if (!fault) {
        if (MotorControl_IsCoupled()) {
            MotorAxisSnapshot other; MotorControl_GetAxis((MotorAxis)(1U-s_view_axis),&other);
            fmt_fixed(v,sizeof(v),to_x10(other.velocity),10);
            UiText_Format(buf,sizeof(buf),ui_strings.fmt_s_m_u_s,state,(unsigned)(2U-s_view_axis),v);
        } else {
            fmt_fixed(v,sizeof(v),to_x100(motor_state.iq),100);
            UiText_Format(buf,sizeof(buf),ui_strings.fmt_s_iq_s_a,state,v);
        }
        if (!MotorControl_IsCoupled() && s_run_mode!=JERRYFOC_MODE_TORQUE) {
            fmt_run_aux(v,*run_target(true));
        } else {
            char step[10]; fmt_fixed(step,sizeof(step),s_run_steps[s_run_mode],scale);
            UiText_Format(v,sizeof(v),ui_strings.fmt_step_s,step);
        }
        int16_t right=(int16_t)(W-4-OLED_GetUTF8Width(v)+x_offset);
        if (4+x_offset+OLED_GetUTF8Width(buf)+4>right) strcpy(buf,state);
        OLED_DrawUTF8((int16_t)(4+x_offset),51,buf);
        OLED_DrawUTF8(right,51,v);
    } else OLED_DrawUTF8((int16_t)(4+x_offset),51,state);
    const char *action=!s_run_active && (!motor_state.aligned || motor_state.aligning) ? ui_strings.text_align :
        (hso || six) && *run_target(true)==0 ? ui_strings.text_stop_138 :
        s_run_active ? (changed ? ui_strings.text_apply : ui_strings.text_stop_138) : ui_strings.text_start;
    UiText_Format(buf,sizeof(buf),independent ? ui_strings.fmt_ok_s_lr_hold_k2_back : ui_strings.fmt_ok_s_k2_back,action);
    draw_footer(x_offset,(uint16_t)W,buf);
}
