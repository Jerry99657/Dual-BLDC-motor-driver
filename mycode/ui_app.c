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
};

/* ---------- 设置菜单下标 ---------- */
enum { INT_LED_BRIGHT, INT_POSITION_SPEED, INT_SPEED_LIMIT };
enum { BOOL_CURRENT_LOOP, BOOL_LED };
enum { CONFIRM_RESET, CONFIRM_MOTOR_2804, CONFIRM_MOTOR_4015 };

/* ---------- 设置变量 ---------- */
static int32_t s_speed_x10;
static int32_t s_voltage_x100;
static int32_t s_position_x100;
static int32_t s_led_brightness;
static int32_t s_position_speed = 30;
static int32_t s_speed_limit = APP_SPEED_LIMIT_DEFAULT;
static bool s_current_loop;
static bool s_led_enable;
/* 下标与 JerryFOC_ControlMode 对应：Torque、Speed、Position。 */
static int32_t s_run_steps[3] = {5, 5, 5};
static int32_t s_step_drafts[3];
static uint8_t s_step_selected;

/* Run 页当前目标 */
static JerryFOC_ControlMode s_run_mode;
static int32_t s_run_speed_x10;
static int32_t s_run_voltage_x100;
static int32_t s_run_position_x100;
static int32_t s_run_current_x100;
static int32_t s_confirmed_current_x100;
static float s_run_position_origin;
static bool s_run_active;
static uint32_t s_last_input_at;
static uint32_t s_last_live_at;

/* ---------- 动态文本 ---------- */
static char s_txt_mode[16];
static char s_txt_motor[8], s_txt_motor_menu[20];
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
static char s_txt_hso_jitter[24], s_txt_hso_reset[24];
static char s_txt_feedback[16];
static uint32_t s_input_repeats;
static uint32_t s_display_errors;

static uint32_t s_info_at;
static bool s_settings_dirty;
static uint32_t s_settings_save_at;
static const char *s_pending_toast;

/* ---------- 工具 ---------- */
static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int32_t to_x10(float v)
{
    return (int32_t)(v * 10.0f + (v >= 0.0f ? 0.5f : -0.5f));
}

static int32_t to_x100(float v)
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

static const char *mode_name(JerryFOC_ControlMode mode)
{
    switch (mode) {
    case JERRYFOC_MODE_VELOCITY: return "Speed";
    case JERRYFOC_MODE_POSITION: return "Position";
    default: return "Torque";
    }
}

static const char *drive_status_text(void)
{
    const char *sampling = MotorSampling_GetStatusText();
    /* Keep an actual ADC/bus/current fault visible after leaving HSO. */
    return JerryFOC_hasHSOFault() && strcmp(sampling, "READY") == 0 ? "HSO STOPPED" : sampling;
}

static void refresh_texts(void)
{
    char v[16];
    snprintf(s_txt_motor, sizeof(s_txt_motor), "%s", JerryFOC_getMotorParams()->name);
    snprintf(s_txt_motor_menu, sizeof(s_txt_motor_menu), "Motor: %s", s_txt_motor);
    snprintf(s_txt_output, sizeof(s_txt_output), "%s",
             !MotorSampling_IsOutputRequested() ? "COAST" :
             MotorSampling_IsOutputEnabled() ? "PWM" : "OFF");
    snprintf(s_txt_repeat, sizeof(s_txt_repeat), "%lu", (unsigned long)s_input_repeats);
    snprintf(s_txt_display_errors, sizeof(s_txt_display_errors), "%lu", (unsigned long)s_display_errors);

    snprintf(s_txt_mode, sizeof(s_txt_mode), "%s", mode_name(JerryFOC_getMode()));

    switch (JerryFOC_getMode()) {
    case JERRYFOC_MODE_VELOCITY:
        fmt_fixed(v, sizeof(v), to_x10(JerryFOC_getTargetVelocity()), 10);
        snprintf(s_txt_target, sizeof(s_txt_target), "%s rad/s", v);
        break;
    case JERRYFOC_MODE_POSITION:
        fmt_fixed(v, sizeof(v), to_x100(JerryFOC_Target_Position), 100);
        snprintf(s_txt_target, sizeof(s_txt_target), "%s rad", v);
        break;
    default:
        fmt_fixed(v, sizeof(v), to_x100(JerryFOC_Target_Iq), 100);
        snprintf(s_txt_target, sizeof(s_txt_target), "%s %s", v,
                 s_current_loop ? "A" : "V");
        break;
    }

    fmt_fixed(v, sizeof(v), to_x10(JerryFOC_getVelocity()), 10);
    snprintf(s_txt_speed, sizeof(s_txt_speed), "%s rad/s", v);

    fmt_fixed(v, sizeof(v), to_x100(JerryFOC_getIq()), 100);
    snprintf(s_txt_iq, sizeof(s_txt_iq), "%s A", v);

    fmt_fixed(v, sizeof(v), to_x100(JerryFOC_getId()), 100);
    snprintf(s_txt_id, sizeof(s_txt_id), "%s A", v);

    fmt_fixed(v, sizeof(v), to_x100(JerryFOC_getPhaseCurrent_A()), 100);
    snprintf(s_txt_ia, sizeof(s_txt_ia), "%s A", v);

    fmt_fixed(v, sizeof(v), to_x100(JerryFOC_getPhaseCurrent_B()), 100);
    snprintf(s_txt_ib, sizeof(s_txt_ib), "%s A", v);

    snprintf(s_txt_enc, sizeof(s_txt_enc), "%s",
             MT6701_IsHealthy() ? "OK" : "ERROR");
    MT6701_2_Snapshot encoder2;
    MT6701_2_GetSnapshot(&encoder2);
    snprintf(s_txt_enc2, sizeof(s_txt_enc2), "%s %u",
             encoder2.healthy ? "OK" : "ERROR", (unsigned)encoder2.raw_angle);
    snprintf(s_txt_enc2_err, sizeof(s_txt_enc2_err), "%lu", (unsigned long)encoder2.errors);
    snprintf(s_txt_drive, sizeof(s_txt_drive), "%s",
             drive_status_text());
    StHsoSnapshot hso;
    StHso_GetSnapshot(&hso);
    snprintf(s_txt_feedback, sizeof(s_txt_feedback), "%s", JerryFOC_getFeedbackText());
    snprintf(s_txt_hso_gate, sizeof(s_txt_hso_gate), "%s",
             StHso_ReasonText(JerryFOC_isHSOActive() ? hso.reason : hso.entry_reason));
    snprintf(s_txt_hso_qual, sizeof(s_txt_hso_qual), "%lu/200 ms %lu/20",
             (unsigned long)(hso.qualified_samples / 10U), (unsigned long)hso.bad_samples);
    snprintf(s_txt_hso_jitter, sizeof(s_txt_hso_jitter), "%lu/50 %lu/10",
             (unsigned long)hso.qualify_paused_samples, (unsigned long)hso.qualify_bad_run);
    fmt_fixed(v, sizeof(v), to_x10(hso.qualify_reset_angle_rad * 57.2957795f), 10);
    snprintf(s_txt_hso_reset, sizeof(s_txt_hso_reset), "%s deg", v);
    snprintf(s_txt_hso_lost, sizeof(s_txt_hso_lost), "%s", JerryFOC_getHSOFaultText());
    fmt_fixed(v, sizeof(v), to_x10(hso.flux_wb * 1000.0f), 10);
    snprintf(s_txt_hso_flux_actual, sizeof(s_txt_hso_flux_actual), "%s mWb", v);
    snprintf(s_txt_hso, sizeof(s_txt_hso), "%s",
             JerryFOC_isHSOActive() ? (JerryFOC_isHSOReturning() ? "RETURNING" : "CONTROL") :
             (JerryFOC_hasHSOFault() ? "LOST" : StHso_StateText(hso.state)));
    fmt_fixed(v, sizeof(v), to_x10(hso.velocity_rad_s), 10);
    snprintf(s_txt_hso_speed, sizeof(s_txt_hso_speed), "%s rad/s", v);
    fmt_fixed(v, sizeof(v), to_x10(hso.angle_error_rad * 57.2957795f), 10);
    snprintf(s_txt_hso_error, sizeof(s_txt_hso_error), "%s deg", v);
    snprintf(s_txt_hso_flux, sizeof(s_txt_hso_flux), "%lu uWb", (unsigned long)(hso.reference_flux_wb * 1000000.0f));
    snprintf(s_txt_hso_time, sizeof(s_txt_hso_time), "%lu us / %lu", (unsigned long)(hso.max_control_cycles / 170U), (unsigned long)hso.overruns);
    snprintf(s_txt_align, sizeof(s_txt_align), "%s",
             AngleSrc_Encoder_IsAligned() ? "YES" : "NO");
    snprintf(s_txt_raw, sizeof(s_txt_raw), "%u",
             (unsigned)MT6701_GetRawAngle());

    fmt_fixed(v, sizeof(v), to_x100(JerryFOC_getElectricalAngle()), 100);
    snprintf(s_txt_elec, sizeof(s_txt_elec), "%s rad", v);

    fmt_fixed(v, sizeof(v), to_x100(JerryFOC_getAngle()), 100);
    snprintf(s_txt_mech, sizeof(s_txt_mech), "%s rad", v);

    snprintf(s_txt_err, sizeof(s_txt_err), "%lu",
             (unsigned long)MT6701_GetErrorCounter());
    MT6701Diagnostics enc;
    MT6701_GetDiagnostics(&enc);
    snprintf(s_txt_enc_crc, sizeof(s_txt_enc_crc), "%lu", (unsigned long)enc.crc_errors);
    snprintf(s_txt_enc_field, sizeof(s_txt_enc_field), "%lu", (unsigned long)enc.field_errors);
    snprintf(s_txt_enc_track, sizeof(s_txt_enc_track), "%lu", (unsigned long)enc.track_errors);
    snprintf(s_txt_enc_spi, sizeof(s_txt_enc_spi), "%lu", (unsigned long)enc.spi_errors);
    snprintf(s_txt_enc_frame, sizeof(s_txt_enc_frame), "%lu", (unsigned long)enc.frame_errors);
    snprintf(s_txt_enc_age, sizeof(s_txt_enc_age), "%u / %u", enc.age_cycles, enc.max_age_cycles);
    snprintf(s_txt_enc_last, sizeof(s_txt_enc_last), "%s", MT6701_ErrorText(enc.last_error));

    MotorSamplingDiagnostics adc;
    MotorSampling_GetDiagnostics(&adc);
    unsigned reference_mv = (unsigned)(adc.reference_voltage * 1000.0f + 0.5f);
    if (adc.reference_updates != 0U)
        snprintf(s_txt_reference, sizeof(s_txt_reference), "%u.%03u V",
                 reference_mv / 1000U, reference_mv % 1000U);
    else
        snprintf(s_txt_reference, sizeof(s_txt_reference), "NO DATA");
    snprintf(s_txt_ref_state, sizeof(s_txt_ref_state), "%s",
             adc.reference_fresh ? "OK" : (adc.reference_updates ? "STALE" : "WAIT"));
    snprintf(s_txt_ref_raw, sizeof(s_txt_ref_raw), "%u / %u",
             (unsigned)adc.reference_raw, (unsigned)adc.reference_cal);
    snprintf(s_txt_ref_count, sizeof(s_txt_ref_count), "%lu", (unsigned long)adc.reference_updates);
    snprintf(s_txt_ref_errors, sizeof(s_txt_ref_errors), "%lu", (unsigned long)adc.reference_errors);
    snprintf(s_txt_adc_revision, sizeof(s_txt_adc_revision), "0x%04X", (unsigned)adc.silicon_revision);
    for (unsigned i = 0U; i < 3U; ++i) {
        snprintf(s_txt_current_adc[i], sizeof(s_txt_current_adc[i]), "%u / %u",
                 (unsigned)adc.current_raw[i], (unsigned)adc.current_zero[i]);
    }
    if (adc.fault_phase != 0U) {
        unsigned i = adc.fault_phase - 1U;
        snprintf(s_txt_current_trip, sizeof(s_txt_current_trip), "%s %c %u",
                 adc.fault_during_zero ? "ZERO" : "RUN", "UVW"[i],
                 (unsigned)adc.fault_current_raw[i]);
    } else {
        snprintf(s_txt_current_trip, sizeof(s_txt_current_trip), "--");
    }
    if (adc.bus_fresh) {
        fmt_fixed(v, sizeof(v), to_x10(adc.bus_voltage), 10);
        snprintf(s_txt_bus, sizeof(s_txt_bus), "%s V", v);
    } else {
        snprintf(s_txt_bus, sizeof(s_txt_bus), "NO DATA");
    }
    snprintf(s_txt_bus_raw, sizeof(s_txt_bus_raw), "%u", (unsigned)adc.bus_raw);
    snprintf(s_txt_bus_count, sizeof(s_txt_bus_count), "%lu", (unsigned long)adc.bus_samples);
    snprintf(s_txt_frame_count, sizeof(s_txt_frame_count), "%lu", (unsigned long)adc.frame_samples);
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
    {"Curr Loop",  KK_UI_MENU_BOOL,   BOOL_CURRENT_LOOP},
    {"Status LED", KK_UI_MENU_BOOL,   BOOL_LED},
    {"LED Bright", KK_UI_MENU_INT,    INT_LED_BRIGHT},
    {"Pos Speed",  KK_UI_MENU_INT,    INT_POSITION_SPEED},
    {"Speed Limit", KK_UI_MENU_INT,   INT_SPEED_LIMIT},
    {"Run Steps",  KK_UI_MENU_PAGE, UI_PAGE_RUN_STEPS},
    {"Motor",      KK_UI_MENU_PAGE, UI_PAGE_MOTOR},
    {"Reset",      KK_UI_MENU_CONFIRM, CONFIRM_RESET},
};

static const KK_UI_MenuItem motor_items[] = {
    {"2804", KK_UI_MENU_CONFIRM, CONFIRM_MOTOR_2804},
    {"4015", KK_UI_MENU_CONFIRM, CONFIRM_MOTOR_4015},
};

static const KK_UI_MenuPage menu_pages[] = {
    {"Settings", settings_items, NULL,
     sizeof(settings_items) / sizeof(settings_items[0])},
    {s_txt_motor_menu, motor_items, NULL,
     sizeof(motor_items) / sizeof(motor_items[0])},
};

static const KK_UI_InfoRow status_rows[] = {
    {"Motor",   s_txt_motor},
    {"Mode",    s_txt_mode},
    {"Bus",     s_txt_bus},
    {"VREF",    s_txt_reference},
    {"RefState",s_txt_ref_state},
    {"Drive",   s_txt_drive},
    {"Output",  s_txt_output},
    {"Run Stop",s_txt_run_stop},
    {"Stop Age",s_txt_stop_age},
    {"Stop Err",s_txt_stop_errors},
    {"I Trip",  s_txt_current_trip},
    {"IU R/Z",  s_txt_current_adc[0]},
    {"IV R/Z",  s_txt_current_adc[1]},
    {"IW R/Z",  s_txt_current_adc[2]},
    {"ChipRev", s_txt_adc_revision},
    {"KeyRpt",  s_txt_repeat},
    {"DispErr", s_txt_display_errors},
    {"Target",  s_txt_target},
    {"Speed",   s_txt_speed},
    {"Iq",      s_txt_iq},
    {"Id",      s_txt_id},
    {"Phase A", s_txt_ia},
    {"Phase B", s_txt_ib},
    {"Encoder", s_txt_enc},
    {"Enc CRC", s_txt_enc_crc},
    {"Enc Field",s_txt_enc_field},
    {"Enc Track",s_txt_enc_track},
    {"Enc SPI", s_txt_enc_spi},
    {"Enc Frame",s_txt_enc_frame},
    {"Enc Age", s_txt_enc_age},
    {"Enc Last",s_txt_enc_last},
    {"Aligned", s_txt_align},
    {"Raw",     s_txt_raw},
    {"Elec",    s_txt_elec},
    {"Mech",    s_txt_mech},
    {"Errors",  s_txt_err},
    {"BusRaw",  s_txt_bus_raw},
    {"BusCnt",  s_txt_bus_count},
    {"FrameCnt",s_txt_frame_count},
    {"ADC1Err", s_txt_adc1_error},
    {"ADC2Err", s_txt_adc2_error},
    {"Ref R/C", s_txt_ref_raw},
    {"RefCnt",  s_txt_ref_count},
    {"RefErr",  s_txt_ref_errors},
    {"Enc2",    s_txt_enc2},
    {"Enc2Err", s_txt_enc2_err},
    {"HSO",     s_txt_hso},
    {"Feedback",s_txt_feedback},
    {"H Gate",  s_txt_hso_gate},
    {"H Qual",  s_txt_hso_qual},
    {"H Jitter",s_txt_hso_jitter},
    {"H Reset", s_txt_hso_reset},
    {"H Lost",  s_txt_hso_lost},
    {"H Speed", s_txt_hso_speed},
    {"H Error", s_txt_hso_error},
    {"H Flux",  s_txt_hso_flux},
    {"H Actual",s_txt_hso_flux_actual},
    {"H Time",  s_txt_hso_time},
};

static const KK_UI_InfoRow about_rows[] = {
    {"FW",      "UI-R26-COAST"},
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
    {"Status", status_rows, sizeof(status_rows) / sizeof(status_rows[0])},
    {"About",  about_rows,  sizeof(about_rows) / sizeof(about_rows[0])},
};

/* 下标 = 页面 ID - 1 */
static const KK_UI_PageRoute routes[] = {
    {KK_UI_PAGE_HOME,   0U},
    {KK_UI_PAGE_CUSTOM, 0U},
    {KK_UI_PAGE_MENU,   0U},
    {KK_UI_PAGE_INFO,   0U},
    {KK_UI_PAGE_INFO,   1U},
    {KK_UI_PAGE_CUSTOM, 1U},
    {KK_UI_PAGE_MENU,   1U},
};

static KK_UI_IntBinding int_bindings[] = {
    [INT_LED_BRIGHT] = {"LED Brightness", &s_led_brightness, 0, 100, 5U, "%", EVT_LED_CHANGED},
    [INT_POSITION_SPEED] = {"Position Speed", &s_position_speed, 1, 60, 1U, "rad/s", EVT_POSITION_SPEED_CHANGED},
    [INT_SPEED_LIMIT] = {"Speed Limit", &s_speed_limit, 1, APP_SPEED_LIMIT_MAX, 1U, "rad/s", EVT_SPEED_LIMIT_CHANGED},
};

static KK_UI_BoolBinding bool_bindings[] = {
    [BOOL_CURRENT_LOOP] = {"Current Loop", &s_current_loop, EVT_CURRENT_LOOP_CHANGED},
    [BOOL_LED]          = {"Status LED",   &s_led_enable,   EVT_LED_CHANGED},
};

static const KK_UI_ConfirmDesc confirm_descs[] = {
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
    .texts = {"Back", "Cancel", "OK", "On", "Off", "Notice"},
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
    /* Entering/backing out of an idle Run page must not interrupt the
     * independent startup alignment. An explicit serial Stop still can. */
    if (!s_run_active && JerryFOC_isAligning()) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    JerryFOC_stop();
    JerryFOC_useCurrentLoop(0U);
    s_run_active = false;
    __set_PRIMASK(primask);
}

static void apply_run_target(void)
{
    if (!s_run_active) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_openloop_debug = 0U;
    JerryFOC_useCurrentLoop(s_current_loop ? 1U : 0U);
    /* Set mode before target so recovery and speed-change policy apply to
     * this command, including the first Run confirmation after an HSO stop. */
    JerryFOC_setMode(s_run_mode);
    switch (s_run_mode) {
    case JERRYFOC_MODE_VELOCITY:
        JerryFOC_setVelocity((float)s_run_speed_x10 / 10.0f);
        break;
    case JERRYFOC_MODE_POSITION:
        /* 本次启动固定原点，连续调参不会随着转子跨圈重新选原点。 */
        JerryFOC_setAbsolutePosition(s_run_position_origin +
            (float)s_run_position_x100 / 100.0f, (float)s_position_speed);
        break;
    default:
        JerryFOC_setCurrent((float)(s_current_loop ? s_run_current_x100 :
                                   s_run_voltage_x100) / 100.0f);
        break;
    }
    (void)JerryFOC_startOutput();
    __set_PRIMASK(primask);
}

static void mark_settings_dirty(uint32_t now_ms)
{
    s_settings_dirty = true;
    s_settings_save_at = now_ms;
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

static uint32_t settings_flags(void)
{
    return (s_current_loop ? APP_FLAG_CURRENT_LOOP : 0U) |
           (s_led_enable ? APP_FLAG_LED_ENABLE : 0U) |
           ((uint32_t)s_position_speed << APP_POSITION_SPEED_SHIFT) |
           ((uint32_t)s_speed_limit << APP_SPEED_LIMIT_SHIFT) |
           (MotorParams_GetActiveProfile() == MOTOR_PROFILE_4015 ? APP_FLAG_MOTOR_4015 : 0U);
}

static bool request_settings_save(void)
{
    AppSettings *st = AppStorage_Get();
    st->speed_x10 = s_speed_x10;
    st->voltage_x100 = s_voltage_x100;
    st->position_x100 = s_position_x100;
    st->led_brightness = (uint32_t)s_led_brightness;
    st->flags = settings_flags();
    st->speed_step_x10 = s_run_steps[JERRYFOC_MODE_VELOCITY];
    st->torque_step_x100 = s_run_steps[JERRYFOC_MODE_TORQUE];
    st->position_step_x100 = s_run_steps[JERRYFOC_MODE_POSITION];

    if (!AppStorage_IsReady()) {
        s_pending_toast = "Save failed";
        return true;
    }
    return AppStorage_RequestSave();
}

static void sync_from_storage(void)
{
    AppSettings *st = AppStorage_Get();

    /* 防止 W25Q 中的旧记录/损坏记录把绑定值带到编辑器范围之外 */
    s_speed_limit = (int32_t)((st->flags & APP_SPEED_LIMIT_MASK) >> APP_SPEED_LIMIT_SHIFT);
    if (s_speed_limit < 1 || s_speed_limit > APP_SPEED_LIMIT_MAX)
        s_speed_limit = APP_SPEED_LIMIT_DEFAULT;
    int32_t profile_max = (int32_t)JerryFOC_getMotorParams()->max_speed_mech;
    int_bindings[INT_SPEED_LIMIT].maximum = profile_max;
    s_speed_limit = clamp_i32(s_speed_limit, 1, profile_max);
    s_speed_x10 = clamp_i32(st->speed_x10, -s_speed_limit * 10, s_speed_limit * 10);
    int32_t voltage_max = to_x100(JerryFOC_getMotorParams()->vel_limit);
    s_voltage_x100 = clamp_i32(st->voltage_x100, -voltage_max, voltage_max);
    s_position_x100 = clamp_i32(st->position_x100, -314, 314);
    s_led_brightness = clamp_i32((int32_t)st->led_brightness, 0, 100);
    s_current_loop = (st->flags & APP_FLAG_CURRENT_LOOP) != 0U;
    s_led_enable = (st->flags & APP_FLAG_LED_ENABLE) != 0U;
    s_position_speed = (int32_t)((st->flags & APP_POSITION_SPEED_MASK) >> APP_POSITION_SPEED_SHIFT);
    if (s_position_speed < 1 || s_position_speed > 60) s_position_speed = 30;
    s_run_steps[JERRYFOC_MODE_VELOCITY] = st->speed_step_x10;
    s_run_steps[JERRYFOC_MODE_TORQUE] = st->torque_step_x100;
    s_run_steps[JERRYFOC_MODE_POSITION] = st->position_step_x100;
    for (unsigned i = 0U; i < 3U; ++i)
        if (s_run_steps[i] < 1 || s_run_steps[i] > 100) s_run_steps[i] = 5;
}

static bool select_motor(MotorProfile profile)
{
    if (profile == MotorParams_GetActiveProfile()) return true;
    stop_run();
    if (!JerryFOC_selectMotor(profile)) {
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
    int_bindings[INT_SPEED_LIMIT].maximum = (int32_t)JerryFOC_getMotorParams()->max_speed_mech;
    s_position_speed = (int32_t)JerryFOC_getMotorParams()->pos_limit;
    AppStorage_Get()->speed_x10 = AppStorage_Get()->voltage_x100 = AppStorage_Get()->position_x100 = 0;
    AppStorage_Get()->flags = settings_flags();
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
    /* 各模式保留自己的草稿，切换后必须再次 OK 才能启动。 */
    KK_UI_Invalidate();
}

/* ---------- 公共接口 ---------- */
const KK_UI_App *ui_app_get(void)
{
    return &s_app;
}

void ui_app_reset(uint32_t now_ms)
{
    clear_run_stop();
    AppStorage_Load();
    sync_from_storage();

    JerryFOC_useCurrentLoop(s_current_loop ? 1U : 0U);

    s_run_mode = JerryFOC_getMode();
    s_run_speed_x10 = s_speed_x10;
    s_run_voltage_x100 = s_voltage_x100;
    s_run_position_x100 = s_position_x100;

    s_run_active = false;
    s_input_repeats = s_display_errors = 0U;
    s_run_current_x100 = s_confirmed_current_x100 = 0;
    s_run_position_origin = JerryFOC_getAngle();
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
    bool save_ok;
    if (AppStorage_PollSaveResult(&save_ok))
        s_pending_toast = save_ok ? "Saved" : "Save failed";

    uint8_t healthy = MT6701_IsHealthy() ? 1U : 0U;
    if (s_last_healthy != 0xFFU && s_last_healthy != 0U && healthy == 0U) {
        AppStorage_Log(APP_LOG_ENCODER_ERROR, MT6701_GetErrorCounter());
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
        const char *reason = NULL;
        if (!JerryFOC_isDriveReady()) reason = drive_status_text();
        else if (!MT6701_IsHealthy()) reason = "ENCODER LOST";
        else if (!AngleSrc_Encoder_IsAligned()) reason = "NOT ALIGNED";
        else if (g_openloop_debug) reason = "OPEN LOOP";
        else if (JerryFOC_getMode() != s_run_mode) reason = "MODE CHANGED";
        else if (!MotorSampling_IsOutputRequested()) reason = "OUTPUT OFF";
        MT6701Diagnostics enc;
        if (reason) MT6701_GetDiagnostics(&enc);
        __set_PRIMASK(mask);
        if (reason) {
            snprintf(s_txt_run_stop, sizeof(s_txt_run_stop), "%s", reason);
            snprintf(s_txt_stop_age, sizeof(s_txt_stop_age), "%u", enc.age_cycles);
            snprintf(s_txt_stop_errors, sizeof(s_txt_stop_errors), "%lu", (unsigned long)enc.errors);
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
        if (request_settings_save()) s_settings_dirty = false;
    }
}

void ui_app_process_events(void)
{
    KK_UI_EventId evt;

    while (KK_UI_PollEvent(&evt)) {
        switch (evt) {
        case EVT_MOTOR_2804_CONFIRMED:
        case EVT_MOTOR_4015_CONFIRMED:
            (void)select_motor(evt == EVT_MOTOR_4015_CONFIRMED ? MOTOR_PROFILE_4015 : MOTOR_PROFILE_2804);
            break;
        case EVT_CURRENT_LOOP_CHANGED:
            stop_run();
            s_run_current_x100 = s_confirmed_current_x100 = 0;
            AppStorage_Get()->flags = settings_flags();
            mark_settings_dirty(HAL_GetTick());
            break;

        case EVT_POSITION_SPEED_CHANGED:
        case EVT_LED_CHANGED:
            AppStorage_Get()->flags = settings_flags();
            AppStorage_Get()->led_brightness = (uint32_t)s_led_brightness;
            mark_settings_dirty(HAL_GetTick());
            break;

        case EVT_SPEED_LIMIT_CHANGED: {
            int32_t limit = s_speed_limit * 10;
            s_speed_x10 = clamp_i32(s_speed_x10, -limit, limit);
            s_run_speed_x10 = clamp_i32(s_run_speed_x10, -limit, limit);
            AppStorage_Get()->flags = settings_flags();
            AppStorage_Get()->speed_x10 = s_speed_x10;
            mark_settings_dirty(HAL_GetTick());
            KK_UI_Invalidate();
            break;
        }

        case EVT_RUN_STEPS_CHANGED:
            for (unsigned i = 0U; i < 3U; ++i) s_run_steps[i] = s_step_drafts[i];
            AppStorage_Get()->speed_step_x10 = s_run_steps[JERRYFOC_MODE_VELOCITY];
            AppStorage_Get()->torque_step_x100 = s_run_steps[JERRYFOC_MODE_TORQUE];
            AppStorage_Get()->position_step_x100 = s_run_steps[JERRYFOC_MODE_POSITION];
            mark_settings_dirty(HAL_GetTick());
            break;

        case EVT_RESET_CONFIRMED:
            if (!select_motor(MOTOR_PROFILE_2804)) break;
            stop_run();
            clear_run_stop();
            AppStorage_ResetDefaults();
            sync_from_storage();
            s_run_speed_x10 = s_speed_x10;
            s_run_voltage_x100 = s_voltage_x100;
            s_run_position_x100 = s_position_x100;
            s_run_current_x100 = s_confirmed_current_x100 = 0;
            s_position_speed = 30;
            mark_settings_dirty(HAL_GetTick());
            s_pending_toast = "Defaults restored";
            KK_UI_Invalidate();
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
    s_run_mode = JerryFOC_getMode();
    stop_run();
    s_run_speed_x10 = s_speed_x10;
    s_run_voltage_x100 = s_voltage_x100;
    s_run_position_x100 = s_position_x100;
    s_run_current_x100 = s_confirmed_current_x100 = 0;
    s_run_position_origin = JerryFOC_getAngle();
    s_last_live_at = HAL_GetTick();
}

void KK_UI_CustomOnLeave(KK_UI_PageId page)
{
    if (page == UI_PAGE_RUN) stop_run();
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
        if (JerryFOC_hasHSOFault()) (void)JerryFOC_useHSO(0U);
        stop_run(); /* 在退出动画开始前停止，不等待 OnLeave。 */
        (void)KK_UI_CustomRequestClose();
        return;
    }
    if (event.action == KK_UI_INPUT_LEFT || event.action == KK_UI_INPUT_RIGHT) {
        if (event.source != KK_UI_INPUT_REPEAT)
            run_mode_step(event.action == KK_UI_INPUT_LEFT ? -1 : 1);
        return;
    }
    if (event.action == KK_UI_INPUT_OK) {
        bool changed = run_target_changed();
        if (JerryFOC_hasHSOFault()) {
            (void)JerryFOC_useHSO(0U);
            stop_run();
            (void)KK_UI_ShowToast("HSO reset", 900U);
        } else if (s_run_active && !changed) {
            stop_run();
        } else if (!JerryFOC_isDriveReady()) {
            (void)KK_UI_ShowToast(MotorSampling_GetStatusText(), 900U);
        } else if (!MT6701_IsHealthy() || !AngleSrc_Encoder_IsAligned()) {
            (void)KK_UI_ShowToast("Encoder not ready", 900U);
        } else if (!s_run_active && !MotorSampling_IsOutputRequested() &&
                   fabsf(JerryFOC_getVelocity()) > 1.0f) {
            (void)KK_UI_ShowToast("Wait for stop", 900U);
        } else {
            /* 运行中确认新位置时保留本次原点，停止后启动才重新捕获。 */
            if (!s_run_active) s_run_position_origin = JerryFOC_getAngle();
            s_run_active = true;
            clear_run_stop();
            apply_run_target();
            *run_target(false) = *run_target(true);
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
                    to_x100(JerryFOC_getMotorParams()->vel_limit);
    if (s_run_mode == JERRYFOC_MODE_TORQUE && s_current_loop)
        limit = to_x100(JerryFOC_getMotorParams()->rated_current);
    int32_t *draft = run_target(true);
    *draft = clamp_i32(*draft + step, -limit, limit);
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
    const int16_t W = (int16_t)OLED_GetWidth();
    const int16_t H = (int16_t)OLED_GetHeight();
    char buf[40];
    char v[16];
    OLED_SetClipWindow(clip_x, 0, clip_width, (uint16_t)H);
    OLED_SetDrawMode(OLED_DRAW_SET);
    if (page == UI_PAGE_RUN_STEPS) {
        OLED_SetFont(ui_font_title);
        OLED_DrawUTF8((int16_t)(4 + x_offset), 1, "Run Steps");
        const char *save = "OK:Save";
        OLED_DrawUTF8((int16_t)(W - 4 - OLED_GetUTF8Width(save) + x_offset), 1, save);
        OLED_DrawHLine(x_offset, 14, (uint16_t)W);
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
    OLED_SetFont(ui_font_title);
    snprintf(buf, sizeof(buf), "< %s >", mode_name(s_run_mode));
    OLED_DrawUTF8((int16_t)(4 + x_offset), 1, buf);
    bool changed = run_target_changed();
    const char *state = changed ? (s_run_active ? "RUN*" : "EDIT") :
                                (s_run_active ? "RUN" : "STOP");
    OLED_DrawUTF8((int16_t)(W - 4 - OLED_GetUTF8Width(state) + x_offset), 1, state);
    OLED_DrawHLine(x_offset, 14, (uint16_t)W);

    int32_t target = *run_target(true);
    const char *unit = s_run_mode == JERRYFOC_MODE_VELOCITY ? "rad/s" :
                       s_run_mode == JERRYFOC_MODE_POSITION ? "rad" :
                       s_current_loop ? "A" : "V";
    fmt_fixed(v, sizeof(v), target, s_run_mode == JERRYFOC_MODE_VELOCITY ? 10 : 100);
    OLED_SetFont(ui_font_big);
    int16_t value_width = (int16_t)OLED_GetUTF8Width(v);
    OLED_SetFont(ui_font_body);
    int16_t unit_width = (int16_t)OLED_GetUTF8Width(unit);
    int16_t value_x = (int16_t)((W - value_width - unit_width - 4) / 2 + x_offset);
    OLED_DrawUTF8((int16_t)(value_x + value_width + 4), 24, unit);
    /* 大字库只包含数字，单位必须使用正文 ASCII 字库。 */
    OLED_SetFont(ui_font_big);
    OLED_DrawUTF8(value_x, 19, v);
    OLED_SetFont(ui_font_body);
    if (s_run_mode == JERRYFOC_MODE_POSITION) {
        fmt_fixed(v, sizeof(v), to_x100(JerryFOC_getAngle() - s_run_position_origin), 100);
        snprintf(buf, sizeof(buf), "Pos %s  Max %ld", v, (long)s_position_speed);
    } else {
        fmt_fixed(v, sizeof(v), to_x10(JerryFOC_getVelocity()), 10);
        if (s_run_mode == JERRYFOC_MODE_VELOCITY)
            snprintf(buf, sizeof(buf), "Act %s  Max %ld", v, (long)s_speed_limit);
        else
            snprintf(buf, sizeof(buf), "Act %s rad/s", v);
    }
    OLED_DrawUTF8((int16_t)(4 + x_offset), 38, buf);
    if (!JerryFOC_isDriveReady()) {
        snprintf(buf, sizeof(buf), "%s", drive_status_text());
    } else if (!s_run_active && strcmp(s_txt_run_stop, "--") != 0) {
        snprintf(buf, sizeof(buf), "%s", s_txt_run_stop);
    } else if (!MT6701_IsHealthy() || !AngleSrc_Encoder_IsAligned()) {
        snprintf(buf, sizeof(buf), "%s", MT6701_IsHealthy() ? "NOT ALIGNED" : "ENCODER ERROR");
    } else {
        fmt_fixed(v, sizeof(v), to_x100(JerryFOC_getIq()), 100);
        snprintf(buf, sizeof(buf), "Iq %s A", v);
    }
    OLED_DrawUTF8((int16_t)(4 + x_offset), 51, buf);
    if ((s_run_active || strcmp(s_txt_run_stop, "--") == 0) &&
        JerryFOC_isDriveReady() && MT6701_IsHealthy() && AngleSrc_Encoder_IsAligned()) {
        fmt_fixed(v, sizeof(v), s_run_steps[s_run_mode], s_run_mode == JERRYFOC_MODE_VELOCITY ? 10 : 100);
        snprintf(buf, sizeof(buf), "Step %s", v);
        OLED_DrawUTF8((int16_t)(W - 4 - OLED_GetUTF8Width(buf) + x_offset), 51, buf);
    }
    OLED_DrawHLine(x_offset, 65, (uint16_t)W);
    OLED_DrawUTF8((int16_t)(4 + x_offset), 67,
                 s_run_active && !changed ? "OK:Cancel  K2:Back" : "OK:Confirm K2:Back");
    OLED_ResetClipWindow();
}
