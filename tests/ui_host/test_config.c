/* Reuse the real UI/controller fixture; keep new bridge scenarios separate so
 * the unchanged page regressions can also compile the R130 renderer. */
#define main ui_fixture_main
#include "test_ui.c"
#undef main

static void check_config_bridge(void) {
    MotorControl_SelectWork(MOTOR_WORK_SINGLE_M1);MotorControl_SetFocus(MOTOR_AXIS_1);reset();MotorControl_StopAll();
    assert(App_Configure(APP_CFG_WORK,MOTOR_AXIS_1,MOTOR_WORK_INDEPENDENT,0));
    assert(App_Configure(APP_CFG_MOTOR,MOTOR_AXIS_2,MOTOR_PROFILE_2804,0));
    press(KK_UI_KEY_OK);settle();press(KK_UI_KEY_RIGHT);press(KK_UI_KEY_UP);
    probe_frame();assert(drew("0.5") && settings.speed_x10==0);
    assert(App_Configure(APP_CFG_MOTOR,MOTOR_AXIS_2,MOTOR_PROFILE_4015,0));
    probe_frame();assert(drew("M1 Speed ENC") && drew("0.5") && settings.speed_x10==0);
    assert(App_Configure(APP_CFG_FOCUS,MOTOR_AXIS_2,0,0));
    assert(MotorControl_GetFocus()==MOTOR_AXIS_2);
    probe_frame();assert(drew("M2 Uq(V) ENC"));
    assert(App_Configure(APP_CFG_FOCUS,MOTOR_AXIS_1,0,0));
    probe_frame();assert(drew("M1 Speed ENC") && drew("0.5"));
    assert(App_Control(0,APP_ACTION_SAVE,0)==1);App_SaveService(tick);
    assert(saved_settings.speed_x10==0 && (saved_settings.axis2_flags&APP_FLAG_MOTOR_4015));
    press(KK_UI_KEY_OK);assert(output_requested && JerryFOC_Target_Velocity==0.5f);
    AppSettings_ResetSave(tick);
    assert(App_Configure(APP_CFG_MOTOR,MOTOR_AXIS_1,MOTOR_PROFILE_2804,0));
    assert(output_requested && JerryFOC_Target_Velocity==0.5f && App_SaveState()!=APP_SAVE_PENDING);
    AppSettings before=settings;
    ControlStub_SetWorkAllowed(false);
    assert(!App_Configure(APP_CFG_WORK,MOTOR_AXIS_1,MOTOR_WORK_SINGLE_M2,0));
    ControlStub_SetWorkAllowed(true);
    assert(!App_Configure(APP_CFG_GEAR,MOTOR_AXIS_1,0,12));
    assert(!App_Configure(APP_CFG_FEEDBACK,MOTOR_AXIS_2,1,0));
    assert(!App_Configure(APP_CFG_SIXSTEP,MOTOR_AXIS_2,1,0));
    assert(!App_Configure(APP_CFG_MOTOR,(MotorAxis)2,MOTOR_PROFILE_4015,0));
    assert(!memcmp(&before,&settings,sizeof(settings)) && output_requested && JerryFOC_Target_Velocity==0.5f);
    assert(MotorControl_GetFocus()==MOTOR_AXIS_1 && MotorControl_GetWork()==MOTOR_WORK_INDEPENDENT);
    MotorControl_StopAll();back();
    assert(App_Configure(APP_CFG_WORK,MOTOR_AXIS_1,MOTOR_WORK_SINGLE_M1,0));
    assert(App_Configure(APP_CFG_GEAR,MOTOR_AXIS_1,-150,23));
    assert(settings.gear_ratio_x100==-150 && settings.gear_offset_x100==23);
    align_request_allowed=false;unsigned requests=align_requests;
    assert(!App_Configure(APP_CFG_ALIGN,MOTOR_AXIS_1,0,0) && align_requests==requests+1);
    align_request_allowed=true;
    puts("PASS: shared configuration bridge preserves foreign-axis drafts, focus/profile metadata and confirmed-only save; idempotent selection, failed work/gear/axis/align do not bypass controller gates");
}
static void check_foreign_throttle_draft(void) {
    MotorControl_SelectWork(MOTOR_WORK_SINGLE_M1);MotorControl_SetFocus(MOTOR_AXIS_1);reset();MotorControl_StopAll();
    assert(App_Configure(APP_CFG_SIXSTEP,MOTOR_AXIS_1,1,0));
    assert(KK_UI_GoTo(UI_PAGE_RUN,tick)==KK_UI_OK);settle();press(KK_UI_KEY_UP);
    probe_frame();assert(drew("M1 Throttle 6STEP") && drew("1"));
    assert(App_Control(2,APP_SET_SPEED_LIMIT,180)==1);
    assert(App_Configure(APP_CFG_GEAR,MOTOR_AXIS_1,-100,10));
    probe_frame();assert(drew("1") && !(settings.control_config&APP_CONFIG_THROTTLE_MASK));
    assert(App_Control(0,APP_ACTION_SAVE,0)==1);App_SaveService(tick);
    assert(!(saved_settings.control_config&APP_CONFIG_THROTTLE_MASK));
    /* Resetting M1 actually resets its throttle draft, even if its accepted
     * throttle was already zero. A foreign-axis refresh must not do that. */
    assert(App_SelectMotor(MOTOR_AXIS_1,MOTOR_PROFILE_4015));
    probe_frame();assert(drew("0"));
    MotorControl_StopAll();assert(App_Configure(APP_CFG_SIXSTEP,MOTOR_AXIS_1,0,0));
    puts("PASS: foreign/global settings preserve an unconfirmed six-step throttle without saving it; an actual M1 model reset clears it");
}
int main(int argc,char **argv) {
    (void)argv;
    if(argc>1) {
        MotorControl_StopAll();AppStorage_ResetDefaults();
        assert(KK_UI_GetCurrentPage()==0);
        assert(App_Configure(APP_CFG_WORK,MOTOR_AXIS_1,MOTOR_WORK_INDEPENDENT,0));
        assert(App_Configure(APP_CFG_GEAR,MOTOR_AXIS_1,-125,12));
        assert(App_Configure(APP_CFG_MOTOR,MOTOR_AXIS_2,MOTOR_PROFILE_4015,0));
        assert(settings.gear_ratio_x100==-125 && settings.gear_offset_x100==12);
        assert(MotorParams_GetAxisProfile(MOTOR_AXIS_2)==MOTOR_PROFILE_4015);
        assert(KK_UI_GetCurrentPage()==0 && !output_requested && save_requests==0);
        puts("PASS: shared configuration business works before any UI/model initialization, with no motion or immediate Flash transaction");
    } else { check_config_bridge();check_foreign_throttle_draft(); }
    return 0;
}
