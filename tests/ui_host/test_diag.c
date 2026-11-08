/* Exercise the real renderer/query adapter with coherent fixture snapshots. */
#define main ui_fixture_main
#include "test_ui.c"
#undef main

unsigned test_format_calls,test_fixed_calls;
static unsigned row_named(unsigned group,const char *name) {
    const KK_UI_InfoPage *page=&ui_app_get()->info_pages[group];
    for(unsigned row=0;row<page->row_count;++row) if(!strcmp(page->rows[row].name,name)) return row;
    assert(!"Missing diagnostic row");return 0;
}
static void expect_row(unsigned axis,unsigned group,unsigned row,const char *expected) {
    char label[24],value[32];uint32_t before=query_model_hash();
    assert(App_ReadRow(axis,group,row,label,value)>0);
    if(strcmp(value,expected)) fprintf(stderr,"M%u %s: %s != %s\n",axis,label,value,expected);
    assert(!strcmp(value,expected) && query_model_hash()==before);
    assert(!strcmp(ui_app_get()->info_pages[group].rows[row].value,expected));
    printf("VALUE M%u G%u R%u %s=%s\n",axis,group,row,label,value);
}
static void zero_cases(void) {
    const struct {
        uint16_t before,after,count,age;
        uint32_t start,end,now;
        const char *values[4];
    } cases[]={
        {1,2,0,0,100,120,130,{"--","--","--","--"}},
        {1,16383,5,0,100,120,130,{"NEAR 5","1","16383","20"}},
        {8192,8200,1,3,100,125,130,{"FAR 1","8192","8200","25"}},
        {1,155,2,4,100,120,130,{"GAP 2","1","155","20"}},
        {65535,65535,65535,65535,0xfffffff0U,0,16,{"GAP 65535","--","--","32"}},
        {65535,0,1,0,0,4,130,{"NEAR 1","65535","0","4"}},
        {128,0,1,3,0,0,0,{"NEAR 1","128","0","0"}},
        {129,0,1,3,0,0,0,{"FAR 1","129","0","0"}},
        {16256,0,1,0,0xfffffff0U,5,16,{"NEAR 1","16256","0","21"}},
        {16255,0,1,0,0,0,0,{"FAR 1","16255","0","0"}},
    };
    for(unsigned axis=1;axis<=2;++axis) for(unsigned c=0;c<sizeof(cases)/sizeof(cases[0]);++c) {
        MotorControl_SelectWork(MOTOR_WORK_INDEPENDENT);MotorControl_SetFocus((MotorAxis)(axis-1));reset();
        MT6701Diagnostics *d=axis==1 ? &enc_diag : &enc2_diag;
        *d=(MT6701Diagnostics){.zero_before=cases[c].before,.zero_after=cases[c].after,
            .zero_count=cases[c].count,.zero_age=cases[c].age,
            .zero_started_at=cases[c].start,.zero_ended_at=cases[c].end};
        tick=cases[c].now;ui_app_reset(tick);
        for(unsigned n=0;n<4;++n) expect_row(axis,3,10+n,cases[c].values[n]);
    }
}
static void observer_cases(void) {
    for(unsigned axis=1;axis<=2;++axis) for(unsigned mode_index=0;mode_index<3;++mode_index) {
        MotorControl_SelectWork(MOTOR_WORK_INDEPENDENT);MotorControl_SetFocus((MotorAxis)(axis-1));reset();
        hso_snapshot=(StHsoSnapshot){.state=mode_index==1 ? ST_HSO_LEARNING : ST_HSO_TRACKING,
            .reason=mode_index==2 ? ST_HSO_REASON_LEARN_TIMEOUT : ST_HSO_REASON_NONE,
            .qualified_samples=1234,.bad_samples=7,.reference_flux_wb=.0075f,
            .max_control_cycles=1701,.overruns=UINT32_MAX};
        six_ui=(SixStepSnapshot){.quality=12,.gate=mode_index ? 'T' : 0,
            .phase_pct=65,.period_pct=146,.period_fraction=11,.startup_ms=65535};
        ui_app_reset(tick);
        expect_row(axis,5,row_named(5,"H Qual"),mode_index ? "123/1000 ms 7/20" : "123/200 ms 7/20");
        expect_row(axis,5,row_named(5,"H Flux"),"7500 uWb");
        expect_row(axis,5,row_named(5,"H Time"),"10 us / 4294967295");
        expect_row(axis,7,row_named(7,"6S ZC"),mode_index ? "12/12 T 65/146.1" : "12/12 - 65/146.1");
        expect_row(axis,7,row_named(7,"6S Hold"),"65535/500 ms");
    }
}
static void work_metrics(void) {
    MotorControl_SelectWork(MOTOR_WORK_INDEPENDENT);MotorControl_SetFocus(MOTOR_AXIS_1);reset();
    enc_diag=enc2_diag=(MT6701Diagnostics){.zero_count=5,.zero_before=1,.zero_after=16383,
        .zero_started_at=10,.zero_ended_at=15};
    hso_snapshot=(StHsoSnapshot){.qualified_samples=1234,.bad_samples=7};
    test_format_calls=test_fixed_calls=0;ui_app_reset(tick);
    printf("LCD %u %u\n",test_format_calls,test_fixed_calls);
    const unsigned groups[]={3,3,3,3,3,5,5,5,5,7,7,7};
    const char *names[]={"E1 Zero","E1 ZPre","E1 ZPost","E1 ZTime","E1 Raw",
        "H Qual","H Flux","H Time","H Speed","6S ZC","6S Hold","6Step Duty"};
    for(unsigned axis=1;axis<=2;++axis) for(unsigned n=0;n<12;++n) {
        unsigned row=row_named(groups[n],names[n]);char label[24],value[32];
        reset_query_reads();test_format_calls=test_fixed_calls=0;
        assert(App_ReadRow(axis,groups[n],row,label,value)>0);
        printf("WORK %u %u %u %u %u %u\n",axis,groups[n],row,test_format_calls,test_fixed_calls,query_reads_total()+control_stub_axis_reads);
    }
}
int main(int argc,char **argv) {
    (void)argv;
    if(argc>1) work_metrics();
    else { zero_cases();observer_cases();puts("PASS: compound LCD/GetRow parity, zero sentinels/boundaries/wrap, HSO windows and cycle truncation, six-step fallback/fraction, frozen UI/settings"); }
    return 0;
}
