/* Real coordinator, M1/M2 outer loops and FOC; reuse peripheral models only.
 * Current plants below are illustrative, not a hardware bandwidth claim. */
#define MOTOR_HOST_DETECT_TEST
#include "test_control.c"
#undef MOTOR_HOST_DETECT_TEST

static void saturation_release(void)
{
    const float limits[]={.5f,1.2f,6.93f,13.86f};
    for(unsigned axis=0;axis<2;++axis)for(unsigned k=0;k<4;++k)for(int sign=-1;sign<=1;sign+=2) {
        boot(MOTOR_WORK_INDEPENDENT);
        float limit=limits[k];
        PIDController pi={.P=.05f,.I=.5f,.limit=limit};
        for(unsigned n=0;n<500;++n)
            assert(MotorControl_VelocityControl((MotorAxis)axis,&pi,sign*100*limit,0)==sign*limit);
        float release=MotorControl_VelocityControl((MotorAxis)axis,&pi,0,0);
        /* At zero error this must not keep a saturated current request.
         * A one-frame trapezoidal-integration remainder is permitted. */
        assert(fabsf(release)<=.02501f*limit);
        float reverse=MotorControl_VelocityControl((MotorAxis)axis,&pi,-sign*100*limit,0);
        assert(reverse==-sign*limit);
    }
    puts("PASS: both encoder speed PIs: blocked reference saturation, neutral release and immediate reverse without a retained limit-level request");
}
static void synchronous_reference(void)
{
    for(int gear=-200;gear<=200;gear+=400)for(unsigned mode=0;mode<2;++mode) {
        boot(MOTOR_WORK_SYNC_POSITION);assert(MotorControl_SetGear(gear,30));
        assert(MotorControl_Command(MOTOR_AXIS_1,JERRYFOC_MODE_POSITION,10,3,false));
        float previous=state(0).target_position;
        for(unsigned n=0;n<300;++n) {
            if(n==120) {
                if(mode) {
                    settings.axis2_flags=(100UL<<APP_SPEED_LIMIT_SHIFT)|AppSettings_EncodePositionSpeed(1);
                    assert(MotorControl_SetOptions(1,settings.axis2_flags));
                } else assert(MotorControl_Command(0,JERRYFOC_MODE_POSITION,-10,3,false));
            }
            ms(1,0);
            float position=state(0).target_position;
            float speed=(position-previous)/.001f;
            for(unsigned a=0;a<2;++a) {
                PIDController probe={.limit=30};
                float v=MotorControl_PositionVelocity((MotorAxis)a,&probe,0);
                float expected=speed*(a ? (float)gear/100 : 1);
                assert(fabsf(v-expected)<.004f);
                if(mode && n>=180)assert(fabsf(v)<(a ? 1.001f : .501f));
            }
            previous=position;
        }
        MotorControl_StopAll();
        PIDController probe={.limit=30};
        assert(MotorControl_PositionVelocity(0,&probe,0)==0);
    }
    puts("PASS: synchronous position uses its planned velocity on both axes, signed 2:1 gearing/offset/reversal, slave Pos Speed limits the shared trajectory, Stop removes feedforward");
}
static void gain_restart(void)
{
    for(unsigned a=0;a<2;++a) {
        MotorAxis slave=(MotorAxis)a;
        boot(a ? MOTOR_WORK_MASTER_M1 : MOTOR_WORK_MASTER_M2);
        assert(MotorControl_SelectProfile(slave,MOTOR_PROFILE_4015));
        if(!a) { JerryFOC_alignStart();JerryFOC_alignFinish();aligned=1; }
        else { MotorControl_Service(now);now+=3000;MotorControl_Service(now);ms(8,0); }
        uint32_t *flags=a ? &settings.axis2_flags : &settings.flags;
        *flags=APP_FLAG_CURRENT_LOOP|(120UL<<APP_SPEED_LIMIT_SHIFT)|AppSettings_EncodePositionSpeed(120);
        assert(MotorControl_SetOptions(slave,*flags));
        float first=0;
        for(unsigned session=0;session<2;++session) {
            assert(MotorControl_StartFollow());
            PIDController pi={.P=.05f,.limit=1.2f};float previous=.05f;
            for(unsigned n=0;n<30;++n) {
                float q=MotorControl_VelocityControl(slave,&pi,1,0);
                assert(q>=previous && q-previous<.011f && q<=.10001f && pi.P==.05f);
                if(!n) { if(session)assert(q==first);else first=q; }
                previous=q;
            }
            assert(MotorControl_VelocityControl(slave,&pi,0,0)==0);
            MotorControl_StopAll();
        }
    }
    puts("PASS: Follow gain transitions bounded in both follower axes; zero-error release and Stop/new Follow resets the gain without leaked session state");
}
static void sync_tracking(float acceleration_gain,bool verify)
{
    boot(MOTOR_WORK_SYNC_POSITION);
    assert(MotorControl_SelectProfile(0,MOTOR_PROFILE_4015));
    assert(MotorControl_SelectProfile(1,MOTOR_PROFILE_4015));
    JerryFOC_alignStart();JerryFOC_alignFinish();aligned=healthy=1;
    MotorControl_Service(now);assert(state(1).aligning);
    now+=3000;MotorControl_Service(now);ms(8,0);
    settings.flags=settings.axis2_flags=APP_FLAG_CURRENT_LOOP|(120UL<<APP_SPEED_LIMIT_SHIFT)|AppSettings_EncodePositionSpeed(120);
    assert(MotorControl_SetOptions(0,settings.flags) && MotorControl_SetOptions(1,settings.axis2_flags));
    assert(MotorControl_SetGear(-100,0));
    assert(MotorControl_Command(0,JERRYFOC_MODE_POSITION,2,3,true));
    float p[2]={0},v[2]={0},iq[2]={0},history[4]={0},sum[2]={0},peak[2]={0};
    const float quantum=2*PI/16384;
    for(unsigned n=0;n<4000;++n) {
        for(unsigned a=0;a<2;++a) {
            MotorAxisSnapshot s=state((MotorAxis)a);
            assert(s.active && fabsf(s.target_iq)<=1.2001f && fabsf(s.target_velocity)<=3.001f);
            iq[a]+=.25f*(s.target_iq-iq[a]);
            v[a]+=.001f*(acceleration_gain*iq[a]-2*v[a]);p[a]+=.001f*v[a];
            peak[a]=fmaxf(peak[a],a ? -p[a] : p[a]);
        }
        hand_position=roundf(p[0]/quantum)*quantum;
        encoder_velocity=(hand_position-history[n%4])/.004f;history[n%4]=hand_position;
        enc2.angle=fmodf(.25f+roundf(p[1]/quantum)*quantum,2*PI);++now;++enc2.samples;
        for(unsigned a=0;a<2;++a) {
            float angle=state((MotorAxis)a).electrical_angle;
            float ia=-iq[a]*sinf(angle),ib=.5f*(sqrtf(3)*iq[a]*cosf(angle)-ia);
            if(a) { frame.m2_current[0]=ia;frame.m2_current[1]=ib; }
            else { frame.m1_current[0]=ia;frame.m1_current[1]=ib;frame.m1_current[2]=-ia-ib; }
        }
        JerryFOC_OuterIRQ();for(unsigned j=0;j<10;++j)JerryFOC_FastIRQ();
        for(unsigned a=0;a<2;++a)if(n>=200 && n<500) {
            float error=state((MotorAxis)a).target_position-p[a];sum[a]+=error*error;
        }
        assert(MotorControl_IsGroupActive());
    }
    printf("SYNC OUTER MODEL ONLY: gain=%.0f reference-error RMS M1/M2 %.5f/%.5f rad; peak %.4f/%.4f rad\n",
        acceleration_gain,sqrtf(sum[0]/300),sqrtf(sum[1]/300),peak[0],peak[1]);
    for(unsigned a=0;verify && a<2;++a) {
        assert(sqrtf(sum[a]/300)<.08f && peak[a]<2.16f);
        assert(fabsf(p[a]-(a ? -2 : 2))<.02f && fabsf(v[a])<.05f);
    }
    MotorControl_StopAll();
}
int main(int argc,char **argv)
{
    bool verify=argc<2 || strcmp(argv[1],"report");
    if(verify) { saturation_release();synchronous_reference();gain_restart(); }
    sync_tracking(300,verify);sync_tracking(1500,verify);sync_tracking(3000,verify);
}
