/* Replay real command/outer-loop limits with the existing peripheral fixture.
 * Counters are injected into a TEST COPY of motor_control.c by check_limits.py. */
#define MOTOR_HOST_DETECT_TEST
#include "test_control.c"
#undef MOTOR_HOST_DETECT_TEST

extern unsigned test_speed_evals,test_position_evals;
static uint32_t bits(float value) { uint32_t out;memcpy(&out,&value,4);return out; }
static unsigned bounded(unsigned n,unsigned fallback) { return n && n<=600U ? n : fallback; }
static void trace(unsigned axis,unsigned speed,unsigned pos,unsigned input,unsigned kind)
{
    MotorAxisSnapshot s=state((MotorAxis)axis);
    printf("VALUE %u %u %u %u %u %08lX %08lX %08lX %u %u\n",axis,speed,pos,input,kind,
        (unsigned long)bits(s.target_velocity),(unsigned long)bits(s.target_position),
        (unsigned long)bits(s.target_iq),s.active,s.mode);
}
int main(void)
{
    const unsigned speed_codes[]={0,1,5,17,120,600,601,1023};
    const unsigned position_codes[]={0,1,5,30,63,64,120,600,601,1023};
    const float targets[]={-700,-600,-17,-0.0f,0.0f,17,600,700};
    const float limits[]={-1,0,.01f,1,5,30,600,700};
    unsigned cases=0;
    for(unsigned profile=0;profile<2;++profile)for(unsigned axis=0;axis<2;++axis) {
        boot(axis ? MOTOR_WORK_SINGLE_M2 : MOTOR_WORK_SINGLE_M1);
        assert(MotorControl_SelectProfile((MotorAxis)axis,(MotorProfile)profile));
        if(axis) { MotorControl_Service(now);now+=3000;MotorControl_Service(now); }
        else { JerryFOC_alignStart();JerryFOC_alignFinish();aligned=1; }
        ms(8,0);
        for(unsigned si=0;si<8;++si)for(unsigned pi=0;pi<10;++pi) {
            unsigned speed=speed_codes[si],position=position_codes[pi];
            MotorControl_StopAll();
            uint32_t flags=MotorControl_GetOptions((MotorAxis)axis)&~(APP_SPEED_LIMIT_MASK|APP_POSITION_SPEED_MASK);
            flags|=(uint32_t)speed<<APP_SPEED_LIMIT_SHIFT;
            flags|=AppSettings_EncodePositionSpeed(position);
            assert(MotorControl_SetOptions((MotorAxis)axis,flags));
            float cap=(float)bounded(speed,100),pcap=fminf((float)bounded(position,30),cap);
            for(unsigned n=0;n<8;++n) {
                MotorControl_StopAll();test_speed_evals=test_position_evals=0;
                assert(MotorControl_Command((MotorAxis)axis,JERRYFOC_MODE_VELOCITY,targets[n],0,false));
                MotorAxisSnapshot s=state((MotorAxis)axis);
                float expected=targets[n]>cap ? cap : targets[n]< -cap ? -cap : targets[n];
                assert(bits(s.target_velocity)==bits(expected));
                trace(axis,speed,position,n,0);
                printf("EVAL %u %u %u %u %u %u %u\n",profile,axis,si,pi,n,test_speed_evals,test_position_evals);
                ++cases;
                MotorControl_StopAll();test_speed_evals=test_position_evals=0;
                assert(MotorControl_Command((MotorAxis)axis,JERRYFOC_MODE_POSITION,1000,limits[n],false));
                unsigned sc=test_speed_evals,pc=test_position_evals;
                HAL_TIM_PeriodElapsedCallback(&(TIM_HandleTypeDef){.Instance=TIM2});
                expected=fminf(limits[n]>0 ? limits[n] : pcap,pcap);
                if(axis)expected=fmaxf(.1f,expected);
                s=state((MotorAxis)axis);
                assert(fabsf(s.target_velocity-expected)<.00001f);
                trace(axis,speed,position,n,1);
                printf("EVAL %u %u %u %u %u %u %u\n",profile,axis,si,pi,n+8,sc,pc);
                assert(MotorControl_GetOptions((MotorAxis)axis)==flags);
                ++cases;
            }
        }
        /* A cap read in one invocation must not cache a later live setting. */
        MotorControl_StopAll();
        assert(MotorControl_SetOptions((MotorAxis)axis,(600UL<<APP_SPEED_LIMIT_SHIFT)|AppSettings_EncodePositionSpeed(600)));
        assert(MotorControl_Command((MotorAxis)axis,JERRYFOC_MODE_VELOCITY,400,0,false));
        assert(MotorControl_SetOptions((MotorAxis)axis,(17UL<<APP_SPEED_LIMIT_SHIFT)|AppSettings_EncodePositionSpeed(600)));
        for(unsigned n=0;n<20;++n)ms(1,0);
        MotorControl_StopAll();
        assert(MotorControl_Command((MotorAxis)axis,JERRYFOC_MODE_VELOCITY,400,0,false));
        assert(state((MotorAxis)axis).target_velocity==17);
    }
    MotorControl_StopAll();printf("CASES %u\n",cases);return 0;
}
