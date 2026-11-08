#define main legacy_foc_main
#include "test_foc.c"
#undef main
extern unsigned test_drive_reads;

int main(void)
{
    unsigned cases=0;
    tim1.ARR=tim8.ARR=8499;
    for (unsigned profile=0;profile<2;++profile) {
        healthy=aligned=ready=calibrated=1;output_requested=false;
        assert(JerryFOC_selectMotor((MotorProfile)profile));
        JerryFOC_setAngleSource(&source);
        for(unsigned loop=0;loop<2;++loop)
        for(unsigned flags=0;flags<16;++flags)
        for(unsigned voltage=0;voltage<3;++voltage) {
            healthy=(flags&1U)!=0;aligned=(flags&2U)!=0;
            ready=(flags&4U)!=0;calibrated=true;output_requested=(flags&8U)!=0;
            bus=(float[]){0,12,20}[voltage];
            frame.m1_current[0]=.2f;frame.m1_current[1]=-.05f;
            JerryFOC_useCurrentLoop(loop);
            JerryFOC_Target_Iq=.3f;test_drive_reads=0;
            JerryFOC_run();
            unsigned reads=test_drive_reads;
            printf("VALUE %u %u %u %u %u %u %u %a %a %a\n",profile,loop,flags,voltage,
                tim1.CCR1,tim1.CCR2,tim1.CCR3,JerryFOC_getId(),JerryFOC_getIq(),JerryFOC_Target_Iq);
            printf("EVAL %u %u %u %u %u\n",profile,loop,flags,voltage,reads);
            ++cases;
        }
    }
    printf("CASES %u\n",cases);
    return 0;
}
