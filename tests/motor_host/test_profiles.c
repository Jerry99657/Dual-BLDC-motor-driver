#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include "motor_params.h"

int main(void)
{
    assert(MotorParams_GetAxis(MOTOR_AXIS_1) == &MOTOR_2804);
    assert(MotorParams_GetAxis(MOTOR_AXIS_2) == &MOTOR_2804);
    assert(MotorParams_SelectAxis(MOTOR_AXIS_2, MOTOR_PROFILE_4015));
    /* Configuring M2 must not replace the M1 FOC/protection preset. */
    assert(MotorParams_GetActive() == &MOTOR_2804);
    assert(MotorParams_GetActiveProfile() == MOTOR_PROFILE_2804);
    assert(MotorParams_GetAxis(MOTOR_AXIS_2)->pole_pairs == 11);
    assert(MotorParams_GetActive()->max_current == 2.0f);
    assert(MotorParams_GetAxis(MOTOR_AXIS_2)->max_current == 2.5f);
    assert(MOTOR_2804.max_speed_mech==600 && MOTOR_4015.max_speed_mech==600);
    assert(MOTOR_4015.rated_speed_mech==104.72f); /* UI ceiling is not a motor rating. */

    MotorParams_Select(MOTOR_PROFILE_4015);
    assert(MotorParams_SelectAxis(MOTOR_AXIS_2, MOTOR_PROFILE_2804));
    assert(MotorParams_GetActive() == &MOTOR_4015);
    assert(MotorParams_GetAxis(MOTOR_AXIS_2) == &MOTOR_2804);
    assert(MotorParams_SelectAxis(MOTOR_AXIS_2, MOTOR_PROFILE_4015));
    assert(MotorParams_GetAxis(MOTOR_AXIS_1) == MotorParams_GetAxis(MOTOR_AXIS_2));
    MotorParams_Select(MOTOR_PROFILE_2804);
    assert(MotorParams_GetAxisProfile(MOTOR_AXIS_2) == MOTOR_PROFILE_4015);

    assert(!MotorParams_SelectAxis(MOTOR_AXIS_COUNT, MOTOR_PROFILE_2804));
    assert(!MotorParams_SelectAxis((MotorAxis)-1, MOTOR_PROFILE_2804));
    assert(!MotorParams_SelectAxis(MOTOR_AXIS_1, MOTOR_PROFILE_COUNT));
    assert(!MotorParams_SelectAxis(MOTOR_AXIS_2, (MotorProfile)-1));
    MotorParams_Select(MOTOR_PROFILE_COUNT);
    assert(MotorParams_GetActiveProfile() == MOTOR_PROFILE_2804);
    assert(MotorParams_GetAxisProfile(MOTOR_AXIS_2) == MOTOR_PROFILE_4015);
    assert(MotorParams_GetAxis(MOTOR_AXIS_COUNT) == NULL);
    assert(MotorParams_GetAxis((MotorAxis)-1) == NULL);
    assert(MotorParams_GetAxisProfile((MotorAxis)-1) == MOTOR_PROFILE_COUNT);
    MotorCalibration measured={(2U<<24)|(1920U<<8)|(uint8_t)-11,440,9290},empty={0};
    assert(MotorParams_SetCalibration(MOTOR_AXIS_1,&measured));
    assert(!MotorParams_IsCalibrated(MOTOR_AXIS_1)); /* Current model is 2804. */
    assert(MotorParams_GetActive()==&MOTOR_2804);
    MotorParams_Select(MOTOR_PROFILE_4015);
    const MotorParams *m=MotorParams_GetActive();
    assert(MotorParams_IsCalibrated(MOTOR_AXIS_1) && m->pole_pairs==11 && m->encoder_direction==1);
    assert(fabsf(m->Rs-1.92f)<1e-6f && fabsf(m->Ls-.00044f)<1e-9f && fabsf(m->psi_f-.00929f)<1e-8f);
    assert(m->max_current==MOTOR_4015.max_current && m->rated_current==MOTOR_4015.rated_current && m->overload_budget_s==MOTOR_4015.overload_budget_s);
    assert(MotorParams_GetAxis(MOTOR_AXIS_2)==&MOTOR_4015);
    assert(MotorParams_SetCalibration(MOTOR_AXIS_2,&measured));
    assert(MotorParams_GetAxis(MOTOR_AXIS_2)->encoder_direction==1); /* M1 sensor polarity is not copied to M2. */
    for(unsigned k=0;k<8;++k) {
        MotorCalibration bad=measured;
        if(k==0) bad.poles_rs&=0xFFFFFFU; /* Unbound v5 result. */
        if(k==1) bad.poles_rs=(3U<<24)|(1920U<<8)|11;
        if(k==2) bad.poles_rs=(2U<<24)|(99U<<8)|11;
        if(k==3) bad.poles_rs=(2U<<24)|(1920U<<8)|33;
        if(k==4) bad.ls_uh=49;
        if(k==5) bad.ls_uh=50001;
        if(k==6) bad.flux_uwb=99;
        if(k==7) bad.flux_uwb=200001;
        assert(!MotorParams_SetCalibration(MOTOR_AXIS_1,&bad));
        assert(MotorParams_GetActive()==m && fabsf(m->Rs-1.92f)<1e-6f);
    }
    MotorParams_Select(MOTOR_PROFILE_2804);assert(!MotorParams_IsCalibrated(MOTOR_AXIS_1));
    MotorParams_Select(MOTOR_PROFILE_4015);assert(MotorParams_IsCalibrated(MOTOR_AXIS_1));
    assert(MotorParams_SetCalibration(MOTOR_AXIS_1,&empty) && MotorParams_GetActive()==&MOTOR_4015);
    assert(MotorParams_IsCalibrated(MOTOR_AXIS_2));
    assert(MotorParams_SetCalibration(MOTOR_AXIS_2,&empty));
    assert(!MotorParams_CustomMask() && !MotorParams_SelectAxis(MOTOR_AXIS_1,MOTOR_PROFILE_CUSTOM1));
    for(unsigned i=0;i<3;++i) {
        MotorCalibration r=measured;r.ls_uh+=i*100;
        assert(MotorParams_SetCustom(i,&r) && MotorParams_CustomMask()==((1U<<(i+1))-1U));
        assert(MotorParams_GetActive()==&MOTOR_4015); /* Saving does not select. */
    }
    for(unsigned a=0;a<2;++a) for(unsigned i=0;i<3;++i) {
        assert(MotorParams_SelectAxis((MotorAxis)a,(MotorProfile)(MOTOR_PROFILE_CUSTOM1+i)));
        const MotorParams *c=MotorParams_GetAxis((MotorAxis)a);
        assert(c->name[7]==(char)('1'+i) && c->pole_pairs==11 && c->encoder_direction==1);
        assert(fabsf(c->Ls-(440+i*100)*1e-6f)<1e-9f && c->max_current==MOTOR_4015.max_current);
        assert(MotorParams_BaseProfile((MotorProfile)(MOTOR_PROFILE_CUSTOM1+i))==MOTOR_PROFILE_4015);
    }
    assert(!MotorParams_SetCustom(3,&measured) && !MotorParams_SetCustom((unsigned)-1,&measured));
    MotorCalibration bad=measured;bad.flux_uwb=0;
    assert(!MotorParams_SetCustom(0,&bad) && MotorParams_CustomMask()==7);
    assert(!MotorParams_SelectAxis(MOTOR_AXIS_1,MOTOR_PROFILE_ESC));
    /* Raw-positive installation needs the opposite direction after Angle=2pi-raw;
     * the record never changes M2's independent installation direction. */
    MotorCalibration positive=measured;
    positive.poles_rs=(positive.poles_rs&~255U)|11U;
    assert(MotorParams_SetCustom(0,&positive));
    assert(MotorParams_SelectAxis(MOTOR_AXIS_1,MOTOR_PROFILE_CUSTOM1));
    assert(MotorParams_GetActive()->encoder_direction==-1);
    assert(MotorParams_SelectAxis(MOTOR_AXIS_2,MOTOR_PROFILE_CUSTOM1));
    assert(MotorParams_GetAxis(MOTOR_AXIS_2)->encoder_direction==1);
    puts("PASS: three shared Custom slots, empty exclusion, axis isolation/polarity, inherited protection, strict records and no implicit selection");
    puts("PASS: calibrated equivalent-star parameters, all bounds/legacy binding, per-axis/profile isolation, direction, preset restoration and unchanged protection caps");
    puts("Independent motor profile tests passed");
    return 0;
}
