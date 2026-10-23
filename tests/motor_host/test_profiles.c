#include <assert.h>
#include <stddef.h>
#include <stdio.h>
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
    puts("Independent motor profile tests passed");
    return 0;
}
