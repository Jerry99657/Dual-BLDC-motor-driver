#include "angle_src_encoder.h"
#include "motor_math.h"
#include <assert.h>
#include <stdio.h>
#include <stdbool.h>

volatile float Angle;
static uint32_t samples;
static uint8_t healthy=1;
static bool time_available;
static float timed_angle;
static uint32_t timed_cycles;
uint8_t MT6701_IsHealthy(void) { return healthy; }
uint32_t MT6701_GetSampleCounter(void) { return samples; }
bool MT6701_GetTimedAngle(unsigned axis,float *angle,uint32_t *cycles)
{ assert(axis==0);if(!time_available)return false;*angle=timed_angle;*cycles=timed_cycles;return true; }

int main(void)
{
    for(int direction=-1;direction<=1;direction+=2) {
        Angle=6.2811853f;++samples;
        AngleSource *source=AngleSrc_Encoder_Init(direction*11);
        AngleSrc_Encoder_Align(0);
        float angle=source->getElectricalAngle();
        assert(AngleSrc_Encoder_IsAligned() && fminf(angle,6.28318530718f-angle)<1e-5f);
        Angle=.001f;++samples;AngleSrc_Encoder_Update(.001f);
        assert(fabsf(source->getMechanicalAngle()-direction*.003f)<2e-6f);
        for(unsigned i=0;i<8;++i) { Angle+=.001f;++samples;AngleSrc_Encoder_Update(.001f); }
        assert(fabsf(source->getVelocity()-direction)<.001f);
        /* Use an independently captured angle, not the global's newer value.
         * Prediction uses the already signed mechanical velocity exactly once. */
        float reference=source->getElectricalAngle();
        timed_angle=Angle;timed_cycles=0xfffffff0U;time_available=true;
        float predicted=AngleSrc_Encoder_At(direction*40.0f,16984U);
        assert(fabsf(sinf(predicted-reference-direction*.044f))<2e-5f);
        assert(fabsf(sinf(AngleSrc_Encoder_At(100,170001U+timed_cycles)-reference))<2e-5f);
        time_available=false;
        assert(AngleSrc_Encoder_At(100,0)==reference);
        float position=source->getMechanicalAngle();
        for(unsigned i=0;i<10;++i) {
            (void)source->getElectricalAngle();(void)source->getVelocity();
            assert(source->getMechanicalAngle()==position);
        }
        AngleSrc_Encoder_Update(.001f);assert(source->getMechanicalAngle()==position);
        healthy=0;Angle=2;AngleSrc_Encoder_Update(.001f);
        healthy=1;++samples;AngleSrc_Encoder_Update(.001f);
        assert(source->getMechanicalAngle()==position && source->getVelocity()==0);
    }
    puts("PASS: real M1 adapter, signed encoder polarity, electrical alignment, wrap, pure getters, duplicate sample and recovery without false motion");
}
