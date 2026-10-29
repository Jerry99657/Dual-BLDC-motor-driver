#include "angle_src_encoder.h"
#include "motor_math.h"
#include <assert.h>
#include <stdio.h>

volatile float Angle;
static uint32_t samples;
static uint8_t healthy=1;
uint8_t MT6701_IsHealthy(void) { return healthy; }
uint32_t MT6701_GetSampleCounter(void) { return samples; }

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
