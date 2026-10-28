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
    float history[4]={0},oracle[4]={0};uint8_t index=0,count=0,oi=0,oc=0;
    for(unsigned i=0;i<10000;++i) {
        float position=sinf((float)i*.005f)*3,dt=i%2 ? .001f : .002f;
        float old=oracle[oi];oracle[oi]=position;oi=(oi+1U)%4U;if(oc<4)++oc;
        assert(MotorMath_WindowVelocity(position,history,&index,&count,dt)==(position-old)/(dt*oc));
    }
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
        AngleSrc_Encoder_Update(.001f);assert(source->getMechanicalAngle()==position);
        healthy=0;Angle=2;AngleSrc_Encoder_Update(.001f);
        healthy=1;++samples;AngleSrc_Encoder_Update(.001f);
        assert(source->getMechanicalAngle()==position && source->getVelocity()==0);
    }
    puts("PASS: common velocity window vs previous arithmetic, signed encoder polarity, electrical alignment, wrap, duplicate sample and recovery without false motion");
}
