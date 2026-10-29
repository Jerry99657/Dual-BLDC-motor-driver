#include "encoder_estimator.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define TAU 6.28318530717958647692f
#define PI (TAU*0.5f)

/* Independent copy of the pre-refactor normal tracking calculation, used as
 * an exact numerical oracle rather than checking only a final average. */
typedef struct { float previous,position,velocity,h[4]; unsigned index,count; } Legacy;
static void legacy_step(Legacy *s,float angle,int direction,float dt)
{
    float delta=angle-s->previous;
    if(delta>PI) delta-=TAU;
    if(delta< -PI) delta+=TAU;
    s->previous=angle;s->position+=delta*direction;
    float old=s->h[s->index];s->h[s->index]=s->position;
    s->index=(s->index+1)%4;if(s->count<4)++s->count;
    s->velocity=(s->position-old)/(dt*s->count);
}
static float wrapped(float a)
{
    a=fmodf(a,TAU);return a<0 ? a+TAU : a;
}
static void trajectories(void)
{
    for(int dir=-1;dir<=1;dir+=2) for(unsigned trial=0;trial<4;++trial) {
        EncoderEstimator s;Legacy old={.previous=6.281f};
        EncoderEstimator_Reset(&s,old.previous,0,true);
        float continuous=old.previous;
        for(unsigned i=1;i<=40000;++i) {
            float dt=trial==3 && i%2 ? .002f : .001f;
            float speed=trial==0 ? 40 : trial==1 ? -120 :
                trial==2 ? .174532925f*TAU*4*cosf((float)i*.001f*TAU*4) :
                .1f*sinf((float)i*.005f);
            continuous+=speed*dt;
            float angle=wrapped(continuous);
            if(trial==3) angle=floorf(angle/TAU*16384)*TAU/16384;
            legacy_step(&old,angle,dir,dt);
            assert(EncoderEstimator_Update(&s,angle,i,true,dir,dt));
            assert(s.position==old.position && s.velocity==old.velocity);
            assert(s.count==old.count && s.index==old.index);
            if(i%17==0) {
                EncoderEstimator before=s;
                assert(!EncoderEstimator_Update(&s,3,i,true,dir,dt));
                assert(memcmp(&s,&before,sizeof(s))==0);
            }
        }
    }
}
static void recovery(void)
{
    EncoderEstimator a={0},b={0};
    assert(!EncoderEstimator_Update(&a,3,0,true,1,.001f));
    assert(EncoderEstimator_Update(&a,3,1,true,1,.001f));
    assert(a.synced && !a.count && a.position==0 && a.velocity==0);
    EncoderEstimator_Reset(&a,TAU-.001f,UINT32_MAX,true);
    EncoderEstimator_Reset(&b,.001f,7,true);
    for(unsigned i=0;i<8;++i) {
        assert(EncoderEstimator_Update(&a,.001f+.001f*i,i,true,1,.001f));
        assert(EncoderEstimator_Update(&b,wrapped(-.001f*i),8+i,true,-1,.001f));
    }
    assert(fabsf(a.position-.009f)<2e-6f && fabsf(b.position-.008f)<2e-6f);
    assert(fabsf(a.velocity-1)<.001f && fabsf(b.velocity-1)<.001f);
    EncoderEstimator untouched=b;
    float position=a.position;
    assert(!EncoderEstimator_Update(&a,2,7,false,1,.001f));
    assert(!a.synced && a.velocity==0 && a.count==0 && a.position==position);
    assert(!EncoderEstimator_Update(&a,2,7,true,1,.001f));
    assert(!a.synced); /* Health alone cannot admit an old frame. */
    assert(EncoderEstimator_Update(&a,2,8,true,1,.001f));
    assert(a.synced && a.velocity==0 && a.position==position && a.count==0);
    assert(EncoderEstimator_Update(&a,2.001f,9,true,1,.001f));
    assert(fabsf(a.velocity-1)<.001f && a.count==1);
    assert(memcmp(&b,&untouched,sizeof(b))==0);
    for(unsigned i=0;i<2;++i) {
        assert(!EncoderEstimator_Update(&a,3,10+i,true,1,i ? -.001f : 0));
        assert(!a.synced && !a.count && a.velocity==0);
    }
    EncoderEstimator_Reset(&a,0,30,true);
    assert(a.position==0 && a.velocity==0 && !a.count);
    /* Sensor validity belongs to the SSI decoder: a valid zero angle remains
     * legal here and signed pole pairs must not be applied a second time. */
    assert(EncoderEstimator_Update(&a,0,31,true,-1,.001f));
    assert(a.count==1 && a.position==0 && a.velocity==0);
}
int main(void)
{
    trajectories();recovery();
    puts("PASS: 320000 samples equal legacy four-point tracking; +/- direction, multi-turn, 4 Hz reversal, quantization, variable step, duplicates, counter rollover, cold start, alignment, outage/recovery and independent instances");
}
