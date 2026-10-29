/* Real modulator and board model; independent current/phase-voltage and timer
 * event models below verify output behavior, not a second copy of the driver. */
#include "test_current.c"
#include "motor_math.h"
extern uint32_t SystemCoreClock;
static unsigned writes,suppressed;
static uint32_t active[3];
static void update_event(TIM_TypeDef *t)
{
    if(t->CR1 & TIM_CR1_UDIS) { ++suppressed;return; }
    active[0]=t->CCR1;active[1]=t->CCR2;active[2]=t->CCR3;
}
void MotorHost_Compare(TIM_HandleTypeDef *h,unsigned ch,uint32_t value)
{
    assert((h->Instance->CR1 & TIM_CR1_UDIS) && host_primask);
    update_event(h->Instance); /* Put an update between every pair of writes. */
    if(ch==TIM_CHANNEL_1) h->Instance->CCR1=value;
    else if(ch==TIM_CHANNEL_2) h->Instance->CCR2=value;
    else h->Instance->CCR3=value;
    ++writes;
}
static float smooth_loss(float i,float dv)
{
    /* Independent piecewise reference for nominal deadtime/conduction loss. */
    float s=i<=-.1f ? -1 : i>=.1f ? 1 : i/.1f;
    return s*dv+.0018f*i;
}
static void voltage_vector(MotorAxis a,float *alpha,float *beta)
{
    TIM_TypeDef *t=a ? &tim8 : &tim1;
    float va=(float)t->CCR1/(float)t->ARR*bus;
    float vb=(float)t->CCR2/(float)t->ARR*bus;
    float vc=(float)t->CCR3/(float)t->ARR*bus;
    *alpha=(2*va-vb-vc)/3;*beta=(vb-vc)/sqrtf(3);
}
static void timestamps_and_update(void)
{
    const uint32_t dtg[]={0,17,127,128,191,192,223,224,255};
    const uint32_t ticks[]={0,17,127,128,254,256,504,512,1008};
    for(unsigned j=0;j<9;++j)for(unsigned k=0;k<3;++k)
        assert(MotorMath_DeadTicks(dtg[j],k)==ticks[j]*(1U<<k));
    assert(MotorMath_DeadTicks(17,3)==0);
    assert(fabsf(MotorMath_PredictAngle(1,1100,17000)-1.11f)<1e-6f);
    assert(MotorMath_PredictAngle(1,1100,170001)==1);
    assert(MotorMath_PredictAngle(1,NAN,100)==1);
    SystemCoreClock=85000000U;
    assert(fabsf(MotorMath_PredictAngle(1,-1100,8500)-.89f)<1e-6f);
    SystemCoreClock=170000000U;
    memset(&frame,0,sizeof(frame));bus=12;
    tim1.CR1=TIM_CR1_CEN;tim1.CCR1=10;tim1.CCR2=20;tim1.CCR3=30;
    update_event(&tim1);unsigned before=suppressed;
    host_primask=0;
    JerryFOC_setAxisVoltage(0,2,.3f,.4f,NULL,NULL);
    assert(suppressed==before+3 && active[0]==10 && active[1]==20 && active[2]==30);
    assert(host_primask==0 && tim1.CR1==TIM_CR1_CEN);
    update_event(&tim1);assert(active[0]==tim1.CCR1 && active[1]==tim1.CCR2 && active[2]==tim1.CCR3);
    host_primask=1;tim1.CR1|=TIM_CR1_UDIS;
    JerryFOC_setAxisVoltage(0,1,0,0,NULL,NULL);
    assert(host_primask==1 && (tim1.CR1 & TIM_CR1_UDIS));
    host_primask=0;tim1.CR1=0;
    puts("PASS: all DTG ranges/CKD and runtime core scaling; atomic three-CCR preload commit, no mixed active vector, existing UDIS/PRIMASK preserved");
}
static void timing_angles(void)
{
    memset(&frame,0,sizeof(frame));frame.timing_valid=true;bus=20;
    for(unsigned a=0;a<2;++a)for(unsigned down=0;down<2;++down)
    for(int dir=-1;dir<=1;dir+=2)for(unsigned wrap=0;wrap<2;++wrap) {
        TIM_TypeDef *t=a ? &tim8 : &tim1;t->CNT=down ? 6000 : 900;t->PSC=0;
        t->CR1=down ? TIM_CR1_DIR : 0;t->BDTR=17;
        uint32_t now=wrap ? 0xfffff000U : 100000U;host_dwt.CYCCNT=now;
        uint32_t elapsed=down ? 2*t->ARR-t->CNT : t->CNT;
        uint32_t offset=a ? 630 : 50;
        frame.current_cycles[a]=now-elapsed+offset;
        float speed=dir*100.0f,theta=.4f,aq,ad,alpha,beta;
        JerryFOC_setEncoderVoltage(a,3,0,theta,speed,&aq,&ad);
        voltage_vector(a,&alpha,&beta);
        float delay=(float)((down ? 3 : 2)*t->ARR-offset)/170000000;
        float expected=theta+11*speed*delay;
        assert(fabsf(alpha+3*sinf(expected))<.005f && fabsf(beta-3*cosf(expected))<.005f);
        assert(fabsf(aq-3)<.005f && fabsf(ad)<.005f);
    }
    frame.timing_valid=false;tim1.CR1=0;tim1.CNT=0;
    /* Bounded near-boundary work must defer coherently to neutral rather
     * than commit a voltage using an angle for the previous update event. */
    frame.timing_valid=true;tim1.CNT=tim1.ARR-64;host_dwt.CYCCNT=123;
    unsigned before=writes;float q=99,d=99;
    JerryFOC_setEncoderVoltage(0,2,0,.4f,100,&q,&d);
    zero_pwm();assert(writes==before+3 && q==0 && d==0);
    frame.timing_valid=false;tim1.CNT=0;
    puts("PASS: encoder voltage angle at next latch plus half-period, both axes/current ranks, up/down late computation, +/-speed and DWT wrap");
}
static void inverter_model(void)
{
    memset(&frame,0,sizeof(frame));
    const float current[]={-.5f,-.1001f,-.1f,-.0999f,-.001f,0,.001f,.0999f,.1f,.1001f,.5f};
    for(unsigned a=0;a<2;++a)for(unsigned v=0;v<2;++v)for(unsigned k=0;k<11;++k)
    for(unsigned dt=0;dt<2;++dt) {
        TIM_TypeDef *t=a ? &tim8 : &tim1;t->BDTR=dt ? 255 : 17;t->CR1=dt ? (2U<<8) : 0;
        t->PSC=0;bus=v ? 20 : 12;
        float ia=current[k],ib=-.4f*ia,ic=-(ia+ib);
        if(a) { frame.m2_current[0]=ia;frame.m2_current[1]=ib; }
        else { frame.m1_current[0]=ia;frame.m1_current[1]=ib;frame.m1_current[2]=ic; }
        float fraction=dt ? .005f : 34.0f/(2*8499);
        float ea=smooth_loss(ia,bus*fraction),eb=smooth_loss(ib,bus*fraction),ec=smooth_loss(ic,bus*fraction);
        float d,q,alpha,beta,theta=.7f;
        JerryFOC_setEncoderVoltage(a,.2f,-.1f,theta,0,&q,&d);
        voltage_vector(a,&alpha,&beta);
        alpha-=(2*ea-eb-ec)/3;beta-=(eb-ec)/sqrtf(3);
        float physical_d=alpha*cosf(theta)+beta*sinf(theta),physical_q=beta*cosf(theta)-alpha*sinf(theta);
        assert(fabsf(physical_d+.1f)<.005f && fabsf(physical_q-.2f)<.005f);
        assert(fabsf(d-physical_d)<.00001f && fabsf(q-physical_q)<.00001f);
    }
    memset(&frame,0,sizeof(frame));frame.m1_current[0]=NAN;
    JerryFOC_setEncoderVoltage(0,2,0,0,0,NULL,NULL);zero_pwm();
    /* Plain modulation used by HSO/alignment/detection/six-step never reads
     * the current-dependent model or the timing metadata. */
    frame.timing_valid=true;float q,d,alpha,beta;
    JerryFOC_setAxisVoltage(0,2,0,.7f,&q,&d);voltage_vector(0,&alpha,&beta);
    assert(fabsf(alpha+2*sinf(.7f))<.005f && fabsf(beta-2*cosf(.7f))<.005f);
    assert(MotorControl_SelectFeedback(true));
    JerryFOC_setEncoderVoltage(0,2,0,.7f,200,&q,&d);voltage_vector(0,&alpha,&beta);
    assert(fabsf(alpha+2*sinf(.7f))<.005f && fabsf(beta-2*cosf(.7f))<.005f);
    assert(MotorControl_SelectFeedback(false));
    /* Zero current error must not integrate rotating CCR rounding into a
     * fictitious voltage error. This failed before the PWM-tick deadband. */
    frame=(MotorSampleFrame){0};tim1.CR1=tim1.CNT=0;tim1.BDTR=17;
    bus=20;MotorControllers loops={0};loops.q.limit=loops.d.limit=10;
    loops.q.integral_prev=.8175f;loops.current_feedforward=1;
    for(unsigned k=0;k<12000;++k)
        JerryFOC_CurrentControl(0,&loops,(float)k*.01f,0,0,0,0,0);
    assert(fabsf(loops.q.integral_prev-.8175f)<1e-6f && fabsf(loops.d.integral_prev)<1e-6f);
    puts("PASS: nominal bridge-loss inverse at 12/20 V, DTG/CKD scaling and 0.5% cap, smooth current zero crossing, CCR-quantized physical-voltage feedback; NaN neutral and HSO/legacy bypass");
}
int main(void)
{
    tim1.ARR=tim8.ARR=8499;ready=calibrated=output_requested=true;healthy=aligned=1;
    JerryFOC_setAngleSource(&source);measured_model(0,440,9290);measured_model(1,440,9290);
    timestamps_and_update();timing_angles();inverter_model();
    assert(writes>0);
}
