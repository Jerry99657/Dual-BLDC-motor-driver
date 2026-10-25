/* Real sensorless adapter; only proprietary ST ARM binaries are mocked. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "st_hso.h"
HostDwt host_dwt;
uint32_t host_primask;
#include "hso_binary_stub.h"
static MotorSampleFrame frame;
static float sample_bus=20;
float MotorSampling_GetBusVoltage(void) { return sample_bus; }
static void step(void) {
    float a=StHso_ControlAngle(),d=binary_zero_current ? 0 : StHso_CurrentD();
    frame.m1_current[0]=d*cosf(a);
    frame.m1_current[1]=d*(-0.5f*cosf(a)+0.86602540378f*sinf(a));
    frame.m1_current[2]=-frame.m1_current[0]-frame.m1_current[1];
    for(unsigned i=0;i<3;++i) { frame.m1_current_raw[i]=2048;frame.m1_voltage_raw[i]=1350; }
    ++frame.sequence;
    StHso_Process(&frame,true,NAN,NAN,false); /* No encoder, ever. */
}
static StHsoSnapshot state(void) { StHsoSnapshot s;StHso_GetSnapshot(&s);return s; }
static void idle(void) { for(unsigned i=0;i<2000;++i) step(); assert(StHso_AtRest()); }
static void init(void) {
    frame=(MotorSampleFrame){0}; binary_emf=true;binary_jump=0;binary_zero_current=false;
    binary_flux=0.012f;binary_d_ripple=binary_phase=0;binary_flux_vary=false;binary_emf_samples=0;
    binary_frequency_ripple=binary_frequency_bias=binary_phase_rate_hz=0;binary_emf_spikes=false;
    binary_frequency_square=binary_emf_alternating=false;
    StHso_Init(&MOTOR_4015); idle();
}
static void reach_control(void) {
    for(unsigned i=0;i<79000 && !StHso_IsUsable();++i) step();
    StHsoSnapshot s=state();
    if(!StHso_IsUsable()) fprintf(stderr,"startup state %d reason %d qual %u hz %f angle %f flux %f\n",s.state,s.reason,s.qualified_samples,s.emf_hz,s.angle_error_rad,s.flux_wb);
    assert(StHso_IsUsable() && StHso_CanTakeover() && s.state==ST_HSO_CONTROL);
}
static void capture_start(void) {
    init();StHsoConfig c;StHso_GetConfig(&c);c.flux_wb=0.012f;
    assert(StHso_SetConfig(&c));idle();assert(StHso_Start(10));
    while(state().state!=ST_HSO_CAPTURE) {
        step();assert(StHso_IsRunning());
        if(state().state==ST_HSO_ALIGN) assert(!binary_offsets);
        if(state().state==ST_HSO_RAMP) assert(binary_offsets && binary_track);
    }
    for(unsigned i=0;i<999;++i) step(); /* First CAPTURE frame already counted. */
    assert(!StHso_IsUsable() && state().qualified_samples==0 && binary_offsets);
    assert(state().emf_offset_v==0 && fabsf(state().emf_offset_limit_v-0.05883f)<1e-6f);
}
static double filtered_high(double bus,double duty,double seconds) {
    /* Independent continuous-time periodic PWM/RC solution, tau=40 us.
     * t=0 is the centre of the high pulse (TIM1 counter valley). */
    const double tau=40e-6,T=100e-6;
    double centre=bus*(1-sinh((1-duty)*T/(2*tau))/sinh(T/(2*tau)));
    return bus+(centre-bus)*exp(-seconds/tau);
}
static void voltage_alignment_tests(void) {
    const double te=(25+47.5)/42500000.0,tl=te+60/42500000.0;
    for(unsigned supply=0;supply<2;++supply) for(unsigned vector=0;vector<2;++vector) {
        init();sample_bus=supply ? 20 : 12;assert(StHso_Start(50));step();
        double d[]={.5,.5,.5};if(vector) {d[0]=.4;d[1]=.6;d[2]=.55;}
        double truth[3];
        for(unsigned i=0;i<3;++i) {
            truth[i]=filtered_high(sample_bus,d[i],te);
            double measured=filtered_high(sample_bus,d[i],i==1 ? te : tl);
            /* Quantize the actual ADC just as on board; keep raw counts. */
            frame.m1_voltage_raw[i]=(uint16_t)lround(measured/35.86*4095);
            frame.m1_voltage[i]=(float)frame.m1_voltage_raw[i]*35.86f/4095;
        }
        MotorSampleFrame original=frame;
        double expected_a=(2*truth[0]-truth[1]-truth[2])/3;
        double expected_b=(truth[1]-truth[2])/sqrt(3.0);
        frame.m1_voltage_high_plateau=false;++frame.sequence;StHso_Process(&frame,true,NAN,NAN,false);
        float raw_a=(float)binary_voltage_in.A/1073741824*72.6f;
        float raw_b=(float)binary_voltage_in.B/1073741824*72.6f;
        assert(hypot(raw_a-expected_a,raw_b-expected_b)>.09);
        frame.m1_voltage_high_plateau=true;++frame.sequence;StHso_Process(&frame,true,NAN,NAN,false);
        float aligned_a=(float)binary_voltage_in.A/1073741824*72.6f;
        float aligned_b=(float)binary_voltage_in.B/1073741824*72.6f;
        assert(hypot(aligned_a-expected_a,aligned_b-expected_b)<.009); /* One divider-referred ADC LSB. */
        for(unsigned i=0;i<3;++i) {
            assert(frame.m1_voltage_raw[i]==original.m1_voltage_raw[i]);
            assert(frame.m1_voltage[i]==original.m1_voltage[i]);
        }
        /* Unknown/coast/edge metadata must not apply a PWM-source model. */
        frame.m1_voltage_high_plateau=false;++frame.sequence;StHso_Process(&frame,true,NAN,NAN,false);
        assert(fabsf((float)binary_voltage_in.A/1073741824*72.6f-raw_a)<1e-5f);
        assert(fabsf((float)binary_voltage_in.B/1073741824*72.6f-raw_b)<1e-5f);
    }
    init();assert(StHso_Start(50));step();frame.m1_voltage_high_plateau=true;
    sample_bus=NAN;++frame.sequence;StHso_Process(&frame,true,NAN,NAN,false);
    assert(!StHso_IsRunning() && state().reason==ST_HSO_REASON_SAMPLE);sample_bus=20;
    puts("PASS: actual adapter against independent periodic PWM/RC/ADC model at 12/20 V, zero and differential vectors, <1 ADC-LSB alignment error, raw preservation, edge/coast bypass and invalid bus rejection");
}
static float wrapped(float a) { return atan2f(sinf(a),cosf(a)); }
static void phase_capture_tests(void) {
    /* The user's -28.8 deg offset and mirror-image reverse rotation are
     * admissible only if stable. Test with independent derivative ripple,
     * across electrical wraps; current-vector continuity includes dq rotation. */
    for(unsigned direction=0;direction<2;++direction) {
        init();StHsoConfig c;StHso_GetConfig(&c);c.flux_wb=0.012f;
        assert(StHso_SetConfig(&c));idle();
        binary_jump=direction ? 0.502654825f : -0.502654825f;
        assert(StHso_Start(direction ? -10 : 10));
        binary_frequency_ripple=3;reach_control();
        assert(fabsf(state().angle_error_rad)>0.4363f && state().capture_resets==0);
        binary_frequency_ripple=0;
        float start=StHso_ControlAngle(),offset=wrapped(state().angle_rad-start);
        float previous_d,previous_q=0;
        StHso_BlendCurrent(&previous_d,&previous_q);
        assert(fabsf(previous_d-0.6f)<1e-5f && previous_q==0);
        for(unsigned i=1;i<=1000;++i) {
            step();assert(StHso_IsUsable());
            float d,q=0;StHso_BlendCurrent(&d,&q);
            /* With zero requested speed-PI torque, the remaining I/F vector
             * must retain its original direction, not rotate with the frame. */
            float expected_angle=wrapped(state().angle_rad-offset);
            if (i<1000) {
                assert(fabsf(wrapped(StHso_ControlAngle()+atan2f(q,d)-expected_angle))<1e-5f);
                assert(fabsf(hypotf(d,q)-0.6f*(1-(float)i/1000))<1e-5f);
            }
            assert(fabsf(d-previous_d)<0.002f && fabsf(q-previous_q)<0.002f);
            previous_d=d;previous_q=q;
            q=direction ? -MOTOR_4015.rated_current : MOTOR_4015.rated_current;
            StHso_BlendCurrent(&d,&q);
            assert(hypotf(d,q)<=MOTOR_4015.rated_current+1e-5f);
        }
        assert(previous_d==0 && previous_q==0);
    }
    /* A small frequency slip can pass the old 15% frequency test but cannot
     * stay in a fixed 10-degree phase window for 200 ms. Never certify it by
     * moving the phase reference along with the drifting observer. */
    capture_start();binary_phase_rate_hz=0.25f;bool saw_drift=false;
    for(unsigned i=0;i<21000 && StHso_IsRunning();++i) {
        step();StHsoSnapshot s=state();
        if(s.capture_reason==ST_HSO_REASON_PHASE_DRIFT) {
            saw_drift=true;assert(fabsf(s.capture_drift_rad)>0.1745f);
        }
        assert(!StHso_IsUsable());
    }
    assert(saw_drift && state().reason==ST_HSO_REASON_CAPTURE_TIMEOUT);
    /* Bounded but oscillatory phase (15 deg, 5 Hz) is also not stable enough;
     * its derivative is <=1.31 Hz, entirely within the +/-2.25 Hz gate. */
    capture_start();saw_drift=false;
    for(unsigned i=0;i<21000 && StHso_IsRunning();++i) {
        float p=(float)i*0.00314159265359f;
        binary_jump=0.261799388f*sinf(p);binary_frequency_bias=1.30899694f*cosf(p);
        step();saw_drift |= state().capture_reason==ST_HSO_REASON_PHASE_DRIFT;
        assert(!StHso_IsUsable());
    }
    assert(saw_drift && state().reason==ST_HSO_REASON_CAPTURE_TIMEOUT);
    /* User-like speed bias still refuses takeover despite a steady angle:
     * accepting a load-angle offset does not waive independent frequency. */
    capture_start();binary_jump=-0.502654825f;binary_frequency_bias=-2.3f;
    for(unsigned i=0;i<21000 && StHso_IsRunning();++i)step();
    assert(!StHso_IsUsable() && state().reason==ST_HSO_REASON_CAPTURE_TIMEOUT &&
           state().capture_reason==ST_HSO_REASON_FREQUENCY);
    puts("PASS: stable +/-28.8 deg I/F offsets with frequency ripple, both directions/wraps, continuous 50 ms frame + 100 ms dq vector crossfade/current bound; phase drift/oscillation and sustained frequency bias cannot take over (ST binaries mocked)");
}
static void capture_tests(void) {
    /* Flux learning may complete just before the old shared 6 s deadline.
     * CAPTURE still needs its full settle + quality window, with a separate
     * bounded lock deadline. No-EMF learning itself still expires at 6 s. */
    init();binary_emf=false;assert(StHso_Start(10));
    while(state().state!=ST_HSO_LEARNING) {step();assert(StHso_IsRunning());}
    unsigned cycle=(unsigned)lroundf(10000/fabsf(binary_hz));
    for(unsigned i=0;i<50U*cycle-1U;++i)step();
    binary_emf=true;reach_control();
    assert(binary_emf_samples>60000 && binary_emf_samples<62000);
    /* Raw derivative ripple +/-3 Hz (angle ripple +/-3.44 deg) repeatedly
     * violates R49's +/-2.25 Hz gate for >2 ms. Its independent mean remains
     * 15 Hz. No tolerance, virtual angle or forced frequency is substituted. */
    capture_start();binary_frequency_ripple=3;reach_control();
    for(unsigned i=0;i<2000;++i)step();
    assert(StHso_IsUsable()); /* Existing CONTROL checks also hold in this model. */
    capture_start();binary_emf_spikes=true;reach_control();
    for(unsigned i=0;i<2000;++i)step();
    assert(StHso_IsUsable());
    /* <=1% brief phase outliers fit the bounded 200 ms admission window. */
    capture_start();
    for(unsigned i=0;i<2100 && !StHso_IsUsable();++i) {binary_jump=i%100U==0 ? 0.6f : 0;step();}
    binary_jump=0;assert(StHso_IsUsable() && state().capture_resets==0 && state().qualified_samples==2000);
    /* 10% bad frames, with no consecutive pair: old unlimited good-sample
     * accumulation could admit this. The new total-error budget must not. */
    capture_start();
    for(unsigned i=0;i<60001 && StHso_IsRunning();++i) {binary_jump=i%10U==0 ? 0.6f : 0;step();}
    StHsoSnapshot s=state();
    assert(!StHso_IsRunning() && s.reason==ST_HSO_REASON_CAPTURE_TIMEOUT && s.capture_resets>0);
    assert(s.capture_reason==ST_HSO_REASON_PHASE_DRIFT && s.entry_reason==ST_HSO_REASON_PHASE_DRIFT);
    assert(fabsf(s.capture_angle_rad-0.6f)<0.02f && fabsf(s.capture_frequency_error_hz)<0.02f &&
           fabsf(s.capture_emf_ratio-1)<0.01f);
    /* Exactly 20 consecutive bad frames still reset; a subsequent good frame
     * cannot erase the rejection record. Sampling/drive do not auto-restart. */
    capture_start();binary_jump=0.6f;for(unsigned i=0;i<20;++i)step();
    s=state();assert(s.capture_resets==1 && s.qualified_samples==0 && s.capture_reason==ST_HSO_REASON_PHASE_DRIFT);
    binary_jump=0;step();assert(state().reason==ST_HSO_REASON_NONE && state().entry_reason==ST_HSO_REASON_PHASE_DRIFT);
    assert(state().capture_resets==1 && fabsf(state().capture_angle_rad-s.capture_angle_rad)<1e-6f);
    reach_control();
    /* Filtering must not admit sustained frequency/phase/amplitude errors. */
    capture_start();binary_frequency_bias=-3;
    for(unsigned i=0;i<60001 && StHso_IsRunning();++i)step();
    s=state();assert(s.reason==ST_HSO_REASON_CAPTURE_TIMEOUT && s.capture_reason==ST_HSO_REASON_FREQUENCY);
    assert(fabsf(s.capture_frequency_error_hz+3)<0.02f && !s.usable);
    capture_start();binary_jump=1.2f; /* Beyond finite 60-degree startup bound. */
    for(unsigned i=0;i<60001 && StHso_IsRunning();++i)step();
    assert(state().reason==ST_HSO_REASON_CAPTURE_TIMEOUT && state().capture_reason==ST_HSO_REASON_ANGLE);
    capture_start();binary_emf=false;
    for(unsigned i=0;i<60001 && StHso_IsRunning();++i)step();
    s=state();assert(s.reason==ST_HSO_REASON_CAPTURE_TIMEOUT && s.capture_reason==ST_HSO_REASON_FLUX && s.capture_emf_ratio<0.01f);
    /* A valid filtered mean alone cannot take over: require an actual good
     * current frame too. Alternating out-of-range raw inputs never get one. */
    for(unsigned k=0;k<2;++k) {
        capture_start();binary_frequency_square=k==0;binary_emf_alternating=k==1;
        for(unsigned i=0;i<60001 && StHso_IsRunning();++i)step();
        s=state();assert(!s.usable && s.reason==ST_HSO_REASON_CAPTURE_TIMEOUT);
        assert(s.qualified_samples==2000 && s.capture_resets==0);
        assert(s.entry_reason==(k==0 ? ST_HSO_REASON_FREQUENCY : ST_HSO_REASON_FLUX));
    }
    puts("PASS: CAPTURE frequency/energy ripple filtering, 100 ms settle + bounded 200 ms quality window, 2.5% total/2 ms run limits, raw final-frame entry, persistent reset cause/angle/frequency/energy diagnostics, sustained faults rejected (ST binaries mocked)");
}
static void command_floor_tests(void) {
    const MotorParams *motors[]={&MOTOR_2804,&MOTOR_4015};
    for(unsigned p=0;p<2;++p) for(unsigned reverse=0;reverse<2;++reverse) {
        init(); StHso_Init(motors[p]); idle();
        float sign=reverse ? -1.0f : 1.0f;
        float below=sign*nextafterf(5.0f,0.0f);
        assert(StHso_MinSpeed()==5.0f);
        assert(!StHso_ValidTarget(below) && !StHso_Start(below));
        assert(!StHso_ValidTarget(0) && !StHso_ValidTarget(NAN) && !StHso_ValidTarget(sign*INFINITY));
        assert(StHso_AtRest() && !StHso_IsRunning());
        assert(StHso_ValidTarget(sign*5.0f) && StHso_Start(sign*5.0f));
        assert(StHso_IsStarting() && !StHso_ValidTarget(-sign*5.0f));
    }
    puts("PASS: R76 real HSO command floor is 5 rad/s for both profiles/directions; immediately lower values rejected, zero/nonfinite and running reverse guards retained");
}
int main(void) {
    command_floor_tests();
    init(); assert(!StHso_Start(0) && !StHso_Start(1) && !StHso_Start(NAN));
    assert(state().emf_offset_v==0 && state().emf_offset_limit_v==0); /* No configured observer yet. */
    StHsoConfig c; StHso_GetConfig(&c); assert(c.flux_wb==0);
    assert(!StHso_SetConfig(NULL));
    for(unsigned field=0;field<5;++field) for(unsigned kind=0;kind<3;++kind) {
        StHsoConfig bad=c;float *v[]={&bad.rs_ohm,&bad.ls_h,&bad.flux_wb,&bad.voltage_pole_rps,&bad.sample_delay_cycles};
        *v[field]=kind==0 ? NAN : kind==1 ? INFINITY : -INFINITY;assert(!StHso_SetConfig(&bad));
    }
    assert(StHso_ValidTarget(200) && !StHso_ValidTarget(600)); /* Independent <400 electrical Hz gate. */
    assert(StHso_Start(10) && StHso_IsStarting() && StHso_CurrentD()==0);
    for(unsigned i=0;i<2500;++i) step();
    assert(!binary_offsets); /* Alignment remains frozen, unlike rotating CAPTURE. */
    assert(fabsf(StHso_CurrentD()-0.6f)<1e-5f);
    assert(!StHso_SetConfig(&c) && !StHso_LearnFlux());
    reach_control();StHso_GetConfig(&c);assert(fabsf(c.flux_wb-0.012f)<0.00002f);
    assert(fabsf(c.sample_delay_cycles-60.0f/42500000*10000)<1e-7f);
    assert(fabsf(binary_ip.KsampleDelay-c.sample_delay_cycles)<1e-7f);
    assert(binary_offsets);
    assert(binary_ip.cycleFreq_Hz==10000 && binary_ip.fullScaleCurrent_A==6.6f && binary_params.FullScaleVoltage_V==72.6f);
    assert(!StHso_Start(-10) && !StHso_ValidTarget(-10));
    for(unsigned i=0;i<1000;++i)step();
    assert(StHso_CurrentD()==0);
    binary_jump=1.0f;step();assert(!StHso_IsRunning() && state().reason==ST_HSO_REASON_ANGLE);
    StHso_Stop();binary_jump=0;idle();assert(!StHso_IsRunning());
    assert(StHso_Start(10));reach_control();binary_emf=false;
    for(unsigned i=0;i<20;++i)step();
    assert(!StHso_IsRunning() && state().reason==ST_HSO_REASON_FLUX);
    StHso_Stop();binary_emf=true;idle();
    assert(StHso_Start(-10));reach_control();assert(StHso_ControlVelocity()<0);
    StHso_Process(&frame,false,0,0,false);assert(!StHso_IsRunning() && state().reason==ST_HSO_REASON_SAMPLE);
    StHso_Stop();idle();assert(StHso_Start(10));step();frame.sequence+=2;step();
    assert(!StHso_IsRunning() && state().reason==ST_HSO_REASON_SEQUENCE);
    init();binary_emf=false;assert(StHso_Start(10));
    for(unsigned i=0;i<60001;++i)step();
    assert(!StHso_IsRunning() && state().reason==ST_HSO_REASON_LEARN_TIMEOUT);
    assert(state().entry_reason==ST_HSO_REASON_FLUX);
    init();StHso_GetConfig(&c);c.flux_wb=0.012f;assert(StHso_SetConfig(&c));idle();
    binary_emf=false;assert(StHso_Start(10));
    for(unsigned i=0;i<60001;++i)step();
    assert(!StHso_IsRunning() && state().reason==ST_HSO_REASON_CAPTURE_TIMEOUT);
    init();binary_zero_current=true;assert(StHso_Start(10));
    for(unsigned i=0;i<10000;++i)step();
    assert(!StHso_IsRunning() && state().reason==ST_HSO_REASON_STARTUP);
    init();assert(StHso_Start(10));step();StHso_RecordControlCycles(17000);
    assert(!StHso_IsRunning() && state().reason==ST_HSO_REASON_TIMING);
    init();assert(StHso_Start(10));step();frame.m1_voltage[0]=NAN;++frame.sequence;
    StHso_Process(&frame,true,0,0,false);assert(!StHso_IsRunning() && state().reason==ST_HSO_REASON_RANGE);
    init();StHso_Stop();frame.m1_voltage[0]=2;
    for(unsigned i=0;i<3000;++i)step();
    assert(!StHso_AtRest() && !StHso_Start(10));
    frame.m1_voltage[0]=0;idle();StHso_Enable(false);assert(!StHso_Start(10));
    /* Instantaneous d/q phase is invalid on EVERY sample. Zero-mean EMI/PWM
     * ripple must average out without selecting only favourable samples. */
    init();binary_d_ripple=0.8f;assert(StHso_Start(10));reach_control();
    StHso_GetConfig(&c);assert(fabsf(c.flux_wb-0.012f)<0.00002f);
    init();binary_d_ripple=0.8f;assert(StHso_Start(-10));reach_control();
    StHso_GetConfig(&c);assert(fabsf(c.flux_wb-0.012f)<0.00002f);
    /* The rounded minimum target still leaves enough bounded time for full
     * electrical cycles, settling, flux learning and HSO qualification. */
    init();binary_d_ripple=0.8f;assert(StHso_Start(5));reach_control();
    /* Finite, stable measurements still must pass strength/direction/phase. */
    const float phases[]={0,3.14159265359f,1.0471975512f};
    const StHsoReason gates[]={ST_HSO_REASON_EMF_LOW,ST_HSO_REASON_FREQUENCY,ST_HSO_REASON_ANGLE};
    for(unsigned k=0;k<3;++k) {
        init();binary_phase=phases[k];if(k==0) binary_flux=0.0005f;
        assert(StHso_Start(10));
        for(unsigned i=0;i<60001;++i) step();
        assert(!StHso_IsRunning() && state().reason==ST_HSO_REASON_LEARN_TIMEOUT && state().entry_reason==gates[k]);
        assert(state().flux_wb>0);StHso_GetConfig(&c);assert(c.flux_wb==0);
    }
    init();binary_flux_vary=true;assert(StHso_Start(10));bool saw_ripple=false;
    for(unsigned i=0;i<60001;++i) {step();saw_ripple |= state().entry_reason==ST_HSO_REASON_FLUX_RIPPLE;}
    StHso_GetConfig(&c);assert(saw_ripple && !StHso_IsRunning() && c.flux_wb==0);
    assert(state().entry_reason==ST_HSO_REASON_FLUX_RIPPLE);
    assert(!strcmp(StHso_ReasonText(ST_HSO_REASON_EMF_LOW),"EMF LOW"));
    assert(!strcmp(StHso_ReasonText(ST_HSO_REASON_LEARN_TIMEOUT),"FLUX TIMEOUT"));
    for(unsigned reason=0;reason<=ST_HSO_REASON_PHASE_DRIFT;++reason)
        assert(strlen(StHso_ReasonText((StHsoReason)reason))<16U);
    for(unsigned state_id=0;state_id<=ST_HSO_FAULT;++state_id)
        assert(strlen(StHso_StateText((StHsoState)state_id))<16U);
    capture_tests();
    phase_capture_tests();
    voltage_alignment_tests();
    puts("PASS: whole-electrical-cycle EMF averages, no favourable-sample selection, zero-mean phase ripple in both directions and low speed, finite candidate diagnostics, strength/direction/phase/variance rejection, timeout retains failing gate (ST binaries mocked)");
    puts("PASS: encoder-free RsDC-style current alignment, I/F flux bootstrap, qualified HSO takeover, both directions, current ramp, no-EMF/alignment timeout, sample/sequence/range/timing/angle faults, idle admission and explicit restart (ST binaries mocked)");
    return 0;
}
