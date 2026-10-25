#include "motor_detect.h"
#include "motor_control.h"
#include "MT6701.h"
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

#define TWO_PI 6.28318530718f
/* At least one mechanical revolution for every admitted pole count. This
 * reduces encoder INL/eccentricity bias compared with a two-pole-pitch sweep. */
#define SWEEP_TICKS 167552U /* 32 electrical turns at 12 electrical rad/s. */
enum { IDLE, RS_LOW, RS_HIGH, FORWARD, HOLD_FORWARD, REVERSE, HOLD_REVERSE,
       L_SETTLE, L_PULSE, FLUX_RAMP, FLUX_MEASURE, DONE, FAILED };
static volatile unsigned phase;
static const char *volatile message="READY";
static uint32_t ticks, last_frame;
static volatile uint32_t result;
static float limit, angle, d_integral, applied;
/* Keep unwrapped mechanical position in native 14-bit encoder counts. */
static int32_t position, previous, origin, middle, forward;
static float sum_i, sum_u, first_i, first_u, resistance;
static volatile uint32_t result_l,result_flux;
static uint32_t candidate,window;
static float inductance,q_integral,speed,wave[20];
static float sum_q,sum_vq,flux_first;

__attribute__((noinline)) bool MotorDetect_Active(void) { return phase-RS_LOW<=FLUX_MEASURE-RS_LOW; }
uint32_t MotorDetect_Result(void) { return result; }
uint32_t MotorDetect_Inductance(void) { return result_l; }
uint32_t MotorDetect_Flux(void) { return result_flux; }
const char *MotorDetect_Status(void) { return message; }
static float clip(float v,float lo,float hi) { return fminf(hi,fmaxf(lo,v)); }
/* Only called for the validated normal ratio .005.. .93. Range reduction
 * gives |z|<=1/3; the atanh series through z^11 has <1.1e-7 absolute error.
 * This avoids pulling the general libm log/error machinery into 128 KiB. */
static float ratio_log(float v) {
    union { float f; uint32_t u; } a={.f=v};
    int exponent=(int)(a.u>>23)-127;
    a.u=(a.u&0x7fffffU)|0x3f800000U;
    float z=(a.f-1)/(a.f+1),power=z,sum=z;
    for(unsigned k=3;k<=11;k+=2) { power*=z*z; sum+=power/(float)k; }
    return 2*sum+(float)exponent*0.69314718056f;
}
static float wrap(float v) {
    if(v>3.14159265359f) v-=TWO_PI;
    if(v< -3.14159265359f) v+=TWO_PI;
    return v;
}
static bool finish(const char *text,bool ok) {
    message=text; phase=ok ? DONE : FAILED;
    /* Mark inactive first: the ordinary coast path also calls Detect_Stop. */
    JerryFOC_stop();
    return true;
}
void MotorDetect_Stop(void) {
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if(MotorDetect_Active()) finish(MotorSampling_IsGlobalStopped() ? "GLOBAL STOP" : "STOPPED",false);
    __set_PRIMASK(mask);
}
__attribute__((noinline)) void MotorDetect_Reset(void) {
    MotorDetect_Stop(); result=result_l=result_flux=0; phase=IDLE; message="READY";
}
bool MotorDetect_Start(unsigned ma) {
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool ok=ma>=100 && ma<=500 && !MotorDetect_Active() &&
        MotorControl_GetWork()==MOTOR_WORK_SINGLE_M1 &&
        !MotorSampling_IsOutputRequested() && !MotorSampling_IsAxisOutputRequested(MOTOR_AXIS_2);
    if(ok) {
        JerryFOC_stop(); /* Zero ordinary targets, cancel pending boot alignment. */
        /* The normal start gate checks ADC/calibration, fresh stopped encoder
         * and alignment; it rejects six-step and HSO's zero torque target. */
        ok=JerryFOC_startOutput()!=0;
    }
    if(ok) {
        ticks=0; result=result_l=result_flux=0; angle=d_integral=applied=0; position=0;
        limit=(float)ma*0.001f;
        previous=MT6701_GetRawAngle();
        last_frame=MotorSampling_GetFrame()->sequence;
        message="RS LOW"; phase=RS_LOW;
    }
    if(!ok && !MotorDetect_Active()) message="Stop/check encoder";
    __set_PRIMASK(mask); return ok;
}
void MotorDetect_Service(uint32_t now) {
    (void)now;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    /* MotorSampling_Service owns the existing 5 ms ADC/bus watchdog, before
     * MotorControl_Service. Retain its fault rather than adding another timer. */
    if(MotorDetect_Active() && !MotorSampling_IsReady()) finish(MotorSampling_GetAxisStatusText(MOTOR_AXIS_1),false);
    __set_PRIMASK(mask);
}
static int poles(int32_t movement) {
    float p=524288.0f/(float)abs(movement); /* 32 * 16384 encoder counts. */
    if(!(p>0.92f && p<32.08f)) return 0;
    int rounded=(int)(p+0.5f);
    return fabsf(p-(float)rounded)<0.08f ? rounded : 0;
}
__attribute__((noinline)) bool MotorDetect_Process(const MotorSampleFrame *f) {
    if(!MotorDetect_Active()) return false;
    unsigned stage=phase;
    if(!MotorSampling_IsReady() || !MotorSampling_IsOutputRequested() || MotorSampling_IsGlobalStopped()) {
        return finish(MotorSampling_IsGlobalStopped() ? "GLOBAL STOP" : MotorSampling_GetAxisStatusText(MOTOR_AXIS_1),false);
    }
    if(f->sequence!=last_frame+1U) return finish("ADC SEQ",false);
    last_frame=f->sequence;
    if(!MT6701_IsHealthy()) {
        /* ADC capture already enforces phase-current/bus/Break protection.
         * Freeze the last bounded PWM vector and all measurement progress;
         * do not finish a measurement using only gap admission. */
        if(stage>=FORWARD && MT6701_HasNearZeroGap()) return true;
        return finish("ENCODER1 LOST",false);
    }
    int32_t raw=MT6701_GetRawAngle(),delta=raw-previous;
    if(delta>8192) delta-=16384;
    if(delta< -8192) delta+=16384;
    if(abs(delta)>130) return finish("E1 JUMP",false);
    previous=raw; position+=delta;
    float iu=f->m1_current[0],iv=f->m1_current[1],iw=f->m1_current[2];
    float hard=limit*1.8f+0.05f;
    if(!isfinite(iu+iv+iw) || fabsf(iu+iv+iw)>0.1f+limit*0.2f ||
        fabsf(iu)>hard || fabsf(iv)>hard || fabsf(iw)>hard) {
        return finish("CURRENT",false);
    }
    float alpha=iu, beta=(iu+2*iv)*0.57735026919f;
    float id,iq; JerryFOC_Park(alpha,beta,angle,&id,&iq);
    ++ticks;
    unsigned ramp=ticks<2000U ? ticks : 2000U;
    float current=limit*(stage==RS_LOW ? (float)ramp*0.00025f : 1);
    if(stage==RS_LOW || stage==RS_HIGH) {
        if(ticks==6000U) {
            sum_i=sum_u=0; middle=position;
        }
        if(ticks>6000U) {
            if(fabsf(id-current)>limit*0.05f+0.025f || abs(position-middle)>26) {
                return finish("RS NOT STEADY",false);
            }
            sum_i+=id; sum_u+=applied;
        }
        if(ticks==8000U) {
            float i=sum_i/2000,u=sum_u/2000;
            if(stage==RS_LOW) { first_i=i; first_u=u; phase=RS_HIGH; message="RS HIGH"; }
            else {
                resistance=(u-first_u)/(i-first_i);
                if(!(resistance>=0.1f && resistance<=50) || i-first_i<limit*0.3f) {
                    return finish("RS RANGE",false);
                }
                origin=position; phase=FORWARD; message="POLES +";
            }
            ticks=0;
        }
    } else if(stage==FORWARD || stage==REVERSE) {
        if(ticks==SWEEP_TICKS/2) middle=position-origin;
        if(ticks>=SWEEP_TICKS) {
            angle=0; ticks=0;
            phase=stage==FORWARD ? HOLD_FORWARD : HOLD_REVERSE;
        } else angle=wrap(angle+(stage==FORWARD ? 0.0012f : -0.0012f));
    } else if(stage==HOLD_FORWARD || stage==HOLD_REVERSE) {
      if(ticks==3000U) {
        int32_t move=position-origin;
        int p=poles(move);
        if(!p || abs(2*middle-move)>abs(move)/5+52) {
            return finish("POLES / SLIP",false);
        }
        if(stage==HOLD_FORWARD) {
            forward=move; origin=position; phase=REVERSE; message="POLES -"; ticks=0;
        } else {
            if(p!=poles(forward) || (move<0)==(forward<0) || 100*abs(move+forward)>3*abs(move)+1300) {
                return finish("POLES / REPEAT",false);
            }
            candidate=(uint8_t)(int8_t)(forward<0 ? -p : p) | ((uint32_t)(resistance*1000+0.5f)<<8);
            phase=L_SETTLE; message="LS HOLD"; ticks=0; middle=position;
        }
      }
    } else if(stage==L_SETTLE) {
        current=limit*0.5f;
        if(ticks==5000U) {
            for(unsigned k=0;k<20;++k) wave[k]=0;
            phase=L_PULSE; message="LS PULSE"; ticks=0; middle=position;
        }
    } else if(stage==L_PULSE) {
        /* Constant voltage for 20 samples (2 ms). Stay on one current polarity
         * to reduce inverter dead-time error. Vary amplitude to average ADC
         * quantisation; sum rising minus falling waveforms before fitting. */
        unsigned k=(ticks-1U)%20U,half=(ticks-1U)/20U;
        if(abs(position-middle)>26) return finish("LS MOVING",false);
        wave[k]+=(half&1U) ? -id : id;
        if(ticks==5120U) {
            float early=0,mid=0,late=0;
            /* Exclude preload/sampling delay and switching transients. Sum
             * three four-sample windows: their successive differences have
             * ratio exp(-R*4*T/L), independent of the unknown DC asymptote. */
            for(unsigned j=4;j<8;++j) {
                early+=wave[j]; mid+=wave[j+4]; late+=wave[j+8];
            }
            float a=(late-mid)/(mid-early);
            if(!(fabsf(mid-early)>0.5f && a>0.005f && a<0.93f)) return finish("LS SIGNAL",false);
            inductance=-resistance*0.0004f/ratio_log(a);
            if(!(inductance>=0.00005f && inductance<=0.05f)) return finish("LS RANGE",false);
            phase=FLUX_RAMP; message="FLUX RAMP"; ticks=0;
            speed=q_integral=0; d_integral=first_u;
        }
        float voltage=first_u+((half&1U) ? -1 : 1)*resistance*limit*(0.12f+0.004f*(float)((half/2U)&7U));
        JerryFOC_setAxisVoltage(MOTOR_AXIS_1,0,voltage,0,NULL,&applied);
        return true;
    } else {
        /* Forced rotating current vector, then two complete mechanical turns
         * checked separately with the measured encoder motion. No preset R/L,
         * flux, pole count or encoder electrical offset is used. */
        speed=fminf(80,speed+0.002f); /* electrical rad/s, 20 rad/s^2 */
        if(stage==FLUX_RAMP && ticks==50000U) {
            phase=FLUX_MEASURE; message="FLUX TEST"; ticks=0;
            window=(uint32_t)((float)abs((int8_t)candidate)*785.398163f+0.5f);
            origin=position; sum_i=sum_q=sum_u=sum_vq=0; flux_first=0;
        }
        if(stage==FLUX_MEASURE) {
            sum_i+=id; sum_q+=iq; sum_u+=applied; sum_vq+=q_integral;
            if(ticks==window) {
                float measured=(float)(position-origin)*(TWO_PI*10000/16384)*(float)(int8_t)candidate/(float)window;
                if(!(measured>72 && measured<88)) return finish("FLUX SLIP",false);
                float ed=(sum_u-resistance*sum_i+measured*inductance*sum_q)/(float)window;
                float eq=(sum_vq-resistance*sum_q-measured*inductance*sum_i)/(float)window;
                float flux=sqrtf(ed*ed+eq*eq)/measured;
                if(!(flux>0.0001f && flux<0.2f)) return finish("FLUX RANGE",false);
                if(flux_first) {
                    if(fabsf(flux-flux_first)>0.1f*flux_first) return finish("FLUX REPEAT",false);
                    result_l=(uint32_t)(inductance*1000000+0.5f);
                    result_flux=(uint32_t)((flux+flux_first)*500000+0.5f);
                    result=candidate; /* Commit marker is written last. */
                    return finish("DONE",true);
                }
                flux_first=flux; ticks=0; origin=position;
                sum_i=sum_q=sum_u=sum_vq=0;
            }
        }
        angle=wrap(angle+speed*0.0001f);
    }
    if(stage>=FLUX_RAMP) {
        /* Conservative 20 V/(A*s) PI integrators, just as the existing DC
         * detector. Actual applied dq values are tracked for voltage limiting. */
        float ceiling=fminf(4,MotorSampling_GetBusVoltage()*0.2f);
        q_integral=clip(q_integral-iq*0.002f,-ceiling,ceiling);
    }
    float ceiling=fminf(4,MotorSampling_GetBusVoltage()*0.2f);
    d_integral=clip(d_integral+(current-id)*0.002f,-ceiling,ceiling);
    JerryFOC_setAxisVoltage(MOTOR_AXIS_1,stage>=FLUX_RAMP ? q_integral : 0,d_integral,angle,&q_integral,&applied);
    return true;
}
