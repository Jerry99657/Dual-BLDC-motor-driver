/* ST ARM binary boundary only. This is a deterministic test signal source,
 * not a replacement observer and not a physical motor model. */
#include "hso.h"
#include "impedcorr.h"
static HSO_Params binary_params;
static IMPEDCORR_Params binary_ip;
static float binary_angle,binary_hz,binary_flux=0.012f,binary_jump;
static float binary_d_ripple,binary_phase;
static float binary_frequency_ripple,binary_frequency_bias,binary_phase_rate_hz;
static bool binary_emf_spikes;
static bool binary_frequency_square,binary_emf_alternating;
static bool binary_flux_vary;
static unsigned binary_emf_samples;
static bool binary_track,binary_emf=true,binary_zero_current,binary_offsets;
static unsigned binary_configurations;
static Voltages_Uab_t binary_voltage_in;
static int32_t binary_q(float x) { return (int32_t)(x*1073741824.0f); }
void HSO_setParams(HSO_Handle h,const HSO_Params *p) { (void)h; binary_params=*p; ++binary_configurations; }
void HSO_setFlag_TrackAngle(HSO_Handle h,bool x) { (void)h; binary_track=x; }
void HSO_setFlag_EnableOffsetUpdate(HSO_Handle h,bool x) { (void)h; binary_offsets=x; }
Vector_ab_t HSO_getOffset_ab_pu(const HSO_Handle h) { (void)h;return (Vector_ab_t){0}; }
fixp30_t HSO_getOffsetLimit(const HSO_Handle h) { (void)h;return binary_q(0.05883f/72.6f); }
void HSO_setAngle_pu(HSO_Handle h,fixp30_t a) { (void)h; binary_angle=(float)a/1073741824.0f*6.28318530718f; }
void HSO_run(HSO_Handle h,Voltages_Uab_t *e,fixp30_t c) {
    (void)h;(void)e;assert(c==0);
    if (binary_track) binary_angle+=(binary_hz+binary_phase_rate_hz)*6.28318530718f*0.0001f;
    binary_angle=fmodf(binary_angle+6.28318530718f,6.28318530718f);
    host_dwt.CYCCNT+=1000;
}
fixp30_t HSO_getAngle_pu(const HSO_Handle h) {
    (void)h;float ripple=binary_frequency_ripple/50*sinf((float)binary_emf_samples*0.0314159265359f);
    return binary_q((binary_angle+binary_jump+ripple)/6.28318530718f);
}
fixp30_t HSO_getEmfSpeed_pu(const HSO_Handle h) {
    (void)h;return binary_q((binary_hz+binary_frequency_bias+binary_phase_rate_hz+
        (binary_frequency_square ? (binary_emf_samples%2U ? 3 : -3) :
        binary_frequency_ripple*cosf((float)binary_emf_samples*0.0314159265359f)))/1000);
}
fixp30_t HSO_getSpeedLP_pu(const HSO_Handle h) { (void)h;return binary_q((binary_hz+binary_frequency_bias+binary_phase_rate_hz)/1000); }
fixp30_t HSO_getFluxAmpl_Wb(const HSO_Handle h) { (void)h; return binary_q(binary_params.Flux_Wb); }
IMPEDCORR_Handle IMPEDCORR_init(void *m,size_t n) { assert(n>=sizeof(IMPEDCORR_Obj)); return m; }
void IMPEDCORR_setParams(IMPEDCORR_Handle h,const IMPEDCORR_Params *p) { (void)h; binary_ip=*p; }
void IMPEDCORR_setRs_si(IMPEDCORR_Handle h,float x) { (void)h; assert(x>0); }
void IMPEDCORR_setLs_si(IMPEDCORR_Handle h,float x) { (void)h; assert(x>0); }
Voltages_Uab_t IMPEDCORR_run(IMPEDCORR_Handle h,const Currents_Iab_t *i,const Voltages_Uab_t *v,fixp30_t w) {
    (void)h;(void)i;binary_voltage_in=*v;
    /* Fixed-frequency source for the capture-ripple test after takeover;
     * do not feed an injected derivative ripple recursively into this source. */
    if (StHso_IsStarting() || binary_frequency_ripple==0) binary_hz=(float)w/1073741824.0f*1000;
    float angle=StHso_ControlAngle(),emf_angle=angle+binary_phase;
    ++binary_emf_samples;
    float variation=binary_flux_vary ? ((binary_emf_samples/3000U)%2U ? 1.5f : 0.5f) : 1;
    if(binary_emf_spikes && binary_emf_samples%40U==0) variation*=2;
    if(binary_emf_alternating) variation=binary_emf_samples%2U ? 0.1f : 2;
    float amplitude=binary_emf ? binary_flux*binary_hz*6.28318530718f/72.6f*variation : 0;
    float ripple=amplitude*binary_d_ripple*(binary_emf_samples%2U ? 1 : -1);
    return (Voltages_Uab_t){binary_q(-amplitude*sinf(emf_angle)+ripple*cosf(angle)),
                           binary_q(amplitude*cosf(emf_angle)+ripple*sinf(angle))};
}
