#include "angle_src_encoder.h"
#include "MT6701.h"
#include "encoder_estimator.h"
#include "motor_math.h"

/* ========== 编码器缓存：仅 AngleSrc_Encoder_Update() 修改 ========== */
static EncoderEstimator encoder;
static float zero_electric_angle = 0.0f;
static uint8_t encoder_aligned = 0U;
static uint8_t encoder_initialized = 0U;
static int pp = 7;             /* 极对数 */
static int DIR = 1;            /* 方向：如果转向/电角度反了，改成 -1 */

static float _normalizeAngle(float angle)
{
    return MotorMath_Wrap(angle);
}

static float angle_meas_electrical(void)
{
    /* Angle 由 SPI DMA 回调以单个 32 位对齐写入；本地快照避免同一次
     * 计算中跨越两个样本。getter 本身不再修改任何跟踪状态。 */
    float sensor_angle = Angle;
    return _normalizeAngle((float)(DIR * pp) * sensor_angle - zero_electric_angle);
}

static float angle_meas_mechanical(void)
{
    return encoder.position;
}
float AngleSrc_Encoder_At(float speed,uint32_t cycles)
{
    float angle;uint32_t at;
    if (!MT6701_GetTimedAngle(0,&angle,&at)) return angle_meas_electrical();
    angle=(float)(DIR*pp)*angle-zero_electric_angle;
    return _normalizeAngle(MotorMath_PredictAngle(angle,(float)pp*speed,(int32_t)(cycles-at)));
}

static float angle_meas_velocity(void)
{
    return encoder.velocity;
}

static AngleSource encoder_src = {
    .getElectricalAngle = angle_meas_electrical,
    .getMechanicalAngle = angle_meas_mechanical,
    .getVelocity        = angle_meas_velocity,
    .providesVelocity   = 1U,
};

AngleSource* AngleSrc_Encoder_Init(int pole_pairs)
{
    DIR=pole_pairs<0 ? -1 : 1;
    pp=pole_pairs*DIR;
    if (!pp) pp=1;
    EncoderEstimator_Reset(&encoder,Angle,MT6701_GetSampleCounter(),false);
    zero_electric_angle = 0.0f;
    encoder_aligned = 0U;
    encoder_initialized = 1U;
    return &encoder_src;
}

void AngleSrc_Encoder_Align(float electrical_angle_rad)
{
    float sensor_angle = Angle;

    /* 记录当前传感器电角度作为零点偏移 */
    zero_electric_angle = _normalizeAngle(
        (float)(DIR * pp) * sensor_angle - electrical_angle_rad);

    /* 标定完成位置定义为机械零点，并从同一份样本重新开始速度估算。 */
    EncoderEstimator_Reset(&encoder,sensor_angle,MT6701_GetSampleCounter(),true);
    encoder_initialized = 1U;
    encoder_aligned = 1U;
}

void AngleSrc_Encoder_Update(float sample_period_s)
{
    bool healthy=encoder_initialized && MT6701_IsHealthy();
    uint32_t sample_counter = MT6701_GetSampleCounter();
    float sensor_angle = Angle;
    (void)EncoderEstimator_Update(&encoder,sensor_angle,sample_counter,healthy,DIR,sample_period_s);
}

uint8_t AngleSrc_Encoder_IsAligned(void)
{
    return encoder_aligned;
}

uint8_t AngleSrc_Encoder_IsSource(const AngleSource *source)
{
    return source == &encoder_src ? 1U : 0U;
}
