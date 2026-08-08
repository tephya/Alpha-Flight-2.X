#include "alg_yaw_estimator.h"
#include <math.h>
#include <string.h>
#include <stdint.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

#define YAW_DEG_TO_RAD  0.01745329252f
#define YAW_GYRO_SIGN   1.0f

/* Mag只承担低频漂移修正，单次不直接覆盖Yaw。
 * 50Hz下0.02对应约1s量级的校正时间常数 */
#define YAW_MAG_CORRECTION_GAIN 0.02f
#define YAW_MAG_REFFERENCE_UPDATE_GAIN 0.02f

#define YAW_MAG_MIN_FIELD_GAUSS 0.05f
#define YAW_MAG_MAX_FIELD_GAUSS 2.00f
#define YAW_MAG_FILED_RATIO_MIN 0.65f
#define YAW_MAG_FILED_RATIO_MAX 1.35f
#define YAW_MAG_INNOVATION_GATE_RAD (30.0f * YAW_DEG_TO_RAD)

#define YAW_MAG_REACQUIRE_REQUIRED_SAMPLES 50U  // Disarmed重新捕获需要连续50个稳定Mag样本，约为1s(@ 50Hz)
#define YAW_MAG_REACQUIRE_MAX_HEADING_STEP_RAD (3.0f * YAW_DEG_TO_RAD) // 相邻Mag航向变化不能超过3°

/* 重新捕获期间不再使用旧field_reference判断，
 * 而是检查相邻样本磁场模长是否稳定。 */
#define YAW_MAG_REACQUIRE_FIELD_STEP_RATIO_MIN 0.90f
#define YAW_MAG_REACQUIRE_FIELD_STEP_RATIO_MAX 1.10f

#define YAW_ESTIMATOR_DT_MIN_S 0.0005f
#define YAW_ESTIMATOR_DT_MAX_S 0.0050f

typedef struct
{
    YawEstimatorDiagnostics_t diag;
    float field_reference_gauss;

    /* Disarmed重新捕获状态 */
    float reacquire_last_yaw_rad;
    float reacquire_last_field_gauss;
    uint16_t reacquire_stable_count;    // count为0表示当前没有正在确认的Mag序列
} YawEstimatorState_t;

static YawEstimatorState_t s_yaw;

static float YawEstimator_WrapPi(float angle_rad)
{
    while(angle_rad > M_PI)
        angle_rad -= 2.0f * M_PI;
    
    while(angle_rad < -M_PI)
        angle_rad += 2.0f * M_PI;

    return angle_rad;
}

static bool YawEstimator_FloatIsFinite(float value)
{
    /* NaN与自身不想等；绝对值门限同时排除Inf和明显损坏值 */
    return value == value && value > -10000.0f && value < 10000.0f;
}

static void YawEstimator_ResetMagReacquire(void)
{
    s_yaw.reacquire_last_yaw_rad = 0.0f;
    s_yaw.reacquire_last_field_gauss = 0.0f;
    s_yaw.reacquire_stable_count = 0U;
}

/**
 * @brief   在Disarmed状态下确认Mag已经连续稳定，并重新建立Yaw基准。
 * 
 * @note    该函数只会在正常Innovation/磁场参考门限拒绝Mag后调用。
 *          Armed时绝不允许直接改变Yaw基准。
 */
static bool YawEstimator_TryReacquireMag(float mag_yaw_rad,
                                            float field_norm_gauss,
                                            bool is_disarmed)
{
    if(!is_disarmed)
    {
        YawEstimator_ResetMagReacquire();
        return false;
    }
    /* 当前样本作为新候选序列的第一个样本。 */
    if(s_yaw.reacquire_stable_count == 0U)
    {
        s_yaw.reacquire_last_yaw_rad = mag_yaw_rad;
        s_yaw.reacquire_last_field_gauss = field_norm_gauss;
        s_yaw.reacquire_stable_count = 1U;
        return false;
    }

    const float heading_step_rad = fabsf(YawEstimator_WrapPi(
        mag_yaw_rad - s_yaw.reacquire_last_yaw_rad));

    const float field_step_ratio = field_norm_gauss / s_yaw.reacquire_last_field_gauss;

    /* 航向或磁场模长出现突变时，不继续累计。
     * 当前样本仍可作为下一段候选序列的起点。 */
    if(heading_step_rad > YAW_MAG_REACQUIRE_MAX_HEADING_STEP_RAD ||
        field_step_ratio < YAW_MAG_REACQUIRE_FIELD_STEP_RATIO_MIN ||
        field_step_ratio > YAW_MAG_REACQUIRE_FIELD_STEP_RATIO_MAX)
    {
        s_yaw.reacquire_last_yaw_rad = mag_yaw_rad;
        s_yaw.reacquire_last_field_gauss = field_norm_gauss;
        s_yaw.reacquire_stable_count = 1U;
        return false;
    }

    s_yaw.reacquire_last_yaw_rad = mag_yaw_rad;
    s_yaw.reacquire_last_field_gauss = field_norm_gauss;

    if(s_yaw.reacquire_stable_count < YAW_MAG_REACQUIRE_REQUIRED_SAMPLES)
    {
        s_yaw.reacquire_stable_count++;
    }

    if(s_yaw.reacquire_stable_count < YAW_MAG_REACQUIRE_REQUIRED_SAMPLES)
    {
        return false;
    }

    s_yaw.diag.yaw_rad = mag_yaw_rad;
    s_yaw.diag.mag_yaw_rad = mag_yaw_rad;
    s_yaw.diag.mag_innovation_rad = 0.0f;
    s_yaw.diag.mag_field_norm_gauss = field_norm_gauss;
    s_yaw.diag.mag_accepted = true;

    s_yaw.field_reference_gauss = field_norm_gauss;

    YawEstimator_ResetMagReacquire();
    return true;
}

static bool YawEstimator_CalculateMagHeading(const MagData_t *mag,
                                            float roll_rad,
                                            float pitch_rad,
                                            float *heading_rad,
                                            float *field_norm_gauss)
{
    if(mag == NULL || heading_rad == NULL || field_norm_gauss == NULL || mag->ovfl)
        return false;

    if (!YawEstimator_FloatIsFinite(mag->MX) ||
        !YawEstimator_FloatIsFinite(mag->MY) ||
        !YawEstimator_FloatIsFinite(mag->MZ))
    {
        return false;
    }

    const float field_norm = sqrtf(mag->MX * mag->MX +
                                   mag->MY * mag->MY +
                                   mag->MZ * mag->MZ);

    if(field_norm < YAW_MAG_MIN_FIELD_GAUSS ||
        field_norm > YAW_MAG_MAX_FIELD_GAUSS)
    {
        return false;
    }

    const float sin_roll = sinf(roll_rad);
    const float cos_roll = cosf(roll_rad);
    const float sin_pitch = sinf(pitch_rad);
    const float cos_pitch = cosf(pitch_rad);

    /* 与原Attitude_CptYaw保持同一坐标约定：
     * Mh = Ry(Pitch) * Rx(Roll) * M */
    const float my_horizontal = cos_roll * mag->MY - sin_roll * mag->MZ;
    const float mz_after_roll = sin_roll * mag->MY + cos_roll * mag->MZ;
    const float mx_horizontal = cos_pitch * mag->MX + sin_pitch * mz_after_roll;
    /* 与当前Gyro Yaw正方向保持一致 */
    const float heading = atan2f(my_horizontal, mx_horizontal);

    if(!YawEstimator_FloatIsFinite(heading))
        return false;

    *heading_rad = heading;
    *field_norm_gauss = field_norm;
    return true;
}

void YawEstimator_Init(void)
{
    memset(&s_yaw, 0, sizeof(s_yaw));
}

void YawEstimator_UpdateGyro(float gz_dps, float dt_s)
{
    if(!s_yaw.diag.initialized ||
        !YawEstimator_FloatIsFinite(gz_dps) ||
        dt_s < YAW_ESTIMATOR_DT_MIN_S ||
        dt_s > YAW_ESTIMATOR_DT_MAX_S)
    {
        return;
    }

    s_yaw.diag.yaw_rad = YawEstimator_WrapPi(
        s_yaw.diag.yaw_rad +
        YAW_GYRO_SIGN * gz_dps * YAW_DEG_TO_RAD * dt_s);
}

bool YawEstimator_CorrectMag(const MagData_t *mag,
                             float roll_rad,
                             float pitch_rad,
                             bool is_disarmed)
{
    float mag_yaw_rad;
    float field_norm_gauss;

    s_yaw.diag.mag_accepted = false;
    
    if(!YawEstimator_CalculateMagHeading(mag,
                                        roll_rad,
                                        pitch_rad,
                                        &mag_yaw_rad, &field_norm_gauss))
    {
        /* 无效样本会中断“连续稳定”的重新捕获确认 */
        YawEstimator_ResetMagReacquire();
        return false;
    }

    s_yaw.diag.mag_yaw_rad = mag_yaw_rad;
    s_yaw.diag.mag_field_norm_gauss = field_norm_gauss;

    /* 首次有效Mag样本建立初始绝对航向，
     * 此路径只发生在YawEstimator_Init之后 */
    if(!s_yaw.diag.initialized)
    {
        s_yaw.diag.yaw_rad = mag_yaw_rad;
        s_yaw.diag.mag_innovation_rad = 0.0f;
        s_yaw.field_reference_gauss = field_norm_gauss;
        s_yaw.diag.initialized = true;
        s_yaw.diag.mag_accepted = true;

        YawEstimator_ResetMagReacquire();
        return true;
    }

    if(s_yaw.field_reference_gauss <= YAW_MAG_MIN_FIELD_GAUSS)
        s_yaw.field_reference_gauss = field_norm_gauss;

    const float field_ratio = field_norm_gauss / s_yaw.field_reference_gauss;
    const float innovation_rad = YawEstimator_WrapPi(
        mag_yaw_rad - s_yaw.diag.yaw_rad);

    s_yaw.diag.mag_innovation_rad = innovation_rad;

    const bool field_rejected =
        field_ratio < YAW_MAG_FILED_RATIO_MIN ||
        field_ratio > YAW_MAG_FILED_RATIO_MAX;

    const bool innovation_rejected = fabsf(innovation_rad) > YAW_MAG_INNOVATION_GATE_RAD;

    if(field_rejected || innovation_rejected)
    {
        /* Armed时继续拒绝异常Mag，
         * Disarmed时则检测Mag是否已连续稳定，满足条件后重新捕获 */
        return YawEstimator_TryReacquireMag(
            mag_yaw_rad,
            field_norm_gauss,
            is_disarmed);
    }

    /* Mag正常通过门限，执行低增益慢校正 */
    s_yaw.diag.yaw_rad = YawEstimator_WrapPi(
        s_yaw.diag.yaw_rad + YAW_MAG_CORRECTION_GAIN * innovation_rad);
    s_yaw.diag.mag_accepted = true;

    /* 正常融合已经恢复，不再需要重新捕获候选序列 */
    YawEstimator_ResetMagReacquire();

    /* 在Disarmed时缓慢更新参考磁场强度 */
    if(is_disarmed)
    {
        s_yaw.field_reference_gauss +=
            YAW_MAG_REFFERENCE_UPDATE_GAIN *
            (field_norm_gauss - s_yaw.field_reference_gauss);
    }

    return true;
}

float YawEstimator_GetYawRad(void)
{
    return s_yaw.diag.yaw_rad;
}

bool YawEstimator_IsInitialized(void)
{
    return s_yaw.diag.initialized;
}

void YawEstimator_CopyDiagnostics(YawEstimatorDiagnostics_t *out)
{
    if(out != NULL)
        *out = s_yaw.diag;
}
