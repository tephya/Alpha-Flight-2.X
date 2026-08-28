/**
 * @file    alg_yaw_estimator.c
 * @brief   Gyro-Mag Yaw 状态估计器实现。
 * 
 * Gyro 积分提供高频航向动态，经过有效性检查的 Mag 航向
 * 用于低频修正长期积分漂移。
 */

#include "alg_yaw_estimator.h"
#include <math.h>
#include <string.h>
#include <stdint.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

#define YAW_DEG_TO_RAD  0.01745329252f      // deg 到 rad 的转换系数。
#define YAW_GYRO_SIGN   1.0f                // Gyro Z 轴与本项目 Yaw 正方向之间的符号关系。

/*
 * Mag 仅承担低频 Yaw 漂移修正，不直接覆盖 Gyro 积分状态。
 * 50 Hz Mag 更新下，0.02 对应约 1 s 量级的校正时间常数。
 */
#define YAW_MAG_CORRECTION_GAIN 0.02f
#define YAW_MAG_REFERENCE_UPDATE_GAIN 0.02f

/* Mag 基础有效性与融合门限。 */

#define YAW_MAG_MIN_FIELD_GAUSS 0.05f       // 允许的最小绝对磁场模长，Gauss。
#define YAW_MAG_MAX_FIELD_GAUSS 2.00f       // 允许的最大绝对磁场模长，Gauss。
#define YAW_MAG_FIELD_RATIO_MIN 0.65f       // 当前场强相对参考场强的最小允许比例。
#define YAW_MAG_FIELD_RATIO_MAX 1.35f       // 当前场强相对参考场强的最大允许比例。
#define YAW_MAG_INNOVATION_GATE_RAD (30.0f * YAW_DEG_TO_RAD)    // Mag 新息绝对值门限，rad。

/*
 * Disarmed Mag Reacquire 条件。
 * 必须连续收到航向与场强均稳定的 Mag 样本，才允许重新建立绝对 Yaw 基准。
 */
#define YAW_MAG_REACQUIRE_REQUIRED_SAMPLES 50U  // Reacquire 所需连续稳定样本数，约 1s @ 50Hz。
#define YAW_MAG_REACQUIRE_MAX_HEADING_STEP_RAD \
    (3.0f * YAW_DEG_TO_RAD)     // 相邻候选 Mag 航向允许的最大变化量，rad。

/*
 * Reacquire 阶段不再依赖可能已经失效的旧 Field Reference，
 * 而通过相邻样本场强比例判断当前磁场环境是否持续稳定。
 */
#define YAW_MAG_REACQUIRE_FIELD_STEP_RATIO_MIN 0.90f
#define YAW_MAG_REACQUIRE_FIELD_STEP_RATIO_MAX 1.10f

/* Gyro Yaw 积分允许的控制周期范围，s。 */
#define YAW_ESTIMATOR_DT_MIN_S 0.0005f
#define YAW_ESTIMATOR_DT_MAX_S 0.0050f

/**
 * @brief   Yaw Estimator 内部状态。
 */
typedef struct
{
    YawEstimatorDiagnostics_t diag;

    float field_reference_gauss;    /**< 当前用于场强比例门限的参考磁场模长，Gauss。 */

    float reacquire_last_yaw_rad;       /**< Reacquire 候选序列上一帧 Mag Yaw，rad。 */
    float reacquire_last_field_gauss;   /**< Reacquire 候选序列上一帧磁场模长，Gauss。 */
    uint16_t reacquire_stable_count;    /**< 当前连续稳定 Mag 样本数；0表示无候选序列。 */
} YawEstimatorState_t;

static YawEstimatorState_t s_yaw;

/* =========================================================================
 * 内部辅助函数
 * ========================================================================= */

/* 将角度归一化到 [-pi, pi]。 */
static float YawEstimator_WrapPi(float angle_rad)
{
    while(angle_rad > M_PI)
        angle_rad -= 2.0f * M_PI;
    
    while(angle_rad < -M_PI)
        angle_rad += 2.0f * M_PI;

    return angle_rad;
}

/*
 * 检查浮点值是否处于可接受的有限范围。
 * value == value 用于排除 NaN，绝对范围同时排除 Inf 和明显损坏值。
 */
static bool YawEstimator_FloatIsFinite(float value)
{
    return value == value &&
           value > -10000.0f &&
           value < 10000.0f;
}

/*
 * 清空 Disarmed 状态下用于重新接纳 Mag 的连续稳定样本跟踪状态。
 * 下一个有效 Mag 样本将作为新候选序列的第一帧。
 */
static void YawEstimator_ResetMagReacquireTracker(void)
{
    s_yaw.reacquire_last_yaw_rad = 0.0f;
    s_yaw.reacquire_last_field_gauss = 0.0f;
    s_yaw.reacquire_stable_count = 0U;
}

/*
 * 在 Disarmed 状态下确认异常后的 Mag 是否已经恢复稳定。
 * 
 * 正常 Field Ratio / Innovation 门限已经拒绝当前 Mag 时才进入该路径。
 * Armed 状态下禁止通过 Reacquire 直接改变绝对 Yaw 基准。
 */
static bool YawEstimator_TryReacquireMag(float mag_yaw_rad,
                                         float field_norm_gauss,
                                         bool is_disarmed)
{
    if(!is_disarmed)
    {
        YawEstimator_ResetMagReacquireTracker();
        return false;
    }
    
    // 第一帧有效 Mag 仅作为新候选稳定序列的起点。
    if(s_yaw.reacquire_stable_count == 0U)
    {
        s_yaw.reacquire_last_yaw_rad = mag_yaw_rad;
        s_yaw.reacquire_last_field_gauss = field_norm_gauss;
        s_yaw.reacquire_stable_count = 1U;
        return false;
    }

    const float heading_step_rad = fabsf(YawEstimator_WrapPi(
        mag_yaw_rad - s_yaw.reacquire_last_yaw_rad));

    const float field_step_ratio = 
        field_norm_gauss / s_yaw.reacquire_last_field_gauss;

    /*
     * 相邻航向或场强发生明显跳变时，认为稳定序列中断。
     * 当前样本仍作为下一段候选序列的第一帧，避免额外丢失一次观测。
     */
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

        if (s_yaw.reacquire_stable_count < YAW_MAG_REACQUIRE_REQUIRED_SAMPLES)
        {
            return false;
        }
    }

    /*
     * 连续稳定条件满足后，直接以当前 Mag 航向重新建立绝对 Yaw，
     * 同时以当前场强建立新的 Field Reference。
     */
    s_yaw.diag.yaw_rad = mag_yaw_rad;
    s_yaw.diag.mag_yaw_rad = mag_yaw_rad;
    s_yaw.diag.mag_innovation_rad = 0.0f;
    s_yaw.diag.mag_field_norm_gauss = field_norm_gauss;
    s_yaw.diag.mag_accepted = true;

    s_yaw.diag.mag_field_ratio = 1.0f;
    s_yaw.diag.mag_reject_reason = YAW_MAG_REJECT_NONE;

    s_yaw.field_reference_gauss = field_norm_gauss;

    YawEstimator_ResetMagReacquireTracker();
    return true;
}

/*
 * 校验 Mag 样本并计算倾斜补偿后的航向与磁场模长。
 * 
 * 本函数只处理单帧几何计算和绝对场强有效性；
 * Field Ratio 与 Innovation 门限由上层融合逻辑判断。
 */
static bool YawEstimator_CalculateMagHeading(const MagData_t *mag,
                                            float roll_rad,
                                            float pitch_rad,
                                            float *heading_rad,
                                            float *field_norm_gauss,
                                            uint8_t *reject_reason)
{
    if(reject_reason == NULL)
        return false;

    *reject_reason = YAW_MAG_REJECT_NONE;

    if(mag == NULL || heading_rad == NULL || field_norm_gauss == NULL || mag->ovfl)
    {
        *reject_reason |= YAW_MAG_REJECT_INVALID_SAMPLE;
        return false;
    }

    if (!YawEstimator_FloatIsFinite(mag->MX) ||
        !YawEstimator_FloatIsFinite(mag->MY) ||
        !YawEstimator_FloatIsFinite(mag->MZ))
    {
        *reject_reason |= YAW_MAG_REJECT_INVALID_SAMPLE;
        return false;
    }

    const float field_norm = sqrtf(mag->MX * mag->MX +
                                   mag->MY * mag->MY +
                                   mag->MZ * mag->MZ);

    // 即使绝对场强门限失败，也保留原始模长供 Diagnostics 使用。
    *field_norm_gauss = field_norm;

    if(field_norm < YAW_MAG_MIN_FIELD_GAUSS ||
        field_norm > YAW_MAG_MAX_FIELD_GAUSS)
    {
        *reject_reason |= YAW_MAG_REJECT_ABSOLUTE_FIELD;
        return false;
    }

    const float sin_roll = sinf(roll_rad);
    const float cos_roll = cosf(roll_rad);
    const float sin_pitch = sinf(pitch_rad);
    const float cos_pitch = cosf(pitch_rad);

    /*
     * 将倾斜状态下的磁场向量补偿到水平面：
     * 
     * M_horizontal = Ry(Pitch) * Rx(Roll) * M
     */
    const float my_horizontal = cos_roll * mag->MY - sin_roll * mag->MZ;
    const float mz_after_roll = sin_roll * mag->MY + cos_roll * mag->MZ;
    const float mx_horizontal = cos_pitch * mag->MX + sin_pitch * mz_after_roll;

    // 按当前 Yaw 坐标方向约定由水平磁场计算绝对航向。
    const float heading = atan2f(my_horizontal, mx_horizontal);

    if(!YawEstimator_FloatIsFinite(heading))
    {
        *reject_reason |= YAW_MAG_REJECT_INVALID_SAMPLE;
        return false;
    }

    *heading_rad = heading;
    return true;
}

/* =========================================================================
 * 航向估计（Yaw Estimator）
 * ========================================================================= */

 void YawEstimator_Init(float field_reference_gauss)
{
    memset(&s_yaw, 0, sizeof(s_yaw));

    /*
     * 有限使用 Hard/Soft-Iron 校准得到的参考场强。
     * 配置无效时保持为零，后续由首次有效 Mag 样本建立参考值。
     */
    if(YawEstimator_FloatIsFinite(field_reference_gauss) &&
        field_reference_gauss > YAW_MAG_MIN_FIELD_GAUSS &&
        field_reference_gauss < YAW_MAG_MAX_FIELD_GAUSS)
    {
        s_yaw.field_reference_gauss = field_reference_gauss;
    }
}

void YawEstimator_UpdateGyro(float gz_dps, float dt_s)
{
    // 未建立绝对 Yaw 前不单独使用 Gyro 启动航向状态。
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
    float mag_yaw_rad = 0.0f;
    float field_norm_gauss = 0.0f;
    uint8_t reject_reason = YAW_MAG_REJECT_NONE;

    // 每帧开始时先清除上一帧的瞬时 Diagnostics 状态。
    s_yaw.diag.mag_accepted = false;
    s_yaw.diag.mag_field_ratio = 0.0f;
    s_yaw.diag.mag_reject_reason = YAW_MAG_REJECT_NONE;

    if(!YawEstimator_CalculateMagHeading(mag,
                                        roll_rad,
                                        pitch_rad,
                                        &mag_yaw_rad, &field_norm_gauss,
                                        &reject_reason))
    {
        s_yaw.diag.mag_field_norm_gauss = field_norm_gauss;
        s_yaw.diag.mag_reject_reason = reject_reason;

        // 无效样本会中断“连续稳定”的 Reacquire 确认。
        YawEstimator_ResetMagReacquireTracker();
        return false;
    }

    s_yaw.diag.mag_yaw_rad = mag_yaw_rad;
    s_yaw.diag.mag_field_norm_gauss = field_norm_gauss;

    /*
     * 首次有效 Mag 样本直接建立绝对 Yaw。
     * 此后高频状态主要由 Gyro 推进，再由 Mag 进行低频修正。
     */
    if(!s_yaw.diag.initialized)
    {
        s_yaw.diag.yaw_rad = mag_yaw_rad;
        s_yaw.diag.mag_innovation_rad = 0.0f;
        s_yaw.diag.mag_reject_reason = YAW_MAG_REJECT_NONE;

        /*
         * 校准 Field Reference 有效时保留配置值；
         * 仅在配置无效时使用首帧磁场模长建立参考。
         */
        if(s_yaw.field_reference_gauss <= YAW_MAG_MIN_FIELD_GAUSS ||
            s_yaw.field_reference_gauss >= YAW_MAG_MAX_FIELD_GAUSS)
        {
            s_yaw.field_reference_gauss = field_norm_gauss;
        }

        s_yaw.diag.mag_field_ratio =
            field_norm_gauss / s_yaw.field_reference_gauss;

        s_yaw.diag.initialized = true;
        s_yaw.diag.mag_accepted = true;

        YawEstimator_ResetMagReacquireTracker();
        return true;
    }

    // 参考值异常时以当前有效 Mag 模长重新建立最基本的场强参考。
    if(s_yaw.field_reference_gauss <= YAW_MAG_MIN_FIELD_GAUSS)
        s_yaw.field_reference_gauss = field_norm_gauss;

    const float field_ratio = field_norm_gauss / s_yaw.field_reference_gauss;

    /*
     * Mag Innovation 表示 Mag 航向观测与当前 Gyro-Mag 融合 Yaw
     * 之间的最短角度偏差。
     */
    const float innovation_rad = YawEstimator_WrapPi(
        mag_yaw_rad - s_yaw.diag.yaw_rad);

    s_yaw.diag.mag_field_ratio = field_ratio;
    s_yaw.diag.mag_innovation_rad = innovation_rad;

    /*
     * Field Ratio 用于检测相对参考地磁场的幅值异常，
     * 可识别部分局部磁干扰或校准失效情况。
     */
    const bool field_rejected =
        field_ratio < YAW_MAG_FIELD_RATIO_MIN ||
        field_ratio > YAW_MAG_FIELD_RATIO_MAX;

    if(field_rejected)
    {
        s_yaw.diag.mag_reject_reason |= YAW_MAG_REJECT_FIELD_RATIO;
    }

    /*
     * Innovation 过大表示 Mag 航向与当前融合状态严重不一致，
     * 不允许该单帧观测直接拉动 Yaw。
     */
    const bool innovation_rejected = 
        fabsf(innovation_rad) > YAW_MAG_INNOVATION_GATE_RAD;

    if(innovation_rejected)
    {
        s_yaw.diag.mag_reject_reason |= YAW_MAG_REJECT_INNOVATION;
    }

    if(s_yaw.diag.mag_reject_reason != YAW_MAG_REJECT_NONE)
    {
        /*
         * Armed 时持续拒绝异常 Mag；
         * Disarmed 时允许通过连续稳定样本确认后重新建立 Yaw 基准。
         */
        return YawEstimator_TryReacquireMag(
            mag_yaw_rad,
            field_norm_gauss,
            is_disarmed);
    }

    /*
     * Mag 正常通过门限后仅执行低增益校正，
     * 保留 Gyro 积分提供的高频动态响应。
     */
    s_yaw.diag.yaw_rad = YawEstimator_WrapPi(
        s_yaw.diag.yaw_rad + YAW_MAG_CORRECTION_GAIN * innovation_rad);
    s_yaw.diag.mag_accepted = true;
    s_yaw.diag.mag_reject_reason = YAW_MAG_REJECT_NONE;

    // 正常融合恢复后，旧 Reacquire 候选序列不再有效。
    YawEstimator_ResetMagReacquireTracker();

    /*
     * Disarmed 时认为外部磁干扰和机体动态较弱，
     * 因此允许参考场强缓慢跟踪长期环境变化。
     */
    if(is_disarmed)
    {
        s_yaw.field_reference_gauss +=
            YAW_MAG_REFERENCE_UPDATE_GAIN *
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
    if(out == NULL)
        return;

    *out = s_yaw.diag;

    // Field Reference 属于内部持久状态，复制时补入公开 Diagnostics。
    out->mag_field_reference_gauss = s_yaw.field_reference_gauss;
}
