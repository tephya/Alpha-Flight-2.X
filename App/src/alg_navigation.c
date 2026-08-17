#include "alg_navigation.h"
#include <stddef.h>
#include <math.h>

/* =========================================================================
 * 常量与宏定义
 * ========================================================================= */

#define NAV_GRAVITY_MPS2 9.80665f
#define NAV_RAD_TO_DEG 57.2957795f

/* -------------------------------------------------------------------------
 * IMU水平Acceleration处理
 * ------------------------------------------------------------------------- */

/*
 * 当前IMU静止时Accel测量代表机体系重力方向。
 * 按现有坐标约定：
 * linear_accel_ne = -R_body_to_ned * accel_measurement
 */
#define NAV_ACCEL_MEASUREMENT_SIGN (-1.0f)
#define NAV_ACCEL_LPF_CUTOFF_HZ 5.0f
#define NAV_ACCEL_LIMIT_MPS2 6.0f
#define NAV_ACCEL_BIAS_LIMIT_MPS2 1.5f
#define NAV_ACCEL_BIAS_TIME_CONSTANT_S 5.0f

/*
 * 飞行中Accel bias观测器参数。
 * GPS Velocity innovation用于缓慢辨识IMU积分产生的长期Acceleration误差。
 */
#define NAV_ACCEL_BIAS_GPS_CORRECTION_GAIN 0.005f
#define NAV_ACCEL_BIAS_GPS_INNOVATION_LIMIT_MPS 0.30f
#define NAV_ACCEL_BIAS_GPS_DT_MIN_S 0.08f
#define NAV_ACCEL_BIAS_GPS_DT_MAX_S 0.40f
#define NAV_ACCEL_BIAS_GPS_STEP_LIMIT_MPS2 0.01f

/* -------------------------------------------------------------------------
 * GPS采样周期与离散融合增益
 * ------------------------------------------------------------------------- */

/*
 * 下列标称增益来自原5Hz配置。
 * 0.20s只作为增益换算基准，实际融合在每条新GPS样本到达时执行。
 */
#define NAV_GPS_GAIN_REFERENCE_DT_S 0.20f
#define NAV_GAIN_DT_RATIO_MIN 0.25f
#define NAV_GAIN_DT_RATIO_MAX 3.00f

/*
 * GPS样本迟到代表观测带宽降低，不能因此获得更大的单帧状态修改权限。
 * 实际dt仍用于时效检查和innovation/dt计算。
 */
#define NAV_GPS_CORRECTION_AUTHORITY_DT_MAX_S 0.15f

/* GPS Position alpha单帧允许直接移动Estimator Position的最大二维距离。 */
#define NAV_POSITION_STATE_CORRECTION_LIMIT_M 0.12f

/* GPS Velocity修正。 */
#define NAV_GPS_CORRECTION_NOMINAL_GAIN 0.35f
#define NAV_GPS_STATE_CORRECTION_RATE_LIMIT_MPS2 2.00f
#define NAV_GPS_INNOVATION_LIMIT_MPS 5.0f
#define NAV_GPS_REACQUIRE_GAP_MS 1500U
#define NAV_GPS_TIMEOUT_MS 600U
#define NAV_ESTIMATED_SPEED_LIMIT_MPS 20.0f

/* -------------------------------------------------------------------------
 * GPS Position alpha-beta修正
 * ------------------------------------------------------------------------- */

#define NAV_POSITION_ALPHA_NOMINAL_GAIN 0.10f
#define NAV_POSITION_BETA_LOW_SPEED_NOMINAL_GAIN 0.06f
#define NAV_POSITION_BETA_RMC_AIDED_NOMINAL_GAIN 0.02f

#define NAV_POSITION_INNOVATION_LIMIT_M 5.0f
#define NAV_POSITION_SAMPLE_DT_MIN_S 0.05f
#define NAV_POSITION_SAMPLE_DT_MAX_S 0.60f
#define NAV_POSITION_SAMPLE_ELAPSED_LIMIT_S 2.0f

/* 低速只约束Velocity模长，方向继续由Position beta提供。 */
#define NAV_VELOCITY_DIRECTION_EPSILON_MPS 0.001f

/*
 * 原先每个5Hz样本最多修正0.20m/s，
 * 等价为1.00m/s²的Velocity状态修正速率。
 */
#define NAV_POSITION_VELOCITY_CORRECTION_RATE_LIMIT_MPS2 1.00f

/* -------------------------------------------------------------------------
 * Velocity Controller
 * ------------------------------------------------------------------------- */

#define VELOCITY_KP 1.80f
#define VELOCITY_KI 0.50f
#define VELOCITY_KD 0.20f

#define VELOCITY_INTEGRAL_ACCEL_LIMIT_MPS2 0.80f
#define VELOCITY_ACCEL_LIMIT_MPS2 2.00f
#define VELOCITY_ACCEL_SLEW_LIMIT_MPS3 8.00f
#define VELOCITY_ANGLE_LIMIT_DEG 12.0f

#define VELOCITY_ARW_GAIN (2.0f / VELOCITY_KP)
#define VELOCITY_INTEGRAL_UNLOAD_GAIN 2.0f

/* -------------------------------------------------------------------------
 * Position Controller
 * ------------------------------------------------------------------------- */

#define NAV_EARTH_RADIUS_M 6378137.0f
#define NAV_DEG_E7_TO_RAD 1.745329252e-9f

#define POSITION_KP 0.70f
#define POSITION_DEADBAND_M 0.10f
#define POSITION_PILOT_SPEED_LIMIT_MPS 1.50f
#define POSITION_VELOCITY_LIMIT_MPS 1.20f
#define POSITION_MAX_ERROR_M 30.0f

#define POSITION_PILOT_ACTIVE_SPEED_MPS 0.02f

/* BRAKING阶段的HOLD锚点捕获条件。 */
#define POSITION_BRAKE_CAPTURE_SPEED_MPS 0.15f
#define POSITION_BRAKE_CAPTURE_OBSERVED_SPEED_MPS 0.25f
/* 只允许在接近锚点的稳定HOLD中建立静态风扰补偿，
 * 防止把正在发展的慢漂或Position瞬态储存到Integral。 */
#define POSITION_HOLD_INTEGRAL_POSITION_ERROR_MAX_M 0.50f
#define POSITION_HOLD_INTEGRAL_EST_SPEED_MAX_MPS 0.60f
#define POSITION_HOLD_INTEGRAL_OBSERVED_SPEED_MAX_MPS 0.70f
#define POSITION_HOLD_INTEGRAL_CONFIRM_TIME_MS 750U
#define POSITION_HOLD_INTEGRAL_MIN_GPS_SAMPLES 4U

#define POSITION_BRAKE_MIN_TIME_S 0.80f
#define POSITION_BRAKE_CAPTURE_GPS_SAMPLES 5U



/* 全局状态变量 */

HorizontalEstimator_t horizontal_estimator;
VelocityController_t velocity_controller;
PositionController_t position_controller;

/* =========================================================================
 * 内部辅助数学函数
 * ========================================================================= */

/**
 * @brief   标量限幅函数
 * @param   value   输入值
 * @param   min_value   最小值
 * @param   max_value   最大值
 * @return 钳制在 [min_value, max_value]范围内的值
 */
static float Navigation_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

/**
 * @brief   2D向量限幅函数（保持向量方向不变，限制其最大模长）
 * @param x     向量X分量指针(In/Out)
 * @param y     向量Y分量指针(In/Out)
 * @param limit 最大模长限制
 */
static void Navigation_LimitVector(float *x, float *y, float limit)
{
    const float magnitude = sqrtf((*x * *x) + (*y * *y));

    if (magnitude > limit && magnitude > 0.0001f)
    {
        const float scale = limit / magnitude;
        *x *= scale;
        *y *= scale;
    }
}

/**
 * @brief   将标称5Hz下的离散一阶融合增益换算到实际GPS采样周期
 *
 * @note    保持单位时间内的滤波带宽基本不变;GPS从5Hz改到10Hz后,
 *          不能继续每帧使用相同增益,否则单位时间修正强度接近翻倍
 */
static float Navigation_GainForSampleDt(float nominal_gain, float sample_dt_s)
{
    const float dt_ratio = Navigation_Clamp(
        sample_dt_s / NAV_GPS_GAIN_REFERENCE_DT_S,
        NAV_GAIN_DT_RATIO_MIN,
        NAV_GAIN_DT_RATIO_MAX);

    return 1.0f - powf(1.0f - nominal_gain, dt_ratio);
}

/**
 * @brief   将标称5Hz alpha-beta滤波器的beta换算到实际GPS采样周期
 * 
 * @note    beta项以innovation/dt修正Velocity,因此按dt平方缩放,
 *          才能避免提高采样率后Velocity修正带宽随之放大
 */
static float Navigation_BetaForSampleDt(float nominal_beta, float sample_dt_s)
{
    const float dt_ratio = Navigation_Clamp(
        sample_dt_s / NAV_GPS_GAIN_REFERENCE_DT_S,
        NAV_GAIN_DT_RATIO_MIN,
        NAV_GAIN_DT_RATIO_MAX);

    return nominal_beta * dt_ratio * dt_ratio;
}

/**
 * @brief   将机体系水平Accel bias投影到当前导航N/E坐标系。
 *
 * @note    机体系采用Forward/Right，导航系采用North/East：
 *              N = cos(yaw) * Forward - sin(yaw) * Right
 *              E = sin(yaw) * Forward + cos(yaw) * Right
 */
static void HorizontalEstimator_ProjectAccelBiasToNe(float sin_yaw, float cos_yaw)
{
    horizontal_estimator.accel_bias_n_mps2 =
        cos_yaw * horizontal_estimator.accel_bias_forward_mps2 -
        sin_yaw * horizontal_estimator.accel_bias_right_mps2;

    horizontal_estimator.accel_bias_e_mps2 =
        sin_yaw * horizontal_estimator.accel_bias_forward_mps2 +
        cos_yaw * horizontal_estimator.accel_bias_right_mps2;
}

/**
 * @brief   刷新公开的N/E bias投影和矫正后水平加速度。
 */
static void HorizontalEstimator_RefreshAccelOutput(float sin_yaw, float cos_yaw)
{
    HorizontalEstimator_ProjectAccelBiasToNe(sin_yaw, cos_yaw);

    horizontal_estimator.accel_n_mps2 =
        horizontal_estimator.accel_lpf_n_mps2 -
        horizontal_estimator.accel_bias_n_mps2;

    horizontal_estimator.accel_e_mps2 =
        horizontal_estimator.accel_lpf_e_mps2 -
        horizontal_estimator.accel_bias_e_mps2;
}

/* =========================================================================
 * 导航估计器(Horizontal Estimator)实现
 * ========================================================================= */

void HorizontalEstimator_Init(void)
{
    HorizontalEstimator_Reset();
}

/**
 * @brief 重置水平估计器的所有状态变量
 */
void HorizontalEstimator_Reset()
{
    horizontal_estimator.velocity_n_mps = 0.0f;
    horizontal_estimator.velocity_e_mps = 0.0f;
    horizontal_estimator.accel_n_mps2 = 0.0f;
    horizontal_estimator.accel_e_mps2 = 0.0f;
    horizontal_estimator.accel_lpf_n_mps2 = 0.0f;
    horizontal_estimator.accel_lpf_e_mps2 = 0.0f;
    horizontal_estimator.accel_bias_forward_mps2 = 0.0f;
    horizontal_estimator.accel_bias_right_mps2 = 0.0f;
    horizontal_estimator.accel_bias_n_mps2 = 0.0f;
    horizontal_estimator.accel_bias_e_mps2 = 0.0f;
    horizontal_estimator.last_gps_sequence = 0U;
    horizontal_estimator.last_gps_tick_ms = 0U;
    horizontal_estimator.gps_accept_count = 0U;
    horizontal_estimator.gps_reject_count = 0U;
    horizontal_estimator.initialized = 0U;
    horizontal_estimator.gps_healthy = 0U;
    horizontal_estimator.last_gps_accepted = 0U;
    horizontal_estimator.gps_accept_streak = 0U;
    horizontal_estimator.last_rmc_vector_used = 0U;

    HorizontalEstimator_ResetPosition();
}

/**
 * @brief 估计器预测步骤(基于高频IMU数据)
 *
 * @param ax_g, ay_g, az_g 机体系三轴加速度(G)
 * @param roll_rad, pitch_rad, yaw_rad 当前机体欧拉角姿态(rad)
 * @param dt 积分时间步长(s)
 * @param learn_accel_bias 是否启用加速度计零偏在线学习
 */
void HorizontalEstimator_Predict(float ax_g,
                                 float ay_g,
                                 float az_g,
                                 float roll_rad,
                                 float pitch_rad,
                                 float yaw_rad,
                                 float dt,
                                 bool learn_accel_bias)
{
    // 防止异常时间步长导致计算发散(0.5ms ~ 5ms之间)
    if (dt < 0.0005f || dt > 0.0050f)
        return;

    const float sr = sinf(roll_rad);
    const float cr = cosf(roll_rad);
    const float sp = sinf(pitch_rad);
    const float cp = cosf(pitch_rad);
    const float sy = sinf(yaw_rad);
    const float cy = cosf(yaw_rad);

    /* NED下的R_body_to_ned前两行。静止时重力旋转只所在Down轴，
     * 因此这里的N/E分量理论上为0（仅代表载体的运动加速度）。
     * 通过旋转矩阵将机体坐标系下的加速度转换到北东地(NED)坐标系的 北(N) 和 东(E) 方向。 */
    const float measured_n_g =
        (cp * cy) * ax_g +
        (sr * sp * cy - cr * sy) * ay_g +
        (cr * sp * cy + sr * sy) * az_g;

    const float measured_e_g =
        (cp * sy) * ax_g +
        (sr * sp * sy + cr * cy) * ay_g +
        (cr * sp * sy - sr * cy) * az_g;

    // 转换单位从 G 到 m/s^2，并应用方向符号约定
    float accel_n_raw = NAV_ACCEL_MEASUREMENT_SIGN * measured_n_g * NAV_GRAVITY_MPS2;
    float accel_e_raw = NAV_ACCEL_MEASUREMENT_SIGN * measured_e_g * NAV_GRAVITY_MPS2;

    // 对单轴原始加速度进行粗略限幅，过滤掉因震动带来的离群尖峰
    accel_n_raw = Navigation_Clamp(accel_n_raw, -NAV_ACCEL_LIMIT_MPS2, NAV_ACCEL_LIMIT_MPS2);
    accel_e_raw = Navigation_Clamp(accel_e_raw, -NAV_ACCEL_LIMIT_MPS2, NAV_ACCEL_LIMIT_MPS2);

    // 一阶低通滤波器(LPF)，平滑加速度数据
    const float tau = 1.0f / (6.2831853f * NAV_ACCEL_LPF_CUTOFF_HZ); // 6.28... 为 2*PI
    const float alpha = dt / (tau + dt);

    horizontal_estimator.accel_lpf_n_mps2 +=
        alpha * (accel_n_raw - horizontal_estimator.accel_lpf_n_mps2);
    horizontal_estimator.accel_lpf_e_mps2 +=
        alpha * (accel_e_raw - horizontal_estimator.accel_lpf_e_mps2);

    if (learn_accel_bias)
    {
        const float bias_alpha =
            dt / (NAV_ACCEL_BIAS_TIME_CONSTANT_S + dt);

        const float observed_forward_mps2 =
            cy * horizontal_estimator.accel_lpf_n_mps2 +
            sy * horizontal_estimator.accel_lpf_e_mps2;

        const float observed_right_mps2 =
            -sy * horizontal_estimator.accel_lpf_n_mps2 +
            cy * horizontal_estimator.accel_lpf_e_mps2;

        horizontal_estimator.accel_bias_forward_mps2 +=
            bias_alpha *
            (observed_forward_mps2 - horizontal_estimator.accel_bias_forward_mps2);

        horizontal_estimator.accel_bias_right_mps2 +=
            bias_alpha *
            (observed_right_mps2 - horizontal_estimator.accel_bias_right_mps2);

        Navigation_LimitVector(
            &horizontal_estimator.accel_bias_forward_mps2,
            &horizontal_estimator.accel_bias_right_mps2,
            NAV_ACCEL_BIAS_LIMIT_MPS2);
    }

    HorizontalEstimator_RefreshAccelOutput(sy, cy);

#if HORIZONTAL_ESTIMATOR_IMU_PREDICTION_ENABLED
    // 若系统已通过GPS初始化，则利用加速度进行航位推算(积分)预测当前速度
    if (horizontal_estimator.initialized)
    {
        horizontal_estimator.velocity_n_mps +=
            horizontal_estimator.accel_n_mps2 * dt;
        horizontal_estimator.velocity_e_mps +=
            horizontal_estimator.accel_e_mps2 * dt;

        // 限制预测出的最大风行速度
        Navigation_LimitVector(&horizontal_estimator.velocity_n_mps,
                               &horizontal_estimator.velocity_e_mps,
                               NAV_ESTIMATED_SPEED_LIMIT_MPS);
    }
#endif

    if (horizontal_estimator.position_initialized)
    {
        /* Position与Velocity属于同一Estimator：高频预测必须使用同一份Velocity。 */
        horizontal_estimator.position_n_m +=
            horizontal_estimator.velocity_n_mps * dt;
        horizontal_estimator.position_e_m +=
            horizontal_estimator.velocity_e_mps * dt;

        horizontal_estimator.position_sample_elapsed_s += dt;
        if (horizontal_estimator.position_sample_elapsed_s >
            NAV_POSITION_SAMPLE_ELAPSED_LIMIT_S)
        {
            horizontal_estimator.position_sample_elapsed_s =
                NAV_POSITION_SAMPLE_ELAPSED_LIMIT_S;
        }
    }
}

/**
 * @brief   使用GPS速度innovation慢速校正飞行中的水平Accel bias。
 *
 * @note    Predict使用：
 *              Velocity += (accel_lpf - accel_bias) * dt
 *
 *          当Estimator速度高于GPS速度时，innovation为负，说明用于积分的
 *          Accel可能偏正，因此需要增加Accel bias。故bias修正符号为负。
 *
 *          本函数只应在Position Hold的HOLD阶段调用。外部还必须排除打杆、
 *          BRAKING和速度源切换阶段。
 */
static void HorizontalEstimator_CorrectAccelBiasFromGps(float innovation_n_mps,
                                                        float innovation_e_mps,
                                                        float gps_dt_s,
                                                        float yaw_rad)
{
    if (gps_dt_s < NAV_ACCEL_BIAS_GPS_DT_MIN_S ||
        gps_dt_s > NAV_ACCEL_BIAS_GPS_DT_MAX_S)
    {
        return;
    }

    const float innovation_mps = sqrtf(
        innovation_n_mps * innovation_n_mps +
        innovation_e_mps * innovation_e_mps);

    if (!(innovation_mps >= 0.0f &&
          innovation_mps <= NAV_ACCEL_BIAS_GPS_INNOVATION_LIMIT_MPS))
    {
        return;
    }

    /*
     * innovation / dt近似Estimator的长期加速度误差。
     * 使用很小的增益更新bias，不能直接快速追踪GPS速度噪声。
     */
    float bias_delta_n_mps2 =
        -NAV_ACCEL_BIAS_GPS_CORRECTION_GAIN *
        innovation_n_mps / gps_dt_s;

    float bias_delta_e_mps2 =
        -NAV_ACCEL_BIAS_GPS_CORRECTION_GAIN *
        innovation_e_mps / gps_dt_s;

    /* 限制单个GPS样本造成的bias改变量。 */
    Navigation_LimitVector(
        &bias_delta_n_mps2,
        &bias_delta_e_mps2,
        NAV_ACCEL_BIAS_GPS_STEP_LIMIT_MPS2);

    const float sy = sinf(yaw_rad);
    const float cy = cosf(yaw_rad);

    const float bias_delta_forward_mps2 =
        cy * bias_delta_n_mps2 + sy * bias_delta_e_mps2;
    const float bias_delta_right_mps2 =
        -sy * bias_delta_n_mps2 + cy * bias_delta_e_mps2;

    horizontal_estimator.accel_bias_forward_mps2 +=
        bias_delta_forward_mps2;
    horizontal_estimator.accel_bias_right_mps2 +=
        bias_delta_right_mps2;

    Navigation_LimitVector(
        &horizontal_estimator.accel_bias_forward_mps2,
        &horizontal_estimator.accel_bias_right_mps2,
        NAV_ACCEL_BIAS_LIMIT_MPS2);

    /* 本帧立即发布新bias对应的N/E加速度，不等下一次Predict */
    HorizontalEstimator_RefreshAccelOutput(sy, cy);
}

/**
 * @brief 估计器修正步骤 (基于低频但长期的GPS绝对测量值)
 * @return 是否成功应用了GPS修正
 */
bool HorizontalEstimator_CorrectGps(float gps_velocity_n_mps,
                                    float gps_velocity_e_mps,
                                    uint32_t gps_sequence,
                                    uint32_t gps_tick_ms,
                                    float yaw_rad,
                                    bool sample_valid,
                                    bool rmc_vector_use_allowed,
                                    bool allow_accel_bias_correction,
                                    bool allow_state_reacquire)
{
    // 利用序列号检查是否为重复的旧数据
    if (gps_sequence == 0U ||
        gps_sequence == horizontal_estimator.last_gps_sequence)
    {
        return false;
    }

    horizontal_estimator.last_gps_sequence = gps_sequence;
    horizontal_estimator.last_gps_accepted = 0U;
    horizontal_estimator.last_rmc_vector_used = 0U;

    const float rmc_speed_mps = sqrtf(
        gps_velocity_n_mps * gps_velocity_n_mps +
        gps_velocity_e_mps * gps_velocity_e_mps);

    /* 检查GPS数据的有效性。
     * 与NaN比较的结果必定为false，因此该范围判断同时拒绝了NaN和Inf等非法浮点数。 */
    if (!sample_valid || !(rmc_speed_mps >= 0.0f && rmc_speed_mps <= 30.0f))
    {
        horizontal_estimator.gps_reject_count++;
        horizontal_estimator.gps_accept_streak = 0U;
        horizontal_estimator.gps_healthy = 0U;
        return false;
    }

    const bool rmc_course_usable =
        rmc_vector_use_allowed &&
        rmc_speed_mps >= NAV_RMC_VECTOR_MIN_SPEED_MPS;

    const uint32_t gps_ms = gps_tick_ms - horizontal_estimator.last_gps_tick_ms;

    const bool initial_alignment =
        !horizontal_estimator.initialized ||
        horizontal_estimator.last_gps_tick_ms == 0U ||
        gps_ms > NAV_GPS_REACQUIRE_GAP_MS;

    // 检查是否需要重新初始化融合滤波器
    if (initial_alignment)
    {
        if (horizontal_estimator.initialized &&
            !allow_state_reacquire)
        {
            horizontal_estimator.gps_reject_count++;
            horizontal_estimator.gps_accept_streak = 0U;
            horizontal_estimator.gps_healthy = 0U;
            return false;
        }

        if (rmc_course_usable)
        {
            horizontal_estimator.velocity_n_mps = gps_velocity_n_mps;
            horizontal_estimator.velocity_e_mps = gps_velocity_e_mps;
            horizontal_estimator.last_rmc_vector_used = 1U;
        }
        else
        {
            horizontal_estimator.velocity_n_mps = 0.0f;
            horizontal_estimator.velocity_e_mps = 0.0f;
        }

        horizontal_estimator.initialized = 1U;
        horizontal_estimator.gps_accept_streak = 1U;
    }
    else
    {
        const float gps_dt_s = (float)gps_ms * 0.001f;

        const float correction_authority_dt_s =
            Navigation_Clamp(
                gps_dt_s,
                NAV_POSITION_SAMPLE_DT_MIN_S,
                NAV_GPS_CORRECTION_AUTHORITY_DT_MAX_S);

        const float correction_gain = Navigation_GainForSampleDt(
            NAV_GPS_CORRECTION_NOMINAL_GAIN,
            correction_authority_dt_s);

        const float max_state_correction_mps =
            NAV_GPS_STATE_CORRECTION_RATE_LIMIT_MPS2 *
            correction_authority_dt_s;

        if(rmc_course_usable)
        {
            const float innovation_n_mps =
                gps_velocity_n_mps - horizontal_estimator.velocity_n_mps;
            const float innovation_e_mps =
                gps_velocity_e_mps - horizontal_estimator.velocity_e_mps;
            const float innovation_mps = sqrtf(
                innovation_n_mps * innovation_n_mps +
                innovation_e_mps * innovation_e_mps);

            const bool innovation_valid =
                innovation_mps >= 0.0f &&
                innovation_mps <= NAV_GPS_INNOVATION_LIMIT_MPS;

            if (!innovation_valid)
            {
                if (!allow_state_reacquire)
                {
                    horizontal_estimator.gps_reject_count++;
                    horizontal_estimator.gps_accept_streak = 0U;
                    horizontal_estimator.gps_healthy = 0U;
                    return false;
                }

                horizontal_estimator.velocity_n_mps = gps_velocity_n_mps;
                horizontal_estimator.velocity_e_mps = gps_velocity_e_mps;
                horizontal_estimator.gps_accept_streak = 1U;
            }
            else
            {
                if (allow_accel_bias_correction)
                {
                    HorizontalEstimator_CorrectAccelBiasFromGps(
                        innovation_n_mps,
                        innovation_e_mps,
                        gps_dt_s,
                        yaw_rad);
                }

                float correction_n_mps =
                    correction_gain * innovation_n_mps;
                float correction_e_mps =
                    correction_gain * innovation_e_mps;

                Navigation_LimitVector(
                    &correction_n_mps,
                    &correction_e_mps,
                    max_state_correction_mps);

                horizontal_estimator.velocity_n_mps +=
                    correction_n_mps;
                horizontal_estimator.velocity_e_mps +=
                    correction_e_mps;

                if (horizontal_estimator.gps_accept_streak < UINT8_MAX)
                    horizontal_estimator.gps_accept_streak++;
            }

            horizontal_estimator.last_rmc_vector_used = 1U;
        }
        else
        {
            /*
             * RMC course在低速时不可信；刚越过低速门限但尚未通过
             * Position Window一致性确认时，同样不能使用其N/E方向。
             *
             * 这里恢复303的模长约束：RMC ground speed可以阻止IMU积分速度
             * 在低速长期漂大，但不会把不可信的course方向写入Estimator。
             */
            const float estimated_speed_mps = sqrtf(
                horizontal_estimator.velocity_n_mps *
                    horizontal_estimator.velocity_n_mps +
                horizontal_estimator.velocity_e_mps *
                    horizontal_estimator.velocity_e_mps);

            const bool estimated_speed_valid =
                estimated_speed_mps >= 0.0f &&
                estimated_speed_mps <= NAV_ESTIMATED_SPEED_LIMIT_MPS;

            const float directionless_speed_upper_bound_mps =
                fmaxf(rmc_speed_mps, NAV_RMC_VECTOR_MIN_SPEED_MPS);

            if (!estimated_speed_valid)
            {
                if (!allow_state_reacquire)
                {
                    horizontal_estimator.gps_reject_count++;
                    horizontal_estimator.gps_accept_streak = 0U;
                    horizontal_estimator.gps_healthy = 0U;
                    return false;
                }

                horizontal_estimator.velocity_n_mps = 0.0f;
                horizontal_estimator.velocity_e_mps = 0.0f;
                horizontal_estimator.gps_accept_streak = 1U;
            }
            else if (estimated_speed_mps > directionless_speed_upper_bound_mps)
            {
                const float speed_excess_mps =
                    estimated_speed_mps - directionless_speed_upper_bound_mps;

                if (speed_excess_mps > NAV_GPS_INNOVATION_LIMIT_MPS)
                {
                    if (!allow_state_reacquire)
                    {
                        horizontal_estimator.gps_reject_count++;
                        horizontal_estimator.gps_accept_streak = 0U;
                        horizontal_estimator.gps_healthy = 0U;
                        return false;
                    }

                    horizontal_estimator.velocity_n_mps = 0.0f;
                    horizontal_estimator.velocity_e_mps = 0.0f;
                    horizontal_estimator.gps_accept_streak = 1U;
                }
                else
                {
                    float speed_correction_mps =
                        correction_gain * speed_excess_mps;

                    speed_correction_mps = Navigation_Clamp(
                        speed_correction_mps,
                        0.0f,
                        max_state_correction_mps);

                    const float corrected_speed_mps =
                        estimated_speed_mps - speed_correction_mps;

                    if (estimated_speed_mps >
                        NAV_VELOCITY_DIRECTION_EPSILON_MPS)
                    {
                        const float velocity_scale =
                            corrected_speed_mps / estimated_speed_mps;

                        horizontal_estimator.velocity_n_mps *= velocity_scale;
                        horizontal_estimator.velocity_e_mps *= velocity_scale;
                    }

                    if (horizontal_estimator.gps_accept_streak < UINT8_MAX)
                        horizontal_estimator.gps_accept_streak++;
                }
            }
            else
            {
                if (horizontal_estimator.gps_accept_streak < UINT8_MAX)
                    horizontal_estimator.gps_accept_streak++;
            }
        }
    }

    horizontal_estimator.last_gps_tick_ms = gps_tick_ms;
    horizontal_estimator.gps_accept_count++;
    horizontal_estimator.gps_healthy = 1U;
    horizontal_estimator.last_gps_accepted = 1U;
    return true;
}

void HorizontalEstimator_UpdateHealth(uint32_t now_ms)
{
    if (!horizontal_estimator.initialized ||
        horizontal_estimator.last_gps_tick_ms == 0U ||
        (uint32_t)(now_ms - horizontal_estimator.last_gps_tick_ms) > NAV_GPS_TIMEOUT_MS)
    {
        horizontal_estimator.gps_healthy = 0U;
    }
}

bool HorizontalEstimator_IsHealthy(uint32_t now_ms)
{
    HorizontalEstimator_UpdateHealth(now_ms);
    return horizontal_estimator.initialized && horizontal_estimator.gps_healthy;
}

/* =========================================================================
 * 速度控制器(Velocity Controller)实现
 * ========================================================================= */

void VelocityController_Init(void)
{
    velocity_controller.kp = VELOCITY_KP;
    velocity_controller.ki = VELOCITY_KI;
    velocity_controller.kd = VELOCITY_KD;
    velocity_controller.arw_gain = VELOCITY_ARW_GAIN;
    VelocityController_Reset();
}

void VelocityController_ResetIntegral(void)
{
    velocity_controller.integral_accel_n_mps2 = 0.0f;
    velocity_controller.integral_accel_e_mps2 = 0.0f;
}

/**
 * @brief 重置控制器积分项及目标输出
 */
void VelocityController_Reset(void)
{
    VelocityController_ResetIntegral();

    velocity_controller.accel_requested_n_mps2 = 0.0f;
    velocity_controller.accel_requested_e_mps2 = 0.0f;
    velocity_controller.accel_target_n_mps2 = 0.0f;
    velocity_controller.accel_target_e_mps2 = 0.0f;
    velocity_controller.p_accel_n_mps2 = 0.0f;
    velocity_controller.p_accel_e_mps2 = 0.0f;
    velocity_controller.d_accel_n_mps2 = 0.0f;
    velocity_controller.d_accel_e_mps2 = 0.0f;
    velocity_controller.roll_target_deg = 0.0f;
    velocity_controller.pitch_target_deg = 0.0f;
    velocity_controller.output_limited = 0U;
    velocity_controller.integral_enabled = 0U;
}

/**
 * @brief   更新单轴Velocity Integral
 */
static float VelocityController_UpdateIntegralAxis(float velocity_error_mps,
                                                    float saturation_error_mps2,
                                                    float integral_accel_mps2,
                                                    VelocityIntegralMode_t integral_mode,
                                                    float dt)
{
    if(integral_mode == VELOCITY_INTEGRAL_MODE_FROZEN)
        return integral_accel_mps2;

    const bool allow_integral_learning =
        integral_mode == VELOCITY_INTEGRAL_MODE_LEARN;

    const bool error_unloads_existing_integral =
        ((integral_accel_mps2 > 0.0f) &&
         (velocity_error_mps < 0.0f)) ||
        ((integral_accel_mps2 < 0.0f) &&
         (velocity_error_mps > 0.0f));

    float effective_velocity_error_mps = 0.0f;

    if(allow_integral_learning)
    {
        effective_velocity_error_mps = velocity_error_mps;

        if(error_unloads_existing_integral)
        {
            effective_velocity_error_mps *=
                VELOCITY_INTEGRAL_UNLOAD_GAIN;
        }
    }
    else if((integral_mode == VELOCITY_INTEGRAL_MODE_UNLOAD_ONLY) &&
                error_unloads_existing_integral)
    {
        effective_velocity_error_mps =
            velocity_error_mps * VELOCITY_INTEGRAL_UNLOAD_GAIN;
    }

    const float candidate_integral_accel_mps2 =
        integral_accel_mps2 +
        velocity_controller.ki *
            (effective_velocity_error_mps +
             velocity_controller.arw_gain * saturation_error_mps2) *
            dt;

    if(allow_integral_learning)
    {
        return candidate_integral_accel_mps2;
    }

    if(integral_accel_mps2 == 0.0f)
    {
        return 0.0f;
    }

    if(((integral_accel_mps2 > 0.0f) &&
        (candidate_integral_accel_mps2 <= 0.0f)) ||
        ((integral_accel_mps2 < 0.0f) &&
        (candidate_integral_accel_mps2 >= 0.0f)))
    {
        return 0.0f;
    }

    if(fabsf(candidate_integral_accel_mps2) < 
        fabsf(integral_accel_mps2))
    {
        return candidate_integral_accel_mps2;
    }

    return integral_accel_mps2;
}

void VelocityController_Update(float velocity_target_n_mps,
                               float velocity_target_e_mps,
                               float velocity_meas_n_mps,
                               float velocity_meas_e_mps,
                               float accel_meas_n_mps2,
                               float accel_meas_e_mps2,
                               float yaw_rad,
                               VelocityIntegralMode_t integral_mode,
                               float dt)
{
    if (dt < 0.0005f || dt > 0.0050f)
        return;

    // 计算速度误差
    const float error_n_mps = velocity_target_n_mps - velocity_meas_n_mps;
    const float error_e_mps = velocity_target_e_mps - velocity_meas_e_mps;

    velocity_controller.integral_enabled =
        (integral_mode == VELOCITY_INTEGRAL_MODE_LEARN) ? 1U : 0U;

    /*
     * D-on-measurement：Velocity的导数就是Acceleration。
     * 使用Estimator已经完成重力分离、bias去除和5Hz LPF的水平Acceleration，
     * 避免对含GPS阶跃的Velocity状态再做数值微分。
     */
    velocity_controller.p_accel_n_mps2 =
        velocity_controller.kp * error_n_mps;
    velocity_controller.p_accel_e_mps2 =
        velocity_controller.kp * error_e_mps;

    velocity_controller.d_accel_n_mps2 =
        -velocity_controller.kd * accel_meas_n_mps2;
    velocity_controller.d_accel_e_mps2 =
        -velocity_controller.kd * accel_meas_e_mps2;

    velocity_controller.accel_requested_n_mps2 =
        velocity_controller.p_accel_n_mps2 +
        velocity_controller.integral_accel_n_mps2 +
        velocity_controller.d_accel_n_mps2;
    velocity_controller.accel_requested_e_mps2 =
        velocity_controller.p_accel_e_mps2 +
        velocity_controller.integral_accel_e_mps2 +
        velocity_controller.d_accel_e_mps2;

    float limited_accel_n_mps2 =
        velocity_controller.accel_requested_n_mps2;
    float limited_accel_e_mps2 =
        velocity_controller.accel_requested_e_mps2;

    Navigation_LimitVector(
        &limited_accel_n_mps2,
        &limited_accel_e_mps2,
        VELOCITY_ACCEL_LIMIT_MPS2);

    const float saturation_error_n_mps2 =
        limited_accel_n_mps2 -
        velocity_controller.accel_requested_n_mps2;
    const float saturation_error_e_mps2 =
        limited_accel_e_mps2 -
        velocity_controller.accel_requested_e_mps2;

    const bool hard_output_limited =
        fabsf(saturation_error_n_mps2) > 0.0001f ||
        fabsf(saturation_error_e_mps2) > 0.0001f;

    float accel_delta_n_mps2 =
        limited_accel_n_mps2 -
        velocity_controller.accel_target_n_mps2;
    float accel_delta_e_mps2 =
        limited_accel_e_mps2 -
        velocity_controller.accel_target_e_mps2;

    const float accel_delta_mps2 = sqrtf(
        accel_delta_n_mps2 * accel_delta_n_mps2 +
        accel_delta_e_mps2 * accel_delta_e_mps2);

    const float max_accel_delta_mps2 =
        VELOCITY_ACCEL_SLEW_LIMIT_MPS3 * dt;

    const bool slew_output_limited =
        accel_delta_mps2 > max_accel_delta_mps2;

    Navigation_LimitVector(
        &accel_delta_n_mps2,
        &accel_delta_e_mps2,
        max_accel_delta_mps2);

    velocity_controller.accel_target_n_mps2 += accel_delta_n_mps2;
    velocity_controller.accel_target_e_mps2 += accel_delta_e_mps2;

    velocity_controller.output_limited =
        (hard_output_limited || slew_output_limited) ? 1U : 0U;

    velocity_controller.integral_accel_n_mps2 =
        VelocityController_UpdateIntegralAxis(
            error_n_mps,
            saturation_error_n_mps2,
            velocity_controller.integral_accel_n_mps2,
            integral_mode,
            dt);
    velocity_controller.integral_accel_e_mps2 =
        VelocityController_UpdateIntegralAxis(
            error_e_mps,
            saturation_error_e_mps2,
            velocity_controller.integral_accel_e_mps2,
            integral_mode,
            dt);

    Navigation_LimitVector(
        &velocity_controller.integral_accel_n_mps2,
        &velocity_controller.integral_accel_e_mps2,
        VELOCITY_INTEGRAL_ACCEL_LIMIT_MPS2);

    const float sy = sinf(yaw_rad);
    const float cy = cosf(yaw_rad);

    const float accel_forward_mps2 =
        cy * velocity_controller.accel_target_n_mps2 +
        sy * velocity_controller.accel_target_e_mps2;
    const float accel_right_mps2 =
        -sy * velocity_controller.accel_target_n_mps2 +
        cy * velocity_controller.accel_target_e_mps2;

    /*
     * 限制的是水平Acceleration矢量，因此转换后不再独立裁剪
     * Roll/Pitch两轴，避免破坏原Acceleration方向。
     */
    velocity_controller.pitch_target_deg =
        -atanf(accel_forward_mps2 / NAV_GRAVITY_MPS2) * NAV_RAD_TO_DEG;
    velocity_controller.roll_target_deg =
        atanf(accel_right_mps2 / NAV_GRAVITY_MPS2) * NAV_RAD_TO_DEG;

    velocity_controller.roll_target_deg = Navigation_Clamp(
        velocity_controller.roll_target_deg,
        -VELOCITY_ANGLE_LIMIT_DEG,
        VELOCITY_ANGLE_LIMIT_DEG);
    velocity_controller.pitch_target_deg = Navigation_Clamp(
        velocity_controller.pitch_target_deg,
        -VELOCITY_ANGLE_LIMIT_DEG,
        VELOCITY_ANGLE_LIMIT_DEG);
}

/**
 * @brief 校验传入的经纬度坐标是否在合法地球标注范围内
 * @param latitude_e7 纬度坐标(° * 1e7)
 * @param longitude_e7 经度坐标(° * 1e7)
 * @return true=坐标符合[-90,90]及[-180,180]规范；false=越界非法数据
 */
static bool Navigation_CoordinateValid(int32_t latitude_e7,
                                       int32_t longitude_e7)
{
    return latitude_e7 >= -900000000 &&
           latitude_e7 <= 900000000 &&
           longitude_e7 >= -1800000000 &&
           longitude_e7 <= 1800000000;
}

void HorizontalEstimator_ResetPosition(void)
{
    horizontal_estimator.position_n_m = 0.0f;
    horizontal_estimator.position_e_m = 0.0f;
    horizontal_estimator.gps_position_n_m = 0.0f;
    horizontal_estimator.gps_position_e_m = 0.0f;

    horizontal_estimator.position_reference_lat_e7 = 0;
    horizontal_estimator.position_reference_lon_e7 = 0;
    horizontal_estimator.meter_per_lat_e7 = 0.0f;
    horizontal_estimator.meter_per_lon_e7 = 0.0f;

    horizontal_estimator.position_sample_elapsed_s = 0.0f;
    horizontal_estimator.position_velocity_correction_n_mps = 0.0f;
    horizontal_estimator.position_velocity_correction_e_mps = 0.0f;

    horizontal_estimator.last_position_sequence = 0U;
    horizontal_estimator.position_accept_count = 0U;
    horizontal_estimator.position_reject_count = 0U;
    horizontal_estimator.position_velocity_correction_count = 0U;

    horizontal_estimator.position_initialized = 0U;
    horizontal_estimator.last_position_accepted = 0U;
    horizontal_estimator.last_position_rejected = 0U;
    horizontal_estimator.last_position_velocity_correction_applied = 0U;

    horizontal_estimator.position_accept_streak = 0U;
}

bool HorizontalEstimator_SetPositionReference(int32_t latitude_e7,
                                              int32_t longitude_e7,
                                              uint32_t gps_sequence)
{
    HorizontalEstimator_ResetPosition();

    if (gps_sequence == 0U ||
        !Navigation_CoordinateValid(latitude_e7, longitude_e7))
    {
        return false;
    }

    const float reference_lat_rad =
        (float)latitude_e7 * NAV_DEG_E7_TO_RAD;

    horizontal_estimator.position_reference_lat_e7 = latitude_e7;
    horizontal_estimator.position_reference_lon_e7 = longitude_e7;
    horizontal_estimator.meter_per_lat_e7 =
        NAV_EARTH_RADIUS_M * NAV_DEG_E7_TO_RAD;
    horizontal_estimator.meter_per_lon_e7 =
        horizontal_estimator.meter_per_lat_e7 * cosf(reference_lat_rad);

    horizontal_estimator.last_position_sequence = gps_sequence;
    horizontal_estimator.position_initialized = 1U;
    return true;
}

bool HorizontalEstimator_CorrectPosition(int32_t latitude_e7,
                                         int32_t longitude_e7,
                                         uint32_t gps_sequence,
                                         bool sample_valid,
                                         bool allow_state_reacquire)
{
    if (!horizontal_estimator.position_initialized ||
        gps_sequence == 0U ||
        gps_sequence == horizontal_estimator.last_position_sequence)
    {
        return false;
    }

    const float gps_sample_dt_s =
        horizontal_estimator.position_sample_elapsed_s;

    horizontal_estimator.position_sample_elapsed_s = 0.0f;
    horizontal_estimator.last_position_sequence = gps_sequence;
    horizontal_estimator.last_position_accepted = 0U;
    horizontal_estimator.last_position_rejected = 0U;
    horizontal_estimator.last_position_velocity_correction_applied = 0U;
    horizontal_estimator.position_velocity_correction_n_mps = 0.0f;
    horizontal_estimator.position_velocity_correction_e_mps = 0.0f;

    if (!sample_valid ||
        !Navigation_CoordinateValid(latitude_e7, longitude_e7) ||
        gps_sample_dt_s < NAV_POSITION_SAMPLE_DT_MIN_S ||
        gps_sample_dt_s > NAV_POSITION_SAMPLE_DT_MAX_S)
    {
        horizontal_estimator.position_reject_count++;
        horizontal_estimator.position_accept_streak = 0U;
        horizontal_estimator.last_position_rejected = 1U;
        return false;
    }

    const int64_t delta_lat_e7 =
        (int64_t)latitude_e7 -
        horizontal_estimator.position_reference_lat_e7;
    const int64_t delta_lon_e7 =
        (int64_t)longitude_e7 -
        horizontal_estimator.position_reference_lon_e7;

    horizontal_estimator.gps_position_n_m =
        (float)delta_lat_e7 * horizontal_estimator.meter_per_lat_e7;
    horizontal_estimator.gps_position_e_m =
        (float)delta_lon_e7 * horizontal_estimator.meter_per_lon_e7;

    const float innovation_n_m =
        horizontal_estimator.gps_position_n_m -
        horizontal_estimator.position_n_m;
    const float innovation_e_m =
        horizontal_estimator.gps_position_e_m -
        horizontal_estimator.position_e_m;
    const float innovation_m = sqrtf(
        innovation_n_m * innovation_n_m +
        innovation_e_m * innovation_e_m);

    const bool innovation_valid =
        innovation_m >= 0.0f &&
        innovation_m <= NAV_POSITION_INNOVATION_LIMIT_M;

    if (!innovation_valid)
    {
        if (!allow_state_reacquire)
        {
            horizontal_estimator.position_reject_count++;
            horizontal_estimator.position_accept_streak = 0U;
            horizontal_estimator.last_position_rejected = 1U;
            return false;
        }

        horizontal_estimator.position_n_m =
            horizontal_estimator.gps_position_n_m;
        horizontal_estimator.position_e_m =
            horizontal_estimator.gps_position_e_m;
        horizontal_estimator.position_accept_count++;
        horizontal_estimator.position_accept_streak = 1U;
        horizontal_estimator.last_position_accepted = 1U;
        return true;
    }

    const float correction_authority_dt_s =
        Navigation_Clamp(
            gps_sample_dt_s,
            NAV_POSITION_SAMPLE_DT_MIN_S,
            NAV_GPS_CORRECTION_AUTHORITY_DT_MAX_S);

    const float position_alpha_gain = Navigation_GainForSampleDt(
        NAV_POSITION_ALPHA_NOMINAL_GAIN,
        correction_authority_dt_s);

    float position_correction_n_m =
        position_alpha_gain * innovation_n_m;
    float position_correction_e_m =
        position_alpha_gain * innovation_e_m;

    /*
     * 限制单个GPS Epoch对Estimator Position的二维修正量，
     * 防止GPS跳点直接造成Position target和姿态指令突变。
     */
    Navigation_LimitVector(
        &position_correction_n_m,
        &position_correction_e_m,
        NAV_POSITION_STATE_CORRECTION_LIMIT_M);

    horizontal_estimator.position_n_m +=
        position_correction_n_m;
    horizontal_estimator.position_e_m +=
        position_correction_e_m;

    const float position_beta_nominal_gain =
        horizontal_estimator.last_rmc_vector_used
            ? NAV_POSITION_BETA_RMC_AIDED_NOMINAL_GAIN
            : NAV_POSITION_BETA_LOW_SPEED_NOMINAL_GAIN;

    const float position_beta_gain = Navigation_BetaForSampleDt(
        position_beta_nominal_gain,
        correction_authority_dt_s);

    float velocity_correction_n_mps =
        position_beta_gain * innovation_n_m / gps_sample_dt_s;
    float velocity_correction_e_mps =
        position_beta_gain * innovation_e_m / gps_sample_dt_s;

    Navigation_LimitVector(
        &velocity_correction_n_mps,
        &velocity_correction_e_mps,
        NAV_POSITION_VELOCITY_CORRECTION_RATE_LIMIT_MPS2 * correction_authority_dt_s);

    const float velocity_before_n_mps =
        horizontal_estimator.velocity_n_mps;
    const float velocity_before_e_mps =
        horizontal_estimator.velocity_e_mps;

    horizontal_estimator.velocity_n_mps += velocity_correction_n_mps;
    horizontal_estimator.velocity_e_mps += velocity_correction_e_mps;

    Navigation_LimitVector(
        &horizontal_estimator.velocity_n_mps,
        &horizontal_estimator.velocity_e_mps,
        NAV_ESTIMATED_SPEED_LIMIT_MPS);

    horizontal_estimator.position_velocity_correction_n_mps =
        horizontal_estimator.velocity_n_mps - velocity_before_n_mps;
    horizontal_estimator.position_velocity_correction_e_mps =
        horizontal_estimator.velocity_e_mps - velocity_before_e_mps;

    horizontal_estimator.position_accept_count++;
    horizontal_estimator.position_velocity_correction_count++;
    if (horizontal_estimator.position_accept_streak < UINT8_MAX)
        horizontal_estimator.position_accept_streak++;
    horizontal_estimator.last_position_accepted = 1U;
    horizontal_estimator.last_position_velocity_correction_applied = 1U;
    return true;
}

static void PositionController_ResetIntegralLearningGate(uint32_t current_gps_sequence)
{
    position_controller.integral_learning_last_gps_sequence = current_gps_sequence;
    position_controller.integral_learning_candidate_start_tick_ms = 0U;
    position_controller.integral_learning_valid_sample_count = 0U;
    position_controller.integral_learning_allowed = 0U;
}

/**
 * @brief 初始化位置控制器模块及其参数设置
 */
void PositionController_Init(void)
{
    PositionController_Reset();
    position_controller.kp = POSITION_KP;
}

/**
 * @brief 复位位置控制器内的参考锚点和累计控制状态
 */
void PositionController_Reset(void)
{
    position_controller.kp = POSITION_KP;

    position_controller.target_n_m = 0.0f;
    position_controller.target_e_m = 0.0f;
    position_controller.error_n_m = 0.0f;
    position_controller.error_e_m = 0.0f;

    position_controller.velocity_target_n_mps = 0.0f;
    position_controller.velocity_target_e_mps = 0.0f;

    position_controller.brake_elapsed_time_s = 0.0f;
    position_controller.brake_last_gps_sequence = 0U;
    position_controller.brake_low_speed_sample_count = 0U;

    position_controller.phase = POSITION_CONTROL_PHASE_INACTIVE;
    position_controller.initialized = 0U;

    PositionController_ResetIntegralLearningGate(0U);
}

/**
 * @brief 锁定当前坐标系原点，使系统进入局部定点控制模式
 * @param latitude_e7 锚点纬度坐标
 * @param longitude_e7 锚点经度坐标
 * @return true=进入成功并建立局部切面系；false=坐标非法拒绝进入
 */
bool PositionController_Enter(const HorizontalEstimator_t *horizontal_state)
{
    PositionController_Reset();

    if (horizontal_state == NULL ||
        !horizontal_state->initialized ||
        !horizontal_state->position_initialized)
    {
        return false;
    }

    position_controller.target_n_m = horizontal_state->position_n_m;
    position_controller.target_e_m = horizontal_state->position_e_m;
    position_controller.brake_last_gps_sequence =
        horizontal_state->last_gps_sequence;
    PositionController_ResetIntegralLearningGate(horizontal_state->last_gps_sequence);
    position_controller.phase = POSITION_CONTROL_PHASE_BRAKING;
    position_controller.initialized = 1U;

    return true;
}

/**
 * @brief 位置控制器核心运算逻辑更新(外环)
 */
bool PositionController_Update(HorizontalEstimator_t *horizontal_state,
                               float pilot_velocity_n_mps,
                               float pilot_velocity_e_mps,
                               bool brake_velocity_observations_consistent,
                               float brake_observed_speed_mps,
                               float dt)
{
    if (horizontal_state == NULL)
        return false;

    const float velocity_est_n_mps = horizontal_state->velocity_n_mps;
    const float velocity_est_e_mps = horizontal_state->velocity_e_mps;

    const bool state_valid =
        horizontal_state->initialized &&
        horizontal_state->position_initialized &&
        velocity_est_n_mps >= -NAV_ESTIMATED_SPEED_LIMIT_MPS &&
        velocity_est_n_mps <= NAV_ESTIMATED_SPEED_LIMIT_MPS &&
        velocity_est_e_mps >= -NAV_ESTIMATED_SPEED_LIMIT_MPS &&
        velocity_est_e_mps <= NAV_ESTIMATED_SPEED_LIMIT_MPS;

    if (!position_controller.initialized ||
        dt < 0.0005f ||
        dt > 0.0050f ||
        !state_valid)
    {
        return false;
    }

    Navigation_LimitVector(&pilot_velocity_n_mps,
                           &pilot_velocity_e_mps,
                           POSITION_PILOT_SPEED_LIMIT_MPS);

    const float pilot_speed_mps = sqrtf(
        pilot_velocity_n_mps * pilot_velocity_n_mps +
        pilot_velocity_e_mps * pilot_velocity_e_mps);

    const float estimated_speed_mps = sqrtf(
        velocity_est_n_mps * velocity_est_n_mps +
        velocity_est_e_mps * velocity_est_e_mps);

    const bool pilot_active =
        pilot_speed_mps > POSITION_PILOT_ACTIVE_SPEED_MPS;

    if (pilot_active)
    {
        position_controller.phase = POSITION_CONTROL_PHASE_MOVING;
        position_controller.brake_elapsed_time_s = 0.0f;
        position_controller.brake_last_gps_sequence =
            horizontal_state->last_gps_sequence;
        position_controller.brake_low_speed_sample_count = 0U;
        PositionController_ResetIntegralLearningGate(horizontal_state->last_gps_sequence);

        /* MOVING期间目标点随Estimator Position移动；
         * Controller不积累人工机动产生的位置误差。 */
        position_controller.target_n_m = horizontal_estimator.position_n_m;
        position_controller.target_e_m = horizontal_estimator.position_e_m;
        position_controller.error_n_m = 0.0f;
        position_controller.error_e_m = 0.0f;
        position_controller.velocity_target_n_mps = pilot_velocity_n_mps;
        position_controller.velocity_target_e_mps = pilot_velocity_e_mps;
        return true;
    }

    if (position_controller.phase == POSITION_CONTROL_PHASE_MOVING ||
        position_controller.phase == POSITION_CONTROL_PHASE_INACTIVE)
    {
        position_controller.phase = POSITION_CONTROL_PHASE_BRAKING;
        position_controller.brake_elapsed_time_s = 0.0f;
        position_controller.brake_last_gps_sequence =
            horizontal_state->last_gps_sequence;
        position_controller.brake_low_speed_sample_count = 0U;
        PositionController_ResetIntegralLearningGate(horizontal_state->last_gps_sequence);
    }

    if (position_controller.phase == POSITION_CONTROL_PHASE_BRAKING)
    {
        position_controller.target_n_m = horizontal_state->position_n_m;
        position_controller.target_e_m = horizontal_state->position_e_m;
        position_controller.error_n_m = 0.0f;
        position_controller.error_e_m = 0.0f;
        position_controller.velocity_target_n_mps = 0.0f;
        position_controller.velocity_target_e_mps = 0.0f;
        position_controller.integral_learning_allowed = 0U;

        if(position_controller.brake_elapsed_time_s <
            POSITION_BRAKE_MIN_TIME_S)
        {
            position_controller.brake_elapsed_time_s += dt;
            if(position_controller.brake_elapsed_time_s > 
                POSITION_BRAKE_MIN_TIME_S)
            {
                position_controller.brake_elapsed_time_s =
                    POSITION_BRAKE_MIN_TIME_S;
            }
        }

        const bool brake_settle_time_ready =
            position_controller.brake_elapsed_time_s >=
            POSITION_BRAKE_MIN_TIME_S;

        const bool new_gps_sample =
            horizontal_state->last_gps_sequence !=
            position_controller.brake_last_gps_sequence;

        if(new_gps_sample)
        {
            position_controller.brake_last_gps_sequence =
                horizontal_state->last_gps_sequence;

            const bool capture_sample_valid =
                brake_settle_time_ready &&
                brake_velocity_observations_consistent &&
                (horizontal_state->last_gps_accepted != 0U) &&
                (horizontal_state->last_position_accepted != 0U) &&
                (brake_observed_speed_mps <= POSITION_BRAKE_CAPTURE_OBSERVED_SPEED_MPS) &&
                (estimated_speed_mps <= POSITION_BRAKE_CAPTURE_SPEED_MPS);
            
            if(capture_sample_valid)
            {
                if(position_controller.brake_low_speed_sample_count <
                    POSITION_BRAKE_CAPTURE_GPS_SAMPLES)
                {
                    position_controller.brake_low_speed_sample_count++;
                }
            }
            else
            {
                position_controller.brake_low_speed_sample_count = 0U;
            }
        }

        if(position_controller.brake_low_speed_sample_count >=
            POSITION_BRAKE_CAPTURE_GPS_SAMPLES)
        {
            position_controller.phase = POSITION_CONTROL_PHASE_HOLD;
            position_controller.target_n_m =
                horizontal_state->position_n_m;
            position_controller.target_e_m =
                horizontal_state->position_e_m;
            PositionController_ResetIntegralLearningGate(horizontal_state->last_gps_sequence);
        }

        return true;
    }

    if (position_controller.phase != POSITION_CONTROL_PHASE_HOLD)
        return false;

    position_controller.error_n_m =
        position_controller.target_n_m - horizontal_state->position_n_m;
    position_controller.error_e_m =
        position_controller.target_e_m - horizontal_state->position_e_m;

    const float error_magnitude = sqrtf(
        position_controller.error_n_m * position_controller.error_n_m +
        position_controller.error_e_m * position_controller.error_e_m);

    if (error_magnitude > POSITION_MAX_ERROR_M)
    {
        PositionController_ResetIntegralLearningGate(horizontal_state->last_gps_sequence);
        return false;
    }

    const bool integral_fast_context_valid =
        error_magnitude <= POSITION_HOLD_INTEGRAL_POSITION_ERROR_MAX_M &&
        estimated_speed_mps <= POSITION_HOLD_INTEGRAL_EST_SPEED_MAX_MPS;

    if(!integral_fast_context_valid)
    {
        PositionController_ResetIntegralLearningGate(horizontal_state->last_gps_sequence);
    }
    else
    {
        const bool new_gps_sample =
            horizontal_state->last_gps_sequence !=
            position_controller.integral_learning_last_gps_sequence;

        if(new_gps_sample)
        {
            position_controller.integral_learning_last_gps_sequence =
                horizontal_state->last_gps_sequence;

            const bool integral_sample_valid =
                brake_velocity_observations_consistent &&
                (horizontal_state->last_gps_accepted != 0U) &&
                (horizontal_state->last_position_accepted != 0U) &&
                (horizontal_state->last_gps_tick_ms != 0U) &&
                (brake_observed_speed_mps <=
                 POSITION_HOLD_INTEGRAL_OBSERVED_SPEED_MAX_MPS);

            if (!integral_sample_valid)
            {
                position_controller.integral_learning_candidate_start_tick_ms = 0U;
                position_controller.integral_learning_valid_sample_count = 0U;
                position_controller.integral_learning_allowed = 0U;
            }
            else
            {
                if (position_controller.integral_learning_valid_sample_count == 0U)
                {
                    position_controller.integral_learning_candidate_start_tick_ms =
                        horizontal_state->last_gps_tick_ms;
                }

                if (position_controller.integral_learning_valid_sample_count < UINT8_MAX)
                {
                    position_controller.integral_learning_valid_sample_count++;
                }

                const uint32_t candidate_elapsed_ms =
                    horizontal_state->last_gps_tick_ms -
                    position_controller.integral_learning_candidate_start_tick_ms;

                position_controller.integral_learning_allowed =
                    (position_controller.integral_learning_valid_sample_count >=
                     POSITION_HOLD_INTEGRAL_MIN_GPS_SAMPLES) &&
                            (candidate_elapsed_ms >=
                             POSITION_HOLD_INTEGRAL_CONFIRM_TIME_MS)
                        ? 1U
                        : 0U;
            }
        }
    }

    float correction_error_n_m = 0.0f;
    float correction_error_e_m = 0.0f;

    if (error_magnitude > POSITION_DEADBAND_M)
    {
        const float scale =
            (error_magnitude - POSITION_DEADBAND_M) / error_magnitude;
        correction_error_n_m = position_controller.error_n_m * scale;
        correction_error_e_m = position_controller.error_e_m * scale;
    }

    position_controller.velocity_target_n_mps =
        position_controller.kp * correction_error_n_m;
    position_controller.velocity_target_e_mps =
        position_controller.kp * correction_error_e_m;

    Navigation_LimitVector(
        &position_controller.velocity_target_n_mps,
        &position_controller.velocity_target_e_mps,
        POSITION_VELOCITY_LIMIT_MPS);

    return true;
}
