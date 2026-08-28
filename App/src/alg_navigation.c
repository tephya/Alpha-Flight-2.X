/**
 * @file    alg_navigation.h
 * @brief   水平状态估计，速度控制及位置控制实现。
 */

#include "alg_navigation.h"
#include <stddef.h>
#include <math.h>

/* =========================================================================
 * 常量与宏定义
 * ========================================================================= */

#define NAV_GRAVITY_MPS2 9.80665f       // 标准重力加速度，m/s^2。
#define NAV_RAD_TO_DEG 57.2957795f      // 弧度到角度转换系数。

/* -------------------------------------------------------------------------
 * IMU水平Acceleration处理
 * ------------------------------------------------------------------------- */

/*
 * 当前 IMU 静止时 Accel 测量主要反映机体系重力方向。
 * 按本项目的传感器轴向和 NED 坐标约定：
 * 
 * linear_accel_ne = -R_body_to_ned * accel_measurement
 */
#define NAV_ACCEL_MEASUREMENT_SIGN (-1.0f)

#define NAV_ACCEL_LPF_CUTOFF_HZ 5.0f        // 水平 Acceleration 一阶低通截止频率，Hz。
#define NAV_ACCEL_LIMIT_MPS2 6.0f           // 单轴原始水平 Acceleration 限幅，m/s^2。
#define NAV_ACCEL_BIAS_LIMIT_MPS2 1.5f      // Accel Bias 二维模长上限，m/s^2。
#define NAV_ACCEL_BIAS_TIME_CONSTANT_S 5.0f     // Disarmed 静止 Bias 学习时间常数，s。

/*
 * 飞行中 Accel bias 慢速观测器参数。
 * 
 * 使用 GPS Velocity Innovation 估计由 IMU Acceleration Bias
 * 长期积分产生的 Velocity 漂移，并反向修正机体系 Accel Bias。
 * 该观测器只做低带宽慢速修正，避免将 GPS 短期噪声学习为 IMU Bias。
 */

#define NAV_ACCEL_BIAS_GPS_CORRECTION_GAIN 0.005f   // GPS Innovation 转换为 Accel Bias 修正量时的观测增益。
#define NAV_ACCEL_BIAS_GPS_INNOVATION_LIMIT_MPS 0.30f   // 允许参与 Bias 学习的最大二维 GPS Velocity Innovation，m/s。
#define NAV_ACCEL_BIAS_GPS_DT_MIN_S 0.08f           // 允许执行 Bias 的最小 GPS 样本间隔，s。
#define NAV_ACCEL_BIAS_GPS_DT_MAX_S 0.40f           // 允许执行 Bias 的最大 GPS 样本间隔，s。

/*
 * 单个 GPS 样本允许产生的最大二维 Accel Bias 修正量，m/s^2。
 * 用于限制异常或瞬时 Innovation 对 Bias 状态造成突变。
 */
#define NAV_ACCEL_BIAS_GPS_STEP_LIMIT_MPS2 0.01f

/* -------------------------------------------------------------------------
 * GPS采样周期与离散融合增益
 * ------------------------------------------------------------------------- */

/*
 * GPS 离散融合增益换算参数。
 * 
 * 下列标称增益均以原 5 Hz GPS 配置为基准，对应参考采样周期 0.20s。
 * 实际融合仍在每个新 GPS 验本到达时执行，并根据实际 Sample Dt
 * 换算等效增益，以保持不同采样率下单位时间内的修正带宽基本一致。
 */

#define NAV_GPS_GAIN_REFERENCE_DT_S 0.20f   // 标称 GPS 融合增益对应的参考采样周期，s。
#define NAV_GAIN_DT_RATIO_MIN 0.25f         // 实际 Sample Dt 与参考周期比值的下限。
#define NAV_GAIN_DT_RATIO_MAX 3.00f         // 实际 Sample Dt 与参考周期比值的上限。

/*
 * GPS 样本迟到代表观测带宽降低，不能因此获得更大的单帧状态修改权限。
 * 实际 Sample dt 仍用于时效检查和 innovation/dt 计算。
 */
#define NAV_GPS_CORRECTION_AUTHORITY_DT_MAX_S 0.15f

/* GPS Position 单帧允许直接移动 Estimator Position 的最大二维距离，m。 */
#define NAV_POSITION_STATE_CORRECTION_LIMIT_M 0.12f

/* GPS Velocity 融合修正。 */
#define NAV_GPS_CORRECTION_NOMINAL_GAIN 0.35f               // 5 Hz 标称采样周期下的 Velocity 融合增益。
#define NAV_GPS_STATE_CORRECTION_RATE_LIMIT_MPS2 2.00f      // Estimator Velocity 最大修正速率，m/s^2。
#define NAV_GPS_INNOVATION_LIMIT_MPS 5.0f       // 常规融合允许的最大二维 Velocity Innovation，m/s。
#define NAV_GPS_REACQUIRE_GAP_MS 1500U          // 超过该样本时间间隔后进入 State Reacquire，ms。
#define NAV_GPS_TIMEOUT_MS 600U                 // 超过该时间未收到有效样本则 GPS 判为不健康，ms。
#define NAV_ESTIMATED_SPEED_LIMIT_MPS 20.0f     // Estimator 最大二维水平速度模长，m/s。

/* -------------------------------------------------------------------------
 * GPS Position alpha-beta修正
 * ------------------------------------------------------------------------- */

#define NAV_POSITION_ALPHA_NOMINAL_GAIN 0.10f           // Position ALpha 标称增益，以 5 Hz GPS 为基准。
#define NAV_POSITION_BETA_LOW_SPEED_NOMINAL_GAIN 0.06f  // 低速，无可靠 RMC Course 时的 Position Beta 标称增益。
#define NAV_POSITION_BETA_RMC_AIDED_NOMINAL_GAIN 0.02f  // RMC Velocity 已提供方向约束时的 Position Beta 标称增益。

#define NAV_POSITION_INNOVATION_LIMIT_M 5.0f        // 常规融合允许的最大二维 Position Innovation，m。
#define NAV_POSITION_SAMPLE_DT_MIN_S 0.05f          // 允许参与 Position 修正的最小 GPS Sample Dt，s。
#define NAV_POSITION_SAMPLE_DT_MAX_S 0.60f          // 允许参与 Position 修正的最大 GPS Sample Dt，s。
#define NAV_POSITION_SAMPLE_ELAPSED_LIMIT_S 2.0f    // Position Sample Dt 累计值的最大保存上限，s。

/* 判断 Velocity 是否具有可保留方向的最小模长，m/s。 */
#define NAV_VELOCITY_DIRECTION_EPSILON_MPS 0.001f

/*
 * 原配置中每个 5Hz Position 样本最多修正 0.20m/s Velocity，
 * 等价为 1.00m/^s 的状态修正速率。
 */
#define NAV_POSITION_VELOCITY_CORRECTION_RATE_LIMIT_MPS2 1.00f

/* -------------------------------------------------------------------------
 * Velocity Controller
 * ------------------------------------------------------------------------- */

#define VELOCITY_KP 1.80f
#define VELOCITY_KI 0.50f
#define VELOCITY_KD 0.20f

/* Velocity Integral 对 Acceleration 指令的最大二维贡献，m/s^2。 */
#define VELOCITY_INTEGRAL_ACCEL_LIMIT_MPS2 0.80f

/* 水平目标 Acceleration 最大模长，m/s^2。 */
#define VELOCITY_ACCEL_LIMIT_MPS2 2.00f

/* 水平目标 Acceleration 最大变化率，m/s^3。 */
#define VELOCITY_ACCEL_SLEW_LIMIT_MPS3 8.00f

/* Roll/Pitch 目标角绝对值上限，deg。 */
#define VELOCITY_ANGLE_LIMIT_DEG 12.0f

/* Back-calculation Anti-windup 增益。 */
#define VELOCITY_ARW_GAIN (2.0f / VELOCITY_KP)

/* Integral 卸载时的 Velocity Error 增强系数。 */
#define VELOCITY_INTEGRAL_UNLOAD_GAIN 2.0f

/* -------------------------------------------------------------------------
 * Position Controller
 * ------------------------------------------------------------------------- */

#define NAV_EARTH_RADIUS_M 6378137.0f           // 用于局部经纬度距离换算的地球参考半径，m。
#define NAV_DEG_E7_TO_RAD 1.745329252e-9f       // 将 E7 格式角度单位转换为 rad 的比例系数。

#define POSITION_KP 0.70f                       // Position 外环比例增益。
#define POSITION_DEADBAND_M 0.10f               // Position Hold 位置误差死区半径，m。
#define POSITION_PILOT_SPEED_LIMIT_MPS 1.50f    // Pilot 水平速度指令最大模长，m/s。
#define POSITION_VELOCITY_LIMIT_MPS 1.20f       // Position 外环输出的最大水平 Velocity Target，m/s。
#define POSITION_MAX_ERROR_M 30.0f              // Position Hold 语训继续控制的最大二维位置误差，m。

/* 超过该 Pilot Velocity 后认为驾驶员正在主动移动飞行器，m/s。 */
#define POSITION_PILOT_ACTIVE_SPEED_MPS 0.02f

/*
 * BRAKING -> HOLD 的锚点捕获条件。
 * Estimator Velocity 与 独立 GPS 速度观测都足够低时，
 * 才认为飞行器已经实际停止并允许锁定当前位置。
 */
#define POSITION_BRAKE_CAPTURE_SPEED_MPS 0.15f              // Estimator 水平速度模长捕获阈值，m/s。
#define POSITION_BRAKE_CAPTURE_OBSERVED_SPEED_MPS 0.25f     // RMC 与 Position-window 速度观测中较大模长捕获阈值，m/s。

/*
 * HOLD 状态下 Velocity Integral 的学习许可条件。
 * 仅在接近锚点且速度稳定时学习静态扰动补偿，
 * 避免把漂移过程或 Position 瞬态积累到 Integral。
 */
#define POSITION_HOLD_INTEGRAL_POSITION_ERROR_MAX_M 0.50f       // 允许 Integral 学习的最大 Position Error，m。
#define POSITION_HOLD_INTEGRAL_EST_SPEED_MAX_MPS 0.60f          // 允许 Integral 学习的最大 Estimator Speed，m/s。
#define POSITION_HOLD_INTEGRAL_OBSERVED_SPEED_MAX_MPS 0.70f     // 允许 Integral 学习的最大 GPS 观测速度，m/s。
#define POSITION_HOLD_INTEGRAL_CONFIRM_TIME_MS 750U             // Integral 学习条件需连续成立的最短时间，ms。
#define POSITION_HOLD_INTEGRAL_MIN_GPS_SAMPLES 4U               // Integral 学习前要求的最少连续有效 GPS 样本数。

/* BRAKING 至少持续该时间后才允许捕获 HOLD 锚点，s。 */
#define POSITION_BRAKE_MIN_TIME_S 0.80f

/* 捕获 HOLD 前要求连续满足低速条件的 GPS 样本数。 */
#define POSITION_BRAKE_CAPTURE_GPS_SAMPLES 5U

/* =========================================================================
 * 全局状态变量
 * ========================================================================= */

HorizontalEstimator_t horizontal_estimator;
VelocityController_t velocity_controller;
PositionController_t position_controller;

/* =========================================================================
 * 内部辅助数学函数
 * ========================================================================= */

/* 将标量限制在 [min_value, max_value]。 */
static float Navigation_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

/*
 * 限制二维向量的最大模长。
 * 当发生限幅时，两轴按同一比例缩放以保持原方向不变。
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

/*
 * 将以 5 Hz 为基准的离散一阶融合增益换算到实际 GPS Sample Dt。
 * 
 * 采用等效离散极点换算，使 GPS 采样率变化后，
 * 单位时间内的滤波带宽保持一致。
 */
static float Navigation_GainForSampleDt(float nominal_gain, float sample_dt_s)
{
    const float dt_ratio = Navigation_Clamp(
        sample_dt_s / NAV_GPS_GAIN_REFERENCE_DT_S,
        NAV_GAIN_DT_RATIO_MIN,
        NAV_GAIN_DT_RATIO_MAX);

    return 1.0f - powf(1.0f - nominal_gain, dt_ratio);
}

/*
 * 将 5 Hz 标称 Alpha-Beta Filter 的 Beta 换算到实际 GPS Sample Dt。
 * 
 * Beta 项通过 Innovation/Dt 修正 Velocity，因此需要按 Dt^2 缩放，
 * 避免提高 GPS 采样率后 Velocity 修正带宽随之增大。
 */
static float Navigation_BetaForSampleDt(float nominal_beta, float sample_dt_s)
{
    const float dt_ratio = Navigation_Clamp(
        sample_dt_s / NAV_GPS_GAIN_REFERENCE_DT_S,
        NAV_GAIN_DT_RATIO_MIN,
        NAV_GAIN_DT_RATIO_MAX);

    return nominal_beta * dt_ratio * dt_ratio;
}

/*
 * 将机体系 Forward/Right Accel Bias 投影到导航系 North/East；
 * 
 * N = cos(yaw) * Forward - sin(yaw) * Right
 * E = sin(yaw) * Forward + cos(yaw) * Right
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

/*
 * 根据当前 Yaw 刷新公开的 N/E Bias 投影，
 * 并发布去除 Bias 后的水平 Acceleration。
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
    horizontal_estimator.last_gps_accepted = 0U;
    horizontal_estimator.last_rmc_vector_used = 0U;

    horizontal_estimator.gps_accept_count = 0U;
    horizontal_estimator.gps_accept_streak = 0U;
    horizontal_estimator.gps_reject_count = 0U;
    horizontal_estimator.initialized = 0U;
    horizontal_estimator.gps_healthy = 0U;

    HorizontalEstimator_ResetPosition();
}

void HorizontalEstimator_Predict(float ax_g,
                                 float ay_g,
                                 float az_g,
                                 float roll_rad,
                                 float pitch_rad,
                                 float yaw_rad,
                                 float dt,
                                 bool learn_accel_bias)
{
    // 拒绝异常时间步长，避免积分和滤波系数失真。
    if (dt < 0.0005f || dt > 0.0050f)
        return;

    const float sr = sinf(roll_rad);
    const float cr = cosf(roll_rad);
    const float sp = sinf(pitch_rad);
    const float cp = cosf(pitch_rad);
    const float sy = sinf(yaw_rad);
    const float cy = cosf(yaw_rad);

    /*
     * 使用 R_body_to_ned 的前两行，将机体系 Accel 投影到 N/E。
     * 静止状态下重力应主要落在 NED Down 轴，因此理想 N/E 分量接近零。
     * 
     * v_ned = Rz(Yaw) · Ry(Pitch) · Rx(Roll) · v_body
     * v_body = [ax, ay, az]
     */
    const float measured_n_g =
        (cp * cy) * ax_g +
        (sr * sp * cy - cr * sy) * ay_g +
        (cr * sp * cy + sr * sy) * az_g;

    const float measured_e_g =
        (cp * sy) * ax_g +
        (sr * sp * sy + cr * cy) * ay_g +
        (cr * sp * sy - sr * cy) * az_g;

    // 根据项目轴向约定调整符号，并由 g 转换为 m/s^2。
    float accel_n_raw = 
        NAV_ACCEL_MEASUREMENT_SIGN * measured_n_g * NAV_GRAVITY_MPS2;
    float accel_e_raw = 
        NAV_ACCEL_MEASUREMENT_SIGN * measured_e_g * NAV_GRAVITY_MPS2;

    // 单轴粗限幅，用于抑制振动或传感器异常产生的瞬时尖峰。
    accel_n_raw = Navigation_Clamp(
        accel_n_raw, 
        -NAV_ACCEL_LIMIT_MPS2, 
        NAV_ACCEL_LIMIT_MPS2);
    accel_e_raw = Navigation_Clamp(
        accel_e_raw, 
        -NAV_ACCEL_LIMIT_MPS2, 
        NAV_ACCEL_LIMIT_MPS2);

    // 一阶 LPF 平滑水平 Acceleration，并根据实际 Dt 计算离散系数。
    const float tau = 1.0f / (6.2831853f * NAV_ACCEL_LPF_CUTOFF_HZ);
    const float alpha = dt / (tau + dt);

    horizontal_estimator.accel_lpf_n_mps2 +=
        alpha * 
        (accel_n_raw - horizontal_estimator.accel_lpf_n_mps2);
    horizontal_estimator.accel_lpf_e_mps2 +=
        alpha * 
        (accel_e_raw - horizontal_estimator.accel_lpf_e_mps2);

    if (learn_accel_bias)
    {
        /*
         * 静止学习时先将导航系 Acceleration 转回机体系，
         * 使 Bias 始终以 Forward/Right 保存，不随 Yaw 改变其物理含义。
         */
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
    // GPS 完成初始对齐后，使用水平 Acceleration 在样本间推进 Velocity。
    if (horizontal_estimator.initialized)
    {
        horizontal_estimator.velocity_n_mps +=
            horizontal_estimator.accel_n_mps2 * dt;
        horizontal_estimator.velocity_e_mps +=
            horizontal_estimator.accel_e_mps2 * dt;

        // 防止长时间失去绝对观测后积分状态无限发散。
        Navigation_LimitVector(&horizontal_estimator.velocity_n_mps,
                               &horizontal_estimator.velocity_e_mps,
                               NAV_ESTIMATED_SPEED_LIMIT_MPS);
    }
#endif

    if (horizontal_estimator.position_initialized)
    {
        /*
         * Position 与 Velocity 属于同一 Estimator 状态，
         * 高频 Position 预测必须使用当前同一份 Velocity。
         */
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
 * 使用 GPS Velocity Innovation 慢速校正飞行中的水平 Accel Bias。
 *
 * Predict使用：
 *      Velocity += (accel_lpf - accel_bias) * dt
 *
 * 当 Estimator 速度高于 GPS 速度时，innovation 为负，说明用于积分的
 * Accel 长期偏正，因此需要增加 Accel bias，故bias修正符号为负。
 *
 * 该机制只适用于稳定 Position Hold。调用方需排除 Pilot 输入，
 * BRAKING 和速度源切换等动态过程。
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

    // 同时排除过大的 Innovation 以及 NaN/Inf 等非法浮点结果。
    if (!(innovation_mps >= 0.0f &&
          innovation_mps <= NAV_ACCEL_BIAS_GPS_INNOVATION_LIMIT_MPS))
    {
        return;
    }

    /*
     * innovation/dt 近似长期 Acceleration 误差。
     * 仅使用很小的观测增益更新 Bias，避免跟随 GPS Velocity 噪声。
     */
    float bias_delta_n_mps2 =
        -NAV_ACCEL_BIAS_GPS_CORRECTION_GAIN *
        innovation_n_mps / gps_dt_s;

    float bias_delta_e_mps2 =
        -NAV_ACCEL_BIAS_GPS_CORRECTION_GAIN *
        innovation_e_mps / gps_dt_s;

    // 限制单个 GPS 样本允许改变的 Bias 大小。
    Navigation_LimitVector(
        &bias_delta_n_mps2,
        &bias_delta_e_mps2,
        NAV_ACCEL_BIAS_GPS_STEP_LIMIT_MPS2);

    const float sy = sinf(yaw_rad);
    const float cy = cosf(yaw_rad);

    // 将 N/E Bias 修正量转回持久保存的 Forward/Right 机体系。
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

    // 立即发布新的 Bias 对应的 N/E Acceleration，不等待下一次 Predict。
    HorizontalEstimator_RefreshAccelOutput(sy, cy);
}

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
    // Sequence 为零或重复时不重复消费同一 GPS 样本。
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

    /*
     * 对 Ground Speed 进行有限范围判断。
     * NaN 会使比较表达式为 false，因此也会被一并拒绝。
     */
    if (!sample_valid || !(rmc_speed_mps >= 0.0f && rmc_speed_mps <= 30.0f))
    {
        horizontal_estimator.gps_reject_count++;
        horizontal_estimator.gps_accept_streak = 0U;
        horizontal_estimator.gps_healthy = 0U;
        return false;
    }

    /*
     * 只有 RMC Course 已通过外部一致性判断，且 Ground Speed 足够高时，
     * 才允许把 RMC 分解得到的 N/E 方向写入 Estimator。
     */
    const bool rmc_course_usable =
        rmc_vector_use_allowed &&
        rmc_speed_mps >= NAV_RMC_VECTOR_MIN_SPEED_MPS;

    const uint32_t gps_ms = 
        gps_tick_ms - horizontal_estimator.last_gps_tick_ms;

    /*
     * 首帧或长时间未收到 GPS 时进入重新对齐路径，
     * 避免用过大的 Sample Dt 继续执行普通增量融合。
     */
    const bool initial_alignment =
        !horizontal_estimator.initialized ||
        horizontal_estimator.last_gps_tick_ms == 0U ||
        gps_ms > NAV_GPS_REACQUIRE_GAP_MS;

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
            // Course 可信时直接以 GPS N/E Velocity 建立初始状态。
            horizontal_estimator.velocity_n_mps = gps_velocity_n_mps;
            horizontal_estimator.velocity_e_mps = gps_velocity_e_mps;
            horizontal_estimator.last_rmc_vector_used = 1U;
        }
        else
        {
            /*
             * 无可靠方向不能凭 Ground Speed 构造任意 N/E 向量。
             * 因此重新捕获时从零 Velocity 开始。
             */
            horizontal_estimator.velocity_n_mps = 0.0f;
            horizontal_estimator.velocity_e_mps = 0.0f;
        }

        horizontal_estimator.initialized = 1U;
        horizontal_estimator.gps_accept_streak = 1U;
    }
    else
    {
        const float gps_dt_s = (float)gps_ms * 0.001f;

        /*
         * Sample Dt 可以真实反应观测间隔，但单帧修正权限没有上限，
         * 防止迟到样本一次性大幅推动 Estimator 状态。
         */
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
                /*
                 * Innovation 超限通常表示 Estimator 已与 GPS 严重脱离。
                 * 仅在上层明确允许 Reacquire 时直接重新对齐。
                 */
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
             * 低速 RMC course 不可靠；刚越过低速门限但尚未通过外部
             * Position Window 一致性确认时，同样不能使用其 N/E 方向。
             *
             * 此时只利用 Ground Speed 对 Estimator Velocity 模长施加上界，
             * 抑制 IMU 积分长期漂大，同时保留原有 Velocity 方向。
             */
            const float estimated_speed_mps = sqrtf(
                horizontal_estimator.velocity_n_mps *
                    horizontal_estimator.velocity_n_mps +
                horizontal_estimator.velocity_e_mps *
                    horizontal_estimator.velocity_e_mps);

            const bool estimated_speed_valid =
                estimated_speed_mps >= 0.0f &&
                estimated_speed_mps <= NAV_ESTIMATED_SPEED_LIMIT_MPS;

            /*
             * 最低保留 NAV_RMC_VECTOR_MIN_SPEED_MPS 的速度上界，
             * 避免低速 GPS 噪声把 Estimator Velocity 反复压到零。
             */
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

                    /*
                     * 仅缩放 Velocity 模长，不采用不可信的RMC Course，
                     * 因此 N/E 两轴必须使用同一比例。
                     */
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
    // 未初始化，从未收到有效样本或超过实现均潘伟 GPS 不健康。
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

/*
 * 更新单轴 Velocity Integral。
 * 
 * LEARN        ：允许正常积累，并加速反向误差对已有 Integral 的卸载。
 * UNLOAD_ONLY  ：只允许 Integral 向零方向变化。
 * FROZEN       ：完全保持当前 Integral。
 * 
 * saturation_error 通过 Back-calculation 将输出饱和反馈到 Integral，
 * 防止控制器持续积累无法实际输出的 Acceleration 请求。
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

        // 误差方向有助于卸载已有 Integral 时，加快回零速度。
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

    /*
     * UNLOAD_ONLY 模式下 Integral 只能减小绝对值。
     * 若候选值跨过零点，则直接钳到零，禁止反向重新累积。
     */
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

    const float error_n_mps = velocity_target_n_mps - velocity_meas_n_mps;
    const float error_e_mps = velocity_target_e_mps - velocity_meas_e_mps;

    velocity_controller.integral_enabled =
        (integral_mode == VELOCITY_INTEGRAL_MODE_LEARN) ? 1U : 0U;

    /*
     * D-on-measurement：Velocity 的时间导数就是 Acceleration。
     * 直接使用 Estimator 已经完成重力分离、Bias 去除和 LPF 的水平
     * Acceleration，可避免对含 GPS 修正阶跃的 Velocity 再做数值微分。
     */
    velocity_controller.p_accel_n_mps2 =
        velocity_controller.kp * error_n_mps;
    velocity_controller.p_accel_e_mps2 =
        velocity_controller.kp * error_e_mps;

    velocity_controller.d_accel_n_mps2 =
        -velocity_controller.kd * accel_meas_n_mps2;
    velocity_controller.d_accel_e_mps2 =
        -velocity_controller.kd * accel_meas_e_mps2;

    // P + I + D 得到限幅和 Slew 处理前的水平 Acceleration 请求。
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

    // 对二维 Acceleration 请求按模长统一限幅，保持原始控制方向。
    Navigation_LimitVector(
        &limited_accel_n_mps2,
        &limited_accel_e_mps2,
        VELOCITY_ACCEL_LIMIT_MPS2);

    /*
     * 饱和误差用于 Back-calculation Anti-windup。
     * 其符号表示实际可输出量相对原始 PID 请求的缺失方向。
     */
    const float saturation_error_n_mps2 =
        limited_accel_n_mps2 -
        velocity_controller.accel_requested_n_mps2;
    const float saturation_error_e_mps2 =
        limited_accel_e_mps2 -
        velocity_controller.accel_requested_e_mps2;

    const bool hard_output_limited =
        fabsf(saturation_error_n_mps2) > 0.0001f ||
        fabsf(saturation_error_e_mps2) > 0.0001f;

    /*
     * 对目标 Acceleration 施加矢量 Slew Rate，
     * 避免 Velocity target 或 GPS 状态变化导致姿态目标瞬间跳变。
     */
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

    // Integral 本身同样按二维模长限幅，避免两轴独立饱和改变补偿方向。
    Navigation_LimitVector(
        &velocity_controller.integral_accel_n_mps2,
        &velocity_controller.integral_accel_e_mps2,
        VELOCITY_INTEGRAL_ACCEL_LIMIT_MPS2);

    const float sy = sinf(yaw_rad);
    const float cy = cosf(yaw_rad);

    // 将导航系 N/E Acceleration 转换到机体系 Forward/Right。
    const float accel_forward_mps2 =
        cy * velocity_controller.accel_target_n_mps2 +
        sy * velocity_controller.accel_target_e_mps2;
    const float accel_right_mps2 =
        -sy * velocity_controller.accel_target_n_mps2 +
        cy * velocity_controller.accel_target_e_mps2;

    /*
     * 水平 Acceleration 已在 N/E 平面按矢量模长统一限幅。
     * 转换为姿态角后只保留最终安全角度限制。
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

/* =========================================================================
 * 位置估计器(Position Estimator)实现
 * ========================================================================= */

/* 检查 E7 经纬度是否处于有效地理范围。 */
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

    /*
     * 建立局部切平面近似。
     * Latitude 每个 E7 单位对应距离近似固定；
     * Longitude 比例根据参考纬度乘 cos(latitude) 修正。
     */
    horizontal_estimator.position_reference_lat_e7 = 
        latitude_e7;
    horizontal_estimator.position_reference_lon_e7 = 
        longitude_e7;

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
    // 只消费新的 Position 样本，并要求局部参考系已经建立。
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

    // Position 样本质量，坐标范围和实际采样周期均需满足融合条件。
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

    // 将当前 GPS 经纬度转换为局部 N/E 平面坐标。
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
        /*
         * Position Innovation 超限时不做渐进修正。
         * 只有上层允许 Reacquire 时才直接将 Position 重新对齐到 GPS。
         */
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

    /*
     * 与 Velocity 修正一致，对单帧融合权限使用受限 Dt；
     * 避免迟到的 GPS Position 获得异常大的状态修正能力。
     */
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
     * 限制单个 GPS Epoch 对 Position 的二维修正量，
     * 防止 GPS 跳点直接导致 Position Error 和姿态指令突变。
     */
    Navigation_LimitVector(
        &position_correction_n_m,
        &position_correction_e_m,
        NAV_POSITION_STATE_CORRECTION_LIMIT_M);

    horizontal_estimator.position_n_m +=
        position_correction_n_m;
    horizontal_estimator.position_e_m +=
        position_correction_e_m;

    /*
     * 当可靠 RMC Vector 已参与 Velocity 修正时，降低 Position Beta 权重；
     * 反之则由 Position 差分承担更多低速方向信息修正。
     */
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

    /*
     * 记录实际应用到 Velocity 状态上的修正量。
     * 若最终 Speed Limit 介入，该值可能小于 Beta 请求。
     */
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

/* =========================================================================
 * 位置控制器(Position Controller)实现
 * ========================================================================= */

/*
 * 清除 Velocity Integral 学习许可的确认状态。
 * current_gps_sequence 作为新的观察起点，避免重复消费旧 GPS 样本。
 */
static void PositionController_ResetIntegralLearningGate(uint32_t current_gps_sequence)
{
    position_controller.integral_learning_last_gps_sequence = current_gps_sequence;
    position_controller.integral_learning_candidate_start_tick_ms = 0U;
    position_controller.integral_learning_valid_sample_count = 0U;
    position_controller.integral_learning_allowed = 0U;
}

void PositionController_Init(void)
{
    PositionController_Reset();
    position_controller.kp = POSITION_KP;
}

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

bool PositionController_Enter(const HorizontalEstimator_t *horizontal_state)
{
    PositionController_Reset();

    if (horizontal_state == NULL ||
        !horizontal_state->initialized ||
        !horizontal_state->position_initialized)
    {
        return false;
    }

    /*
     * 进入 Position Control 时先以当前位置建立临时目标，
     * 并从 BRAKING 开始等待飞行器真正停止后再捕获 HOLD 锚点。
     */
    position_controller.target_n_m = horizontal_state->position_n_m;
    position_controller.target_e_m = horizontal_state->position_e_m;
    position_controller.brake_last_gps_sequence =
        horizontal_state->last_gps_sequence;
    PositionController_ResetIntegralLearningGate(horizontal_state->last_gps_sequence);
    position_controller.phase = POSITION_CONTROL_PHASE_BRAKING;
    position_controller.initialized = 1U;

    return true;
}

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

    /*
     * Position Controller 依赖完整的 Velocity + Position Estimator 状态。
     * 同时检查单轴 Velocity 范围，以拒绝明显异常或非法状态。
     */
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

    // Pilot Velocity 先按二维模长限幅，保持摇杆指令方向。
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
        /*
         * Pilot 主动输入时直接进入 MOVING。
         * 目标点持续跟随当前 Estimator Position，
         * 从而不积累人工移动产生的大量 Position Error。
         */
        position_controller.phase = POSITION_CONTROL_PHASE_MOVING;
        position_controller.brake_elapsed_time_s = 0.0f;
        position_controller.brake_last_gps_sequence =
            horizontal_state->last_gps_sequence;
        position_controller.brake_low_speed_sample_count = 0U;
        PositionController_ResetIntegralLearningGate(horizontal_state->last_gps_sequence);

        position_controller.target_n_m = horizontal_estimator.position_n_m;
        position_controller.target_e_m = horizontal_estimator.position_e_m;
        position_controller.error_n_m = 0.0f;
        position_controller.error_e_m = 0.0f;
        position_controller.velocity_target_n_mps = pilot_velocity_n_mps;
        position_controller.velocity_target_e_mps = pilot_velocity_e_mps;
        return true;
    }

    /*
     * Pilot 松杆后从 MOVING 进入 BRAKING。
     * 此阶段暂不固定 Position 锚点，先等待真实水平速度稳定下降。
     */
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
        /*
         * BRAKING 中目标点继续跟随 Estimator Position，
         * Velocity Target 置零，由 Velocity Controller 负责减速，
         * 避免在尚未停止时建立固定 Position Error。
         */
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

        /*
         * HOLD 捕获条件只在新 GPS Epoch 到达时重新判定，
         * 防止高频 Control Loop 对同一 GPS 数据重复累计确认次数。
         */
        const bool new_gps_sample =
            horizontal_state->last_gps_sequence !=
            position_controller.brake_last_gps_sequence;

        if(new_gps_sample)
        {
            position_controller.brake_last_gps_sequence =
                horizontal_state->last_gps_sequence;

            /*
             * 同时要求：
             * 1. BRAKING 已持续足够时间；
             * 2. RMC 与 Position-window Velocity 观测一致；
             * 3. 本轮 Velocity/Position GPS 样本均被接受；
             * 4. Estimator 与独立观测速度均足够低。
             */
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

    // HOLD 中 Position Error 定义为 Target - Estimator Position。
    position_controller.error_n_m =
        position_controller.target_n_m - horizontal_state->position_n_m;
    position_controller.error_e_m =
        position_controller.target_e_m - horizontal_state->position_e_m;

    const float error_magnitude = sqrtf(
        position_controller.error_n_m * position_controller.error_n_m +
        position_controller.error_e_m * position_controller.error_e_m);

    /*
     * 超大 Position Error 表示局部状态可能已经失效，
     * 不继续输出可能具有危险意义的追赶指令。
     */
    if (error_magnitude > POSITION_MAX_ERROR_M)
    {
        PositionController_ResetIntegralLearningGate(horizontal_state->last_gps_sequence);
        return false;
    }

    /*
     * Velocity Integral 只允许在接近锚点且 Estimator Speed 较低时学习，
     * 快速条件一旦失效，立即撤销已有确认过程。
     */
    const bool integral_fast_context_valid =
        error_magnitude <= POSITION_HOLD_INTEGRAL_POSITION_ERROR_MAX_M &&
        estimated_speed_mps <= POSITION_HOLD_INTEGRAL_EST_SPEED_MAX_MPS;

    if(!integral_fast_context_valid)
    {
        PositionController_ResetIntegralLearningGate(horizontal_state->last_gps_sequence);
    }
    else
    {
        /*
         * 更严格的 Integral 学习条件按 GPS Epoch 判断，
         * 不能以高频控制周期重复累计同一份低频观测。
         */
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
                // 第一条有效样本建立连续稳定窗口的起始时间。
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

                /*
                 * 同时满足最少独立 GPS 样本数和最短持续时间后，
                 * 才认为当前误差足够稳定，可作为稳态扰动交给 Integral 学习。
                 */
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

    /*
     * Position Deadband 在二维误差模长上处理，
     * 超出 Deadband 的部分仍保持原 N/E 方向。
     */
    float correction_error_n_m = 0.0f;
    float correction_error_e_m = 0.0f;

    if (error_magnitude > POSITION_DEADBAND_M)
    {
        const float scale =
            (error_magnitude - POSITION_DEADBAND_M) / error_magnitude;
        correction_error_n_m = position_controller.error_n_m * scale;
        correction_error_e_m = position_controller.error_e_m * scale;
    }

    // Position P 外环将位置误差转换为 Velocity Target。
    position_controller.velocity_target_n_mps =
        position_controller.kp * correction_error_n_m;
    position_controller.velocity_target_e_mps =
        position_controller.kp * correction_error_e_m;

    // 对二维 Velocity Target 统一限幅，保持目标运动方向。
    Navigation_LimitVector(
        &position_controller.velocity_target_n_mps,
        &position_controller.velocity_target_e_mps,
        POSITION_VELOCITY_LIMIT_MPS);

    return true;
}
