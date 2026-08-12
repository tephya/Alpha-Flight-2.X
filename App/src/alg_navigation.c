#include "alg_navigation.h"
#include <stddef.h>
#include <math.h>

/* =========================================================================
 * 常量与宏定义
 * ========================================================================= */

#define NAV_GRAVITY_MPS2 9.80665f
#define NAV_RAD_TO_DEG 57.2957795f

#define NAV_ACCEL_MEASUREMENT_SIGN (-1.0f) /* 当前IMU静止时az约±1g，姿态算法把Accel解释为机体系重力方向。 \
                                            * 对这种约定，线加速度水平分量 = -R_body_to_ned * accel_measurement */
#define NAV_ACCEL_LPF_CUTOFF_HZ 5.0f        // 加速度低通滤波器截止频率(Hz)
#define NAV_ACCEL_LIMIT_MPS2 6.0f           // 单轴加速度限幅(m/s^2)
#define NAV_ACCEL_BIAS_LIMIT_MPS2 1.5f      // 加速度计零偏估值限幅(m/s^2)
#define NAV_ACCEL_BIAS_TIME_CONSTANT_S 5.0f     // 零偏学习的时间常数(s)

/* 水平状态估计器(GPS融合)相关参数 */
#define NAV_GPS_CORRECTION_GAIN 0.35f           // GPS位置/速度修正增益(互补滤波权重)
#define NAV_GPS_INNOVATION_LIMIT_MPS 5.0f       // GPS信息(测量值与估计值偏差)限度，超限则拒绝该次修正
#define NAV_GPS_REACQUIRE_GAP_MS 1500U          // GPS重新捕获的超时时间阈值(ms)
#define NAV_GPS_TIMEOUT_MS 600U                 // GPS健康状态判定超时时间(ms)
#define NAV_ESTIMATED_SPEED_LIMIT_MPS 20.0f     // 估计器输出的绝对速度限速(m/s)

/* 速度控制器(PI控制)相关参数 */
#define VELOCITY_KP 1.20f
#define VELOCITY_KI 0.50f
#define VELOCITY_INEGRAL_ACCEL_LIMIT_MPS2 0.80f     // 积分项加速度限幅(m/s^2)，防止积分饱和
#define VELOCITY_ACCEL_LIMIT_MPS2 2.00f             // 总目标加速度输出限幅
#define VELOCITY_ANGLE_LIMIT_DEG 12.0f              // 输出姿态角(Roll/Pitch)限幅(度)
#define VELOCITY_ANGLE_SLEW_DPS 40.0f               // 目标姿态角的最大变化率限制(度/秒)

/* 位置控制器相关参数 */
#define NAV_EARTH_RADIUS_M 6378137.0f               // WGS84地球赤道半径基准，单位：米
#define NAV_DEG_E7_TO_RAD 1.745329252e-9f           // 经纬度(扩展1e7倍)度数转为弧度的转换常数(PI / 180 / 1e7)

#define POSITION_KP 0.70f                           // 位置控制比例P增益系数
#define POSITION_DEADBAND_M 0.10f                   // 位置死区半径(m)，进入此范围消除静态差补偿，防常态GPS漂移引发的晃动
#define POSITION_PILOT_SPEED_LIMIT_MPS 1.50f        // 外部摇杆干预改变目标点时的最高前馈移速限制(m/s)
#define POSITION_VELOCITY_LIMIT_MPS 1.20f           // 最终输出给内环(速度环)的N/E合成目标速度绝对上限(m/s)
#define POSITION_MAX_ERROR_M 30.0f                  // 单次允许的最大位置误差跟踪阈值(m)

#define POSITION_PILOT_ACTIVE_SPEED_MPS 0.02f
#define POSITION_BRAKE_CAPTURE_SPEED_MPS 0.15f
#define POSITION_BRAKE_CAPTURE_TIME_S 0.50f
#define POSITION_BRAKE_MAX_TIME_S 1.50f

/*
 * 位置估计每个控制周期使用Velocity积分推进；
 * 新GPS位置到达时，只吸收20%的GPS位置innovation，
 * 避免约5Hz原始经纬度阶跃直接形成剧烈倾角指令。 
 */
#define POSITION_GPS_CORRECTION_GAIN 0.20f
#define POSITION_GPS_INNOVATION_LIMIT_M 5.0f

/* alpha-beta融合中的beta项：
 * GPS Position innovation除修正Position外，也要小幅修正Velocity，
 * 避免“位置认为仍在远离目标，速度却认为正在返回”。
 * 0.02配合约5Hz GPS只产生温和的Velocity约束，不会直接追踪单帧位置噪声。 */
#define POSITION_GPS_VELOCITY_CORRECTION_GAIN 0.02f

/* 仅当GPS样本间隔可信时才计算innovation / dt
 * 防止重复样本或超时恢复样本制造Velocity尖峰。 */
#define POSITION_GPS_SAMPLE_DT_MIN_S 0.05f
#define POSITION_GPS_SAMPLE_DT_MAX_S 0.60f
#define POSITION_GPS_SAMPLE_ELAPSED_LIMIT_S 2.0f

/* 单帧GPS Position最多只能把融合Velocity改变0.20m/s
 * 该限幅保护的是位置噪声对速度状态的瞬时感染，
 * 不改变原有的Horizontal Estimator的总速度上限 */
#define POSITION_GPS_VELOCITY_CORRECTION_LIMIT_MPS 0.20f

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
    if(value < min_value)
        return min_value;
    if(value > max_value)
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

    if(magnitude > limit && magnitude > 0.0001f)
    {
        const float scale = limit / magnitude;
        *x *= scale;
        *y *= scale;
    }
}

/**
 * @brief 限制数值的变化率(Slew rate limiter)
 * @param current 当前值
 * @param target 期望目标值
 * @param max_rate_per_s 每秒最大允许变化量
 * @param dt 控制周期(s)
 * @return 经过斜率限制后的下一时刻值
 */
static float Navigation_Slew(float current,
                             float target,
                             float max_rate_per_s,
                             float dt)
{
    const float max_step = max_rate_per_s * dt;
    const float error = target - current;

    // 如果目标值与当前值的偏差超过了单次步长允许的最大范围，则只走最大补偿
    if(error > max_step)
        return current + max_step;
    if(error < -max_step)
        return current - max_step;
    return target;
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
    horizontal_estimator.accel_bias_n_mps2 = 0.0f;
    horizontal_estimator.accel_bias_e_mps2 = 0.0f;
    horizontal_estimator.last_gps_sequence = 0U;
    horizontal_estimator.last_gps_tick_ms = 0U;
    horizontal_estimator.gps_accept_count = 0U;
    horizontal_estimator.gps_reject_count = 0U;
    horizontal_estimator.initialized = 0U;
    horizontal_estimator.gps_healthy = 0U;
    horizontal_estimator.last_gps_accepted = 0U;
}

/**
 * @brief 估计器预测步骤(基于高频IMU数据)
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
    if(dt < 0.0005f || dt > 0.0050f)
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
    const float tau = 1.0f / (6.2831853f * NAV_ACCEL_LPF_CUTOFF_HZ);    // 6.28... 为 2*PI
    const float alpha = dt / (tau + dt);

    horizontal_estimator.accel_lpf_n_mps2 +=
        alpha * (accel_n_raw - horizontal_estimator.accel_lpf_n_mps2);
    horizontal_estimator.accel_lpf_e_mps2 +=
        alpha * (accel_e_raw - horizontal_estimator.accel_lpf_e_mps2);

    // 动态学习并消除加速度计在水平方向上的恒定零偏(通常由于标定误差或温度漂移引起)
    if(learn_accel_bias)
    {
        const float bias_alpha = dt / (NAV_ACCEL_BIAS_TIME_CONSTANT_S + dt);

        horizontal_estimator.accel_bias_n_mps2 +=
            bias_alpha * (horizontal_estimator.accel_lpf_n_mps2 -
                          horizontal_estimator.accel_bias_n_mps2);

        horizontal_estimator.accel_bias_e_mps2 +=
            bias_alpha * (horizontal_estimator.accel_lpf_e_mps2 -
                          horizontal_estimator.accel_bias_e_mps2);

        // 限制零偏估值的最大范围，防止误将长时间机动识别为零偏
        horizontal_estimator.accel_bias_n_mps2 = Navigation_Clamp(
            horizontal_estimator.accel_bias_n_mps2, -NAV_ACCEL_BIAS_LIMIT_MPS2, NAV_ACCEL_BIAS_LIMIT_MPS2);
        horizontal_estimator.accel_bias_e_mps2 = Navigation_Clamp(
            horizontal_estimator.accel_bias_e_mps2, -NAV_ACCEL_BIAS_LIMIT_MPS2, NAV_ACCEL_BIAS_LIMIT_MPS2);
    }

    // 得到最终用于速度预测的纯净运动加速度
    horizontal_estimator.accel_n_mps2 =
        horizontal_estimator.accel_lpf_n_mps2 - horizontal_estimator.accel_bias_n_mps2;
    horizontal_estimator.accel_e_mps2 =
        horizontal_estimator.accel_lpf_e_mps2 - horizontal_estimator.accel_bias_e_mps2;

#if HORIZONTAL_ESTIMATOR_IMU_PREDICTION_ENABLED
    // 若系统已通过GPS初始化，则利用加速度进行航位推算(积分)预测当前速度
    if(horizontal_estimator.initialized)
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
}

/**
 * @brief 估计器修正步骤 (基于低频但长期的GPS绝对测量值)
 * @return 是否成功应用了GPS修正
 */
bool HorizontalEstimator_CorrectGps(float gps_velocity_n_mps,
                                    float gps_velocity_e_mps,
                                    uint32_t gps_sequence,
                                    uint32_t gps_tick_ms,
                                    bool sample_valid)
{
    // 利用序列号检查是否为重复的旧数据
    if(gps_sequence == 0U ||
        gps_sequence == horizontal_estimator.last_gps_sequence)
    {
        return false;
    }

    horizontal_estimator.last_gps_sequence = gps_sequence;
    horizontal_estimator.last_gps_accepted = 0U;

    const float gps_speed = sqrtf(gps_velocity_n_mps * gps_velocity_n_mps +
                                  gps_velocity_e_mps * gps_velocity_e_mps);
    
    /* 检查GPS数据的有效性。
     * 与NaN比较的结果必定为false，因此该范围判断同时拒绝了NaN和Inf等非法浮点数。 */
    if(!sample_valid || !(gps_speed >= 0.0f && gps_speed <= 30.0f))
    {
        horizontal_estimator.gps_reject_count++;
        horizontal_estimator.gps_healthy = 0U;
        return false;
    }

    const uint32_t gps_ms = gps_tick_ms - horizontal_estimator.last_gps_tick_ms;

    // 检查是否需要重新初始化融合滤波器
    if(!horizontal_estimator.initialized ||
        horizontal_estimator.last_gps_tick_ms == 0U ||
        gps_ms > NAV_GPS_REACQUIRE_GAP_MS)
    {
        /* 首帧或长时间失锁后的第一帧直接对齐GPS，避免拿旧速度缓慢追赶。 */
        horizontal_estimator.velocity_n_mps = gps_velocity_n_mps;
        horizontal_estimator.velocity_e_mps = gps_velocity_e_mps;
        horizontal_estimator.initialized = 1U;
    }
    else
    {
        // 计算新息(Innovation): GPS观测速度与当前预测速度的误差
        const float innovation_n =
            gps_velocity_n_mps - horizontal_estimator.velocity_n_mps;
        const float innovation_e =
            gps_velocity_e_mps - horizontal_estimator.velocity_e_mps;
        const float innovation =
            sqrtf(innovation_n * innovation_n + innovation_e * innovation_e);

        // 如果误差过大(可能发生GPS阶跃/多径突变)，则拒绝本次融合
        if(innovation > NAV_GPS_INNOVATION_LIMIT_MPS)
        {
            horizontal_estimator.gps_reject_count++;
            horizontal_estimator.gps_healthy = 0U;
            return false;
        }

        // 应用修正：基于增益(Gain)按比例修正当前预估速度
        horizontal_estimator.velocity_n_mps +=
            NAV_GPS_CORRECTION_GAIN * innovation_n;
        horizontal_estimator.velocity_e_mps +=
            NAV_GPS_CORRECTION_GAIN * innovation_e;
    }

    horizontal_estimator.last_gps_tick_ms = gps_tick_ms;
    horizontal_estimator.gps_accept_count++;
    horizontal_estimator.gps_healthy = 1U;
    horizontal_estimator.last_gps_accepted = 1U;
    return true;
}

void HorizontalEstimator_UpdateHealth(uint32_t now_ms)
{
    if(!horizontal_estimator.initialized ||
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

    velocity_controller.accel_target_n_mps2 = 0.0f;
    velocity_controller.accel_target_e_mps2 = 0.0f;
    velocity_controller.roll_target_deg = 0.0f;
    velocity_controller.pitch_target_deg = 0.0f;
}

/**
 * @brief 
 */
void VelocityController_Update(float velocity_target_n_mps,
                               float velocity_target_e_mps,
                               float velocity_meas_n_mps,
                               float velocity_meas_e_mps,
                               float yaw_rad,
                               float dt)
{
    if(dt < 0.0005f || dt > 0.0050f)
        return;

    // 计算速度误差
    const float error_n = velocity_target_n_mps - velocity_meas_n_mps;
    const float error_e = velocity_target_e_mps - velocity_meas_e_mps;

    // 计算暂定输出：P项 + 当前时刻的I项，用于后续的抗积分饱和判定
    float pre_accel_n =
        velocity_controller.kp * error_n +
        velocity_controller.integral_accel_n_mps2;
    float pre_accel_e =
        velocity_controller.kp * error_e +
        velocity_controller.integral_accel_e_mps2;

    const float pre_magnitude =
        sqrtf(pre_accel_n * pre_accel_n + pre_accel_e * pre_accel_e);

    /* 抗积分饱和机制(Anti-windup):
     * integral_dot_error 是当前积分量与当前误差的内积。
     * 当 Integral_dot_error < 0 时，说明误差符号与积分项符号相反（即误差正试图卸载现有的积分值）。
     * 策略：未饱和时正常积分；控制量饱和时，只允许反向误差卸载已有Integral，禁止继续朝着饱和方向积累。 */
    const float integral_dot_error =
        velocity_controller.integral_accel_n_mps2 * error_n +
        velocity_controller.integral_accel_e_mps2 * error_e;

    if(pre_magnitude < VELOCITY_ACCEL_LIMIT_MPS2 || integral_dot_error < 0.0f)
    {
        velocity_controller.integral_accel_n_mps2 +=
            velocity_controller.ki * error_n * dt;
        velocity_controller.integral_accel_e_mps2 +=
            velocity_controller.ki * error_e * dt;

        // 独立限制积分项的最大权重，防止长时间小误差导致积分爆表
        Navigation_LimitVector(
            &velocity_controller.integral_accel_n_mps2,
            &velocity_controller.integral_accel_e_mps2,
            VELOCITY_INEGRAL_ACCEL_LIMIT_MPS2);
    }

    // 计算最终的 NED 坐标系下的目标加速度
    velocity_controller.accel_target_n_mps2 =
        velocity_controller.kp * error_n +
        velocity_controller.integral_accel_n_mps2;
    velocity_controller.accel_target_e_mps2 =
        velocity_controller.kp * error_e +
        velocity_controller.integral_accel_e_mps2;

    // 限制总体目标加速度
    Navigation_LimitVector(&velocity_controller.accel_target_n_mps2,
                           &velocity_controller.accel_target_e_mps2,
                           VELOCITY_ACCEL_LIMIT_MPS2);

    const float sy = sinf(yaw_rad);
    const float cy = cosf(yaw_rad);

    /* 坐标系转换：利用航向角，将大地N/E坐标系下的目标加速度，旋转到机体坐标系的 前/右 轴上。 */
    const float accel_forward =
        cy * velocity_controller.accel_target_n_mps2 +
        sy * velocity_controller.accel_target_e_mps2;
    const float accel_right =
        -sy * velocity_controller.accel_target_n_mps2 +
        cy * velocity_controller.accel_target_e_mps2;

    /* 物理模型转化：期望加速度 -> 期望欧拉角
     * 飞机通过倾斜自身，利用升力(抵抗重力的力)的水平分量来产水水平加速度 。
     * a = g * tan(theta) => theta = atan(a/g)
     * 注意：机头下沉(Pitch为负)产生向前的加速度，机身右倾(Roll为正)产生向右的加速度。*/
    float desired_pitch_deg =
        -atanf(accel_forward / NAV_GRAVITY_MPS2) * NAV_RAD_TO_DEG;
    float desired_roll_deg =
        atanf(accel_right / NAV_GRAVITY_MPS2) * NAV_RAD_TO_DEG;

    // 限制单轴最终输出的最大倾斜角限制
    desired_roll_deg = Navigation_Clamp(desired_roll_deg,
                                        -VELOCITY_ANGLE_LIMIT_DEG,
                                        VELOCITY_ANGLE_LIMIT_DEG);
    desired_pitch_deg = Navigation_Clamp(desired_pitch_deg,
                                        -VELOCITY_ANGLE_LIMIT_DEG,
                                        VELOCITY_ANGLE_LIMIT_DEG);

    // 平滑输出(Slew rate limiter)，防止指令突变引起机体剧烈震荡
    velocity_controller.roll_target_deg = Navigation_Slew(
        velocity_controller.roll_target_deg,
        desired_roll_deg,
        VELOCITY_ANGLE_SLEW_DPS,
        dt);
    velocity_controller.pitch_target_deg = Navigation_Slew(
        velocity_controller.pitch_target_deg,
        desired_pitch_deg,
        VELOCITY_ANGLE_SLEW_DPS,
        dt);
}

/**
 * @brief 校验传入的经纬度坐标是否在合法地球标注范围内
 * @param latitude_e7 纬度坐标(° * 1e7)
 * @param longitude_e7 经度坐标(° * 1e7)
 * @return true=坐标符合[-90,90]及[-180,180]规范；false=越界非法数据
 */
static bool PositionController_CoordinateValid(int32_t latitude_e7,
                                              int32_t longitude_e7)
{
    return latitude_e7 >= -900000000 &&
           latitude_e7 <= 900000000 &&
           longitude_e7 >= -1800000000 &&
           longitude_e7 <= 1800000000;
}

/**
 * @brief   用GPS Position innovation约束水平Velocity状态。
 * 
 * @param[in,out]   horizontal_state    待修正的水平状态估计器
 * @param[in]   innovation_n_m  GPS北向位置测量 - 预测位置(m)
 * @param[in]   innovation_e_m  GPS东向位置测量 - 预测位置(m)
 * @param[in]   gps_sample_dt_s 前后两条新RMC样本的时间间隔
 * 
 * @note    alpha-beta关系中，Velocity修正量为 beta * position_innovation / dt。
 *          修正量先单帧限幅，再对最终Velocity做原有总速度限幅。
 */
static void PositionController_CorrectVelocity(HorizontalEstimator_t *horizontal_state,
                                               float innovation_n_m,
                                               float innovation_e_m,
                                               float gps_sample_dt_s)
{
    position_controller.velocity_correction_n_mps = 0.0f;
    position_controller.velocity_correction_e_mps = 0.0f;
    position_controller.last_velocity_correction_applied = 0U;

    if(horizontal_state == NULL ||
        !horizontal_state->initialized ||
        gps_sample_dt_s < POSITION_GPS_SAMPLE_DT_MIN_S ||
        gps_sample_dt_s > POSITION_GPS_SAMPLE_DT_MAX_S)
    {
        return;
    }

    float correction_n_mps =
        POSITION_GPS_VELOCITY_CORRECTION_GAIN * innovation_n_m / gps_sample_dt_s;
    float correction_e_mps =
        POSITION_GPS_VELOCITY_CORRECTION_GAIN * innovation_e_m / gps_sample_dt_s;

    Navigation_LimitVector(
        &correction_n_mps,
        &correction_e_mps,
        POSITION_GPS_VELOCITY_CORRECTION_LIMIT_MPS);

    const float velocity_before_n_mps = horizontal_state->velocity_n_mps;
    const float velocity_before_e_mps = horizontal_state->velocity_e_mps;

    horizontal_state->velocity_n_mps += correction_n_mps;
    horizontal_state->velocity_e_mps += correction_e_mps;

    Navigation_LimitVector(
        &horizontal_state->velocity_n_mps,
        &horizontal_state->velocity_e_mps,
        NAV_ESTIMATED_SPEED_LIMIT_MPS);
    
    /* 记录限幅后真正进入Estimator的修正量，
     * 便于后续黑匣子诊断。 */
    position_controller.velocity_correction_n_mps = 
        horizontal_state->velocity_n_mps - velocity_before_n_mps;
    position_controller.velocity_correction_e_mps =
        horizontal_state->velocity_e_mps - velocity_before_e_mps;
    position_controller.velocity_correction_count++;
    position_controller.last_velocity_correction_applied = 1U;
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

    position_controller.reference_lat_e7 = 0;
    position_controller.reference_lon_e7 = 0;
    position_controller.meter_per_lat_e7 = 0.0f;
    position_controller.meter_per_lon_e7 = 0.0f;

    position_controller.position_n_m = 0.0f;
    position_controller.position_e_m = 0.0f;
    position_controller.gps_position_n_m = 0.0f;
    position_controller.gps_position_e_m = 0.0f;

    position_controller.target_n_m = 0.0f;
    position_controller.target_e_m = 0.0f;
    position_controller.error_n_m = 0.0f;
    position_controller.error_e_m = 0.0f;

    position_controller.velocity_target_n_mps = 0.0f;
    position_controller.velocity_target_e_mps = 0.0f;

    position_controller.brake_stable_time_s = 0.0f;
    position_controller.brake_elapsed_time_s = 0.0f;
    position_controller.phase = POSITION_CONTROL_PHASE_INACTIVE;

    position_controller.gps_sample_elapsed_s = 0.0f;
    position_controller.velocity_correction_n_mps = 0.0f;
    position_controller.velocity_correction_e_mps = 0.0f;

    position_controller.last_gps_sequence = 0U;
    position_controller.gps_correction_count = 0U;
    position_controller.gps_reject_count = 0U;
    position_controller.velocity_correction_count = 0U;

    position_controller.initialized = 0U;
    position_controller.last_gps_accepted = 0U;
    position_controller.last_gps_rejected = 0U;
    position_controller.last_velocity_correction_applied = 0U;
}

/**
 * @brief 锁定当前坐标系原点，使系统进入局部定点控制模式
 * @param latitude_e7 锚点纬度坐标
 * @param longitude_e7 锚点经度坐标
 * @return true=进入成功并建立局部切面系；false=坐标非法拒绝进入
 */
bool PositionController_Enter(int32_t latitude_e7,
                              int32_t longitude_e7,
                              uint32_t gps_sequence)
{
    PositionController_Reset();

    if(gps_sequence == 0U ||
        !PositionController_CoordinateValid(latitude_e7, longitude_e7))
    {
        return false;
    }

    const float reference_lat_rad =
        (float)latitude_e7 * NAV_DEG_E7_TO_RAD;

    position_controller.reference_lat_e7 = latitude_e7;
    position_controller.reference_lon_e7 = longitude_e7;

    /* 建立从经纬度度数转为平面米单位的换算系数。
     * 纬度转换保持恒定；经度换算需要结合当前纬度角的余弦值缩小圈长。 */
    position_controller.meter_per_lat_e7 =
        NAV_EARTH_RADIUS_M * NAV_DEG_E7_TO_RAD;
    position_controller.meter_per_lon_e7 =
        position_controller.meter_per_lat_e7 * cosf(reference_lat_rad);

    /*
     * 进入模式时当前位置即为局部原点和初始目标。
     * 记录当前sequence，避免同一个GPS样本被进入函数和Update重复处理。 
     */
    position_controller.last_gps_sequence = gps_sequence;
    position_controller.initialized = 1U;
    position_controller.brake_stable_time_s = 0.0f;
    position_controller.brake_elapsed_time_s = 0.0f;
    position_controller.phase = POSITION_CONTROL_PHASE_BRAKING;

    return true;
}

/**
 * @brief 位置控制器核心运算逻辑更新(外环)
 */
bool PositionController_Update(int32_t latitude_e7,
                               int32_t longitude_e7,
                               uint32_t gps_sequence,
                               HorizontalEstimator_t *horizontal_state,
                               float pilot_velocity_n_mps,
                               float pilot_velocity_e_mps,
                               float dt)
{
    if(horizontal_state == NULL)
        return false;

    const float velocity_est_n_mps = horizontal_state->velocity_n_mps;
    const float velocity_est_e_mps = horizontal_state->velocity_e_mps;

    const bool velocity_valid =
        horizontal_state->initialized &&
        velocity_est_n_mps >= -NAV_ESTIMATED_SPEED_LIMIT_MPS &&
        velocity_est_n_mps <= NAV_ESTIMATED_SPEED_LIMIT_MPS &&
        velocity_est_e_mps >= -NAV_ESTIMATED_SPEED_LIMIT_MPS &&
        velocity_est_e_mps <= NAV_ESTIMATED_SPEED_LIMIT_MPS;

    if(!position_controller.initialized ||
        gps_sequence == 0U ||
        dt < 0.0005f || 
        dt > 0.0050f ||
        !velocity_valid ||
        !PositionController_CoordinateValid(latitude_e7, longitude_e7))
    {
        return false;
    }

    /* 累计距离上一条新RMC的时间。
     * 只限制内部计时器的最大值，不会把超时样本伪装成正常dt。 */
    position_controller.gps_sample_elapsed_s += dt;
    if(position_controller.gps_sample_elapsed_s > 
        POSITION_GPS_SAMPLE_ELAPSED_LIMIT_S)
    {
        position_controller.gps_sample_elapsed_s =
            POSITION_GPS_SAMPLE_ELAPSED_LIMIT_S;
    }

    /* 高频预测：使用同一份融合Velocity推进连续Position。 */
    position_controller.position_n_m += velocity_est_n_mps * dt;
    position_controller.position_e_m += velocity_est_e_mps * dt;

    /* 低频GPS联合修正：统一RMC sequence只处理一次。 */
    if(gps_sequence != position_controller.last_gps_sequence)
    {
        const float gps_sample_dt_s =
            position_controller.gps_sample_elapsed_s;

        position_controller.gps_sample_elapsed_s = 0.0f;
        position_controller.last_gps_sequence = gps_sequence;
        position_controller.last_gps_accepted = 0U;
        position_controller.last_gps_rejected = 0U;
        position_controller.velocity_correction_n_mps = 0.0f;
        position_controller.velocity_correction_e_mps = 0.0f;
        position_controller.last_velocity_correction_applied = 0U;

        // 利用于参考点的差值和换算系数，算出在局部坐标系中的平面坐标（单位：米）
        const int64_t delta_lat_e7 =
            (int64_t)latitude_e7 - position_controller.reference_lat_e7;
        const int64_t delta_lon_e7 =
            (int64_t)longitude_e7 - position_controller.reference_lon_e7;

        position_controller.gps_position_n_m =
            (float)delta_lat_e7 * position_controller.meter_per_lat_e7;
        position_controller.gps_position_e_m =
            (float)delta_lon_e7 * position_controller.meter_per_lon_e7;

        const float innovation_n_m =
            position_controller.gps_position_n_m - position_controller.position_n_m;
        const float innovation_e_m =
            position_controller.gps_position_e_m - position_controller.position_e_m;

        const float innovation_magnitude =
            sqrtf(innovation_n_m * innovation_n_m +
                  innovation_e_m * innovation_e_m);

        if(innovation_magnitude <= POSITION_GPS_INNOVATION_LIMIT_M)
        {
            /* alpha项：修正Position。 */
            position_controller.position_n_m +=
                POSITION_GPS_CORRECTION_GAIN * innovation_n_m;
            position_controller.position_e_m +=
                POSITION_GPS_CORRECTION_GAIN * innovation_e_m;

            /* beta项：用同一个innovation约束Velocity。
             * 这使Position外环和Velocity内环对“正在往哪里移动”
             * 使用同一套约束的状态 */
            PositionController_CorrectVelocity(
                horizontal_state,
                innovation_n_m,
                innovation_e_m,
                gps_sample_dt_s);

            position_controller.gps_correction_count++;
            position_controller.last_gps_accepted = 1U;
        }
        else
        {
            /* 拒绝位置跳变时，Position和Velocity都不能吸收该innovation，
             * 否则两个状态会再次失去一致性。 */
            position_controller.gps_reject_count++;
            position_controller.last_gps_rejected = 1U;
        }
    }

    // 限制摇杆目标速度的二维模长，避免对角打杆超过单轴约定速度。
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

    if(pilot_active)
    {
        position_controller.phase = POSITION_CONTROL_PHASE_MOVING;
        position_controller.brake_stable_time_s = 0.0f;
        position_controller.brake_elapsed_time_s = 0.0f;

        position_controller.target_n_m = position_controller.position_n_m;
        position_controller.target_e_m = position_controller.position_e_m;
        position_controller.error_n_m = 0.0f;
        position_controller.error_e_m = 0.0f;

        position_controller.velocity_target_n_mps = pilot_velocity_n_mps;
        position_controller.velocity_target_e_mps = pilot_velocity_e_mps;
        return true;
    }

    if(position_controller.phase == POSITION_CONTROL_PHASE_MOVING ||
        position_controller.phase == POSITION_CONTROL_PHASE_INACTIVE)
    {
        position_controller.phase = POSITION_CONTROL_PHASE_BRAKING;
        position_controller.brake_stable_time_s = 0.0f;
        position_controller.brake_elapsed_time_s = 0.0f;
    }

    if(position_controller.phase == POSITION_CONTROL_PHASE_BRAKING)
    {
        position_controller.target_n_m = position_controller.position_n_m;
        position_controller.target_e_m = position_controller.position_e_m;
        position_controller.error_n_m = 0.0f;
        position_controller.error_e_m = 0.0f;
        position_controller.velocity_target_n_mps = 0.0f;
        position_controller.velocity_target_e_mps = 0.0f;

        position_controller.brake_elapsed_time_s += dt;
        if(position_controller.brake_elapsed_time_s > 
            POSITION_BRAKE_MAX_TIME_S)
        {
            position_controller.brake_elapsed_time_s = POSITION_BRAKE_MAX_TIME_S;
        }

        if(estimated_speed_mps <= POSITION_BRAKE_CAPTURE_SPEED_MPS)
        {
            position_controller.brake_stable_time_s += dt;

            if(position_controller.brake_stable_time_s >= 
                POSITION_BRAKE_CAPTURE_TIME_S)
            {
                position_controller.brake_stable_time_s = POSITION_BRAKE_CAPTURE_TIME_S;
            }
        }
        else
        {
            position_controller.brake_stable_time_s = 0.0f;
        }

        const bool speed_capture_ready =
            position_controller.brake_stable_time_s >=
            POSITION_BRAKE_CAPTURE_TIME_S;

        const bool brake_timeout =
            position_controller.brake_elapsed_time_s >=
            POSITION_BRAKE_MAX_TIME_S;

        if(speed_capture_ready || brake_timeout)
        {
            position_controller.phase = POSITION_CONTROL_PHASE_HOLD;
            position_controller.target_n_m =
                position_controller.position_n_m;
            position_controller.target_e_m =
                position_controller.position_e_m;
        }

        return true;
    }

    if(position_controller.phase != POSITION_CONTROL_PHASE_HOLD)
    {
        return false;
    }

    position_controller.error_n_m =
        position_controller.target_n_m - position_controller.position_n_m;
    position_controller.error_e_m =
        position_controller.target_e_m - position_controller.position_e_m;

    const float error_magnitude = sqrtf(
        position_controller.error_n_m * position_controller.error_n_m +
        position_controller.error_e_m * position_controller.error_e_m);

    if(error_magnitude > POSITION_MAX_ERROR_M)
        return false;

    float correction_error_n_m = 0.0f;
    float correction_error_e_m = 0.0f;

    /* 径向deadband：消除误差模长的前POSITION_DEADBAND_M。
     * 避免GPS单次小幅跳动直接形成位置修正。 */
    if(error_magnitude > POSITION_DEADBAND_M)
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

    // 对输出速度指令进行整体二维模长限幅保护
    Navigation_LimitVector(
        &position_controller.velocity_target_n_mps,
        &position_controller.velocity_target_e_mps,
        POSITION_VELOCITY_LIMIT_MPS);

    return true;
}
