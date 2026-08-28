/**
 * @file    alg_controller.c
 * @brief   姿态角度控制、角速度控制器及遥控通道映射实现。
 */

#include "alg_controller.h"
#include <math.h>

/**
 * Yaw Rate 积分增长抑制区间。
 * 当前 Heading Hold 外环 Kp = 1.0，因此航向误差数值约等于目标角速度。
 * 大机动时逐步限制积分继续增长，但仍允许已有积分卸载。
 */
#define YAW_RATE_I_RELAX_START_DPS 5.0f
#define YAW_RATE_I_RELAX_END_DPS 20.0f

/** Roll/Pitch Rate Feedforward 增益及输出限幅。 */
#define ROLL_RATE_FF_GAIN 0.50f
#define PITCH_RATE_FF_GAIN 0.40f
#define RATE_FF_LIMIT 10.0f

/* 遥控器归一化通道范围。 */
#define RC_NORMALIZED_MIN 172U
#define RC_NORMALIZED_MID 992U
#define RC_NORMALIZED_MAX 1811U

/** 全局姿态角度控制器实例。 */
AngleController_t angle_controller;

/** 全局角速度控制器实例。 */
RateController_t rate_controller;

/** Roll/Pitch Rate Feedforward 启用状态。 */
static uint8_t s_roll_pitch_rate_ff_enabled;

/**
 * @brief   将数值限制在对称区间 [-limit, limit]。
 */
static float RateController_Limit(float value, float limit)
{
    if(value > limit)
        return limit;
    if(value < -limit)
        return -limit;

    return value;
}

/**
 * @brief   根据 Yaw 目标角速度计算积分增长缩放系数。
 * 
 * 小目标角速度下允许积分正常增长；随着目标角速度增大，
 * 逐步降低积分增长速度，避免大机动期间产生过多积分累积。
 * 
 * @param[in]   yaw_rate_target Yaw 目标角速度，deg/s。
 * @return  积分增长缩放系数，范围 [0.0, 1.0]。
 */
static float RateController_GetYawIntegralGrowthScale(float yaw_rate_target)
{
    const float target_abs = fabsf(yaw_rate_target);

    if(target_abs <= YAW_RATE_I_RELAX_START_DPS)
        return 1.0f;

    if(target_abs >= YAW_RATE_I_RELAX_END_DPS)
        return 0.0f;

    return (YAW_RATE_I_RELAX_END_DPS - target_abs) /
           (YAW_RATE_I_RELAX_END_DPS - YAW_RATE_I_RELAX_START_DPS);
}

/**
 * @brief   将带中位的遥控器通道映射为对称输出量。
 * 
 * 中位对应 0，两端分别对应 -magnitude 和 +magnitude。
 * 
 * @param[in]   ch  遥控器原始通道值。
 * @param[in]   magnitude   输出最大绝对值。
 * @return  映射结果，范围 [-magnitude,magnitude]。
 */
static float Map_CenteredChannel(uint16_t ch, float magnitude)
{
    if (ch >= RC_NORMALIZED_MAX)
        return magnitude;
    if (ch <= RC_NORMALIZED_MIN)
        return -magnitude;
    if (ch == RC_NORMALIZED_MID)
        return 0.0f;

    if (ch > RC_NORMALIZED_MID)
    {
        return (float)(ch - RC_NORMALIZED_MID) /
               (float)(RC_NORMALIZED_MAX - RC_NORMALIZED_MID) * magnitude;
    }

    return -(float)(RC_NORMALIZED_MID - ch) /
           (float)(RC_NORMALIZED_MID - RC_NORMALIZED_MIN) * magnitude;
}

/*============================= 姿态角度控制器（外环） ==============================*/

void AngleController_Init(void)
{
    PID_Init(&angle_controller.roll, 4.20f, 0.0f, 0.0f, 250.0f);
    PID_Init(&angle_controller.pitch, 4.20f, 0.0f, 0.0f, 250.0f);

    angle_controller.roll_rate_target = 0.0f;
    angle_controller.pitch_rate_target = 0.0f;
}

void AngleController_Reset(void)
{
    PID_Reset(&angle_controller.roll);
    PID_Reset(&angle_controller.pitch);

    angle_controller.roll_rate_target = 0.0f;
    angle_controller.pitch_rate_target = 0.0f;
}

void AngleController_Update(float roll_target,
                            float pitch_target,
                            float roll_meas,
                            float pitch_meas,
                            float dt)
{
    PID_SetTarget(&angle_controller.roll, roll_target);
    PID_SetTarget(&angle_controller.pitch, pitch_target);

    angle_controller.roll_rate_target = PID_Update(&angle_controller.roll, roll_meas, dt);
    angle_controller.pitch_rate_target = PID_Update(&angle_controller.pitch, pitch_meas, dt);
}

/*============================= 角速度控制器（内环） ============================*/

void RateController_Init(void)
{
    PID_Init(&rate_controller.roll,  1.30f, 0.10f, 0.000f, 120.0f);
    PID_Init(&rate_controller.pitch, 1.25f, 0.10f, 0.000f, 120.0f);
    PID_Init(&rate_controller.yaw,   2.00f, 0.15f, 0.000f, 200.0f);
}

void RateController_Reset(void)
{
    PID_Reset(&rate_controller.roll);
    PID_Reset(&rate_controller.pitch);
    PID_Reset(&rate_controller.yaw);

    rate_controller.roll_output = 0.0f;
    rate_controller.pitch_output = 0.0f;
    rate_controller.yaw_output = 0.0f;

    s_roll_pitch_rate_ff_enabled = 0U;
}

void RateController_Update(float roll_rate_target,
                            float pitch_rate_target,
                            float yaw_rate_target,
                            float roll_rate,
                            float pitch_rate,
                            float yaw_rate,
                            float dt)
{
    PID_SetTarget(&rate_controller.roll, roll_rate_target);
    PID_SetTarget(&rate_controller.pitch, pitch_rate_target);
    PID_SetTarget(&rate_controller.yaw, yaw_rate_target);

    /*
     * Rate Feedforward 根据目标角速度直接产生即使控制量，
     * 避免已有 Rate Integral 抵消新产生的 Position Hold 纠偏指令。
     * Feedforward 只改善响应建立速度，闭环误差仍由 Rate PID 修正。
     */
    if(s_roll_pitch_rate_ff_enabled != 0U)
    {
        rate_controller.roll.ff = RateController_Limit(
            roll_rate_target * ROLL_RATE_FF_GAIN,
            RATE_FF_LIMIT);

        rate_controller.pitch.ff = RateController_Limit(
            pitch_rate_target * PITCH_RATE_FF_GAIN,
            RATE_FF_LIMIT);
    }
    else
    {
        rate_controller.roll.ff = 0.0f;
        rate_controller.pitch.ff = 0.0f;
    }

    rate_controller.roll_output = PID_Update(&rate_controller.roll, roll_rate, dt);
    rate_controller.pitch_output = PID_Update(&rate_controller.pitch, pitch_rate, dt);

    // 大 Yaw 指令下限制积分继续增长，降低机动结束后的过冲风险。
    const float yaw_integral_growth_scale =
        RateController_GetYawIntegralGrowthScale(yaw_rate_target);

    rate_controller.yaw_output = PID_UpdateWithIntegralScale(
        &rate_controller.yaw,
        yaw_rate,
        dt,
        yaw_integral_growth_scale);
}

void RateController_SetRollPitchFeedForwardEnabled(uint8_t enabled)
{
    s_roll_pitch_rate_ff_enabled = (enabled != 0U) ? 1U : 0U;
}

/*================================ 航向锁定 ===================================*/

float YawHeadingHold_Update(PID_t *pid_yaw,
                            float yaw_target_deg,
                            float yaw_meas_deg,
                            float dt)
{
    float yaw_err = yaw_target_deg - yaw_meas_deg;
    
    // 将航向误差限制到 [-180, 180] deg，使控制始终选择最短旋转方向。
    if(yaw_err > 180.0f)
        yaw_err -= 360.0f;
    if(yaw_err < -180.0f)
        yaw_err += 360.0f;

    PID_SetTarget(pid_yaw, 0.0f);

    return PID_Update(pid_yaw, -yaw_err, dt);
}

/*================================ 遥控映射 ===================================*/

float Map_Roll(uint16_t ch)
{
    return Map_CenteredChannel(ch, 30.0f);
}

float Map_Pitch(uint16_t ch)
{
    return -Map_CenteredChannel(ch, 30.0f);
}

float Map_Yaw(uint16_t ch)
{
    return -Map_CenteredChannel(ch, 90.0f);
}

uint16_t Map_Throttle(uint16_t ch)
{
    if(ch <= 172U)
        return 0U;
    
    if(ch >= 1811U)
        return (uint16_t)ALG_THROTTLE_MAX;

    return (uint16_t)((float)((ch - 172) / 1639.0f) 
                        * ALG_THROTTLE_MAX);
}
