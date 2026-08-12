#include "alg_controller.h"
#include <math.h>

/* 当前Heading Hold外环Kp为1.0，因此5°航向误差对应5°/s目标。
 * 超过此范围时，Yaw Rate I只允许卸载，不允许继续增大。 */
#define YAW_RATE_I_RELAX_START_DPS 5.0f
#define YAW_RATE_I_RELAX_END_DPS 20.0f

/*========== 遥控通道映射 =========*/

#define RC_NORMALIZED_MIN 172U
#define RC_NORMALIZED_MID 992U
#define RC_NORMALIZED_MAX 1811U

AngleController_t angle_controller;
RateController_t rate_controller;

/**
 * @brief   根据Yaw目标角速度计算积分项(I)的增长缩放因子。
 * 
 * 用于大机动时抑制积分项的累积，防止过冲与积分饱和。
 * 
 * @param[in]   yaw_rate_target 目标Yaw角速度(deg/s)。
 * @return float    积分缩放系数，范围[0.0,1.0]。
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
 * @brief   将带中位的遥控器通道值映射为正负对称的物理量。
 * 
 * @param[in]   ch  遥控器原始通道值。
 * @param[in]   magnitude   映射输出的最大绝对值。
 * @return float    映射后的结果，范围 [-magnitude,magnitude]。
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

/*============================= 角度控制器(PID外环) ==============================*/

void AngleController_Init(void)
{
    PID_Init(&angle_controller.roll, 3.5f, 0.0f, 0.0f, 250.0f);
    PID_Init(&angle_controller.pitch, 3.5f, 0.0f, 0.0f, 250.0f);

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

/*============================= 角速度控制器(PID内环) ============================*/

void RateController_Init(void)
{
    PID_Init(&rate_controller.roll,  1.10f, 0.10f, 0.000f, 120.0f);
    PID_Init(&rate_controller.pitch, 1.05f, 0.10f, 0.000f, 120.0f);
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

    rate_controller.roll_output = PID_Update(&rate_controller.roll, roll_rate, dt);
    rate_controller.pitch_output = PID_Update(&rate_controller.pitch, pitch_rate, dt);

    const float yaw_integral_growth_scale =
        RateController_GetYawIntegralGrowthScale(yaw_rate_target);

    rate_controller.yaw_output = PID_UpdateWithIntegralScale(
        &rate_controller.yaw,
        yaw_rate,
        dt,
        yaw_integral_growth_scale);
}

/*================================ 航向锁定 ===================================*/
float YawHeadingHold_Update(PID_t *pid_yaw,
                            float yaw_target_deg,
                            float yaw_meas_deg,
                            float dt)
{
    float yaw_err = yaw_target_deg - yaw_meas_deg;
    
    // 处理角度环绕，保证总是沿最短路径旋转
    if(yaw_err > 180.0f)
        yaw_err -= 360.0f;
    if(yaw_err < -180.0f)
        yaw_err += 360.0f;

    PID_SetTarget(pid_yaw, 0.0f);

    return PID_Update(pid_yaw, -yaw_err, dt);
}

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
