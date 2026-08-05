#include "alg_controller.h"

AngleController_t angle_controller;
RateController_t rate_controller;

/*========== 角度控制器(PID外环) =========*/
void AngleController_Init(void)
{
    PID_Init(&angle_controller.roll, 2.5f, 0.0f, 0.0f, 250.0f);
    PID_Init(&angle_controller.pitch, 2.5f, 0.0f, 0.0f, 250.0f);

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

/*========== 角速度控制器(PID内环) ==========*/
void RateController_Init(void)
{
    /* 诊断阶段先使用P-only并限制最大姿态修正
     * 防止再次失稳时Mixer产生过大的电机差动。 */
    PID_Init(&rate_controller.roll,  0.50f, 0.00f, 0.000f, 120.0f);
    PID_Init(&rate_controller.pitch, 0.50f, 0.00f, 0.000f, 120.0f);
    /* Yaw暂时保持原参数，本轮主要定位Roll/Pitch振荡。 */
    PID_Init(&rate_controller.yaw,   2.00f, 0.10f, 0.000f, 200.0f);
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
    rate_controller.yaw_output = PID_Update(&rate_controller.yaw, yaw_rate, dt);
}

/*========= 航向锁定 ==========*/
float YawHeadingHold_Update(PID_t *pid_yaw,
                            float yaw_target_deg,
                            float yaw_meas_deg,
                            float dt)
{
    float yaw_err = yaw_target_deg - yaw_meas_deg;
    
    if(yaw_err > 180.0f)
        yaw_err -= 360.0f;
    if(yaw_err < -180.0f)
        yaw_err += 360.0f;

    PID_SetTarget(pid_yaw, 0.0f);

    return PID_Update(pid_yaw, -yaw_err, dt);
}

/*========== 遥控通道映射 =========*/
#define RC_NORMALIZED_MIN 172U
#define RC_NORMALIZED_MID 992U
#define RC_NORMALIZED_MAX 1811U

static float Map_CenteredChannel(uint16_t ch, float magnitude)
{
    if(ch >= RC_NORMALIZED_MAX)
        return magnitude;
    if(ch <= RC_NORMALIZED_MIN)
        return -magnitude;
    if(ch == RC_NORMALIZED_MID)
        return 0.0f;
    
    if(ch > RC_NORMALIZED_MID)
    {
        return (float)(ch - RC_NORMALIZED_MID) /
               (float)(RC_NORMALIZED_MAX - RC_NORMALIZED_MID) * magnitude;
    }

    return -(float)(RC_NORMALIZED_MID - ch) /
           (float)(RC_NORMALIZED_MID - RC_NORMALIZED_MIN) * magnitude;
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
    return Map_CenteredChannel(ch, 90.0f);
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
