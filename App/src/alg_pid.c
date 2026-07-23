#include "alg_pid.h"

/**
 * @brief  初始化 PID 控制器
 * @param  *pid			要初始化的 PID 控制器实体指针
 * @param  kp				初始化的 P 项系数
 * @param  ki				初始化的 I 项系数
 * @param  kd				初始化的 D 项系数
 * @param  output_limit	设定的 PID 控制器输出上限值
 */
void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral_limit = 200.0f;
    pid->output_limit = output_limit;

    PID_Reset(pid);
}

/**
 * @brief  复位 PID 控制器
 * @param  *pid	 要复位的 PID 控制器实体指针
 */
void PID_Reset(PID_t *pid)
{
    pid->target = 0.0f;

    pid->measurement = 0.0f;

    pid->error = 0.0f;
    pid->last_error = 0.0f;

    pid->integral = 0.0f;

    pid->derivative = 0.0f;
    pid->derivative_raw = 0.0f;
    pid->derivative_lpf = 0.0f;

    pid->last_measurement = 0.0f;
    pid->first_update = 1;
    pid->ff = 0.0f;

    pid->output = 0.0f;
}

/**
 * @brief  设定 PID 控制器的目标输出值
 * @param  *pid	 要更新的 PID 控制器实体指针
 * @param  target	 要设定的目标输出值
 */
void PID_SetTarget(PID_t *pid, float target)
{
    pid->target = target;
}

/**
 * @brief  动态设置 PID 增益参数 (Kp, Ki, Kd)
 * @param  *pid	要更新的 PID 控制器实体指针
 * @param  kp		设定的 kp 系数
 * @param  ki		设定的 ki 系数
 * @param  kd		设定的 kd 系数
 */
void PID_SetGain(PID_t *pid,
                 float kp,
                 float ki,
                 float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief  设定 PID 控制器的积分最大值（包含上下限）
 * @param  *pid	 要更新的 PID 控制器实体指针
 * @param  limit	 要设定的积分最大值
 */
void PID_SetIntegralLimit(PID_t *pid,
                          float limit)
{
    pid->integral_limit = limit;
}

/**
 * @brief  设定 PID 控制器的输出上限
 * @param  *pid	 要更新的 PID 控制器实体指针
 * @param  limit	 要设定的输出上限值
 */
void PID_SetOutputLimit(PID_t *pid,
                        float limit)
{
    pid->output_limit = limit;
}

/**
 * @brief  动态更新 PID 输出值
 * @param  *pid	 		要更新的 PID 控制器实体指针
 * @param  measurement		当前测量值
 * @param  dt				距离上一次更新的时间间隔
 */
float PID_Update(PID_t *pid,
                 float measurement,
                 float dt)
{
    if (dt < 0.0005f)
        dt = 0.0005f;

    pid->measurement = measurement;
    pid->error = pid->target - measurement;

    /* Integral - Dynamic Anti-Windup */
    uint8_t stop_integration = 0;

    // 判断上一帧的输出是否已经顶满，且当前误差仍在要求电机朝着极限方向死磕
    if ((pid->output >= pid->output_limit && pid->error > 0) ||
        (pid->output <= -pid->output_limit && pid->error < 0))
    {
        stop_integration = 1;
    }

    if (!stop_integration)
    {
        pid->integral += pid->error * dt;
    }

    // 静态保底限幅，防止变量在长时间轻微误差下缓慢越界
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    if (pid->first_update)
    {
        pid->last_measurement = measurement; // 强行对齐，消除首帧偏差
        pid->first_update = 0;
    }

    /* Derivative on Measurement */
    pid->derivative_raw = (pid->last_measurement - measurement) / dt;

    pid->derivative_lpf =
        0.2f * pid->derivative_raw +
        0.8f * pid->derivative_lpf;

    pid->derivative = pid->derivative_lpf;

    // 分别计算各独立项
    float p_term = pid->kp * pid->error;
    float i_term = pid->ki * pid->integral; // 当前的理论 I 项输出
    float d_term = pid->kd * pid->derivative;

    // 计算未限幅的理论总输出
    float pre_out = p_term + i_term + d_term + pid->ff;

    // Back-Calculation 动态反算机制
    if (pre_out > pid->output_limit)
    {
        // 算出超标了多少
        float excess = pre_out - pid->output_limit;

        // 如果 Ki 不为 0，把超标的部分直接从积分池里按比例扣除（强行挤水）
        if (pid->ki > 0.0001f)
        {
            pid->integral -= (excess / pid->ki);
        }
        pid->output = pid->output_limit;
    }
    else if (pre_out < -pid->output_limit)
    {
        float excess = pre_out - (-pid->output_limit);

        if (pid->ki > 0.0001f)
        {
            pid->integral -= (excess / pid->ki);
        }
        pid->output = -pid->output_limit;
    }
    else
    {
        pid->output = pre_out;
    }

    // 最后的静态保底限幅 (防止在未触发 Output 满载的情况下，长时间微小误差导致 I 越界)
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    pid->last_measurement = measurement;
    pid->last_error = pid->error;

    return pid->output;
}
