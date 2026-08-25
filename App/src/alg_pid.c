/**
 * @file    alg_pid.c
 * @brief   通用 PID 控制器实现。
 * 
 * 支持 D-on-measurement，Integral Growth Scaling，
 * Conditional Integration 及 Back-Calculation Anti-Windup。
 */

#include "alg_pid.h"

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

void PID_SetTarget(PID_t *pid, float target)
{
    pid->target = target;
}

void PID_SetGain(PID_t *pid,
                 float kp,
                 float ki,
                 float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_SetIntegralLimit(PID_t *pid,
                          float limit)
{
    pid->integral_limit = limit;
}

void PID_SetOutputLimit(PID_t *pid,
                        float limit)
{
    pid->output_limit = limit;
}

/*
 * PID 核心更新。
 * 
 * Integral_growth_scale 仅限制 Integral 沿当前方向继续增长的速度；
 * 当误差有助于卸载已有 Integral 时，始终允许以完整速度回零。
 */
static float PID_UpdateInternal(PID_t *pid,
                                float measurement,
                                float dt,
                                float integral_growth_sacle)
{
    // 限制最小 Dt，避免 Derivative 计算因异常小时间步长而产生尖峰。
    if (dt < 0.0005f)
        dt = 0.0005f;

    pid->measurement = measurement;
    pid->error = pid->target - measurement;

    // Integral Growth Scale 仅允许处于 [0,1]。
    if(integral_growth_sacle < 0.0f)
        integral_growth_sacle = 0.0f;
    else if(integral_growth_sacle > 1.0f)
        integral_growth_sacle = 1.0f;

    /*
     * Conditional Integral Anti-Windup：
     * 若上一周期输出已经饱和，且当前误差仍要求输出继续向同一方向增大，
     * 则停止 Integral 的进一步累积。
     */
    uint8_t stop_integration = 0;

    if ((pid->output >= pid->output_limit && pid->error > 0) ||
        (pid->output <= -pid->output_limit && pid->error < 0))
    {
        stop_integration = 1;
    }

    /*
     * 即使 Integral Growth 被限制，也始终允许反向误差卸载已有 Integral，
     * 避免 Integral Growth Scaling 将已有稳态补偿锁死。
     */
    const uint8_t integral_unwinding =
        ((pid->integral > 0.0f) && (pid->error < 0.0f)) ||
        ((pid->integral < 0.0f) && (pid->error > 0.0f));

    if (!stop_integration)
    {
        if(integral_unwinding)
        {
            pid->integral += pid->error * dt;
        }
        else if(integral_growth_sacle > 0.0f)
        {
            pid->integral += pid->error * dt * integral_growth_sacle;
        }
    }

    // Integral 状态绝对值限幅。
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    /*
     * 首次更新时将历史 Measurement 与当前值对齐，
     * 避免未初始化历史值产生 Derivative Kick。
     */
    if (pid->first_update)
    {
        pid->last_measurement = measurement;
        pid->first_update = 0;
    }

    /* 
     * Derivative on Measurement：
     * 对 Measurement 而非 Error 求导，避免 Target 阶跃直接产生 Derivative Kick。
     * 符号通过 Last_measurement - measurement 保持负测量导数形式。
     */
    pid->derivative_raw = (pid->last_measurement - measurement) / dt;

    // 固定一阶低通：20% 新 Derivative，80% 历史状态。
    pid->derivative_lpf =
        0.2f * pid->derivative_raw +
        0.8f * pid->derivative_lpf;

    pid->derivative = pid->derivative_lpf;

    const float p_term = pid->kp * pid->error;
    const float i_term = pid->ki * pid->integral;
    const float d_term = pid->kd * pid->derivative;

    // Feedforward 与 PID 三项共同形成限幅前理论输出。
    const float pre_out = p_term + i_term + d_term + pid->ff;

    /*
     * Back-Calculation Anti-Windup:
     * 当理论输出超过执行范围时，将超出的输出量按 Ki 折算回 Integral，
     * 使 Integral 状态与可实现输出保持一致。
     */
    if (pre_out > pid->output_limit)
    {
        float excess = pre_out - pid->output_limit;
        
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

    /*
     * Back-Calculation 可能直接修改 Integral，
     * 因此在最终输出后再次执行绝对值限幅作为状态保护。
     */
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    pid->last_measurement = measurement;
    pid->last_error = pid->error;

    return pid->output;
}

float PID_Update(PID_t *pid,
                 float measurement,
                 float dt)
{
    return PID_UpdateInternal(pid, measurement, dt, 1.0f);
}

float PID_UpdateWithIntegralScale(PID_t *pid,
                                  float measurement,
                                  float dt,
                                  float integral_growth_scale)
{
    return PID_UpdateInternal(pid, measurement, dt, integral_growth_scale);
}
