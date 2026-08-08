#ifndef __ALG_PID_H
#define __ALG_PID_H

#include <stdint.h>

typedef struct
{
    float target;           // 设定的目标值
    float measurement;      // 实际测量值

    float error;            // 误差值
    float last_error;       // 上一次计算的误差值

    float kp;       // P 项增益系数
    float ki;       // I 项增益系数
    float kd;       // D 项增益系数

    float integral;         // 积分值
    float integral_limit;   // 积分最大值（包含上下限）

    float derivative;           // 微分项计算值
    float derivative_raw;       // 原始微分计算值
    float derivative_lpf;       // 微分项低通滤波器，20% 信任新变化
    float last_measurement;     // 上一次测量值
    uint8_t first_update;       // 记录 是否为首次更新的变量； 值=1:首次更新, 值=0:非首次更新

    /* FeedForward (Reserved) */
    float ff;       // PID之外的固定增益项，初值为0（保留）

    float output;           // PID 输出值
    float output_limit;     // PID 输出上限

} PID_t;


void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float output_limit);

void PID_Reset(PID_t *pid);

void PID_SetTarget(PID_t *pid, float target);

void PID_SetGain(PID_t *pid,
                 float kp,
                 float ki,
                 float kd);

void PID_SetIntegralLimit(PID_t *pid,
                          float limit);

void PID_SetOutputLimit(PID_t *pid,
                        float limit);

float PID_Update(PID_t *pid,
                 float measurement,
                 float dt);

/**
 * @brief   更新PID，并按比例控制Integral继续增长的速度。
 * @param   integral_growth_scale   积分增长比例，范围0.0~1.0，
 *                                  反向误差卸载Integral时不受此比例限制。
 */
float PID_UpdateWithIntegralScale(PID_t *pid,
                                  float measurement,
                                  float dt,
                                  float integral_growth_scale);

#endif
