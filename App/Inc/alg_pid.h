/**
 * @file    alg_pid.h
 * @brief   通用 PID 控制器接口。
 */

#ifndef __ALG_PID_H
#define __ALG_PID_H

#include <stdint.h>

/**
 * @brief   PID 控制器状态与参数。
 */
typedef struct
{
    float target;           /**< 目标值。 */
    float measurement;      /**< 当前测量值。 */

    float error;            /**< 当前误差，Target - Measurement。 */
    float last_error;       /**< 上一周期误差。 */

    float kp;       /**< Proportional 增益。 */
    float ki;       /**< Integral 增益。 */
    float kd;       /**< Derivative 增益。 */

    float integral;         /**< Integral 累积状态。 */
    float integral_limit;   /**< Integral 绝对值上限。 */

    float derivative;           /**< 滤波后的 Derivative 值。 */
    float derivative_raw;       /**< 未滤波的原始 Derivative 值。 */
    float derivative_lpf;       /**< Derivative 低通滤波状态。 */
    float last_measurement;     /**< 上一周期测量值，用于 D-on-measurement。 */
    uint8_t first_update;       /**< 首次更新标志，1=首次，0=已完成初始化更新。 */

    float ff; /**< Feedforward 控制贡献。 */

    float output;           /**< 当前 PID 最终输出。 */
    float output_limit;     /**< PID 输出绝对值上限。 */

} PID_t;

/**
 * @brief 初始化 PID 控制器。
 *
 * @param[in,out] pid           PID 实例。
 * @param[in]     kp            Proportional 增益。
 * @param[in]     ki            Integral 增益。
 * @param[in]     kd            Derivative 增益。
 * @param[in]     output_limit  输出绝对值上限。
 */
void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float output_limit);

/**
 * @brief 清除 PID 动态状态并保留当前配置参数。
 */
void PID_Reset(PID_t *pid);

/**
 * @brief 设置 PID 目标值。
 */
void PID_SetTarget(PID_t *pid, float target);

/**
 * @brief 设置 PID 的 P/I/D 增益。
 */
void PID_SetGain(PID_t *pid,
                 float kp,
                 float ki,
                 float kd);

/**
 * @brief 设置 Integral 绝对值上限。
 */
void PID_SetIntegralLimit(PID_t *pid,
                          float limit);

/**
 * @brief 设置 PID 输出绝对值上限。
 */
void PID_SetOutputLimit(PID_t *pid,
                        float limit);

/**
 * @brief 执行一次标准 PID 更新。
 *
 * @param[in,out] pid          PID 实例。
 * @param[in]     measurement  当前测量值。
 * @param[in]     dt           控制周期，s。
 *
 * @return PID 输出。
 */
float PID_Update(PID_t *pid,
                 float measurement,
                 float dt);

/**
 * @brief 执行 PID 更新，并限制 Integral 的正向增长速度。
 *
 * @param[in,out] pid                    PID 实例。
 * @param[in]     measurement            当前测量值。
 * @param[in]     dt                     控制周期，s。
 * @param[in]     integral_growth_scale  Integral 增长比例，范围 [0.0, 1.0]。
 *
 * @return PID 输出。
 *
 * @note 当误差方向有助于卸载已有 Integral 时，不受
 *       integral_growth_scale 限制。
 */
float PID_UpdateWithIntegralScale(PID_t *pid,
                                  float measurement,
                                  float dt,
                                  float integral_growth_scale);

#endif
