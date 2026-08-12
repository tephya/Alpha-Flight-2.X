#ifndef __ALG_CONTROLLER_H
#define __ALG_CONTROLLER_H

#include <stdint.h>
#include "alg_pid.h"

// 遥控油门映射后的控制量上限
#define ALG_THROTTLE_MAX 1152.0f

/**
 * @brief 角度控制器（姿态外环）。
 * 
 * 输入：目标角度与实测角度，单位：°；
 * 输出：目标角速度，单位 °/s，作为 Rate Controller 的输入。
 * 
 * @note 当前仅控制 Roll/Pitch，Yaw Heading Hold使用独立外环实现。
 */
typedef struct
{
    PID_t roll;
    PID_t pitch;

    float roll_rate_target;     // Roll目标角速度
    float pitch_rate_target;    // Pitch目标角速度
} AngleController_t;

extern AngleController_t angle_controller;

/**
 * @brief   初始化姿态角度控制器(外环)。
 *
 * 配置 Roll 和 Pitch 轴的角度 PID 参数，并清零输出。
 */
void AngleController_Init(void);

/**
 * @brief   复位姿态角度控制器状态。
 *
 * 清除外环 PID 的积分器历史状态及当前目标角速度。
 */
void AngleController_Reset(void);

/**
 * @brief   更新姿态角度控制器（外环），计算期望角速度。
 *
 * 根据期望欧拉角与当前测量欧拉角，计算出内环所需的期望角速度。
 * 结果直接存入 angle_controller.roll_rate_target / pitch_rate_target 中。
 *
 * @param[in]   roll_target   Roll目标角度，°
 * @param[in]   pitch_target  Pitch目标角度，°
 * @param[in]   roll_meas     Roll实测角度，°
 * @param[in]   pitch_meas    Pitch实测角度，°
 * @param[in]   dt            控制周期，s
 * 
 * @pre 必须先调用 AngleController_Init()。
 */
void AngleController_Update(float roll_target,
                            float pitch_target,
                            float roll_meas,
                            float pitch_meas,
                            float dt);

/**
 * @brief 角度控制器（姿态外环）。
 * 
 * 输入：目标角速度与实测角速度，单位：°/s；
 * 输出：无量纲控制量，直接送入 Mixer
 */
typedef struct
{
    PID_t roll;
    PID_t pitch;
    PID_t yaw;

    float roll_output;      // Roll 控制输出
    float pitch_output;     // Pitch 控制输出
    float yaw_output;       // Yaw 控制输出
} RateController_t;

extern RateController_t rate_controller;

/**
 * @brief   初始化角速度控制器（内环）。
 */
void RateController_Init(void);

/**
 * @brief   复位角速度控制器状态。
 * 
 * 清除内环 PID 积分器历史状态，并将环控输出量归零。
 */
void RateController_Reset(void);

/**
 * @brief   更新角速度控制器（内环），计算混空气输入量。
 *
 * 包含常规的 Roll/Pitch 闭环，以及带有动态积分缩放的 Yaw 闭环。
 * 结果直接存入 rate_controller 的 output 字段中。
 *
 * @param[in]   roll_rate_target  Roll 目标角速度，°/s
 * @param[in]   pitch_rate_target Pitch 目标角速度，°/s
 * @param[in]   yaw_rate_target   Yaw 目标角速度，°/s
 * @param[in]   roll_rate         Roll 实测角速度，°/s
 * @param[in]   pitch_rate        Pitch 实测角速度，°/s
 * @param[in]   yaw_rate          Yaw 实测角速度，°/s
 * @param[in]   dt                控制周期，s
 */
void RateController_Update(float roll_rate_target,
                           float pitch_rate_target,
                           float yaw_rate_target,
                           float roll_rate,
                           float pitch_rate,
                           float yaw_rate,
                           float dt);

/**
 * @brief Yaw Heading Hold外环。
 * 
 * @param[in,out]   pid_yaw   Yaw外环PID示例。
 * @param[in]       yaw_target_deg  期望航向角(deg)。
 * @param[in]       yaw_meas_deg    当前测量航向角(deg)。
 * @param[in]       dt              控制周期(s)。
 * @return  float   目标Yaw角速度(deg/s)。
 */
float YawHeadingHold_Update(PID_t *pid_yaw,
                            float yaw_target_deg,
                            float yaw_meas_deg,
                            float dt);

/**
 * @brief   将遥控器通道映射为 Roll 目标角度。
 * @param[in]   ch  原始通道值。
 * @return  float    目标角度，范围 [-30.0,30.0]。
 */
float Map_Roll(uint16_t ch);

/**
 * @brief   将遥控器通道映射为 Pitch 目标角度。
 * 
 * @param[in]   ch  原始通道值。
 * @return  float   目标角度，范围 [-30.0,30.0]。
 * 
 * @note    结果取反。适配摇杆前推（数值增大）产生低头（负俯仰）的航模习惯。
 */
float Map_Pitch(uint16_t ch);

/**
 * @brief   将遥控器通道映射为 Yaw 目标角速度。
 * 
 * @param[in]   ch  原始通道值。
 * @return  float   目标角速度，范围[-90.0,90.0]
 * 
 * @note    返回结果取反。适配具体的偏航坐标系方向定义。
 */
float Map_Yaw(uint16_t ch);

/**
 * @brief   将遥控器油门通道映射为内部控制量。
 * 
 * @param[in]   ch  原始通道值。
 * @return uint16_t 映射后的油门量，范围 [0, ALG_THROTTLE_MAX]。
 */
uint16_t Map_Throttle(uint16_t ch);

#endif
