/**
 * @file    app_arm.h
 * @brief   飞行器解锁，停桨及 Crash Protection 状态机接口。
 */

#ifndef __APP_ARM_H
#define __APP_ARM_H

#include "app_shared_types.h"
#include "bsp_elrs.h"
#include <stdbool.h>

/**
 * @brief   初始化 ARM 状态机。
 * 
 * 上电后默认进入 Disarmed 状态，并清除开关边沿及 Crash 检测状态。
 */
void Arm_Init(void);

/**
 * @brief   更新 ARM 状态机。
 * 
 * 根据 RC ARM 开关，油门，姿态就绪状态及运行时故障
 * 执行 Arm/Disarmed 状态转换。
 * 
 * @param[in] rc    当前 RC 通道及链路状态。
 * @param[in] roll_meas 当前 Roll，deg。
 * @param[in] pitch_meas 当前 Pitch，deg。
 * @param[in] dt    控制周期，s。
 */
void Arm_Update(const RCChannelData_t *rc,
                float roll_meas,
                float pitch_meas,
                float dt);

/**
 * @brief   立即撤销 Armed 状态。
 * 
 * 仅修改 ARM 状态并发布日志/指示事件；
 * 电机实际停桨由 FlightControl 末端安全门同一执行。
 */
void Arm_ForceDisarm(void);

#endif
