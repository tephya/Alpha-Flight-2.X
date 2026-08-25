/**
 * @file    app_flightctrl.h
 * @brief   飞行控制任务及控制状态快照接口。
 */

#ifndef __APP_FLIGHTCTRL_H
#define __APP_FLIGHTCTRL_H

#include <stdint.h>

/**
 * @brief   飞行控制周期内的主要目标值，测量值与控制输出。
 */
typedef struct
{
    float roll_target;          /**< Roll 目标角，deg，由 RC 或水平辅助控制生成。 */
    float pitch_target;         /**< Pitch 目标角，deg，由 RC 或水平辅助控制生成。 */
    float yaw_target;           /**< Heading Hold 锁定的目标航向，deg。 */

    uint8_t yaw_mode;           /**< Yaw 模式：0=Heading Hold，1=Manual Rate。 */
    uint8_t airmode_active;     /**< Airmode 状态：0=关闭，1=保持姿态控制权限。 */

    float roll_rate_target;     /**< Roll 目标角速度，deg/s。由 Angle Controller 输出。 */
    float pitch_rate_target;    /**< Pitch 目标角速度，deg/s。由 Angle Controller 输出。 */
    float yaw_rate_target;      /**< Yaw 目标角速度，deg/s。 */

    float roll_cmd;         /**< Roll Rate Controller 输出，送入 Mixer。 */
    float pitch_cmd;        /**< Pitch Rate Controller 输出，送入 Mixer。 */
    float yaw_cmd;          /**< Yaw Rate Controller 输出，送入 Mixer。 */

    uint16_t throttle;      /**< RC 映射后的基础油门指令。 */

    float roll_meas;        /**< 当前融合 Roll，deg。 */
    float pitch_meas;       /**< 当前融合 Pitch，deg。 */
    float yaw_meas;         /**< 当前融合 Yaw，deg。 */
} FlightControl_t;

/**
 * @brief   飞行控制主任务入口。
 * 
 * 由 Active IMU 新数据驱动姿态估计，导航估计，ARM 状态机，
 * 串级 PID，Mixer，点击输出及 Blackbox 记录。
 * 
 * @param[in] argument  RTOS Task 参数，当前未使用。
 */
void App_FlightCtrl_Task(void *argument);

#endif
