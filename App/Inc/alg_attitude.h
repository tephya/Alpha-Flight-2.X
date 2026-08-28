/**
 * @file    alg_attitude.h
 * @brief   姿态角计算接口。
 * 
 * 提供使用骆驼椅-加速度计互补滤波的横滚/俯仰（Roll/Pitch）估计，
 * 以及使用磁力计测量值计算倾斜补偿的偏航角（Yaw）。
 * 所有姿态角均已弧度为单位。
 */

#ifndef __ALG_ATTITUDE_H
#define __ALG_ATTITUDE_H

#include "bsp_icm42688.h"
#include "bsp_qmc5883.h"
#include <stdbool.h>

/** 为没有 M_PI 的库提供单精度后备定义。 */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


/**
 * @brief   姿态状态和基于加速度计推导的观测角。
 */
typedef struct
{
    float roll;     /**< 横滚角（Roll），单位：弧度。 */
    float pitch;    /**< 俯仰角（Pitch），单位：弧度。 */
    float yaw;      /**< 偏航角（Yaw），单位：弧度，通常在 [-pi, pi] 范围内。 */

    float accel_roll;   /**< 由重力推导出的横滚观测角，单位：弧度。 */
    float accel_pitch;  /**< 由重力推导出的俯仰观测角，单位：弧度。 */
} Attitude_t;

/**
 * @brief   飞控 (FlightControl)使用的全局姿态状态。
 * 
 * 横滚和俯仰由本模块中的互补滤波器进行更新。
 * 偏航角也可以由 YawEstimator 进行更新。
 */
extern Attitude_t attitude;

/**
 * @brief   从加速度计测量值计算横滚/俯仰观测角
 * 
 * 计算出的角度会被写入 accel_roll 和 accel_pitch，
 * 并被互补滤波器用作低频校正项。
 * 
 * @param[in]       imu 当前IMU数据；加速度值以 g 为单位。
 * @param[in,out]   att 接收加速度推导角的姿态状态。
 * 
 * @note    线性加速度会引入暂时的姿态观测误差，
 *          因为此时加速度计的测量值不再仅代表重力。
 */
void Attitude_ComputeAccelAngles(const IcmData_t *imu, Attitude_t *att);

/**
 * @brief   使用互补滤波更新横滚/俯仰角。
 * 
 * 陀螺仪积分提供短期动态响应，而加速度计观测值则用来抑制长期的积分漂移。偏航角（Yaw）不会被修改。
 * 
 * @param[in]       imu 当前IMU数据；陀螺仪值以 dps（度/秒）为单位。
 * @param[in,out]   att 需要更新的姿态状态。
 * @param[in]       dt 更新周期，单位：秒。
 * @param[in]       use_weak_accel_correction
 *                      为true时：在解锁（Armed）状态下使用较弱的加速度计校正； 
 *                      为false时：在锁定（Disarmed）状态下使用常规校正。
 */
void Attitude_Update(const IcmData_t *imu,
                     Attitude_t *att,
                     float dt,
                     bool use_weak_accel_correction);

/**
 * @brief   从磁力计测量值计算倾斜补偿后的偏航角（Yaw）。
 * 
 * 使用当前的横滚/俯仰估计值，将磁场向量投影到水平面上。结果将直接写入 yaw。
 * 
 * @param[in]       mag 单位转换和 NED 坐标轴对齐后的磁力计数据。 
 * @param[in,out]   att 提供横滚/俯仰角并接收偏航角的姿态状态。
 * 
 * @note    输出通常在 [-pi,pi] 范围内。
 * @note    此函数仅执行几何倾斜补偿和航向计算。
 *          磁力计数据的有效性检查、新息拒绝（innovation rejection）、滤波，
 *          以及陀螺仪融合必须由调用者处理。
 */
void Attitude_CptYaw(const MagData_t *mag, Attitude_t *att);

#endif
