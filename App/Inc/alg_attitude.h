/**
 * @file    alg_attitude.h
 * @brief   姿态角计算接口。
 * 
 * 本模块使用Accel与Gyro的互补滤波计算Roll/Pitch，并提供基于Mag的
 * 倾斜补偿Yaw计算函数。所有姿态角统一使用rad。
 */

#ifndef __ALG_ATTITUDE_H
#define __ALG_ATTITUDE_H

#include "bsp_icm42688.h"
#include "bsp_qmc5883.h"
#include <stdbool.h>

/* 某些C库未定义M_PI，再次提供单精度兜底值 */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


/**
 * @brief   姿态角及Accel中间计算结果。
 */
typedef struct
{
    float roll;     // Roll，单位rad。
    float pitch;    // Pitch，单位rad。 
    float yaw;      // Yaw，单位rad，范围通常为[-pi,pi]。

    float accel_roll;   // 根据重力方向计算的Roll，单位rad。
    float accel_pitch;  // 根据重力方向计算的Pitch，单位rad。
} Attitude_t;

/**
 * @brief   FlightControl使用的全局姿态状态。
 * 
 * Roll/Pitch由本模块的互补滤波更新；Yaw当前也可能由YawEstimator更新
 */
extern Attitude_t attitude;

/**
 * @brief   根据Accel测量值计算Roll/Pitch观测角。
 * 
 * 计算结果写入att->accel_roll和att->accel_pitch，不直接修改最终的
 * att->roll和att->pitch。该结果主要作为互补滤波的低频修正量。
 * 
 * @param[in]   imu 当前IMU数据，Accel单位为g
 * @param[in,out] att 姿态状态，写入Accel观测角。
 * 
 * @note    飞行器存在明显线加速度时，Accel测得的不再只有重力，
 *          此时计算出的姿态角会暂时包含运动加速度误差。
 * @pre     imu和att必须为有效指针。
 */
void Attitude_ComputeAccelAngles(const IcmData_t *imu, Attitude_t *att);

/**
 * @brief   执行一次Roll/Pitch互补滤波更新。
 * 
 * Gyro积分提供短期动态响应，Accel观测角用于抑制长期积分漂移。
 * 本函数只更新att->roll和att->pitch，不更新Yaw。
 * 
 * @param[in]   imu 当前IMU数据，Gyro单位为dps。
 * @param[in,out] att 姿态状态。
 * @param[in]   dt 更新周期，单位s。
 * @param[in]   use_weak_accel_correction true: 弱Accel校正，用于Armed状态；
 *                                        false: 正常Accel校正，用于Disarmed状态。
 * 
 * @pre     imu和att必须为有效指针，dt必须为有效正数。
 */
void Attitude_Update(const IcmData_t *imu,
                     Attitude_t *att,
                     float dt,
                     bool use_weak_accel_correction);

/**
 * @brief   根据Mag数据计算经过Roll/Pitch倾斜补偿的Yaw。
 * 
 * 计算结果直接写入att->yaw，输出范围为[-pi,pi]。
 * 
 * @param[in]   mag 已完成单位转换及NED轴向对齐的Mag数据。
 * @param[in,out] att 当前姿态状态；读取Roll/Pitch并写入Yaw。
 * 
 * @note    本函数只完成倾斜补偿和几何角度计算，不执行Mag有效性检查、
 *          Innovation拒绝、滤波或Gyro融合。调用方必须先排除ovfl及异常数据。
 * @pre     mag和att必须为有效指针。
 */
void Attitude_CptYaw(const MagData_t *mag, Attitude_t *att);

#endif
