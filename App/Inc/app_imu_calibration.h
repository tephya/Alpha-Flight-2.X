/**
 * @file    app_imu_calibration.h
 * @brief   双 IMU Gyro Bias 上电校准接口。
 */

#ifndef __APP_IMU_CALIBRATION_H
#define __APP_IMU_CALIBRATION_H

#include "bsp_icm42688.h"
#include <stdbool.h>
#include <stdint.h>

/* 本次启动中需要完成 Gyro Bias 校准的 IMU 位掩码。 */
#define IMU_CAL_REQUIRED_IMU1 (1U << 0)     // IMU1必须完成本次启动校准
#define IMU_CAL_REQUIRED_IMU2 (1U << 1)     // IMU2必须完成本次启动校准

/**
 * @brief   初始化本次启动的 Gyro Bias 校准。
 * 
 * @param[in] required_mask 需要参与校准的 IMU 位掩码：
 *                          bit0=IMU1，bit1=IMU2。
 * 
 * @note    校准结果仅在本次运行期间有效，不写入 Flash。
 */
void ImuCalibration_Init(uint8_t required_mask);

/**
 * @brief   使用新的 IMU 样本推进 Gyro Bias 校准状态机。
 *
 * @param[in] imu1          IMU1 当前样本。
 * @param[in] imu1_fresh    IMU1 本周期是否有新样本。
 * @param[in] imu2          IMU2 当前样本。
 * @param[in] imu2_fresh    IMU2 本周期是否有新样本。
 * 
 * @return  true 表示所有 Required IMU 已完成校准；
 *          false 表示校准尚未完成。
 * 
 * @note    校准完成前，调用方应保持 Motor Output 为零并跳过正常控制计算。
 */
bool ImuCalibration_Update(const IcmData_t *imu1, bool imu1_fresh, const IcmData_t *imu2, bool imu2_fresh);

/**
 * @brief   使用对应 IMU 的启动 Gyro Bias 修正一帧数据。
 * 
 * @param[in] instance IMU 实例。
 * @param[in,out] data  待修正的 IMU 数据。
 * 
 * @note    校准尚未完成时不修改输入数据。
 */
void ImuCalibration_Apply(IcmInstance_t instance, IcmData_t *data);

/**
 * @brief   查询本次启动的 Gyro Bias 校准是否完成。
 * 
 * @return  true 表示所有 Required IMU 均已完成校准。
 */
bool ImuCalibration_IsReady(void);

#endif
