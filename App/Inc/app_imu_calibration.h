#ifndef __APP_IMU_CALIBRATION_H
#define __APP_IMU_CALIBRATION_H

#include "bsp_icm42688.h"
#include <stdbool.h>
#include <stdint.h>

/* 本轮启动的必须参与者(WHO_AM_I校准成功的IMU) */
#define IMU_CAL_REQUIRED_IMU1 (1U << 0)     // IMU1必须完成本次启动校准
#define IMU_CAL_REQUIRED_IMU2 (1U << 1)     // IMU2必须完成本次启动校准

/**
 * @brief   开始本次上电的Gyro bias校准。
 * @param   required_mask   需要参数校准的IMU位码：bit0=IMU1，bit1=IMU2。
 * @note    校准结果值在本次运行期间有效，不写入Flash。
 */
void ImuCalibration_Init(uint8_t required_mask);

/**
 * @brief   输入本轮新线IMU数据并推进校准状态机。
 * @retval  true    校准已完成，之后可进入姿态解锁。
 * @retval  false   尚未完成：调用方必须保持电机为零并跳过控制解算。
 */
bool ImuCalibration_Update(const IcmData_t *imu1, bool imu1_fresh, const IcmData_t *imu2, bool imu2_fresh);

/* 以对应IMU的启动零偏修正一帧数据；校准未完成时不修改 */
void ImuCalibration_Apply(IcmInstance_t instance, IcmData_t *data);

bool ImuCalibration_IsReady(void);

#endif
