/**
 * @file    app_level_trim.h
 * @brief   双 IMU Level Trim 校准，持久化及补偿接口。
 */

#ifndef __APP_LEVEL_TRIM_H
#define __APP_LEVEL_TRIM_H

#include "bsp_elrs.h"
#include "bsp_icm42688.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief   初始化 Level Trim 模块并从 Config Flash 加载最近一次有效结果。
 * 
 * @param[in]   required_mask   实际初始化成功的IMU掩码：
 *                              bit0=IMU1，bit1=IMU2。
 * 
 * @note    只有所有 Required IMU 均存在有效 Trim 时，已存配置才整体生效。
 */
void LevelTrim_Init(uint8_t required_mask);

/**
 * @brief   处理 RC 长按触发手势。
 * 
 * @param[in] rc    当前最新 RC 数据。
 * 
 * @note    只有 Disarmed，低油门，Gyro Calibration 完成，
 *          Mag Calibration 未运行时，持续按住 SE 达到规定时间才启动校准。
 */
void LevelTrim_HandleRc(const RCChannelData_t *rc);

/**
 * @brief   向 Level Trim 状态机提交双 IMU 的新样本。
 *
 * @param[in] imu1  IMU1 数据。
 * @param[in] imu1_fresh IMU1 本周期是否为新样本。
 * @param[in] imu2  IMU2 数据。
 * @param[in] imu2_fresh IMU2 本周期是否为新样本。
 *
 * @note    只在校准激活时消费样本；移动机体会清空已累计样本。
 */
void LevelTrim_UpdateSamples(const IcmData_t *imu1, bool imu1_fresh,
                             const IcmData_t *imu2, bool imu2_fresh);

/**
 * @brief   从指定 IMU 的 Accel 姿态角观测中扣除 Level Trim。
 *
 * @param[in]   instance    IMU 实例。
 * @param[in,out]   accel_roll_rad  Accel Roll 观测，rad。
 * @param[in,out]   accel_pitch_rad  Accel Pitch 观测，rad。
 */
void LevelTrim_Apply(IcmInstance_t instance, float *accel_roll_rad, float *accel_pitch_rad);

/**
 * @brief   查询 Level Trim 是否正在占用校准流程。
 * 
 * @return  true 表示正在采样或等待姿态滤波器重新稳定，
 *          此时禁止解锁。
 */
bool LevelTrim_IsActive(void);

/**
 * @brief   获取指定 IMU 当前实际生效的 Level Trim。
 *
 * @param[in]   instance    IMU 实例。
 * @param[out]  roll_offset_rad Roll Trim，rad。
 * @param[out]  pitch_offset_rad Pitch Trim，rad。
 * 
 * @retval  true 表示该 IMU 存在有效 Trim。
 */
bool LevelTrim_GetOffsets(IcmInstance_t instance,
                          float *roll_offset_rad,
                          float *pitch_offset_rad);

/**
 * @brief   查询所有 Required IMU 是否均存在有效 Level Trim。
 * 
 * @return true 表示当前可用 IMU 均已有有效 Trim。
 */
bool LevelTrim_IsReady(void);

#endif
