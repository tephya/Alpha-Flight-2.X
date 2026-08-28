/**
 * @file    app_imu2_redundancy.h
 * @brief   双 IMU 冗余管理，健康判定及 Active IMU 数据输出接口。
 */

#ifndef __APP_IMU2_REDUNDANCY_H
#define __APP_IMU2_REDUNDANCY_H

#include "bsp_icm42688.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief   单次 IMU 冗余更新结果。
 */
typedef enum
{
    IMU_UPDATE_ACTIVE_FRAME = 0,    /**< 获得新的有效 Active IMU 帧，可执行控制计算。 */
    IMU_UPDATE_STANDBY_ONLY,        /**< 仅 Standby IMU 更新，Active 数据未变化。 */
    IMU_UPDATE_CALIBRATION,         /**< Gyro Bias 校准尚未完成。 */
    IMU_UPDATE_TIMEOUT,             /**< 等待 IMU DRDY 超时。 */
    IMU_UPDATE_DUAL_FAULT,          /**< 当前判定双 IMU 均不能可靠使用。 */
    IMU_UPDATE_TIMING_ANOMALY,      /**< Active IMU 有新帧，但采样时间间隔异常。 */
} ImuUpdateResult_t;

/**
 * @brief   初始化双 IMU 冗余管理模块。
 * 
 * 个你底层初始化结果建立可用 IMU 集合，初始 Active IMU，
 * 并启动 Gyro Bias Calibration，Level Trim 及 Cycle Counter。
 * 
 * @param[in] init_fail_mask    ICM_InitAll() 返回的初始化失败位掩码。
 */
void ImuRedundancy_Init(uint8_t init_fail_mask);

/**
 * @brief   等待并处理一轮双 IMU 数据更新。
 * 
 * 完成 DRDY 等待，SPI 数据刷新，Gyro Bias 校准，数据新鲜度检查，
 * 双 IMU CrossCheck，健康状态更新，Active IMU 选择及真实控制周期计算。
 * 
 * @param[out] out  当前 Active IMU 数据。
 * @param[out] dt_s 相邻有效 Active IMU 帧之间的实际周期，s。
 * @param[out] fresh_flags  本轮收到新 DRDY 的 IMU 位标志，可为 NULL。
 * 
 * @return  本轮 IMU 更新结果，见 ImuUpdateResult_t。
 */
ImuUpdateResult_t ImuRedundancy_Update(IcmData_t *out, float *dt_s, uint8_t *fresh_flags);


#endif
