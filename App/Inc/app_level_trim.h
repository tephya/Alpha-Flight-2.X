#ifndef __APP_LEVEL_TRIM_H
#define __APP_LEVEL_TRIM_H

#include "bsp_elrs.h"
#include "bsp_icm42688.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief   初始化Level Trim模块并从Config Flash加载最近一次有效结果。
 * @param   required_mask   实际初始化成功的IMU掩码，bit0=IMU1，bit1=IMU2。
 */
void LevelTrim_Init(uint8_t required_mask);

/**
 * @brief   处理SE长按触发手势：应有Task_FlightCtrl使用最新RC缓存调用。
 * @note    只有Disarmed、SD(按键)关闭、低油门且SE(CH7)持续按住3秒才会开始校准。
 */
void LevelTrim_HandleRc(const RCChannelData_t *rc);

/**
 * @brief   向Level Trim状态机提交两颗IMU的新鲜样本。
 * @note    只在校准激活时消费样本；移动机体会清空已累计样本。
 */
void LevelTrim_UpdateSamples(const IcmData_t *imu1, bool imu1_fresh,
                             const IcmData_t *imu2, bool imu2_fresh);

/**
 * @brief   从指定IMU计算出的Accel姿态角中扣除持久化Level Trim。
 */
void LevelTrim_Apply(IcmInstance_t instance, float *accel_roll_rad, float *accel_pitch_rad);

/** @retval true    正在采样或等待姿态滤波器重新稳定，此时禁止解锁。 */
bool LevelTrim_IsActive(void);

/**
 * @brief   读取指定IMU当前实际生效的Level Trim。
 * @retval  true    该IMU存在有效Trim，输出单位为rad。
 */
bool LevelTrim_GetOffsets(IcmInstance_t instance,
                          float *roll_offset_rad,
                          float *pitch_offset_rad);

/** @retval true    当前所有可用IMU均有有效Level Trim。 */
bool LevelTrim_IsReady(void);

#endif
