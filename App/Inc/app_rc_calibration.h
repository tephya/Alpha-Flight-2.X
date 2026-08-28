/**
 * @file    app_rc_calibration.h
 * @brief   RC 通道端点，中位点及 Deadband 校准接口。
 */

#ifndef __APP_RC_CALIBRATION_H
#define __APP_RC_CALIBRATION_H

#include "bsp_elrs.h"
#include <stdbool.h>
#include <stdint.h>

#define RC_CAL_CHANNEL_COUNT 4U     // 参与校准的主控制通道数量：Roll，Pitch，Throttle，Yaw。

/**
 * @brief   RC 主控制通道校准参数。
 */
typedef struct
{
    uint16_t min[RC_CAL_CHANNEL_COUNT];     /**< 各通道采集到的最小原始值。 */
    uint16_t mid[RC_CAL_CHANNEL_COUNT];     /**< 各通道中位值。 */
    uint16_t max[RC_CAL_CHANNEL_COUNT];     /**< 各通道采集到的最大原始值。 */
    uint16_t deadband;                      /**< 归一化后的 Roll/Pitch/Yaw 中心 Deadband。 */
    uint16_t reserved;                      /**< 保留字段，用于后续扩展。 */
} RcCalibration_t;

/**
 * @brief   初始化 RC Calibration 模块。
 * 
 * 从 Config Flash 加载最近一份有效校准参数；
 * 若无有效记录，则加载标准 CRSF 默认参数，但保持 Not Ready。
 */
void RcCalibration_Init(void);

/**
 * @brief   使用未归一化的原始 CRSF 数据推进 RC Calibration 状态机。
 * 
 * @param[in] raw_rc    当前原始 RC 通道数据。
 */
void RcCalibration_Update(const RCChannelData_t *raw_rc);

/**
 * @brief   将主控制通道映射到标准 CRSF 范围并应用中心 Deadband。
 * 
 * @param[in,out] rc    待归一化的 RC 数据。
 * 
 * @note    仅处理 channles[0..3]；其他开关通道保持原始值。
 */
void RcCalibration_Apply(RCChannelData_t *rc);

/**
 * @brief   查询是否已经加载或升成有效 RC Calibration。
 * 
 * @return  true 表示当前校准参数有效。
 */
bool RcCalibration_IsReady(void);

/**
 * @brief   查询 RC Calibration 状态机是否正在运行。
 * 
 * @return  true 表示正在采集端点或中心位置。
 */
bool RcCalibration_IsActive(void);

/**
 * @brief   获取当前归一化后的中心 Deadband。
 * 
 * @return  Roll/Pitch/Yaw 使用的中心 Deadband，CRSF Count。
 */
uint16_t RcCalibration_GetNormalizedDeadband(void);

#endif
