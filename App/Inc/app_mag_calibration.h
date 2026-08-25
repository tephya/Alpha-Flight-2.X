/**
 * @file    app_mag_calibration.h
 * @brief   Mag 原始数据采集状态机及 Hard/Soft-Iron 校正接口。
 */

#ifndef __APP_MAG_CALIBRATION_H
#define __APP_MAG_CALIBRATION_H

#include "bsp_elrs.h"
#include "bsp_qmc5883.h"
#include <stdbool.h>

/** 
 * @brief  初始化 Mag 校准采集状态机 
 */
void MagCalibration_Init(void);

/**
 * @brief 处理 RC 手势并推进 Mag Calibration 采集状态机。
 *
 * @param[in] rc  当前最新 RC 数据。
 *
 * @note 使用 SB（CRSF CH8 / channels[7]）控制采集流程。
 *       由 FlightCtrl Task 在取得最新 RC 数据后周期调用。
 */
void MagCalibration_HandleRc(const RCChannelData_t *rc);

/**
 * @brief 记录一帧尚未进行 Hard/Soft-Iron 校正的 Mag 数据。
 *
 * @param[in] mag  已转换为 Gauss 并完成机体坐标系对齐的原始 Mag 数据。
 *
 * @note 由 Nav Task 在 QMC 新数据读取成功后调用。
 *       非采集状态或 Overflow 样本会立即丢弃。
 */
void MagCalibration_LogSample(const MagData_t *mag);

/**
 * @brief 对 Mag 数据原地应用已拟合的 Hard/Soft-Iron 校正。
 *
 * @param[in,out] mag  待校正 Mag 数据。
 *
 * @note 输入必须已经转换为 Gauss，并完成机体 NED 轴对齐。
 *       Overflow 样本不执行校正，ovfl 标志保持不变。
 */
void MagCalibration_Apply(MagData_t *mag);

/**
 * @brief 获取拟合时得到的参考磁场模长。
 *
 * @return 校正后的参考磁场模长，Gauss。
 */
float MagCalibration_GetFieldReferenceGauss(void);

/**
 * @brief 查询 Mag Calibration 采集流程是否正在占用系统。
 *
 * @return true 表示正在执行触发长按、等待文件打开、
 *         数据采集或等待文件关闭。
 *
 * @note 返回 true 时 Arm 状态机必须禁止解锁。
 */
bool MagCalibration_IsActive(void);

#endif
