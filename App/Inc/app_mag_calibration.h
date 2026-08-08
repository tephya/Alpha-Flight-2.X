#ifndef __APP_MAG_CALIBRATION_H
#define __APP_MAG_CALIBRATION_H

#include "bsp_elrs.h"
#include "bsp_qmc5883.h"
#include <stdbool.h>

/** @brief  初始化Mag校准采集状态机 */
void MagCalibration_Init(void);

/**
 * @brief   处理SB(CH8/channels[7])触发与采集文件启停。
 * @note    由FlightCtrl Task在读取最新RC数据后调用。
 */
void MagCalibration_HandleRc(const RCChannelData_t *rc);

/**
 * @brief   记录一帧尚未进行Hard/Soft-Iron校正的NED轴Mag数据。
 * @note    由Nav Task每次QMC读取成功后调用；非采集状态下立即返回。
 */
void MagCalibration_LogSample(const MagData_t *mag);

/**
 * @brief   对Mag数据原地应用已拟合的Hard/Soft-Iron校正。
 * @note    输入必须已经转换为Gauss，并完成机体NED轴对齐。
 *          保留ovfl标志；发生溢出的样本不执行校正。
 */
void MagCalibration_Apply(MagData_t *mag);

/**
 * @brief   获取Hard/Soft-Iron拟合得到的参考磁场模长。
 * @return  校正后磁场参考值，单位Gauss。
 */
float MagCalibration_GetFieldReferenceGauss(void);

/**
 * @brief   查询校准采集是否正在触发、打开、采集或关闭阶段。
 * @note    true时Arm状态机必须禁止解锁。
 */
bool MagCalibration_IsActive(void);

#endif
