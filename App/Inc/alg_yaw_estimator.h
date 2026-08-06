#ifndef __ALG_YAW_ESTIMATOR_H
#define __ALG_YAW_ESTIMATOR_H

#include "bsp_qmc5883.h"
#include <stdbool.h>

typedef struct
{
    float yaw_rad;                  // Gyro积分并由Mag慢矫正后的航向
    float mag_yaw_rad;              // 最近一次倾斜补偿后的原始Mag航向
    float mag_innovation_rad;       // wrap(mag_yaw - fused_yaw)
    float mag_field_norm_gauss;     // 最近一次Mag三轴模长
    bool initialized;               
    bool mag_accepted;              // 最近一次Mag样本是否通过门限并参与校正
} YawEstimatorDiagnostics_t;

/** @brief  初始化Yaw estimator；首次有效Mag样本将建立绝对航向。 */
void YawEstimator_Init(void);

/**
 * @brief   使用active IMU的Z轴Gyro推进航向。
 * @param   gz_dps  active IMU的Z轴角速度，单位deg/s。
 * @param   dt_s    本次有效控制周期，单位s。
 */
void YawEstimator_UpdateGyro(float gz_dps, float dt_s);

/**
 * @brief   使用倾斜补偿后的Mag航向对Gyro积分进行慢校正。
 * @param   allow_reference_update  true时允许慢速更新磁场模长基准；仅应在Disarmed时传入true
 * @retval  true    本次Mag样本被接收并用于初始化/校正。
 */
bool YawEstimator_CorrectMag(const MagData_t *mag,
                             float roll_rad,
                             float pitch_rad,
                             bool allow_reference_update);

float YawEstimator_GetYawRad(void);
bool YawEstimator_IsInitialized(void);
void YawEstimator_CopyDiagnostics(YawEstimatorDiagnostics_t *out);

#endif
