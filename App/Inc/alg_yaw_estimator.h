/**
 * @file    alg_yaw_estimator.h
 * @brief   Yaw 状态估计器接口。
 * 
 * 使用 Gyro Z 轴积分推进 Yaw，并通过经过有效性检查的 Mag 航向
 * 对长期积分漂移进行慢速修正。
 */

#ifndef __ALG_YAW_ESTIMATOR_H
#define __ALG_YAW_ESTIMATOR_H

#include "bsp_qmc5883.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief   Mag 样本拒绝原因位掩码。
 */
typedef enum
{
    YAW_MAG_REJECT_NONE = 0U,                  /**< 样本未被拒绝。 */
    YAW_MAG_REJECT_INVALID_SAMPLE = (1U << 0), /**< Mag 样本本身无效。 */
    YAW_MAG_REJECT_ABSOLUTE_FIELD = (1U << 1), /**< 磁场绝对模长超出允许范围。 */
    YAW_MAG_REJECT_FIELD_RATIO = (1U << 2),    /**< 磁场模长与参考值比例异常。 */
    YAW_MAG_REJECT_INNOVATION = (1U << 3),     /**< Mag Yaw Innovation 超出门限。 */
} YawMagRejectReason_t;

/**
 * @brief   Yaw Estimator 运行状态与诊断信息。
 */
typedef struct
{
    float yaw_rad;                  /**< Gyro 积分并经 Mag 修正后的融合 Yaw，rad。 */
    float mag_yaw_rad;              /**< 最近一次倾斜补偿后的原始 Mag Yaw，rad。 */
    float mag_innovation_rad;       /**< Mag 新息；Mag 航向观测与当前融合 Yaw 的最短角度偏差，rad。 */
    float mag_field_norm_gauss;     /**< 最近一次 Mag 三轴磁场模长，Gauss。 */

    float mag_field_reference_gauss;    /**< 当前磁场有效性判断使用的参考模长，Gauss。 */
    float mag_field_ratio;              /**< 当前磁场模长与参考模长的比值。 */
    uint8_t mag_reject_reason;          /**< 最近样本的拒绝原因，YawMagRejectResson_t 位掩码。 */

    bool initialized;               /**< 是否已建立有效的绝对 Yaw 状态。 */
    bool mag_accepted;              /**< 最近一次 Mag 样本是否实际参与 Yaw 修正。 */
} YawEstimatorDiagnostics_t;

/**
 * @brief   初始化 Yaw Estimator。
 * 
 * 初始化后等待首次有效 Mag 样本建立绝对航向。
 * 
 * @param[in] field_reference_gauss Hard/Soft-Iron 校准得到的参考磁场模长，Gauss。
 */
void YawEstimator_Init(float field_reference_gauss);

/**
 * @brief   使用 Gyro Z 轴角速度推进 Yaw。
 * 
 * @param[in] gz_dps Active IMU 的 Z轴 角速度，deg/s。
 * @param[in] dt_s 本次有效积分周期，s。
 */
void YawEstimator_UpdateGyro(float gz_dps, float dt_s);

/**
 * @brief   使用倾斜补偿后的 Mag 航向修正 Gyro 积分 Yaw。
 * 
 * @param[in] mag   当前 Mag 数据。
 * @param[in] roll_rad  当前 Roll，rad。
 * @param[in] pitch_rad 当前 Pitch，rad。
 * @param[in] is_disarmed   当前是否处于 Disarmed 状态。
 * 
 * @return  true 表示本次 Mag 样本完成初始化，正常校正或重新捕获；
 *          false 表示样本无效，被拒绝或仍在等待重新捕获确认。
 * 
 * @note    仅在 Disarmed 状态下允许更新磁场参考值及执行安全 Mag Reacquire。
 */
bool YawEstimator_CorrectMag(const MagData_t *mag,
                             float roll_rad,
                             float pitch_rad,
                             bool is_disarmed);

/**
 * @brief   获取当前融合 Yaw。
 * 
 * @return  当前 Yaw，rad。
 */
float YawEstimator_GetYawRad(void);

/**
 * @brief   判断 Yaw Estimator 是否已完成初始化。
 * 
 * @return  true 表示已有有效绝对 Yaw；false 表示仍未初始化。
 */
bool YawEstimator_IsInitialized(void);

/**
 * @brief   复制当前 Yaw Estimator 诊断状态。
 * 
 * @param[out]  out 用于接收诊断数据的结构体。
 */
void YawEstimator_CopyDiagnostics(YawEstimatorDiagnostics_t *out);

#endif
