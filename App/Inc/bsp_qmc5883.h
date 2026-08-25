/**
 * @file    bsp_qmc5883.h
 * @brief   QMC 磁力计初始化，采样及数据访问接口。
 */

#ifndef __BSP_QMC5883_H
#define __BSP_QMC5883_H

#include "i2c.h"
#include "stdbool.h"

/**
 * @brief   磁力计最近一次有效采样数据。
 */
typedef struct
{
    float MX;           /**< NED 坐标系 X 轴磁场强度，Gauss。 */
    float MY;           /**< NED 坐标系 Y 轴磁场强度，Gauss。 */
    float MZ;           /**< NED 坐标系 Z 轴磁场强度，Gauss。 */

    bool ovfl; /**< true=本次磁场测量发生 Overflow，数据不可信。 */
} MagData_t;

/**
 * @brief   初始化 QMC 磁力计。
 * 
 * 配置量程，Oversampling，ODR 及 Continuous Measurement Mod。
 * 
 * @return  HAL_OK  初始化成功。
 * @return  其他值 I2C 通信失败对应的 HAL Error Code。
 */
HAL_StatusTypeDef QMC_Init(void);

/**
 * @brief   读取并转换一组磁力计数据。
 * 
 * 读取 Overflow 状态及三轴 Raw Data，
 * 转换为 Gauss 并映射到飞控统一使用的 NED 坐标系。
 * 
 * @return HAL_OK   读取成功。
 * @return 其他值   I2C 通信失败对应的 HAL Error Code。
 */
HAL_StatusTypeDef QMC_ReadData(void);

/**
 * @brief   复制最近一次磁力计采样结果。
 * 
 * @param[out]  out 调用方提东的磁力计数据对象。
 */
void QMC_CopyTo(MagData_t *out);

#endif
