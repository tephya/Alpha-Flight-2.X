/**
 * @file    bsp_power.h
 * @brief   电池电压与电流 ADC 采样接口。
 */

#ifndef __BSP_POWER_H
#define __BSP_POWER_H

#include <stdint.h>

/**
 * @brief   电源采样数据。
 */
typedef struct
{
    float vbat;         /**< 电池总线电压，V。 */
    float current;      /**< 电调电流检测值，A。 */
} PowerData_t;

/**
 * @brief   初始化电源采样模块的软件状态。
 * 
 * ADC GPIO，Channel，Rank 等底层配置由 CubeMX 生成代码完成。
 */
void BSP_Power_Init(void);

/**
 * @brief   执行一次电池电压与电流 ADC Scan，并返回转换结果。
 * 
 * @param[out] out  调用方提供的电源数据对象。
 */
void BSP_Power_Read(PowerData_t *out);

#endif
