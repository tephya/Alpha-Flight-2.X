/**
 * @file    bsp_icm42688.h
 * @brief   双 ICM IMU 驱动接口。
 */

#ifndef __BSP_ICM_42688_H
#define __BSP_ICM_42688_H

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include <stdbool.h>

/**
 * @brief   IMU Data Ready Event Flags 句柄。
 * 
 * 由 IMU Redundancy Task 等上层模块等待，
 * ISR 在对应 IMU DRDY 到达时设置相应 Flag。
 */
extern osEventFlagsId_t g_icmDataReadyEvtId;
#define ICM1_DRDY_FLAG (1U << 0)    // IMU1 Data Ready Event Flag。
#define ICM2_DRDY_FLAG (1U << 1)    // IMU2 Data Ready Event Flag。
    
#define ICM_ODR_HZ 800  // IMU 输出速率，Hz。

/**
 * @brief   ICM 实例编号。
 */
typedef enum
{
    ICM_INSTANCE_1 = 0, /**< IMU1：SPI1，CS=PC4，INT1=PA4。 */
    ICM_INSTANCE_2 = 1, /**< IMU2：SPI3，CS=PB3，INT1=PA15。 */

    ICM_INSTANCE_MAX    /**< ICM 实例总数，仅用于数组边界。 */
} IcmInstance_t;

/**
 * @brief 单颗 ICM 最近一次有效采样数据。
 */
typedef struct
{
    float ax;           /**< X 轴加速度，g。 */
    float ay;           /**< Y 轴加速度，g。 */
    float az;           /**< Z 轴加速度，g。 */

    float gx;                   /**< X 轴角速度，deg/s。 */
    float gy;                   /**< Y 轴角速度，deg/s。 */
    float gz;                   /**< Z 轴角速度，deg/s。 */

    uint32_t timestamp_cycle; /**< 本次有效读取完成时的 DWT->CYCCNT 快照。 */
} IcmData_t;

/**
 * @brief   初始化全部 ICM 实例。
 * 
 * 完成 Event Flags 创建，两颗 IMU 的寄存器初始化，
 * 并主动读取一次数据寄存器以清除上电阶段可能已经锁存的 DRDY 状态。
 * 
 * @return  初始化失败位掩码：
 *          bit0=IMU1 初始化失败；
 *          bit1=IMU2 初始化失败；
 *          0=全部成功。
 */
uint8_t ICM_InitAll(void);

/**
 * @brief   IMU Data Ready 外部中断分发入口。
 * 
 * 由 GPIO EXTI Callback 根据实际触发引脚调用，
 * 这里只设置对应 Event Flag，不执行 SPI 读取。
 * 
 * @param[in] inst  触发的 DRDY 的 ICM 实例。
 */
void ICM_OnDataReady_ISR(IcmInstance_t inst);

/**
 * @brief   读取并转换指定 ICM 实例的一组 Accel / Gyro 数据。
 * 
 * @param[in] inst  ICM 实例。
 */
void ICM_TriggerRead(IcmInstance_t inst);

/**
 * @brief   复制指定 ICM 实例最近一次有效采样。
 * 
 * @param[in] inst  ICM 实例。
 * @param[out] out  调用方提供的数据对象。
 */
void ICM_CopyTo(IcmInstance_t inst, IcmData_t *out);

#endif
