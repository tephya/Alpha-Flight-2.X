/**
 * @file    bsp_elrs.h
 * @brief   ELRS / CRSF 遥控数据接收与解析接口。
 */

#ifndef __BSP_ELRS_H
#define __BSP_ELRS_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief   ELRS 接收端发布的最新 RC 状态。
 */
typedef struct
{
    uint16_t channels[16]; /**< 16 路 CRSF Channel 原始值，11-bit 编码。 */

    uint8_t rssi;       /**< Uplink RSSI，信号强度。 */
    uint8_t lq;         /**< Uplink Link Quality，0~100。 */
    int8_t snr;         /**< Uplink SNR，信噪比。 */

    bool link_ok; /**< 最近是否持续收到有效 RC Channel Frame。 */
} RCChannelData_t;

/**
 * @brief   初始化 ELRS UART DMA 接收。
 * 
 * @note    USART2 与 RX DMA 参数由 CubeMX 初始化；
 *          RX DMA 必须配置为 Circular Mode。
 */
void ELRS_Init(void);

/**
 * @brief   消费 UART DMA Circular Buffer 中的新数据并解析 CRSF Frame。
 * 
 * 同时根据最近一条有效 RC Channel Frame 的时间维护 Link Failsafe 状态。
 */
void ELRS_Poll(void);

/**
 * @brief   复制当前最新 RC 状态。
 * 
 * @param[out] out  RC 状态输出对象。
 */
void ELRS_CopyTo(RCChannelData_t *out);

#endif
