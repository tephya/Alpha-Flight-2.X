/**
 * @file    bsp_sdcard.h
 * @brief   SD Card SPI Mode Black 读写接口。
 */

#ifndef __BSP_SDCARD_H
#define __BSP_SDCARD_H

#include "cmsis_os2.h"
#include <stdint.h>

/**
 * @brief   SD Card 地址类型。
 */
typedef enum
{
    SD_TYPE_UNKNOWN = 0, /**< 尚未完成卡类型识别。 */
    SD_TYPE_SDSC,        /**< Standard Capacity，Byte Addressing。 */
    SD_TYPE_SDHC         /**< High Capacity，Block Addressing。 */
} SdCardType_t;

/**
 * @brief   初始化 SD Card SPI Mode。
 * 
 * 创建 DMA 同步对象，执行 SPI Mode 上电初始化序列，
 * 并是被 SDSC / SDHC 地址类型。
 * 
 * @return 0    初始化成功。
 * @return <0   初始化失败，具体错误码见实现。
 */
int8_t BSP_SD_Init(void);

/**
 * @brief   读取一个 512 Byte SD Block。
 * 
 * @param[in]   block   BLack Index。
 * @param[out]  buf     接收 Buffer，容量至少 512 Byte。
 * 
 * @return 0    读取成功。
 * @return <0   读取失败。
 */
int8_t BSP_SD_ReadBlock(uint32_t block, uint8_t *buf);

/**
 * @brief   写入一个 512 Byte SD Block。
 * 
 * @param[in] block Block Index。
 * @param[in] buf   待写入的 512 Byte 数据。
 * 
 * @return 0    写入成功。
 * @return <0   写入失败。
 */
int8_t BSP_SD_WriteBlock(uint32_t block, const uint8_t *buf);

/**
 * @brief   将 SPI2 切换到 SD Card 运行阶段的最高安全频率。
 */
void BSP_SD_SetSpeedFast(void);

#endif
