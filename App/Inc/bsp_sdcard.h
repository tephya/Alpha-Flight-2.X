#ifndef __BSP_SDCARD_H
#define __BSP_SDCARD_H

#include "cmsis_os2.h"
#include <stdint.h>

typedef enum
{
    SD_TYPE_UNKNOWN = 0,
    SD_TYPE_SDSC = 0, // 字节寻址
    SD_TYPE_SDHC = 1, // Block寻址
} SdCardType_t;

int8_t BSP_SD_Init(void);
int8_t BSP_SD_ReadBlock(uint32_t block, uint8_t *buf);
int8_t BSP_SD_WriteBlock(uint32_t block, const uint8_t *buf);
void BSP_SD_SetSpeed(uint32_t prescaler);

#endif
