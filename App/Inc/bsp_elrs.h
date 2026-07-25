#ifndef __BSP_ELRS_H
#define __BSP_ELRS_H

#include <stdint.h>
#include <stdbool.h>

// CRSF一帧最多16个通道，11-bit压缩格式，原始值范围172~1811
typedef struct
{
    uint16_t channels[16]; // 16路遥控通道值(172~1811)
    uint8_t rssi;          // 信号强度
    uint8_t lq;         // ELRS链路信号质量(LQ)(0~100)
    int8_t snr;         // 信噪比
    bool link_ok;       // 距离上一次解析成功的有效帧是否超过failsafe阈值
} RCChannelData_t;

void ELRS_Init(void);
void ELRS_Poll(void);
void ELRS_CopyTo(RCChannelData_t *out);

#endif
