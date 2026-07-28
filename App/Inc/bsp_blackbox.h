#ifndef __BSP_BLACKBOX_H
#define __BSP_BLACKBOX_H

#include <stdint.h>

/*====== 帧同步：每条记录的第一字节固定是这个magic，供上位机解析器丢失同步后重新扫描定位 ======
 * 记录格式统一为：[MAGIC(1B)][type(1B)][...记录内容...]
 * 后续解析对不上号时，逐字节向后扫描找下一个MAGIC字节，从哪里重新按type分发，
 * 代价是每条记录多1字节开销*/
#define BB_FRAME_MAGIC 0xAAU

/*====== 帧类型tag，写入文件的每条记录第一字节，供离线解析脚本区分记录种类 ======*/
typedef enum
{
    BB_REC_MOTION = 0x01,        // 高频运动帧
    BB_REC_ARM_CHANGED = 0x02,       // 低频事件：解锁状态变化
    BB_REC_DUAL_FAULT = 0x03,    // 低频事件：双IMU失效
    BB_REC_VOLTAGE_FAULT = 0x04, // 低频事件：持续低压
    BB_REC_IMU_SWITCH = 0x05,    // 低频事件：主备IMU切换
} BB_RecType_t;

int8_t BB_Init(void);
void BB_BufferInit(void);
/*====== 生产者接口：各Task/模块调用，写入一条记录 ======*/
/* 高频运动帧：由Task_FlightCtrl每控制周期调用 */
int8_t BB_LogMotion(uint16_t time_ms, const int16_t angle_cdeg[3],
                    const uint16_t motor[4], const int8_t target_cdeg[3]);

/* 低频事件帧，只在状态跳变时调用 */
int8_t BB_LogArmChanged(uint16_t time_ms, uint8_t armed);
int8_t BB_LogDualFault(uint16_t time_ms);
int8_t BB_LogVoltageFault(uint16_t time_ms);
int8_t BB_LogImuSwitch(uint16_t time_ms, uint8_t new_active_imu);

int8_t BB_WaitReady(uint32_t timeout_ms);
void BB_Process(void);
int8_t BB_Close(void);
uint8_t BB_GetErrorFlags(void);

#endif
