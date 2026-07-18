#ifndef __BLACKBOX_H
#define __BLACKBOX_H

#include "stm32f4xx.h"
#include "ff.h"

#define LOG_BUFF_SIZE	512U

/*
 * 定义SD生产者结构体，单次记录 19 Bytes
 * 作者用的 SD 卡为 Block(512Bytes) 写入，因此记录 27 帧后就写入一次 SD 卡
 * 当前结构体下，第27帧跨Block边界，前18 Byte进入当前Block，最后1 Byte成为下一Block的开头
 */
typedef __packed struct {
	// 状态量
    uint16_t time_ms;
	
	// 测量值
	/*
	 * roll、pitch 范围 -π ~ π rad，保留4位小数记录
	 * Yaw 范围 -2π ~ 2π rad， 缩放5000
	 */
	int16_t angle_cdeg[3];

	// 输出值
    uint16_t motor[4];	
	
	// 目标值
	/*
	 * target roll、pitch 范围 -20° ~ 20°
	 * target yaw 范围 -90° ~ 90°
	 */
    int8_t target_cdeg[3];
} BB_Frame_t;

/* 定义消费者状态 */
typedef enum
{
	BB_BUF_FREE,
	BB_BUF_FILLING,
	BB_BUF_READY,
	BB_BUF_WRITING,
	BB_BUF_ERROR
} BB_BufferState_t;

/* 定义SD消费者结构体 */
typedef struct {
	uint8_t data[LOG_BUFF_SIZE]; // 缓冲区
	uint16_t pos;				 // 
	volatile BB_BufferState_t state;
} BB_Buffer_t;

int8_t BB_Init(void);               		// 初始化：挂载SD卡，创建日志文件
void BB_BufferInit(void);
int8_t BB_Log(const BB_Frame_t *f);        // 写入一帧
void BB_Process();
int8_t BB_Close(void);              		// 关闭文件

#endif