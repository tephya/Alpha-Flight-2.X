#ifndef __BLACKBOX_H
#define __BLACKBOX_H

#include "stm32f4xx.h"
#include "ff.h"

#define LOG_BUFF_SIZE	512
#define FRAME_SIZE		19U

/*
 * 定义块内紧凑黑盒记录结构体，单次记录 19 Bytes
 * 作者用的 SD 卡为 Block(512Bytes) 写入，因此记录 27 帧后就写入一次 SD 卡
 * 当前结构体下，最后一帧会少记录一个 Byte，留存为下一个 Block 的头
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



int8_t BB_Init(void);               		// 初始化：挂载SDka，创建日志文件
FRESULT BB_Log(const BB_Frame_t *f);        	// 写入一帧
int8_t BB_Close(void);              		// 关闭文件

#endif