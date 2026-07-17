#include "blackbox.h"

#include <stdio.h>
#include <string.h>

static FATFS fs;		// 文件系统对象
static FIL fil;			// 文件对象
static char log_buf[LOG_BUFF_SIZE] = {0};		// SD 卡写入缓冲区
static uint16_t log_pos = 0;	// 下一次写入缓冲区开始的位置

// 检查结构体大小是否发生改变
typedef char BB_Frame_Size_Check[
	(sizeof(BB_Frame_t) == FRAME_SIZE) ? 1 : -1
];


/**
  * @brief  初始化黑匣子 (Blackbox) 文件系统并创建新日志文件
  * @note   自动遍历寻找 LOG001.CSV ~ LOG999.CSV 中第一个未被占用的文件名，
  * 		完成创建并写入 CSV 数据表头。
  * @retval  0 : 初始化成功
  * 		-1 : SD 卡挂载失败 (FATFS mount error)
  * 		-2 : 文件创建/打开失败
  */
int8_t BB_Init(void){
	char filename[16];
	FILINFO	fno;
	UINT bw;			// 实际写入字节数
	int8_t res = 0;     // [修复Bug] 必须初始化为 0，否则后续判断会使用随机内存值

	if(f_mount(&fs, "", 1) != FR_OK) return -1;		// 挂载SD card，同时FATFS自动调用SD_Init()
	
	// 找第一个不存在的LOGxxx.CSV
	for(int i = 1; i < 1000; i++){
		snprintf(filename, sizeof(filename), "LOG%03d.BIN", i);
		if(f_stat(filename, &fno) == FR_NO_FILE){
			// 这个不存在，就用它
			break;
		}
	}
	
	if(f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return -2;
	
	return 0;
}

/**
  * @brief  记录单帧飞行数据到黑匣子日志文件
  * @param  record 指向当前飞行状态数据帧 (BB_Frame_t) 的指针
  * @retval  0 : 单帧写入成功
  *			 1 : FR_DISK_ERR
  * 		 res : 其他 FRESULT
  */
FRESULT BB_Log(const BB_Frame_t *record){
	const uint8_t *src = (const uint8_t *)record;
	uint16_t remaining = FRAME_SIZE;
	
	while(remaining > 0){
		uint16_t space = LOG_BUFF_SIZE - log_pos;		// 缓冲区剩余可用空间
		uint16_t copy_size = (remaining < space) ? remaining : space;
		
		memcpy(&log_buf[log_pos], src, copy_size);
		
		// 如果本次没拷贝完，会将剩余部分作为下一 Block 的头
		log_pos += copy_size;
		src += copy_size;
		remaining -= copy_size;
		
		// 一个 块 大小的缓冲区写满后，写入 SD 卡
		if(log_pos == LOG_BUFF_SIZE){
			UINT bw;
			
			FRESULT result = f_write(&fil, log_buf, LOG_BUFF_SIZE, &bw);
			
			if(result != FR_OK || bw != LOG_BUFF_SIZE)
				return (result != FR_OK) ? result : FR_DISK_ERR;
			
			log_pos = 0;
		}
	}
	
	return FR_OK;
}

/**
  * @brief  安全关闭黑匣子文件，刷新缓冲
  * @note   将 FAT 簇链和目录项物理写入 SD 卡。建议在检测到飞机锁定 (Disarm) 
  * 		或触发严重失效 (Failsafe) 坠机前调用，防止最后一批日志丢失。
  * @retval  0 : 成功关闭
  */
int8_t BB_Close(void){
	f_close(&fil);
	return 0;
}