#include "blackbox.h"
#include "ff.h"
#include <stdio.h>
#include <string.h>

static FATFS fs;		// 文件系统对象
static FIL fil;			// 文件对象
static char line_buf[256];


/**
  * @brief  初始化黑匣子 (Blackbox) 文件系统并创建新日志文件
  * @note   自动遍历寻找 LOG001.CSV ~ LOG999.CSV 中第一个未被占用的文件名，
  * 完成创建并写入 CSV 数据表头。
  * @retval  0 : 初始化成功
  * -1 : SD 卡挂载失败 (FATFS mount error)
  * -2 : 文件创建/打开失败
  * -3 : 表头写入失败
  */
int8_t BB_Init(void){
	char filename[16];
	FILINFO	fno;
	UINT bw;			// 实际写入字节数
	int8_t res = 0;     // [修复Bug] 必须初始化为 0，否则后续判断会使用随机内存值

	if(f_mount(&fs, "", 1) != FR_OK) return -1;		// 挂载SD card，同时FATFS自动调用SD_Init()
	
	// 找第一个不存在的LOGxxx.CSV
	for(int i = 1; i < 1000; i++){
		snprintf(filename, sizeof(filename), "LOG%03d.CSV", i);
		if(f_stat(filename, &fno) == FR_NO_FILE){
			// 这个不存在，就用它
			break;
		}
	}
	
	if(f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return -2;
	
	const char *header =
		"time_ms,"
		"gx,gy,gz,"
		"ax,ay,az,"
		"mx,my,mz,"
		"roll,pitch,yaw,"
		"m1,m2,m3,m4,"
		"roll_cmd,pitch_cmd,yaw_cmd,"
		"roll_target,pitch_target,yaw_rate_target,"
		"roll_rate_target,pitch_rate_target,"
		"thr,"
		"ch0,ch1,ch2,ch3,ch4,"
		"vbat,current,thr_lim,fs,rth,"
		"lat,lon,sat\r\n";
        
    // 获取 f_write 的返回值赋给 res
	res = f_write(&fil, header, strlen(header), &bw);
	if (res != FR_OK || bw != strlen(header)) return -3;
	
	return 0;
}

/**
  * @brief  记录单帧飞行数据到黑匣子日志文件
  * @note   [性能警告] 内部包含大量浮点数 (%%.2f) 的 snprintf 格式化转换以及 
  * 阻塞式 SD 卡写入。强烈建议在 Main Loop 中以低频 (如 500ms/2Hz) 调用，
  * 绝对禁止将其放入 TIM 高频中断或姿态解算核心链路中！
  * @param  f 指向当前飞行状态数据帧 (BB_Frame_t) 的指针
  * @retval  0 : 单帧写入成功
  * -1 : 写入失败
  */
int8_t BB_Log(BB_Frame_t *f){
	int len;
	UINT bw;

	len = snprintf(line_buf, sizeof(line_buf),
		"%u,"
		"%.2f,%.2f,%.2f,"
		"%.2f,%.2f,%.2f,"
		"%.2f,%.2f,%.2f,"
		"%.2f,%.2f,%.2f,"
		"%u,%u,%u,%u,"
		"%.2f,%.2f,%.2f,"
		"%.2f,%.2f,%.2f,"
		"%.2f,%.2f,"
		"%u,"
		"%u,%u,%u,%u,%u,"
		"%.2f,%.2f,%.0f,%u,%u,"
		"%.6f,%.6f,%u\r\n",

		f->time_ms,
		f->gx, f->gy, f->gz,
		f->ax, f->ay, f->az,
		f->mx, f->my, f->mz,
		f->roll, f->pitch, f->yaw,
		f->m1, f->m2, f->m3, f->m4,
		f->pro, f->ppo, f->pyo,
		f->target_roll, f->target_pitch, f->target_yaw,
		f->roll_rate_target, f->pitch_rate_target,
		f->throttle,
		f->ch0, f->ch1, f->ch2, f->ch3, f->ch4,
		f->vbat, f->current, f->throttle_limit,
		f->failsafe_active, f->rth_state,
		f->latitude, f->longitude, f->satellites);

	if(f_write(&fil, line_buf, len, &bw) != FR_OK) return -1;
	return 0;
}

/**
  * @brief  安全关闭黑匣子文件，刷新缓冲
  * @note   将 FAT 簇链和目录项物理写入 SD 卡。建议在检测到飞机锁定 (Disarm) 
  * 或触发严重失效 (Failsafe) 坠机前调用，防止最后一批日志丢失。
  * @retval  0 : 成功关闭
  */
int8_t BB_Close(void){
	f_close(&fil);
	return 0;
}