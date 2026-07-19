#include "blackbox.h"

#include <stdio.h>
#include <string.h>

#define FRAME_SIZE 19U

static FATFS fs;		// 文件系统对象
static FIL fil;			// 文件对象

static BB_Buffer_t bb_buf[2];
static uint8_t fill_index = 0; // 当前正在写入缓冲区的下标
static uint8_t overflow = 0;
static volatile uint8_t bb_write_error = 0;

// 检查结构体大小是否发生改变
typedef char BB_Frame_Size_Check[(sizeof(BB_Frame_t) == FRAME_SIZE) ? 1 : -1];

/**
 * @brief  初始化黑匣子 (Blackbox) 文件系统并创建新日志文件
 * @note   自动遍历寻找 LOG001.CSV ~ LOG999.CSV 中第一个未被占用的文件名，
 * 		完成创建并写入 CSV 数据表头。
 * @retval  0 : 初始化成功
 * 		-1 : SD 卡挂载失败 (FATFS mount error)
 * 		-2 : 文件创建/打开失败
 */
int8_t BB_Init(void)
{
	char filename[16];
	FILINFO	fno;
	FRESULT res;
	
	res = f_mount(&fs, "", 1);
	if(res != FR_OK) return -1;		// 挂载SD card，同时FATFS自动调用SD_Init()
	
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
 * @brief	初始化消费者状态
 */
void BB_BufferInit(void)
{
	bb_buf[0].pos = 0;
	bb_buf[0].state = BB_BUF_FILLING;

	bb_buf[1].pos = 0;
	bb_buf[1].state = BB_BUF_FREE;

	fill_index = 0;
}

/**
 * @brief	转换写入缓冲区，检查并更新消费者状态
 * @note	1块缓冲区写满后，自动调用此函数
 * @retval	-1 : 缓冲区溢出（两块缓冲区都没被消费完）
 * 			 0 : 转换消费者状态成功
 */
static int8_t BB_PublishCurrentBlock()
{
	uint8_t next = fill_index ^ 1U;

	/* 另一块还没写完，两块 buffer 都被占用 */
	if(bb_buf[next].state != BB_BUF_FREE){
		overflow = 1;
		return -1;
	}

	bb_buf[fill_index].state = BB_BUF_READY;

	fill_index = next;
	bb_buf[fill_index].pos = 0;
	bb_buf[fill_index].state = BB_BUF_FILLING;

	return 0;
}

/**
  * @brief  填充 Buffer
  * @param  record 指向当前飞行状态数据帧 (BB_Frame_t) 的指针
  * @retval  0 : 单帧写入成功
  *			 1 : FR_DISK_ERR
  * 		 res : 其他 FRESULT
  */
int8_t BB_Log(const BB_Frame_t *record){
	const uint8_t *src = (const uint8_t *)record;
	uint16_t remaining = FRAME_SIZE;
	
	while(remaining > 0){
		uint16_t space = LOG_BUFF_SIZE - bb_buf[fill_index].pos;		// 缓冲区剩余可用空间
		uint16_t copy_size = (remaining < space) ? remaining : space;
		
		memcpy(&bb_buf[fill_index].data[bb_buf[fill_index].pos], src, copy_size);
		
		// 如果本次没拷贝完，会将剩余部分作为下一 Block 的头
		bb_buf[fill_index].pos += copy_size;
		src += copy_size;
		remaining -= copy_size;
		
		// 一个 块 大小的缓冲区写满后，写入 SD 卡
		if (bb_buf[fill_index].pos == LOG_BUFF_SIZE)
		{
			if(BB_PublishCurrentBlock() != 0)
				return -1;
		}
	}
	
	return 0;
}

/**
  * @brief  消费者消费(将缓冲区内容写入SD)，并检查更新消费者状态
  */
void BB_Process(){
	int8_t index = -1;

	if(bb_buf[0].state == BB_BUF_READY)
		index = 0;
	else if(bb_buf[1].state == BB_BUF_READY)
		index = 1;
	else
		return;

	bb_buf[index].state = BB_BUF_WRITING;

	UINT bw;
	
	FRESULT result = f_write(&fil, bb_buf[index].data, LOG_BUFF_SIZE, &bw);
	
	if(result == FR_OK && bw == LOG_BUFF_SIZE){
		bb_buf[index].pos = 0;
		bb_buf[index].state = BB_BUF_FREE;
	}else{
		bb_write_error = 1;
		bb_buf[index].state = BB_BUF_ERROR;
	}
}

/**
 * @brief  将当前未满 512 Byte 的尾部缓存写入日志文件
 * @note   该函数仅处理当前 FILLING Buffer 中尚未写出的有效数据。
 *         调用前应确保所有 BB_BUF_READY 状态的完整 Buffer 已完成写入。
 * @retval FR_OK       : 尾部数据写入并同步成功
 * @retval FR_DISK_ERR : 实际写入长度与请求长度不一致
 * @retval 其他值      : FatFS 返回的具体错误码
 */
static FRESULT BB_Flush(void)
{
	/* 获取当前填充缓冲区中的有效数据长度 */
	uint16_t len = bb_buf[fill_index].pos;

	/* 缓冲区为空，无需写入 */
	if (len == 0)
		return FR_OK;

	UINT written = 0;

	/* 仅写入实际有效数据，不使用无效数据补满 512 Byte */
	FRESULT result = f_write(
		&fil,
		bb_buf[fill_index].data,
		len,
		&written);

	/* 检查 FatFS 返回值和实际写入长度 */
	if (result != FR_OK || written != len)
		return (result != FR_OK) ? result : FR_DISK_ERR;

	/* 尾部数据已提交，清空当前缓冲区位置 */
	bb_buf[fill_index].pos = 0;

	/* 将文件数据、FAT 簇链和目录信息同步至存储介质 */
	return f_sync(&fil);
}

/**
 * @brief  安全停止 Blackbox 记录并关闭日志文件
 * @note   调用该函数前，应停止继续调用 BB_Log()，并确保完整的
 *         READY Buffer 已由 BB_Process() 写入。
 *         函数会将当前不足 512 Byte 的尾部数据写出，然后关闭文件。
 * @retval  0 : 日志尾部写入且文件关闭成功
 *         -1 : 尾部数据写入或同步失败
 *         -2 : 文件关闭失败
 */
int8_t BB_Close(void)
{
	FRESULT flush_result = BB_Flush();
	FRESULT close_result = f_close(&fil);

	if (flush_result != FR_OK)
		return -1;

	if (close_result != FR_OK)
		return -2;

	return 0;
}