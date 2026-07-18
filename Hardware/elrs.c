#include "elrs.h"
#include "Buzzer.h"

#define BUFF_SIZE 64	/* Sync(1) + Len(1) + Type(1) + Payload(0~60) + CRC8(1) */
#define DMA_BUF_SIZE 128


static uint8_t state = 0;		/* 当前状态 */
static uint8_t idx = 0;			/* 缓冲区写入位置 */
static volatile uint8_t frame_len = 0;	/* 还要收多少字节 */
static volatile uint8_t frame_ready = 0;	/* 帧接收完毕标志， 0： 帧接收完毕， 1： 帧未接收完 */
static uint16_t read_ptr = 0;

/* 帧接收缓冲区 
 * 大小： 64 Bytes */
static uint8_t frame_buf[BUFF_SIZE];	
uint8_t dma_rx_buf[DMA_BUF_SIZE];

CRSF_Data_t crsf_data = {0};
CRSF_Data_t temp_data = {0};

/* 8bit CRC8 码表 
 * 查表方式速度更快 */
static const uint8_t crc8_table[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
    0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
    0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
    0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
    0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
    0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
    0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
    0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
    0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
    0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
    0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};


/**
  * @brief  初始化 ELRS (CRSF) 串口接收与 DMA 外设
  * @note   使用 USART2 (PA2/PA3)，波特率 420000 bps (CRSF 标准)。
  * 		开启 DMA 循环模式 (Circular)，硬件自动将串口数据搬运至 dma_rx_buf，
  * 		全程零 CPU 介入，杜绝数据阻塞。
  * @param  None
  * @retval None
  */
void ELRS_Init(void)
{

	USART_InitTypeDef usart;
	GPIO_InitTypeDef gpio;
	DMA_InitTypeDef dma;
	
	/* 开启时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	/* USART引脚功能重复用 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);	// TX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);	// RX
	
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);
	
	usart.USART_BaudRate = 420000;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &usart);
	
	DMA_DeInit(DMA1_Stream5);
	dma.DMA_Channel = DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
	dma.DMA_Memory0BaseAddr = (uint32_t)dma_rx_buf;
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma.DMA_BufferSize = DMA_BUF_SIZE;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_VeryHigh;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5, &dma);
		
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	USART_Cmd(USART2, ENABLE);
	DMA_Cmd(DMA1_Stream5, ENABLE);
}

/**
  * @brief  轮询读取 DMA 接收环形缓冲区中的新数据
  * @note   通过计算 DMA 硬件剩余传输量得出 write_ptr，与本地 read_ptr 进行追赶。
  * 		将追赶过程中的新字节逐个喂入 CRSF 协议状态机进行组帧。
  * @param  None
  * @retval None
  */
void ELRS_Poll(void)
{
	uint16_t counter =  DMA_GetCurrDataCounter(DMA1_Stream5);
	uint16_t write_ptr = (DMA_BUF_SIZE - counter) % DMA_BUF_SIZE;
	
	/* 当读写指针不一致时，说明有新数据 */
	while(read_ptr != write_ptr){
		uint8_t byte = dma_rx_buf[read_ptr];
		
		ELRS_StateMachine(byte);
		/* 指针循环移动 */
		read_ptr = (read_ptr + 1) % DMA_BUF_SIZE;
	}
}	

/**
  * @brief  CRSF 协议帧解析状态机 (组帧)
  * @note   标准 CRSF 帧结构: [Sync(0xC8)] [Length] [Type] [Payload] [CRC8]
  * 		严格处理了上一帧未被消费时的丢包逻辑，防止半帧覆盖引发系统崩溃。
  * @param  byte  来自串口 DMA 的单个字节
  * @retval None
  */
void ELRS_StateMachine(uint8_t byte)
{	
	/* 状态机判断 */
	switch(state)
	{
		case 0:						/* 等Sync */
			if(byte == 0xC8)
			{
			if(frame_ready) break;		/* 上一帧还没被解析，果断丢弃新帧保平安 */
				state = 1;
				frame_buf[idx++] = 0xC8;
			}
			break;
			
		case 1: 					/* 读Len */
			if(byte > 62){			/* Len最大62，防止噪声干扰导致数组越界 */
				state = 0;
				idx = 0;
				break;
			}
			state = 2;
			frame_buf[idx++] = byte;
			frame_len = byte + 2;   /* 总帧长 = Sync(1) + Len(1) + Len里的数值 */
			break;
			
		case 2:						/* 收数据 */
			frame_buf[idx++] = byte;
			if(idx == frame_len)
			{
				state = 0;
				frame_ready = 1;        /* 完整帧组装完毕，等待上层解析 */
				idx = 0;
			}
			break;
			
		default:	
			return;
	}
}

/**
  * @brief  计算 CRSF 数据帧的 CRC8 校验码
  * @note   CRSF 协议规范：CRC 校验范围仅包含 Type 字节 和 Payload 数据区，
  * 		不包含 Sync 字节(0) 和 Length 字节(1)。
  * @param  None
  * @retval 计算出的 8 位 CRC 校验值
  */
uint8_t CRSF_CRC8(void){
	uint8_t crc = 0;

	for(int i = 2; i < frame_len - 1; i++)
		crc = crc8_table[crc ^ frame_buf[i]];
		
	return crc;
}

/**
  * @brief  消费并解析已就绪的 CRSF 数据帧 (解包)
  * @note   支持解析 0x16 帧 (RC 通道，11-bit 高效压缩格式) 和 
  * 		0x14 帧 (链路统计信息 Link Statistics)。
  * 		只有通过了严格的 CRC8 校验的数据才会被更新到系统 temp_data 中。
  * @param  None
  * @retval  1 : 成功解析并更新了一帧数据
  * 		-1 : 无就绪帧，或 CRC 校验失败 (脏数据)
  */
int8_t CRSF_PARSE_FRAME(void)
{
	uint8_t offset, byte_index, start_bit;
	uint32_t raw;
	
	if(frame_ready)
	{
		frame_ready = 0;		/* 消费该帧，清除就绪标志 */
		
		if(frame_buf[frame_len - 1] != CRSF_CRC8()) return -1;		/* CRC 校验失败，拒绝解析 */
		
		switch(frame_buf[2])					/* 根据帧类型(Type)进行分发处理 */
		{
			case 0x16:  /* 摇杆通道数据 */
			{
				for(int i = 0; i < 16; i++){
					start_bit = i * 11;
					byte_index = start_bit / 8;
					offset = start_bit % 8;				
					{		
					    /* 跨 3 个字节读取 32 位原始数据，以防 11 bit 跨越边界 */
						raw = (uint32_t)frame_buf[byte_index + 3] 		/* 跳过 Sync + Len + Type (3Bytes) */
							| (uint32_t)frame_buf[byte_index + 4] << 8
							| (uint32_t)frame_buf[byte_index + 5] << 16;
					}				
					/* 右移偏移量，并用 0x7FF (二进制 11 个 1) 掩码提取真实的 11 位通道值 */
					temp_data.channels[i] = (raw >> offset) & 0x7FF;
				}
				break;
			}
			
			case 0x14:  /* 链路质量数据 */
			{
				temp_data.rssi = frame_buf[3];  // RSSI 天线信号强度
				temp_data.lq = frame_buf[5];    // LQ 链路质量 (0-100%)
				temp_data.snr = frame_buf[6];   // SNR 信噪比
				break;
			}
			
			default:
				break;
		}
		temp_data.updated = 1;				/* 数据有效更新标志位 */
		return 1;
	}
	else
		return -1;			/* 数据正在收集中 */
}