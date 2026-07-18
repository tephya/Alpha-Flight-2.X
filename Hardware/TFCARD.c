#include "TFCARD.h"
#include "Delay.h"

#include <stddef.h>

#define SD_CS_LOW() GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define SD_CS_HIGH()                      \
	do                                    \
	{                                     \
		uint8_t _r;                       \
		GPIO_SetBits(GPIOB, GPIO_Pin_12); \
		SD_SPI_RWByte(&_r, 0xFF);         \
	} while (0)

#define SD_DMA_TIMEOUT_MS	10U
#define SD_BUSY_TIMEOUT_MS	500U

uint8_t SD_CardType = 3;
static volatile uint8_t sd_dma_done = 0;
static volatile uint8_t sd_dma_error = 0;

/* SPI 全双工发送时必须读DR，避免RX溢出 */
static uint8_t sd_dma_rx_sink;

/**
  * @brief  初始化 SD 卡底层 SPI 外设与 GPIO。
  * @note   使用 SPI2 (PB13/SCK, PB14/MISO, PB15/MOSI)，CS 为普通 GPIO (PB12)。
  * 		根据 SD 卡协议规范，初始化阶段的 SPI 速率必须低于 400kHz，此处预分频设为 128。
  * @param  None
  * @retval None
  */
void SD_SPI_Init(void){
	// 开启时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	// SPI2引脚复用功能重映射
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);		// SCK
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);		// MISO
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);		// MOSI
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;		// CS
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;	// SD Initializtion need below 400kHz
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	// SD Support: Mode1 or Mode 3
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;		// SD Support
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI2, &SPI_InitStructure);
	
	SPI_Cmd(SPI2, ENABLE);
	
	SD_CS_HIGH();
}

/**
 * @brief  执行 SD 卡 SPI 模式的上电初始化时序。
 * @note   标准流程：发送至少 74 个 dummy clock -> CMD0 (复位) -> CMD8 (校验电压)
 * 			-> ACMD41 (内部初始化就绪等待) -> CMD58 (读取 OCR 寄存器判断 SDHC/SDSC)。
 * 			全局变量 SD_CardType 被更新以适配后续的 Block 寻址机制。
 * @param  None
 * @retval 0 成功；负数代表对应步骤初始化失败的错误码。
 */
int8_t SD_Init()
{
	uint8_t res, dummy, buf[4];

	SD_CS_HIGH();
	for (int i = 0; i < 10; i++)
	{
		SD_SPI_RWByte(&res, 0xFF);
	}

	// CMD0 带重试
	for (int i = 0; i < 10; i++)
	{
		SD_CS_LOW();
		res = SD_SendCmd(0, 0x00000000);
		SD_CS_HIGH();
		SD_SPI_RWByte(&dummy, 0xFF);
		if (res == 0x01)
			break;
	}
	if (res != 0x01)
		return -1;

	// CMD8
	SD_CS_LOW();
	res = SD_SendCmd(8, 0x000001AA);
	for (int i = 0; i < 4; i++)
	{
		SD_SPI_RWByte(&buf[i], 0xFF);
	}
	SD_CS_HIGH();
	if (res != 0x01)
		return -2;
	if (buf[3] != 0xAA)
		return -3;

	// ACMD41
	for (int i = 0; i < 500; i++)
	{
		SD_CS_LOW();
		SD_SendCmd(55, 0x00000000);
		SD_CS_HIGH();

		SD_CS_LOW();
		res = SD_SendCmd(41, 0x40000000);
		SD_CS_HIGH();
		if (res == 0x00)
			break;
	}
	if (res != 0x00)
		return -4;

	// CMD58
	SD_CS_LOW();
	res = SD_SendCmd(58, 0x00000000);
	for (int i = 0; i < 4; i++)
	{
		SD_SPI_RWByte(&buf[i], 0xFF);
	}
	SD_CS_HIGH();
	if (buf[0] & 0x40)
	{ // SDHC block寻址
		SD_CardType = 1;
	}
	else
	{ // SDSC 字节寻址
		SD_CardType = 0;
	}

	return 0;
}

/**
 * @brief	初始化 SPI2 TX/RX DMA。
 * @note	SPI2_RX(MISO): DMA1_Stream3_Ch0
 * 			SPI2_TX(MOSI): DMA1_Stream4_Ch0
 * @param 	timeout_ms: 允许SPI DMA传输的最大时间
 * @retval	None
 */
void SD_SPI_DMA_Init(void)
{
	DMA_InitTypeDef dma;
	NVIC_InitTypeDef nvic;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	/* SPI2_RX(MISO): DMA1 Stream3 Channel0 */
	DMA_DeInit(DMA1_Stream3);
	while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE)
		;

	dma.DMA_Channel = DMA_Channel_0;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
	dma.DMA_Memory0BaseAddr = (uint32_t)&sd_dma_rx_sink;
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma.DMA_BufferSize = 1;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Disable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Normal;
	dma.DMA_Priority = DMA_Priority_Low; // 仲裁时优先级较高的Stream先执行
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull; // 无关配置项，因为FIFO禁用了
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init(DMA1_Stream3, &dma);
	DMA_ITConfig(DMA1_Stream3,
				 DMA_IT_TC | DMA_IT_TE | DMA_IT_DME, // Transfer Complete | Error | Direct Mode Error
				 ENABLE);

	/* SPI2_TX(MOSI): DMA1 Stream4 Channel0 */
	DMA_DeInit(DMA1_Stream4);
	while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE)
		;

	dma.DMA_Memory0BaseAddr = (uint32_t)&sd_dma_rx_sink;
	dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;

	DMA_Init(DMA1_Stream4, &dma);
	DMA_ITConfig(DMA1_Stream4,
				 DMA_IT_DME | DMA_IT_TE,
				 ENABLE);

	/* SD DMA终端低于IMU、Control和GPS */
	nvic.NVIC_IRQChannel = DMA1_Stream3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 4;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	nvic.NVIC_IRQChannel = DMA1_Stream4_IRQn;
	NVIC_Init(&nvic);
}

static int8_t SD_DMA_WaitComplete(uint32_t timeout_ms)
{
	uint32_t start_ms = SysTick_ms;
	
	while(!sd_dma_done &&!sd_dma_error){
		if((uint32_t)(SysTick_ms - start_ms) >= timeout_ms){
			return -1;
		}

		/**
		 * CPU停止执行普通指令，等待中断唤醒。
		 * SysTick、控制中断和DMA完成中断均可唤醒。
		 */
		__DSB(); // 确保在 __DSB 之前发起的全部显式内存访问（读/写）彻底完成
		__WFI();
	}

	if(sd_dma_error)
		return -2;

	return 0;
}

/**
 * @brief	通过DMA传输缓冲区记录内容到SPI寄存器
 * @note	后续再写入SD中
 * @param	buf: 待写入SD的缓冲区
 * @param	len: 写入数据的长度，单位：Byte
 * @retval	 0: 传输成功
 * 			-1: 传输超时
 * 			-2:	传输错误
 */
int8_t SD_SPI_DMA_Transmit(const uint8_t *buf, uint16_t len)
{	
	if(buf == NULL || len == 0)
		return -1;

	SPI_DMACmd(SPI2, SPI_DMAReq_Tx, DISABLE);
	SPI_DMACmd(SPI2, SPI_DMAReq_Rx, DISABLE);

	while(DMA_GetCmdStatus(DMA1_Stream3) != DISABLE)
		;
	while(DMA_GetCmdStatus(DMA1_Stream4) != DISABLE)
		;

	DMA_ClearFlag(
		DMA1_Stream3,
		DMA_FLAG_FEIF3 | DMA_FLAG_DMEIF3 |
			DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3 |
			DMA_FLAG_TCIF3);

	DMA_ClearFlag(
		DMA1_Stream4,
		DMA_FLAG_FEIF4 | DMA_FLAG_DMEIF4 |
			DMA_FLAG_TEIF4 | DMA_FLAG_HTIF4 |
			DMA_FLAG_TCIF4);

	DMA1_Stream3->M0AR = (uint32_t)&sd_dma_rx_sink;
	DMA1_Stream3->NDTR = len;

	DMA1_Stream4->M0AR = (uint32_t)buf;
	DMA1_Stream4->NDTR = len;

	sd_dma_done = 0;
	sd_dma_error = 0;

	/* 先准备RX，避免第一个接收Byte丢失 
	 * 先使能 DMA 通道，保证开启瞬间就能响应收发请求 */
	DMA_Cmd(DMA1_Stream3, ENABLE);
	DMA_Cmd(DMA1_Stream4, ENABLE);

	SPI_DMACmd(SPI2, SPI_DMAReq_Rx, ENABLE);
	SPI_DMACmd(SPI2, SPI_DMAReq_Tx, ENABLE);

	/**
	 * 裸机架构下，先在这里等待，但IMU和Control高优先级中断仍可抢占
	 * FreeRTOS架构下替换为xSemaphoreTake()
	 */
	int8_t wait_result = SD_DMA_WaitComplete(SD_DMA_TIMEOUT_MS);

	DMA_Cmd(DMA1_Stream3, DISABLE);
	DMA_Cmd(DMA1_Stream4, DISABLE);

	SPI_DMACmd(SPI2, SPI_DMAReq_Tx, DISABLE);
	SPI_DMACmd(SPI2, SPI_DMAReq_Rx, DISABLE);

	if(wait_result != 0)
		return wait_result;

	/* 等待传输完成 */
	while (SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY) == SET)
		;

	return 0;
}

/**
 * @brief	DMA1 Stream3 中断服务函数
 * @note	用于处理 SPI RX DMA数据传输的完成与错误状态
 * @retval	None
 */
void DMA1_Stream3_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) != RESET){
		DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
		sd_dma_done = 1;
	}

	if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TEIF3) != RESET){
		DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TEIF3);
		sd_dma_error = 1;
	}

	if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_DMEIF3) != RESET){
		DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_DMEIF3);
		sd_dma_error = 1;
	}
}

/**
 * @brief	DMA1 Stream4 中断服务函数
 * @note	用于处理 SPI TX DMA数据传输的完成与错误状态
 * @retval	None
 */
void DMA1_Stream4_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream4, DMA_IT_TEIF4) != RESET){
		DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TEIF4);
		sd_dma_error = 1;
	}

	if (DMA_GetITStatus(DMA1_Stream4, DMA_IT_DMEIF4) != RESET)
	{
		DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_DMEIF4);
		sd_dma_error = 1;
	}
}

/**
  * @brief  动态切换 SPI 通信速率
  * @note   在 SD_Init 初始化成功后调用，降低预分频值（如 SPI_BaudRatePrescaler_2）以切入全速模式，
  * 提升 FatFS 的数据吞吐量。
  * @param  prescaler: SPI 波特率预分频参数 (例如 SPI_BaudRatePrescaler_X)
  * @retval None
  */
void SD_SPI_SetSpeed(uint16_t prescaler)
{
	SPI_Cmd(SPI2, DISABLE);
	SPI2->CR1 = (SPI2->CR1 & ~SPI_CR1_BR) | prescaler;
	SPI_Cmd(SPI2, ENABLE);
}

/**
  * @brief  底层阻塞式 SPI 单字节全双工收发
  * @note   轮询 TXE 标志发送数据，轮询 RXNE 标志接收数据。
  * @param  RXD: 指向接收数据存储地址的指针
  * @param  TXD: 需要发送的单字节数据
  * @retval None
  */
void SD_SPI_RWByte(uint8_t *RXD, uint8_t TXD)
{
	while(SPI_GetFlagStatus(SPI2, SPI_FLAG_TXE) != SET) continue;
	SPI_SendData(SPI2, TXD);
	while(SPI_GetFlagStatus(SPI2, SPI_FLAG_RXNE) != SET) continue;
	*RXD = SPI_ReceiveData(SPI2);
}

/**
  * @brief  向 SD 卡发送标准 6 字节 Command 并等待响应
  * @note   Command 帧格式：1Byte CMD Index + 4Byte Argument + 1Byte CRC。
  * 默认情况下 SPI 模式禁用 CRC 校验，但 CMD0 和 CMD8 强制要求合法 CRC。
  * @param  cmd: SD 卡指令序号 (例如 0, 8, 41, 55, 58)
  * @param  arg: 32-bit 指令参数
  * @retval SD 卡返回的 R1 响应字节 (Response)
  */
uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg)
{
	uint8_t res, dummy;
	uint8_t crc = 0x01;			// 默认：SD卡SPI模式下不校验，停止位即可
	
	if(cmd == 0) crc = 0x95;	// CMD0 强制校验
	if(cmd == 8) crc = 0x87;	// CMD8 强制校验
	
	
	SD_SPI_RWByte(&dummy, 0x40 | cmd);				// send index of cmd
	SD_SPI_RWByte(&dummy, (arg >> 24) & 0xFF);		// cmd-arg:bit31~bit24
	SD_SPI_RWByte(&dummy, (arg >> 16) & 0xFF);		// cmd-arg:bit23~bit16
	SD_SPI_RWByte(&dummy, (arg >> 8) & 0xFF);		// cmd-arg:bit15~bit8
	SD_SPI_RWByte(&dummy, arg & 0xFF);				// cmd-arg:bit7~bit0
	SD_SPI_RWByte(&dummy, crc);					// CRC(Usually Reset), bit0 = 1 ---> Stop bit
	
	for(int i=0 ; i<8; i++){
		SD_SPI_RWByte(&res, 0xFF);
		if((res & 0x80) == 0) break;
	}
	
	return res;
}




/**
  * @brief  从 SD 卡读取单个数据块 (Block, 标准 512 字节)
  * @note   使用 CMD17 指令。若卡类型为 SDHC 则参数为 Block 序号；若为 SDSC 则参数转为物理字节地址。
  * 成功响应后轮询等待 Data Token (0xFE)，接收完毕后丢弃后续的 2 Byte CRC。
  * @param  block: 目标读取的 Block 索引
  * @param  buf: 指向存储读取数据的 512 字节 Buffer 指针
  * @retval 0 成功；负数错误码代表指令无响应或未收到有效 Token
  */
int8_t SD_ReadBlock(uint32_t block, uint8_t *buf)
{
	uint8_t res, dummy;
	uint32_t addr = (SD_CardType) ? block : block * 512;
	SD_CS_LOW();
	
	for(int i=0;i<200;i++){
		res = SD_SendCmd(17, addr);
		if(res == 0x00) break;
	}
	if(res != 0x00) return -1;
	
	for(int i=0;i<500;i++){
		SD_SPI_RWByte(&res, 0xFF);
		if(res == 0xFE) break;
	}
	if(res != 0xFE) return -2;
	
	for(int i=0; i<512; i++){
		SD_SPI_RWByte(&buf[i], 0xFF);
	}
	SD_SPI_RWByte(&dummy, 0xFF);
	SD_SPI_RWByte(&dummy, 0xFF);
	
	SD_CS_HIGH();
	
	return 0;
}

/**
  * @brief  将单个数据块 (Block, 512 字节) 写入 SD 卡
  * @note   使用 CMD24 指令。寻址逻辑同 ReadBlock。
  * 		在数据前发送 Data Token (0xFE)，随后发送 512 字节及 2 Byte Dummy CRC。
  * 		通过解析 Data Response 判断写入被接受 (0x05)，随后轮询 MISO 直至卡内部擦写结束 (脱离 Busy 状态)。
  * @param  block: 目标写入的 Block 索引
  * @param  buf: 待写入的 512 字节数据 Buffer 指针
  * @retval  0 : 写入成功
  * 		-1 : 发送 CMD24 超时
  * 		-2 : SPI DMA 数据块传输失败
  * 		-3 : 写入数据被 SD 卡拒绝(Data Response 校验异常)
  * 		-4 : Busy 状态超时
  */
int8_t SD_WriteBlock(uint32_t block, const uint8_t *buf)
{
	int8_t ret = 0;
	uint8_t res, dummy;
	uint32_t addr = (SD_CardType) ? block : block * 512U;
	SD_CS_LOW();
	
	// 发送CMD24
	for(int i = 0; i < 200; i++){
		res = SD_SendCmd(24, addr);
		if(res == 0x00) break;
	}
	
	if(res != 0x00){
		ret = -1;
		goto exit;
	}
	
	SD_SPI_RWByte(&dummy, 0xFF);
	SD_SPI_RWByte(&dummy, 0xFE);

	if(SD_SPI_DMA_Transmit(buf, 512) != 0){
		ret = -2;
		goto exit;
	}

	// 发两字节CRC后，紧跟着获取data response
	SD_SPI_RWByte(&dummy, 0xFF);		// CRC1
	SD_SPI_RWByte(&dummy, 0xFF);		// CRC2

	// 等待SD卡内部校验
	for(int i = 0; i < 10; i++){
		SD_SPI_RWByte(&res, 0xFF);
		if((res & 0x11) == 0x01) break;
	}

	if((res & 0x1F) != 0x05){
		ret = -3;
		goto exit;
	}
	
	// 等待卡结束内部写操作（MISO 为高表示闲）
	uint32_t start_time = SysTick_ms;
	do {
		SD_SPI_RWByte(&res, 0xFF);

		if(SysTick_ms - start_time >= SD_BUSY_TIMEOUT_MS){
			SD_CS_HIGH();
			ret = -4;
			goto exit;
		}
	} while (res == 0x00);

exit:
	SD_CS_HIGH();
	return 0;
}
