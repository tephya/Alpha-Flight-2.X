#include "TFCARD.h"
#include "Delay.h"

uint8_t SD_CardType = 3;


/**
  * @brief  初始化 SD 卡底层 SPI 外设与 GPIO
  * @note   使用 SPI2 (PB13/SCK, PB14/MISO, PB15/MOSI)，CS 为普通 GPIO (PB12)。
  * 根据 SD 卡协议规范，初始化阶段的 SPI 速率必须低于 400kHz，此处预分频设为 128。
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
  * @brief  动态切换 SPI 通信速率
  * @note   在 SD_Init 初始化成功后调用，降低预分频值（如 SPI_BaudRatePrescaler_2）以切入全速模式，
  * 提升 FatFS 的数据吞吐量。
  * @param  prescaler: SPI 波特率预分频参数 (例如 SPI_BaudRatePrescaler_X)
  * @retval None
  */
void SD_SPI_SetSpeed(uint16_t prescaler){
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
void SD_SPI_RWByte(uint8_t *RXD, uint8_t TXD){
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
uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg){
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
  * @brief  执行 SD 卡 SPI 模式的上电初始化时序
  * @note   标准流程：发送至少 74 个 dummy clock -> CMD0 (复位) -> CMD8 (校验电压) 
  * -> ACMD41 (内部初始化就绪等待) -> CMD58 (读取 OCR 寄存器判断 SDHC/SDSC)。
  * 全局变量 SD_CardType 被更新以适配后续的 Block 寻址机制。
  * @param  None
  * @retval 0 成功；负数代表对应步骤初始化失败的错误码
  */
int8_t SD_Init(){
	uint8_t res, dummy, buf[4];
	
	SD_CS_HIGH();
	for(int i = 0; i < 10; i++){
		SD_SPI_RWByte(&res, 0xFF);
	}

	// CMD0 带重试
	for (int i = 0; i < 10; i++) {
		SD_CS_LOW();
		res = SD_SendCmd(0, 0x00000000);
		SD_CS_HIGH();
		SD_SPI_RWByte(&dummy, 0xFF);
		if (res == 0x01) break;
	}
	if (res != 0x01) return -1;
	
	// CMD8
	SD_CS_LOW();
	res = SD_SendCmd(8, 0x000001AA);
	for(int i=0; i<4; i++){
		SD_SPI_RWByte(&buf[i], 0xFF);
	}
	SD_CS_HIGH();
	if(res != 0x01) return -2;
	if(buf[3] != 0xAA) return -3;
	
	// ACMD41
	for(int i=0; i<500; i++){
		SD_CS_LOW();
		SD_SendCmd(55, 0x00000000);
		SD_CS_HIGH();
		
		SD_CS_LOW();
		res = SD_SendCmd(41, 0x40000000);
		SD_CS_HIGH();
		if(res == 0x00) break;
	}
	if(res != 0x00) return -4;
	
	// CMD58
	SD_CS_LOW();
	res = SD_SendCmd(58, 0x00000000);
	for(int i=0; i<4; i++){
		SD_SPI_RWByte(&buf[i], 0xFF);
	}
	SD_CS_HIGH();
	if(buf[0] & 0x40){		// SDHC block寻址
		SD_CardType = 1;
	}
	else{					// SDSC 字节寻址
		SD_CardType = 0;
	}
	
	return 0;
}

/**
  * @brief  从 SD 卡读取单个数据块 (Block, 标准 512 字节)
  * @note   使用 CMD17 指令。若卡类型为 SDHC 则参数为 Block 序号；若为 SDSC 则参数转为物理字节地址。
  * 成功响应后轮询等待 Data Token (0xFE)，接收完毕后丢弃后续的 2 Byte CRC。
  * @param  block: 目标读取的 Block 索引
  * @param  buf: 指向存储读取数据的 512 字节 Buffer 指针
  * @retval 0 成功；负数错误码代表指令无响应或未收到有效 Token
  */
int8_t SD_ReadBlock(uint32_t block, uint8_t *buf){
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
  * 在数据前发送 Data Token (0xFE)，随后发送 512 字节及 2 Byte Dummy CRC。
  * 通过解析 Data Response 判断写入被接受 (0x05)，随后轮询 MISO 直至卡内部擦写结束 (脱离 Busy 状态)。
  * @param  block: 目标写入的 Block 索引
  * @param  buf: 待写入的 512 字节数据 Buffer 指针
  * @retval 0 成功；负数错误码代表指令拒绝、写入数据被拒或擦写超时
  */
int8_t SD_WriteBlock(uint32_t block, const uint8_t *buf){
	uint8_t res, dummy;
	uint32_t addr = (SD_CardType) ? block : block * 512;
	SD_CS_LOW();
	
	for(int i = 0; i < 200; i++){
		res = SD_SendCmd(24, addr);
		if(res == 0x00) break;
	}if(res != 0x00) return -1;
	
	SD_SPI_RWByte(&dummy, 0xFF);
	
	SD_SPI_RWByte(&dummy, 0xFE);
	for(int i = 0; i < 512; i++)
		SD_SPI_RWByte(&dummy, buf[i]);
	
	// 发两字节CRC后，紧跟着获取data response
	SD_SPI_RWByte(&dummy, 0xFF);		// CRC1
	SD_SPI_RWByte(&dummy, 0xFF);		// CRC2

	for(int i = 0; i < 10; i++){
		SD_SPI_RWByte(&res, 0xFF);
		if((res & 0x11) == 0x01) break;
	}
	if((res & 0x1F) != 0x05) return -2;
	
	// 等待卡结束内部写操作（MISO 为高表示闲）
	uint32_t timeout = 100000;
	do {
		SD_SPI_RWByte(&res, 0xFF);
	} while (res == 0x00 && --timeout);
	if(res == 0x00) return -3;
	
	SD_CS_HIGH();
	
	return 0;
}
