#include "ICM_42688P.h"
#include "Delay.h"
#include <stdio.h>


IMU_Data_t imu1_data = {0};
IMU_Data_t imu2_data = {0};

/* DMA 缓冲区：1字节寄存器地址 + 12字节数据 (ACC*6 + GYRO*6) */
uint8_t spi1_dma_tx_buf[13] = {0};
uint8_t spi1_dma_rx_buf[13] = {0};


/**
  * @brief  初始化 2 * 6轴传感器SPI1，SPI3相关引脚
  * @param  None
  * @retval None
  */
void ICM_SPI_Init(void){
	// 使能SPI1 GPIO引脚时钟 AHB1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// SPI1引脚复用功能重映射
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;		// PA4: ICM_INT1，ICM输出中断采集信号给IMU
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	// PC4: SPI1_CSB
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
	// 使能SPI1时钟 APB2 84MHz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;		// 第二个边沿采样（Mode3）
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;			// 空闲时Clock为高（Mode3）
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	// MSB先发
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;			// CS由软件控制
	SPI_Init(SPI1, &SPI_InitStructure);
	
	SPI_Cmd(SPI1, ENABLE);
	
	GPIO_SetBits(GPIOC, GPIO_Pin_4);
}

/**
  * @brief  初始化 TIM7 作为 IMU 硬件采样的周期性 Pacing Trigger (2ms)
  */
void ICM_TIM7_Trigger_Init(void){
    TIM_TimeBaseInitTypeDef tim;
    NVIC_InitTypeDef nvic;
    
    /* 开启 TIM7 时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    
    /* 定时器配置: 84MHz / 84 = 1 MHz（1us），2000下即2ms */
    tim.TIM_Period = 2000 - 1;        
    tim.TIM_Prescaler = 84 - 1;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM7, &tim);
    
    /* 清除挂起标志，并使能更新中断 */
    TIM_ClearFlag(TIM7, TIM_FLAG_Update);
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    
    /* 配置 NVIC 中断优先级 */
    nvic.NVIC_IRQChannel = TIM7_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;     // 硬件触发保持最高优先级
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    TIM_Cmd(TIM7, DISABLE);
}

/**
  * @brief  TIM7 中断服务函数：纯粹的底层硬件触发源
  */
void TIM7_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
        
        /* 纯粹触发 DMA 连读，不掺杂任何控制层业务 */
        ICM_Start_DMA_Transfer(); 
    }
}

/**
  * @brief  初始化 SPI1 的 DMA (TX: DMA2_Stream3_CH3, RX: DMA2_Stream0_CH3)
  */
void ICM_SPI1_DMA_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    /* SPI1_TX: DMA2_Stream3, Channel 3 */
    DMA_DeInit(DMA2_Stream3);
    DMA_InitStructure.DMA_Channel = DMA_Channel_3;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(SPI1->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)spi1_dma_tx_buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = 13;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream3, &DMA_InitStructure);

    /* SPI1_RX: DMA2_Stream0, Channel 3 */
    DMA_DeInit(DMA2_Stream0);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)spi1_dma_rx_buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);

    /* 配置 DMA RX 完毕中断 */
    DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 与原 TIM7 优先级一致
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
}

/**
  * @brief  触发一次非阻塞的 DMA 连读 (读取 ACC 和 GYRO 共 12 字节)
  */
void ICM_Start_DMA_Transfer(void)
{
    /* 0x0C 是 ACCEL_DATA_X1，Bit7=1 表示读 */
    spi1_dma_tx_buf[0] = 0x0C | 0x80; 

    /* 拉低 CS 开启传输 */
    GPIO_ResetBits(GPIOC, GPIO_Pin_4);

    DMA_Cmd(DMA2_Stream3, DISABLE);
    DMA_Cmd(DMA2_Stream0, DISABLE);

    DMA_SetCurrDataCounter(DMA2_Stream3, 13);
    DMA_SetCurrDataCounter(DMA2_Stream0, 13);

    /* 先开 RX，再开 TX */
    DMA_Cmd(DMA2_Stream0, ENABLE);
    DMA_Cmd(DMA2_Stream3, ENABLE);
}

void DMA2_Stream0_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
        
        GPIO_SetBits(GPIOC, GPIO_Pin_4); // 1. 传输结束，拉高 CS

        /* 2. 解析 DMA 搬运来的原始数据并转换单位 */
        int16_t raw_ax = (int16_t)(spi1_dma_rx_buf[1] << 8 | spi1_dma_rx_buf[2]);
        int16_t raw_ay = (int16_t)(spi1_dma_rx_buf[3] << 8 | spi1_dma_rx_buf[4]);
        int16_t raw_az = (int16_t)(spi1_dma_rx_buf[5] << 8 | spi1_dma_rx_buf[6]);
        int16_t raw_gx = (int16_t)(spi1_dma_rx_buf[7] << 8 | spi1_dma_rx_buf[8]);
        int16_t raw_gy = (int16_t)(spi1_dma_rx_buf[9] << 8 | spi1_dma_rx_buf[10]);
        int16_t raw_gz = (int16_t)(spi1_dma_rx_buf[11] << 8 | spi1_dma_rx_buf[12]);

        imu1_data.ax = raw_ay * LSB_ACC;
        imu1_data.ay = raw_ax * LSB_ACC;
        imu1_data.az = raw_az * LSB_ACC;
        imu1_data.gx = raw_gy * LSB_GYO;
        imu1_data.gy = raw_gx * LSB_GYO;
        imu1_data.gz = raw_gz * LSB_GYO;

        /* 数据准备好了，触发 TIM6 软件中断去算 PID */
        /* 注意这里使用的是 TIM6_DAC_IRQn */
        NVIC_SetPendingIRQ(TIM6_DAC_IRQn); 
    }
}

/**
  * @brief  通过SPI总线向ICM读写一个字节（全双工）
  * @param  RXData 接收缓冲区指针
  * @param  TXData 待发送字节
  * @retval None
  */
void ICM_SPI_RWByte(SPI_TypeDef * SPIx, uint8_t *RXData, uint8_t TXData){
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) != SET) continue;
	SPI_I2S_SendData(SPIx, TXData);
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) != SET) continue;
	*RXData = SPI_I2S_ReceiveData(SPIx);
}
/**
  * @brief  读ICM寄存器
  * @param  reg 寄存器地址
  * @retval 寄存器值
  */
uint8_t ICM_ReadReg(SPI_TypeDef * SPIx, uint8_t reg){
	uint8_t dummy = 0, Data = 0;
	
	if(SPIx == SPI1) GPIO_ResetBits(GPIOC, GPIO_Pin_4);
	else			 GPIO_ResetBits(GPIOB, GPIO_Pin_3);
	
	ICM_SPI_RWByte(SPIx, &dummy, reg|0x80);	// bit7=1：读操作
	ICM_SPI_RWByte(SPIx, &Data, 0x00);		// 发0x00占位符，收数据
	
	if(SPIx == SPI1) GPIO_SetBits(GPIOC, GPIO_Pin_4);
	else			 GPIO_SetBits(GPIOB, GPIO_Pin_3);	
	
	return Data;
}
/**
  * @brief  向ICM写寄存器
  * @param  reg 寄存器地址
  * @param  Data 待写入数据
  * @retval None
  */
void ICM_WriteReg(SPI_TypeDef * SPIx, uint8_t reg, uint8_t Data){
	uint8_t dummy = 0;
	
	if(SPIx == SPI1) GPIO_ResetBits(GPIOC, GPIO_Pin_4);
	else			 GPIO_ResetBits(GPIOB, GPIO_Pin_3);	
	
	ICM_SPI_RWByte(SPIx, &dummy, reg&0x7F);	// bit7=0：写操作
	ICM_SPI_RWByte(SPIx, &dummy, Data);
	
	if(SPIx == SPI1) GPIO_SetBits(GPIOC, GPIO_Pin_4);
	else			 GPIO_SetBits(GPIOB, GPIO_Pin_3);	
}
/**
  * @brief  配置ICM基本寄存器
  * @param  None
  * @retval 初始化状态
  *		@arg 0: 初始化正常
  *		@arg 1: WHO_AM_I校验失败，SPI通信异常
  */
uint8_t ICM_Init(uint8_t who1, uint8_t who2){
	ICM_WriteReg(SPI1, 0x4A, 0xA5);		// 软件复位
	Delay_ms(20);
	
    who1 = ICM_ReadReg(SPI1, 0x01);
	Delay_us(100);
    
    if(who1 != 0x6A){
		return 1;
	}
	
	ICM_WriteReg(SPI1, 0x7D, 0x0E);		// PWR_CTRL: 使能ACC、Gyro、温度传感器
	Delay_ms(20);
	
	ICM_WriteReg(SPI1, 0x41, 0x03);		// ACC_RANGE: +/-16G
	Delay_ms(5);	
	
	ICM_WriteReg(SPI1, 0x40, 0x8C);		// ACC_CONF: 高性能模式，OSR4，1600Hz
	Delay_ms(5);
	
	ICM_WriteReg(SPI1, 0x43, 0x00);		// GYR_RANGE: +/-2000dps
	Delay_ms(5);	

	ICM_WriteReg(SPI1, 0x42, 0xCD);		// GYR_CONF: 高性能模式，OSR4，3200Hz
	Delay_ms(10);

	return 0;
}

/**
  * @brief  读加速度计三轴数据
  * @param  AX X轴加速度（单位：g）
  * @param  AY Y轴加速度（单位：g）
  * @param  AZ Z轴加速度（单位：g）
  * @retval None
  */
void ICM_ReadACCData(SPI_TypeDef * SPIx, IMU_Data_t *imu_data){
	int16_t rawX, rawY, rawZ;
	rawX = (int16_t)(ICM_ReadReg(SPIx, 0x0C) << 8 | ICM_ReadReg(SPIx, 0x0D));
	rawY = (int16_t)(ICM_ReadReg(SPIx, 0x0E) << 8 | ICM_ReadReg(SPIx, 0x0F));
	rawZ = (int16_t)(ICM_ReadReg(SPIx, 0x10) << 8 | ICM_ReadReg(SPIx, 0x11));
	
	imu_data->ax = rawX * LSB_ACC;
	imu_data->ay = rawY * LSB_ACC;
	imu_data->az = rawZ * LSB_ACC;
	
	/* AX AY重映射 */
	float tmp;
	
	tmp = imu_data->ax;
	imu_data->ax = imu_data->ay;
	imu_data->ay = tmp;
}
