#include "DSHOT.h"
#include "control.h"

/*==================================================================
 * DSHOT600 四路输出 (PA8/PA9/PA10/PA11 = TIM1 CH1~CH4)
 * 使用 TIM1_UP DMA burst 模式，一次 update事件写 CCR1~CCR4 四个寄存器
 *==================================================================*/

#define DSHOT_ARR			279		/* 168MHz / 280 = 600khz = DSHOT600*/
#define DSHOT_BIT_0			105		/* 37.5% duty = 625ns */
#define DSHOT_BIT_1 		210		/* 75% duty = 1250ns */
#define DSHOT_FRAME_LEN		18		/* 16 data bits + 2 zero padding */
#define DSHOT_CHANNELS		4

/* 缓冲区： 每次 TIM1_UP 事件 DMA 传 4 个 halfword (CCR1~CCR4)
 * 总大小 = 18 halfword * 4 channels = 72 halfword */ 
static uint16_t dshot_buf[DSHOT_FRAME_LEN * DSHOT_CHANNELS] = {0};

/**
  * @brief  编码单通道 DSHOT 数据帧并计算 CRC 校验码
  * @note   DSHOT 数据帧共 16-bit：11-bit 油门值 + 1-bit Telemetry (此处默认设为 0) + 4-bit CRC。
  * @param  throttle: 11-bit 的原始油门输入值 (0~2047)
  * @retval 经过 CRC 计算和移位拼接后的 16-bit 完整数据帧
  */
static uint16_t dshot_encode_frame(uint16_t throttle){
	uint16_t val = (throttle << 1) | 0;
	uint16_t crc = (val ^ (val >> 4) ^ (val >> 8)) & 0x0F;
	return (val << 4) | crc;
}

/**
  * @brief  将 4 路电机的 DSHOT 帧解析并填充至 DMA 传输 Buffer
  * @note   根据 TIM DMA Burst 的内存排布规则，每一行的 4 个 HalfWord 分别对应 TIM1 的 CCR1~CCR4。
  * 总共 16 行为有效数据位（将 16-bit 的 DSHOT 帧逐位拆解为 PWM 脉宽），最后补充 2 行为 Padding（全 0）以生成帧间隔。
  * @param  t1: 1 号电机 DSHOT 数值
  * @param  t2: 2 号电机 DSHOT 数值
  * @param  t3: 3 号电机 DSHOT 数值
  * @param  t4: 4 号电机 DSHOT 数值
  * @retval None
  */
static void DSHOT_FillBuffer(uint16_t t1, uint16_t t2, uint16_t t3, uint16_t t4){
	uint16_t frame[4];
	frame[0] = dshot_encode_frame(t1);
	frame[1] = dshot_encode_frame(t2);
	frame[2] = dshot_encode_frame(t3);
	frame[3] = dshot_encode_frame(t4);
	
	/* 布局： [CCR1_bit0, CCR2_bit0, CCR3_bit0, CCR4_bit0, CCR1_bit1...] */
	for(int bit = 0; bit < 16; bit++){
		int base = bit * DSHOT_CHANNELS;
		for(int ch = 0; ch < 4; ch++){
			dshot_buf[base + ch] = ((frame[ch] >> (15 - bit)) & 1 )
									? DSHOT_BIT_1 : DSHOT_BIT_0;
		}
	}
	
	/* 末尾 2 个 padding 周期，全通道输出 0 */
	for(int pad = 16; pad < 18; pad++){
		int base = pad * DSHOT_CHANNELS;
		for(int ch = 0; ch < 4; ch++)
			dshot_buf[base + ch] = 0;
	}
}

/**
  * @brief  初始化 4 通道 DSHOT 输出相关的 GPIO、Timer 与 DMA
  * @note   使用 TIM1_CH1~CH4 (PA8~PA11)，配置为 PWM1 模式。
  * 利用 TIM1 Update Event 触发 DMA2_Stream5 Channel 6 进行 DMA Burst 传输。
  * 每次传输自动向 DMAR 寄存器连续写入 4 个 HalfWord（映射至 CCR1~CCR4），实现硬件级并行输出。
  * @param  None
  * @retval None
  */
void DSHOT4_Init(){	
	/* 开启时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	/* TIM1引脚复用功能重映射 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);	
	
	/* 配置GPIOA */
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &gpio);
	
	/* 配置TIM1 Timebase */
	TIM_TimeBaseInitTypeDef tim;
	tim.TIM_Prescaler = 0;
	tim.TIM_Period = DSHOT_ARR;
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_RepetitionCounter = 0;
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM1, &tim);
	
	/* 配置 TIM1 Output Compare 
	 * CH1 ~ CH4 全部 PWM1 模式*/
	TIM_OCInitTypeDef oc;
	oc.TIM_OCMode = TIM_OCMode_PWM1;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_OutputNState = TIM_OutputNState_Disable;
	oc.TIM_Pulse = 0;
	oc.TIM_OCPolarity = TIM_OCPolarity_High;
	oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
	oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	
	TIM_OC1Init(TIM1, &oc);
	TIM_OC2Init(TIM1, &oc);
	TIM_OC3Init(TIM1, &oc);
	TIM_OC4Init(TIM1, &oc);
	
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	/* DMA burst: 每次 TIM1_UP 写 CCR1~CCR4	*/
	TIM_DMAConfig(TIM1, TIM_DMABase_CCR1, TIM_DMABurstLength_4Transfers);
	
	/* DMA2 Stream5 Channel6 = TIM1_UP*/
	DMA_DeInit(DMA2_Stream5);
	DMA_InitTypeDef dma;
	dma.DMA_BufferSize = DSHOT_FRAME_LEN * DSHOT_CHANNELS;	/* 72 */
	dma.DMA_Channel = DMA_Channel_6;
	dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	dma.DMA_Memory0BaseAddr = (uint32_t)dshot_buf;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_Mode = DMA_Mode_Normal;		/* 发完一次自动停止 */
	dma.DMA_PeripheralBaseAddr = (uint32_t)&TIM1->DMAR;		/* burst 模式写 DMA Register */
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	dma.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_HalfWord;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA2_Stream5, &dma);
	
	/* TIM1 update event 触发 DMA */
	TIM_DMACmd(TIM1, TIM_DMA_Update, ENABLE);
	
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

/**
  * @brief  将控制器的目标油门值映射至 DSHOT 协议有效范围
  * @note   DSHOT 协议要求 0 为电机停止，1~47 为特殊命令 (Telemtry/Beep 等)，48~2047 才是实际转速对应范围。
  * @param  thr: 控制器计算出的基础油门值
  * @param  arm_state: 解锁状态 (0=上锁, 非0=解锁)
  * @retval 映射后的 11-bit DSHOT 指令值，如果未解锁则直接返回 0。
  */
static uint16_t thr_to_dshot(uint16_t thr, uint8_t arm_state)
{
    if(!arm_state) return 0;
    return thr + 48; 
}

/**
  * @brief  执行一次 4 通道 DSHOT 数据帧发送
  * @note   进行油门映射转换及 Buffer 填充，安全重置并开启 DMA 进行数据流传输。
  * 采用阻塞等待上一帧 DMA 完成的方式，确保 Buffer 数据在传输途中不被篡改。
  * @param  t1: 1 号电机目标油门值
  * @param  t2: 2 号电机目标油门值
  * @param  t3: 3 号电机目标油门值
  * @param  t4: 4 号电机目标油门值
  * @retval None
  */
void DSHOT4_Send(uint16_t t1, uint16_t t2, uint16_t t3, uint16_t t4)
{
	t1 = thr_to_dshot(t1, arm_state);
	t2 = thr_to_dshot(t2, arm_state);
	t3 = thr_to_dshot(t3, arm_state);
	t4 = thr_to_dshot(t4, arm_state);
	
	DSHOT_FillBuffer(t1, t2, t3, t4);
	
	DMA_Cmd(DMA2_Stream5, DISABLE);						/* 防止上一帧还没发完的情况出现 */
	while(DMA_GetCmdStatus(DMA2_Stream5) != DISABLE);		/* 等 DMA 真正停下来 */
	
	/* 清除上一帧的状态标志；
	 * TCIFx: Transfer Complete Interrupt Flag
	 * HTIFx: Half Transfer
	 * TEIFx: Transfer Error
	 * DMEIFx: Direct Mode Error
	 * FEIFx: FIFO Error
	 * x: Steamx */
	DMA_ClearFlag(DMA2_Stream5,
			DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_TEIF5 |
			DMA_FLAG_DMEIF5 | DMA_FLAG_FEIF5);
	DMA_SetCurrDataCounter(DMA2_Stream5, DSHOT_FRAME_LEN * DSHOT_CHANNELS);
	DMA_Cmd(DMA2_Stream5, ENABLE);
}