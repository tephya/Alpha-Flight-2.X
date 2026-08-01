#include "bsp_dshot.h"
#include "tim.h"

/*=============================================================
 * DSHOT600 四路输出(PA8/PA9/PA10/PA11 = TIM1 CH1~CH4)
 * TIM1_UP事件触发DMA2_Stream5 burst，一次写入CCR1~CCR4四个寄存器
 *=============================================================*/

 #define DSHOT_BIT_0 105U       // 37.5% duty = 625ns @168MHz/280
 #define DSHOT_BIT_1 210U       // 75% duty = 1250ns
 #define DSHOT_FRAME_LEN 18U    // 16 data bits + 2 padding
 #define DSHOT_CHANNELS 4U

/**
 * DMA burst缓冲区，注意：元素类型必须是uint16_t——实际传输粒度由
 * CubeMX里hdma_tim1_up.Init.MemDataAlignment=DMA_MDATAALIGN_HALFWORD决定，
 * 跟HAL_TIM_DMABurst_MultiWriteStart()函数原型接收uint32_t*无关，
 * 调用时强转指针类型即可，缓存区本身不能声明为uint32_t，否则实际发出去的半子数据会错位。
 */
#pragma arm section zidata = "DMA_SAFE_SRAM"
static uint16_t s_dshot_buf[DSHOT_FRAME_LEN * DSHOT_CHANNELS];
#pragma arm section zidata

/**
 * @brief   编码单通道DSHOT数据帧并计算 CRC 校验码
 * @note    DSHOT数据帧共16bit：11 data bits + 1bit Tele + 4bit CRC
 * @param   payload 原始油门输入值
 * @retval  经过CRC计算和移位拼接后的完整油门数据帧
 */
static uint16_t DSHOT_EncodeFrame(uint16_t payload)
{
    uint16_t val = (uint16_t)((payload << 1) | 0U);     // bit0: Telemetry = 0
    uint16_t crc = (uint16_t)((val ^ (val >> 4) ^ (val >> 8)) & 0x0FU);
    return (uint16_t)((val << 4) | crc);
}

static void DSHOT_FillBuffer(uint16_t p1, uint16_t p2, uint16_t p3, uint16_t p4)
{
    uint16_t frame[DSHOT_CHANNELS];
    frame[0] = DSHOT_EncodeFrame(p1);
    frame[1] = DSHOT_EncodeFrame(p2);
    frame[2] = DSHOT_EncodeFrame(p3);
    frame[3] = DSHOT_EncodeFrame(p4);

    for (uint32_t bit = 0; bit < 16U; bit++)
    {
        uint32_t base = bit * DSHOT_CHANNELS;
        for (uint32_t ch = 0; ch < DSHOT_CHANNELS; ch++)
        {
            s_dshot_buf[base + ch] = ((frame[ch] >> (15U - bit)) & 1U) 
                                    ? DSHOT_BIT_1 : DSHOT_BIT_0;
        }
    }

    // 末尾2个padding周期，四路全0，形成帧间隔
    for (uint32_t pad = 16U; pad < DSHOT_FRAME_LEN; pad++)
    {
        uint32_t base = pad * DSHOT_CHANNELS;
        for (uint32_t ch = 0; ch < DSHOT_CHANNELS; ch++)
            s_dshot_buf[base + ch] = 0U;
    }
}



/**
 * @brief   初始化DSHOT600四路输出
 * @note    TIM1/DMA2_Stream5的始终、GPIO、PWM通道、hdma_tim1_up均已由CubeMX生成，
 *          这里只负责启动PWM通道输出和清空burst buffer，
 *          必须在第一次调用BSP_DSHOT_Send前调用一次
 */
void BSP_DSHOT_Init(void)
{
    for (uint32_t i = 0; i < DSHOT_FRAME_LEN * DSHOT_CHANNELS; i++)
        s_dshot_buf[i] = 0;

    /**
     * MX_TIM1_Init只做了Base/PWM通道配置和Break/DeadTime配置，没有真正启动输出
     * HAL_TIM_PWM_Start对Break型定时器(TIM1)会顺带使能MOE位
     * 否则BDTR.AutomaticOutput=DISABLE的情况下CCR再怎么变化外部引脚也不会翻转
     */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

/**
 * @brief   编码并发送一帧DSHOT600数据到四路电机
 * @note    纯执行层：不做ARM判断、不做偏移映射，四个入参直接当做DSHOT 11-bit payload编码。
 *          payload=0表示停转指令；48~2047为有效油门区间；
 *          1~47是协议保留区间，调用方不应传入。
 *          是否解锁、要不要加48偏移，是由调用方(ARM状态机)决定后传入成品值
 */
void BSP_DSHOT_Send(uint16_t p1, uint16_t p2, uint16_t p3, uint16_t p4)
{
    DSHOT_FillBuffer(p1, p2, p3, p4);

    /**
     * 上一帧DMA若还没发完就先停掉再重启。DSHOT600单帧总时长约30us，
     * 远小于800Hz(1.25ms)控制周期，正常情况这里不会真的碰到BUSY，
     * 只是给异常场景留个非阻塞的兜底，不采用旧代码那种忙等到DMA停稳的写法
     */
    if(HAL_DMA_GetState(htim1.hdma[TIM_DMA_ID_UPDATE]) == HAL_DMA_STATE_BUSY)
    {
        HAL_TIM_DMABurst_WriteStop(&htim1, TIM_DMA_UPDATE);
    }

    HAL_TIM_DMABurst_MultiWriteStart(&htim1, TIM_DMABASE_CCR1, TIM_DMA_UPDATE,
                                     (uint32_t *)s_dshot_buf,
                                     TIM_DMABURSTLENGTH_4TRANSFERS,
                                     DSHOT_FRAME_LEN * DSHOT_CHANNELS);
}
