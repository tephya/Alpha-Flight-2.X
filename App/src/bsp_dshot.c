/**
 * @file    bsp_dshot.c
 * @brief   基于 TIM1 PWM + DMA Burst 的四路 DShot600 输出实现。
 */

#include "bsp_dshot.h"
#include "tim.h"

/*
 * DShot600 输出链路：
 * 
 * TIM1 CH1~CH4 对应四路 Motor Output。
 * 每个 TIM1 Update Event 触发一次 DMA Burst，
 * 连续写入 CCR1~CCR4，使四路电机在同一个 DShot Bit 周期内同步更新。
 * 
 * 当前 TIM1 配置下，每个 DShot Bit 占 280 个 Timer Count。
 */
 #define DSHOT_BIT_0 105U       // Logic 0 High Time：37.5% Duty。
 #define DSHOT_BIT_1 210U       // Logic 1 High Time：75% Duty。
 #define DSHOT_FRAME_LEN 18U    // 16 个 Data Bit + 2 个低电平 Padding 周期。
 #define DSHOT_CHANNELS 4U      // 四路同步 DShot 输出。

/*
 * DMA Burst Buffer 按以下顺序排列：
 *
 * [bit15_CH1][bit15_CH2][bit15_CH3][bit15_CH4]
 * [bit14_CH1][bit14_CH2][bit14_CH3][bit14_CH4]
 * ...
 * [bit0_CH1 ][bit0_CH2 ][bit0_CH3 ][bit0_CH4 ]
 * [padding...]
 * 
 * 每个 TIM1 Update Event 通过 DMA Burst 连续更新 CCR1~CCR4。
 * 
 * DMA Memory Data Alignment 配置为 Halfword，因此 Buffer 元素必须保持
 * uint16_t。HAL_DMABurst_MultiWriteStart() 的 Buffer 参数虽然声明为
 * uint32_t *，这里的强制转换仅用于适配 HAL API，不改变实际 DMA 传输粒度。 
 */
#pragma arm section zidata = "DMA_SAFE_SRAM"

static uint16_t s_dshot_buf[DSHOT_FRAME_LEN * DSHOT_CHANNELS];

#pragma arm section zidata

/*
 * 将单路 11-bit Payload 编码为完整 16-bit DShot Frame：
 * 
 * [11-bit Payload][1-bit Telemetry][4-bit CRC]
 * 
 * 当前不请求 ESC Telemetry，因此 Telemetry Bit 固定为 0。
 */
static uint16_t DSHOT_EncodeFrame(uint16_t payload)
{
    uint16_t val = (uint16_t)((payload << 1) | 0U);

    uint16_t crc =
        (uint16_t)((val ^
                    (val >> 4) ^
                    (val >> 8)) &
                   0x0FU);

    return (uint16_t)((val << 4) | crc);
}

/*
 * 将四路 DShot Frame 展开为 DMA Burst Buffer。
 * 
 * DShot 按 MSB First 发送，每个 Bit 对应四个 CCR 值：
 * Buffer 尾部追加两个全 0 周期，使四路输出保持低电平形成帧瞬间。
 */
static void DSHOT_FillBuffer(uint16_t p1, uint16_t p2, uint16_t p3, uint16_t p4)
{
    uint16_t frame[DSHOT_CHANNELS];
    frame[0] = DSHOT_EncodeFrame(p1);
    frame[1] = DSHOT_EncodeFrame(p2);
    frame[2] = DSHOT_EncodeFrame(p3);
    frame[3] = DSHOT_EncodeFrame(p4);

    for (uint32_t bit = 0; bit < 16U; bit++)
    {
        uint32_t base = 
            bit * DSHOT_CHANNELS;

        for (uint32_t ch = 0; ch < DSHOT_CHANNELS; ch++)
        {
            /*
             * 从 bit15 到 bit0 依次展开。
             * 
             * Logic 1 / 0 通过不同 CCR 值改变 PWM High Time，
             * Bit 周期本身保持不变。
             */
            s_dshot_buf[base + ch] = 
                ((frame[ch] >> (15U - bit)) & 1U) 
                    ? DSHOT_BIT_1 
                    : DSHOT_BIT_0;
        }
    }

    /*
     * 最后两个 Bit 周期四路 CCR 全部置 0，
     * 保持输出低电平并形成 DShot Frame 间隔。
     */
    for (uint32_t pad = 16U; pad < DSHOT_FRAME_LEN; pad++)
    {
        uint32_t base = pad * DSHOT_CHANNELS;
        for (uint32_t ch = 0; ch < DSHOT_CHANNELS; ch++)
            s_dshot_buf[base + ch] = 0U;
    }
}

void BSP_DSHOT_Init(void)
{
    /*
     * 启动前清空整个 DMA Buffer，
     * 保证尚未发送有效 Frame 时 CCR 更新数据均为 0。
     */
    for (uint32_t i = 0; i < DSHOT_FRAME_LEN * DSHOT_CHANNELS; i++)
        s_dshot_buf[i] = 0;

    /*
     * CubeMX 只完成 TIM1 Base / PWM / Break-DeadTime 等参数配置，
     * PWM Channel 仍需要显式 Start 才会真正输出。
     * 
     * TIM1 属于 Advanced-control Timer，
     * HAL_TIM_PWM_Start() 同时会处理 Main Output Enable，
     * 使 CH1~CH4 的 PWM 波形能够实际输出到引脚。
     */
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void BSP_DSHOT_Send(uint16_t p1, uint16_t p2, uint16_t p3, uint16_t p4)
{
    DSHOT_FillBuffer(p1, p2, p3, p4);

    /*
     * 正常情况下，上一帧 DMA 应在下一次控制周期到来前早已完成。
     * 
     * 若异常情况下 DMA Burst 仍处于 Busy，则先停止旧传输，
     * 再发送最新的一帧，避免在实时控制路径中 Busy Wait。
     */
    if(htim1.DMABurstState != HAL_DMA_STATE_READY)
    {
        (void)HAL_TIM_DMABurst_WriteStop(&htim1, TIM_DMA_UPDATE);
    }

    /*
     * TIM1 Update Event 每触发一次 DMA Burst，
     * 按 CCR1 -> CCR4 连续写入四个 Halfword。
     * 
     * 18 组 Burst 对应：
     * 16 个 DShot Data Bit + 2 个 Padding 周期。
     */
    if(HAL_TIM_DMABurst_MultiWriteStart(&htim1, TIM_DMABASE_CCR1, TIM_DMA_UPDATE,
                                     (uint32_t *)s_dshot_buf,
                                     TIM_DMABURSTLENGTH_4TRANSFERS,
                                     DSHOT_FRAME_LEN * DSHOT_CHANNELS) != HAL_OK)
    {
        /*
         * TODO：
         * 记录 DShot DMA 启动失败次数，并纳入 Blackbox / Health 诊断，
         * 用于统计实际 Motor Command 丢帧情况。
         */
    }
}
