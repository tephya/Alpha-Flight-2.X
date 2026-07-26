#include "bsp_power.h"
#include "adc.h"

// 分压比：(75k+10k)/10k = 8.5
#define VBAT_ADC_TO_VOLT (3.3f / 4095.0f * 8.5f)

// 电流采样：11.75mV/A，即除0.01175；预留，本次不参与任何判断
#define CURRENT_ADC_TO_AMP (3.3f / 4095.0f / 0.01175f)

static PowerData_t s_power = {0};

void BSP_Power_Init(void)
{
    s_power.vbat = 0.0f;
    s_power.current = 0.0f;
}

/**
 * @brief   触发一次ADC1 Scan转换(VBAT+电流两通道)并拷贝结果
 * @note    阻塞式，两通道总转换事件(144+15 cycles @ ADC clock(APB2 84MHz))量级在微秒，
 *          不会对100ms周期的Task造成影响，不用DMA
 * @param   out 调用方提供的接收结构体指针
 */
void BSP_Power_Read(PowerData_t *out)
{
    HAL_ADC_Start(&hadc1);

    // Rank1: VBAT(ADC_CHANNEL_10)
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    s_power.vbat = HAL_ADC_GetValue(&hadc1) * VBAT_ADC_TO_VOLT;

    // Rank2: ESC(ADC_CHANNEL_11)
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    s_power.current = HAL_ADC_GetValue(&hadc1) * CURRENT_ADC_TO_AMP;

    HAL_ADC_Stop(&hadc1);

    *out = s_power;
}
