/**
 * @brief   bsp_power.c
 * @brief   电池电压与电流 ADC 采样实现。
 */

#include "bsp_power.h"
#include "adc.h"

/*
 * VBAT Voltage Divider:
 *
 * R_high = 75 kΩ
 * R_low  = 10 kΩ
 *
 * Divider Ratio = (75 + 10) / 10 = 8.5 / 1
 * 
 * ADC Raw -> Pin Voltage -> Battery Voltage。
 */
#define VBAT_ADC_TO_VOLT (3.3f / 4095.0f * 8.5f)

/*
 * Current Sense 灵敏度为 11.75 mV/A。
 * 
 * ADC Raw -> Sense Voltage -> Current。
 */
#define CURRENT_ADC_TO_AMP (3.3f / 4095.0f / 0.01175f)

#define ADC_POLL_TIMEOUT_MS 20U     // 单次 ADC Conversion 等待超时时间，ms。

/** 最近一次完成的电源采样结果。 */
static PowerData_t s_power = {0};

void BSP_Power_Init(void)
{
    /*
     * ADC 硬件由 MX_ADC1_Init() 完成配置，
     * 此处只初始化软件侧缓存。
     */
    s_power.vbat = 0.0f;
    s_power.current = 0.0f;
}

void BSP_Power_Read(PowerData_t *out)
{
    if(out == NULL)
    {
        return;
    }

    /*
     * ADC 1 使用 Scan Conversion：
     * 
     * Rank1 -> VBAT
     * Rank2 -> ESC Current
     * 
     * 当前调用频率较低，因此采用阻塞式 Polling，
     * 不额外引入 DMA 数据链。
     */
    HAL_ADC_Start(&hadc1);

    /* Rank1：VBAT。 */
    HAL_ADC_PollForConversion(&hadc1, ADC_POLL_TIMEOUT_MS);
    s_power.vbat = HAL_ADC_GetValue(&hadc1) * VBAT_ADC_TO_VOLT;

    /* Rank2：ESC Current Sense。 */
    HAL_ADC_PollForConversion(&hadc1, ADC_POLL_TIMEOUT_MS);
    s_power.current = HAL_ADC_GetValue(&hadc1) * CURRENT_ADC_TO_AMP;

    HAL_ADC_Stop(&hadc1);

    *out = s_power;
}
