#include "protection.h"

#include "ADC1in10.h"
#include "Delay.h"

#include "control.h"

#include <math.h>

#define Kp 15
#define THROTTLE_MAX 1152.0f
#define FS_ACTIVE 	1
#define FS_STANDBY  0

float throttle_limit = 0;
static uint32_t tilt_start_time = 0;
uint8_t failsafe_active = FS_STANDBY;
BuzzerMode_t buzzer_mode = BUZZ_NORMAL;

/**
  * @brief  初始化CRT（of ESC）引脚， 并设置油门上限
  */
void Protection_Init(){
	/* 开启时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_AN;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Pin = GPIO_Pin_1;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &gpio);				/* CRT 引脚 */
	
	/* ADC1 Initializtion in ADC1in10.c */
	
	throttle_limit = THROTTLE_MAX;
}

/**
  * @brief  根据测量的ESC电流值动态更新油门上限， 并记录电流值在黑盒实体指针中
  * @param	*f		黑盒实体指针
  */
void Protection_Update(BB_Frame_t *f){
	float measurement, current, error;
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);
	ADC_SoftwareStartConv(ADC1);
	
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) != SET)	continue;
	uint16_t raw = ADC_GetConversionValue(ADC1);
	
	measurement = raw * 0.0008058608058f;		/* 3.3/4095.0f */
	current = measurement * 85.1063829787f;		/* 11.75mV / A & 1.0/0.01175f */
	
	f->current = current;
	
	error = current - 44.0f;
	if(error > 0)
		throttle_limit -= Kp * error;
	else
		throttle_limit += 10.0f;
		
	if(throttle_limit > THROTTLE_MAX) throttle_limit = THROTTLE_MAX;
	if(throttle_limit < 100.0f) throttle_limit = 100.0f;
}

/**
  * @brief  更新并设置当前运行的安全状态，并设置蜂鸣器为对应模式
  */
void Protection_SetMode(void)
{
    if(vbat <= 14.0f)						// 电压过低状态
        buzzer_mode = BUZZ_LOW_VOLTAGE;
    else if(throttle_limit < 950.0f)		// 过流状态
        buzzer_mode = BUZZ_OVERCURRENT;
    else if(end_count > 0)					// 手动停机倒计时状态
        buzzer_mode = BUZZ_DISARMING;
    else									// 正常工作状态
        buzzer_mode = BUZZ_NORMAL;
}

/**
  * @brief  倾角检测
  * @param	roll_deg: 	当前的横滚角
  * @param	pitch_deg：	当前的俯仰角
  */
uint8_t Tilt_Check(float roll_deg, float pitch_deg){
    if(fabsf(roll_deg) > TILT_ANGLE_LIMIT || fabsf(pitch_deg) > TILT_ANGLE_LIMIT){
        if(tilt_start_time == 0)
            tilt_start_time = SysTick_ms;
        else if(SysTick_ms - tilt_start_time >= TILT_TIMEOUT_MS)
            return 1;
    } else {
        tilt_start_time = 0;
    }
    return 0;
}