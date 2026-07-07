#include "control.h"

#include "elrs.h"
#include "ICM_42688P.h"
#include "DSHOT.h"

#include "attitude.h"
#include "flight_controller.h"
#include "protection.h"

#include <math.h>


#define Disarmed  0
#define Armed	  1

volatile uint8_t arm_state = Disarmed;	/* arm 初始为待解锁状态 */
uint8_t end_count = 0;					/* 停机确认计数变量 */

Motor_Output_t motor_thr_data = {0};	/* 记录电机输出值的结构体 */
volatile uint16_t throttle = 0;

/**
  * @brief	初始化飞控核心解算任务的软件中断通道 (TIM6_IRQn)
  */
void Control_SoftwareTask_Init(void){
    /* 仅作为 NVIC 调度载体，无需开启 TIM6 计数时钟 */
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;  // 低优先级：允许被 DMA(1) 和 ELRS(2) 抢占
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

/**
  * @brief	执行飞控核心解算任务
  */
void TIM6_DAC_IRQHandler(void)
{
    // 软件触发的中断，无需清除定时器标志位，NVIC 会自动清除 Pending 状态
    const float dt = 0.002f;

    /* 姿态解算 */
    ICM_GetRollPitch(&imu1_data, &att1);
    Attitude_Update(&imu1_data, &att1, dt);

    fc.roll_meas  = att1.roll * 57.29578f;
    fc.pitch_meas = att1.pitch * 57.29578f;
	// 受IIC读写速度和器件读写要求，yaw_meas 在 Main Loop 间隔 20ms 以上进行采样
	
    /* 核心控制链路更新 */
    FlightController_Update(dt, imu1_data.gx, imu1_data.gy, imu1_data.gz);

    /* Mixer 混控与动力输出 */
    Mixer(fc.roll_cmd, fc.pitch_cmd, fc.yaw_cmd, fc.throttle,
          &motor_thr_data.m1, &motor_thr_data.m2,
          &motor_thr_data.m3, &motor_thr_data.m4);

    DSHOT4_Send(motor_thr_data.m1, motor_thr_data.m2,
                motor_thr_data.m3, motor_thr_data.m4);
}


/**
  * @brief  电机输出限位器，保证电机输出值在一定范围内
  * @param  val			油门值
  * @param  min			设定的最小电机输出值
  * @param  max			设定的最大电机输出值
  * @retval 经过限位器处理后的电机输出值
  */
static uint16_t clamp(float val, float min, float max)
{
    if(val < min) return (uint16_t)min;
    if(val > max) return (uint16_t)max;
    return (uint16_t)val;
}

/**
  * @brief  电机输出混控器
  * @param  roll_cmd	roll输出值
  * @param  pitch_cmd	pitch输出值
  * @param  yaw_cmd		yaw输出值
  * @param  throttle	油门基准
  * @param  m1			电机1获得的油门值
  * @param  m2			电机2获得的油门值
  * @param  m3			电机3获得的油门值
  * @param  m4			电机4获得的油门值
  */
void Mixer(float roll_cmd,
           float pitch_cmd,
           float yaw_cmd,
           uint16_t throttle,
           uint16_t *m1,
           uint16_t *m2,
           uint16_t *m3,
           uint16_t *m4)
{
	/* 混控器里的输出上限要大于 Throttle MAX，
	 * 目的是让姿态控制有更多裕量来调整 */
    *m1 = clamp(throttle - pitch_cmd - roll_cmd - yaw_cmd, 0, 1352);	
    *m2 = clamp(throttle + pitch_cmd - roll_cmd + yaw_cmd, 0, 1352);
    *m3 = clamp(throttle - pitch_cmd + roll_cmd + yaw_cmd, 0, 1352);
    *m4 = clamp(throttle + pitch_cmd + roll_cmd - yaw_cmd, 0, 1352);
}


/**
  * @brief  检查遥控是否与飞控 ARM
  * @param  None
  * @retval Armed：		true
			Disarmed：	false
  */
bool IS_ARMED(void){
	/* 1792： Press Down，191： Press Up 
	 * 值大于 1000 视为 “按下开关” */
	return crsf_data.channels[4] >= 1000;			
}

/**
  * @brief  检查遥控油门摇杆是否在最低位
  * @param  None
  * @retval 油门摇杆在最低位：		true
			油门摇杆不在最低位：	false
  */
bool Thr_LDet(void){
	return crsf_data.channels[2] <= 180;
}


