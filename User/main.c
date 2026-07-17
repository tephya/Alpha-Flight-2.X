#include "stm32f4xx.h"
#include "stm32f4xx_iwdg.h"

#include "GPS.h"
#include "Delay.h"
#include "Buzzer.h"
#include "ICM_42688P.h"
#include "USART3.h"
#include "ADC1in10.h"
#include "TFCARD.h"
#include "DSHOT.h"
#include "elrs.h"
#include "SW_I2C.h"

#include "blackbox.h"
#include "attitude.h"
#include "control.h"
#include "protection.h"
#include "flight_controller.h"
#include "navigation.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#ifndef Armed
#define Armed 1
#endif

#ifndef Disarmed
#define Disarmed  0
#endif

int fputc(int ch, FILE *f);
void IWDG_Init(void);
void IWDG_Feed(void);


/* Main控制流程：
 * 1. 初始化
 * 2. ARM ESC
 * 3. 等GPS搜星， 并记录首航坐标
 * 4. ARM 遥控
 * 5. Main Loop -- 时间片轮询核心控制之外的进程
 * 		退出条件： 5.1 电压过低
 * 				 5.2 倾角保护
 * 				 5.3 手动退出
 * 6. 延时等待 SD 写盘， 确保日志文件不受损
 * 7. 退出 */
int main(void){
/*====================== Local variable declarations Start ===========================*/
    uint8_t imu1_who = 0, imu2_who = 0;    
    uint32_t last_failsafe = 0;
    uint32_t last_slow = 0;
    uint32_t last_EFC = 0;        /* Last End Flight Confirm */
    uint32_t last_qmc = 0;
	uint32_t last_gps = 0;
	uint32_t last_time_32 = 0;	// 保存上一次记录的时间
	
    uint32_t last_dshot = 0;
    
    BB_Frame_t record = {0};
/*===================== Local variable declarations End ==============================*/

/*========================= Initialization Start ==============================*/    
    SystemInit();
    SysTick_Init();
    
    /* Peripheral Init */
    DSHOT4_Init();
    USART3_Init();
    Init_ADC1();
	GPS_Init();
    Protection_Init();
    SD_SPI_Init();
    BB_Init();
    IIC_Init();
    BUZZER_Init();
    ELRS_INIT();
    
    /* IMU & New Architecture Init */
    ICM_SPI_Init();
    ICM_Init(imu1_who, imu2_who);
    ICM_SPI1_DMA_Init();         /* 初始化 SPI DMA 通道 */
    Control_SoftwareTask_Init(); /* 初始化控制层软件中断 */
    ICM_TIM7_Trigger_Init();     /* 初始化 IMU 硬件触发定时器 */
    QMC_Init();
	   
    /* GPIO Output Config (LED: PB9) */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_9);
    
    /* PID & Controller Model Init */
    FlightController_Init();
/*========================== Initialization End ==============================*/    

/*========================= Arm Delay Start ==============================*/        
    /* 等 4 秒让 ESC 完成启动 */
    BUZZER_Start();
    Delay_ms(4000);
    BUZZER_Stop();
    Delay_ms(1000);
    
    /* ESC 安全握手：throttle=0 持续 3 秒 */
    BUZZER_Start();
    for (int i = 0; i < 3000; i++) {
        if(i == 999 || i == 2999) BUZZER_Stop();
        if(i == 1999) BUZZER_Start();
        DSHOT4_Send(0, 0, 0, 0);
        Delay_ms(1);
    }
/*========================== Arm Delay End ==============================*/    

/*========================== GPS Set Home Start ==============================*/
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	while(!gps_home.valid){
		GPS_SetHome();
	}
	
	BUZZER_Start();
    Delay_ms(4000);
	BUZZER_Stop();
	USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);
/*========================== GPS Set Home End ==============================*/	

/*========================= Arm Lock Loop Start ==============================*/        
    while(!arm_state){
        ELRS_Poll();    
        if(CRSF_PARSE_FRAME() == 1){
            __disable_irq();
            memcpy(&crsf_data, &temp_data, sizeof(CRSF_Data_t));
            __enable_irq();
        }
        
        if(SysTick_ms - last_dshot >= 20){
            last_dshot = SysTick_ms;
            DSHOT4_Send(0, 0, 0, 0);
        }
        
        /* 此时 TIM7 尚未开启，采用原阻塞读取进行安全倾角检测 */
        ICM_ReadACCData(SPI1, &imu1_data);
        float r_acc = atan2f(imu1_data.ay, imu1_data.az) * 57.2958f;
        float p_acc = atan2f(-imu1_data.ax, sqrtf(imu1_data.ay*imu1_data.ay + imu1_data.az*imu1_data.az)) * 57.2958f;
        
        uint8_t tilt_ok = (fabsf(r_acc) < 30.0f && fabsf(p_acc) < 30.0f);
        
        if(IS_ARMED() && Thr_LDet() && tilt_ok){
            arm_state = Armed;
            BUZZER_Stop();
            
            /* 重置时间戳并立刻激活高频硬件时钟触发链 */
            TIM_SetCounter(TIM7, 0);
            TIM_Cmd(TIM7, ENABLE);
			USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
        }
        else if(IS_ARMED() && Thr_LDet() && !tilt_ok){
            if((SysTick_ms / 150) % 2){ BUZZER_Start(); GPIO_SetBits(GPIOB, GPIO_Pin_9); } 
            else { BUZZER_Stop(); GPIO_ResetBits(GPIOB, GPIO_Pin_9); }
        }
        else if(IS_ARMED() && !Thr_LDet()){
            if((SysTick_ms / 250) % 2){ BUZZER_Start(); GPIO_SetBits(GPIOB, GPIO_Pin_9); } 
            else { BUZZER_Stop(); GPIO_ResetBits(GPIOB, GPIO_Pin_9); }
        }
        else{
            if((SysTick_ms / 600) % 2){ BUZZER_Start(); GPIO_SetBits(GPIOB, GPIO_Pin_9); } 
            else { BUZZER_Stop(); GPIO_ResetBits(GPIOB, GPIO_Pin_9); }
        }
    }
    
    /* 锁定初始航向角 */
    for(int i = 0; i < 20; i++)
    {
        if(QMC_ReadRaw_3Axis(&qmc_data) == 0)
        {
            QMC_Raw2Gauss(&qmc_data);
            Attitude_CptYaw(&qmc_data, &att1);
            fc.yaw_meas = att1.yaw * 57.2958f;
            break;
        }
        Delay_ms(25);
    }

    FlightController_Reset();
	
	/* SD卡读写速度设置为2.625MHz，留有充分写入裕量（速度越快写入越不稳定，过低速度又太快了） */
    SD_SPI_SetSpeed(SPI_BaudRatePrescaler_16); 		
    
	/* 标定时间片轮询时间基准 */
    last_qmc = SysTick_ms;
    last_slow = SysTick_ms;
    last_failsafe = SysTick_ms;
    last_EFC = SysTick_ms;
	last_gps = SysTick_ms;

	IWDG_Init();
/*================================ Main Loop Start (时间片轮询) ================================*/    
    while(1){
		IWDG_Feed();		// 1s 要喂一次狗
		
        /* 接收遥控器数据 */
        ELRS_Poll();
        if(CRSF_PARSE_FRAME() == 1){
            __disable_irq();
            memcpy(&crsf_data, &temp_data, sizeof(CRSF_Data_t));
            __enable_irq();
        }
    
        /* 慢速 I2C 任务：每 20ms 读取磁力计并单向注入航向角更新 */
        if(SysTick_ms - last_qmc >= 20)
        {
            last_qmc = SysTick_ms;
            if(QMC_ReadRaw_3Axis(&qmc_data) == 0)
            {
                QMC_Raw2Gauss(&qmc_data);
                Attitude_CptYaw(&qmc_data, &att1);
                FlightController_UpdateYaw(att1.yaw * 57.2958f);
            }
        }
		
		/* 慢速任务：每 100ms 更新一次 GPS 和返航逻辑 */
        if(SysTick_ms - last_gps >= 100)
        {
            last_gps = SysTick_ms;
            
            // GPS 基础解析 (只要不开 RTH，仅读取 GPS 数据是安全的，可以用于黑匣子记录)
            GPS_Poll();

#if ENABLE_AUTO_RTH
            /* * [TODO] 危险代码段：RTH 控制逻辑
             * 当前状态：逻辑已编写，但未进行实机调参和边界测试。
             * 依赖项：需要确保磁力计 (QMC) 无干扰，且悬停油门 (hover_throttle) 已精准标定。
             */
            float bearing = GPS_BearingTo(gps_home.latitude, gps_home.longitude);
            float dist    = GPS_DistanceTo(gps_home.latitude, gps_home.longitude);
            
            RTH_Update(bearing, dist, att1.yaw * 57.2958f, 
                       &fc.roll_target, &fc.pitch_target, &fc.yaw_target, &fc.throttle);
#endif
        }
        
        /* 安全防御任务：每 100ms 检视机身状态，电压检测与过流 */
        if(SysTick_ms - last_failsafe >= 100){
            last_failsafe = SysTick_ms;
            
            /* 异常倾角紧急停机坠机保护 */
            if(Tilt_Check(fc.roll_meas, fc.pitch_meas)){
                FlightController_Reset();
                TIM_Cmd(TIM7, DISABLE);
                arm_state = Disarmed;
                break;
            }
			
			ReadVBAT();
            if(vbat <= 13.2f) /* 强制坠机以保护锂电池 */
            {
                FlightController_Reset();
                TIM_Cmd(TIM7, DISABLE);
                DSHOT4_Send(0, 0, 0, 0);
                arm_state = Disarmed;
                break;
            }
            
            Protection_Update();
            Protection_SetMode();
            Buzzer_Drive();
        }

        /* 黑匣子阻塞写盘 
		 * 后续改进方向： 采用环形缓冲区 / 双缓冲 + DMA */
        if(SysTick_ms - last_slow >= 500){
            last_slow = SysTick_ms;
			uint32_t delta_time = SysTick_ms - last_time_32;
			last_time_32 = SysTick_ms;
			
			record.time_ms = (uint16_t)delta_time;
            
            /* 临界区保护：防止写盘拼装时被高频控制快照打断导致数据撕裂 */
            __disable_irq();
            record.angle_cdeg[0] = (int16_t)(att1.roll * 10000.0f);
			record.angle_cdeg[1] = (int16_t)(att1.pitch * 10000.0f);
			record.angle_cdeg[2] = (int16_t)(att1.yaw * 5000.0f);
			
			record.target_cdeg[0] = (int8_t)fc.roll_target;
			record.target_cdeg[1] = (int8_t)fc.pitch_target;
			record.target_cdeg[2] = (int8_t)fc.yaw_target;
			
			record.motor[0] = motor_thr_data.m1;
			record.motor[1] = motor_thr_data.m2;
			record.motor[2] = motor_thr_data.m3;
			record.motor[3] = motor_thr_data.m4;		
            __enable_irq(); 
                    
            /* 允许在此执行慢速阻塞写盘，高频控制链路已通过软件中断实现越顶 */
            BB_Log(&record);
        }
        
        /* 落地停机判定：拨下 Arm 且零油门保持 6 秒则切断动力 */
        if(SysTick_ms - last_EFC >= 2000){
            last_EFC = SysTick_ms;
            if(Thr_LDet() && !IS_ARMED()){
                if(++end_count >= 3){
                    FlightController_Reset();
                    BUZZER_Stop();
                    TIM_Cmd(TIM7, DISABLE);
                    arm_state = Disarmed;
                    break;
                }
            }
            else{
                end_count = 0;
            }
        }
    }
/*================================ Main Loop End================================*/    
    
    /* 停机落盘安全收尾流程 */
    BB_Close();
    DSHOT4_Send(0, 0, 0, 0);
    Delay_ms(3000);
    
    while(1){
        BUZZER_Start();
        GPIO_SetBits(GPIOB, GPIO_Pin_9);
        Delay_ms(1000);
        BUZZER_Stop();
        GPIO_ResetBits(GPIOB, GPIO_Pin_9);
        Delay_ms(1000);
    }

    return 0;
}

int fputc(int ch, FILE *f){
    while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    USART_SendData(USART3, ch);
    return ch;
}

void IWDG_Init(void){
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_32);		/* LSI = 32MHz */
	IWDG_SetReload(1000);						/* 1000ms 喂一次 */
	IWDG_ReloadCounter();
	IWDG_Enable();
}

void IWDG_Feed(void){
	IWDG_ReloadCounter();
}