#ifndef __BLACKBOX_H
#define __BLACKBOX_H

#include "stm32f4xx.h"

typedef struct{
    uint32_t time_ms;
    float gx, gy, gz;
    float ax, ay, az;
    float mx, my, mz;
    float roll, pitch, yaw;
    uint16_t m1, m2, m3, m4;
    // PID
    float pro, ppo, pyo;
    float target_roll, target_pitch, target_yaw;
	
	float roll_rate_target;
	float pitch_rate_target;
    // 遥控器
    uint16_t throttle;
    uint16_t ch0, ch1, ch2, ch3, ch4;
    // 系统状态
    float vbat, current;
    float throttle_limit;
    uint8_t failsafe_active;
    uint8_t rth_state;
    // GPS
    double latitude, longitude;
    uint8_t satellites;
    // QMC self test（首帧）
    int16_t rmx1, rmy1, rmz1, rmx2, rmy2, rmz2;
} BB_Frame_t;

int8_t BB_Init(void);               // 初始化：挂载SDka，创建日志文件
int8_t BB_Log(BB_Frame_t *f);        // 写入一帧
int8_t BB_Close(void);              // 关闭文件

#endif