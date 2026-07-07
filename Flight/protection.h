#ifndef __PROTECTION_H
#define __PROTECTION_H

#include "stm32f4xx.h"
#include "blackbox.h"

#define TILT_ANGLE_LIMIT  60.0f
#define TILT_TIMEOUT_MS   500


typedef enum {
    BUZZ_NORMAL,      // 正常飞行，慢间隔响（比如2s周期，响200ms）
    BUZZ_LOW_VOLTAGE, // 14V警告，快闪（比如200ms周期）
    BUZZ_DISARMING,   // 停机确认中
    BUZZ_OVERCURRENT  // 过流保护
} BuzzerMode_t;

extern BuzzerMode_t buzzer_mode;
extern uint8_t failsafe_active;
extern float throttle_limit;

void Protection_Init(void);
void Protection_Update(BB_Frame_t *f);
void Protection_SetMode(void);
uint8_t Tilt_Check(float roll_deg, float pitch_deg);



#endif
