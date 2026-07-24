#ifndef __BSP_ICM_42688_H
#define __BSP_ICM_42688_H

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include <stdbool.h>

typedef enum
{
    ICM_INSTANCE_1 = 0, // SPI1, CS=PC4, INT1=PA4
    ICM_INSTANCE_2 = 1, // SPI3, CS=PB3, INT1=PA15
    ICM_INSTANCE_MAX
} IcmInstance_t;

typedef struct
{
    float ax, ay, az;   // g
    float gx, gy, gz;   // dps
    uint32_t timestamp_cycle;   // 本次读取时DWT->CYCCNT快照
} IcmData_t;

uint8_t ICM_InitAll(void);
void ICM_OnDataReady_ISR(IcmInstance_t inst);
void ICM_TriggerRead(IcmInstance_t inst);
void ICM_CopyTo(IcmInstance_t inst, IcmData_t *out);


/*====== 事件标志句柄，供app_imu2_redundancy.c的osEventFlagWait使用 ======*/
extern osEventFlagsId_t g_icmDataReadyEvtId;
#define ICM1_DRDY_FLAG (1U << 0)
#define ICM2_DRDY_FLAG (1U << 1)

/*====== ODR=800Hz对应周期，供超时计算使用 ======*/
#define ICM_ODR_HZ 800
#define ICM_PERIOD_MS 2             // 1.25ms向上取整为2ms tick，若tick更细可改用us级定时器
#define ICM_DRDY_TIMEOUT_MS 4       // 3倍周期，按ms tick向上取整

#endif
