#include "app_power_monitor.h"
#include "app_shared_types.h"
#include "bsp_power.h"
#include "cmsis_os2.h"

#define VBAT_DISARM_THRESHOLD 13.2f     // 4S单片3.3V下限，换电池时改这一处
#define VBAT_RECOVER_THRESHOLD 13.5f    // 迟滞：回升到此值以上才重新置VOLTAGE_OK，防止临界点抖动
#define VBAT_FAULT_CONFIRM_COUNT 5      // 连续5次(≈500ms @100ms周期)低于阈值才确认，滤掉大机动瞬时压降造成的误判

#define TASK_PM_PERIOD_MS 100U

extern osEventFlagsId_t SystemReadyEventGroupHandle;

void App_PowerMonit_Task(void *argument)
{
    (void)argument;
    BSP_Power_Init();

    uint32_t low_count = 0;
    PowerData_t pwr;

    for (;;)
    {
        BSP_Power_Read(&pwr);
        g_heartbeat.powermonit_last_tick = osKernelGetTickCount();

        if(pwr.vbat <= VBAT_DISARM_THRESHOLD)
        {
            if(low_count < VBAT_FAULT_CONFIRM_COUNT)
                low_count++;
            
            if(low_count > VBAT_FAULT_CONFIRM_COUNT)
            {
                g_power_health.voltage_fault = true;
                osEventFlagsClear(SystemReadyEventGroupHandle, SYSREADY_BIT_VOLTAGE_OK);
            }
        }
        else
        {
            low_count = 0;

            if(pwr.vbat >= VBAT_RECOVER_THRESHOLD)
            {
                g_power_health.voltage_fault = false;
                osEventFlagsSet(SystemReadyEventGroupHandle, SYSREADY_BIT_VOLTAGE_OK);
            }
        }

        osDelay(TASK_PM_PERIOD_MS);
    }
}
