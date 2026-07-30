#include "app_power_monitor.h"
#include "app_shared_types.h"
#include "bsp_power.h"
#include "bsp_blackbox.h"
#include "cmsis_os2.h"

#define VBAT_DISARM_THRESHOLD 13.2f     // 4S单片3.3V下限，换电池时改这一处
#define VBAT_RECOVER_THRESHOLD 13.5f    // 迟滞：回升到此值以上才重新置VOLTAGE_OK，防止临界点抖动
#define VBAT_FAULT_CONFIRM_COUNT 5      // 连续5次(≈500ms @100ms周期)低于阈值才确认，滤掉大机动瞬时压降造成的误判

#define VBAT_LOW_WARNING_THRESHOLD 14.0f // 早期预警阈值，跟触发disarm的13.2V是两回事，只提示不动作
#define VBAT_LOW_WARNING_RECOVER 14.2 // 迟滞：回升到此值以上才允许下次再报警，防止临界点反复提示

#define CRITICAL_BATTERY_REPOST_MS 1000U // 持续报警重发间隔，需大于EVT_CRITICAL_BATTERY节拍自身播放时长(约740ms)

#define TASK_PM_PERIOD_MS 100U

extern osEventFlagsId_t SystemReadyEventGroupHandle;
extern osMessageQueueId_t IndicatorEventQueueHandle;

static bool s_low_battery_warned = false;   // 早期预警跳变检测，独立于VOLTAGE_OK那套confirm-count逻辑
static bool s_critical_battery_was_active = false;  // CRITICAL_BATTERY持续报警状态追踪
static uint32_t s_critical_battery_last_repost_tick = 0;

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
                if (!g_power_health.voltage_fault)          // 跳边沿才记，持续锁存期间不用重复写
                    BB_LogVoltageFault(osKernelGetTickCount());

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

        if(g_power_health.voltage_fault)
        {
            if(!s_critical_battery_was_active ||
                (osKernelGetTickCount() - s_critical_battery_last_repost_tick) >= CRITICAL_BATTERY_REPOST_MS)
            {
                IndicatorEvent_t evt = EVT_CRITICAL_BATTERY;
                osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0, 0);
                s_critical_battery_last_repost_tick = osKernelGetTickCount();
            }
        }
        s_critical_battery_was_active = g_power_health.voltage_fault;

        // 早期预警：不动作，只提示
        if(pwr.vbat <= VBAT_LOW_WARNING_THRESHOLD)
        {
            if(!s_low_battery_warned)
            {
                IndicatorEvent_t evt = EVT_LOW_BATTERY;
                osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0, 0);
                s_low_battery_warned = true;
            }
        }
        else if(pwr.vbat >= VBAT_LOW_WARNING_RECOVER)
        {
            s_low_battery_warned = false;
        }

        osDelay(TASK_PM_PERIOD_MS);
    }
}
