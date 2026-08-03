#include "app_power_monitor.h"
#include "app_shared_types.h"
#include "bsp_power.h"
#include "bsp_blackbox.h"
#include "cmsis_os2.h"

/*================================ 低电压检测宏 ====================================*/
#define VBAT_DISARM_THRESHOLD 13.2f     // 4S单片3.3V下限，换电池时改这一处
#define VBAT_RECOVER_THRESHOLD 13.5f    // 迟滞：回升到此值以上才重新置VOLTAGE_OK，防止临界点抖动
#define VBAT_FAULT_CONFIRM_COUNT 25      // 连续25次(≈500ms @20ms周期)低于阈值才确认，滤掉大机动瞬时压降造成的误判
#define CRITICAL_BATTERY_REPOST_MS 1500U // 持续报警重发间隔，需大于EVT_CRITICAL_BATTERY节拍自身播放时长(约740ms)

#define VBAT_LOW_WARNING_THRESHOLD 14.0f // 早期预警阈值，跟触发disarm的13.2V是两回事，只提示不动作
#define VBAT_LOW_WARNING_RECOVER 14.2f // 迟滞：回升到此值以上才允许下次再报警，防止临界点反复提示

/*================================ 过流检测宏 ====================================*/
#define CURRENT_LIMIT_A 44.0f           // 2200mAh、25C电池理论持续电流约为55A。44A约为其80%
                                        
#define CURRENT_LIMIT_MIN_SCALE 0.60f   // 最多把collective throttle压到60%，避免导致飞行器直接失去维持高度和姿态的能力
#define CURRENT_FILTER_TAU_S 0.10f      // 电流系统滤波时间常数。用于滤除电机换相、姿态修正产生的短时电流尖峰
#define CURRENT_ATTACK_TAU_S 0.15f      // Attack较快：检测到过流后约在数百毫秒内明显降低油门
#define CURRENT_RELEASE_TAU_S 1.00f     // Release较慢：电流恢复后缓慢释放限制，避免反复振荡

#define CURRENT_LIMIT_REPOST_MS 2000U


#define TASK_PM_PERIOD_MS 20U
#define TASK_PM_DT_S 0.02f

extern osEventFlagsId_t SystemReadyEventGroupHandle;
extern osMessageQueueId_t IndicatorEventQueueHandle;

static bool s_low_battery_warned = false;   // 早期预警跳变检测
static bool s_critical_battery_was_active = false;  // CRITICAL_BATTERY持续报警状态追踪
static uint32_t s_critical_battery_last_repost_tick = 0;

static float s_current_filtered = 0.0f;
static float s_current_scale = 1.0f;
static bool s_current_initialized = false;
static bool s_current_limit_reported = false;   // 本轮持续限流是否已经成功投递过提示
                                                    // 队列满导致投递失败时保持false，下一周期继续尝试
static uint32_t s_current_limit_last_report_tick = 0U;

/**
 * @brief   根据整机总电流计算油门缩放比例
 * @note    1.这是闭环软限流，不假设DShot输出与电流呈线性关系
 *          2.超过44A时降低collective throttle
 *          3.不直接修改Roll/Pitch/Yaw修正量，优先保留姿态控制能力
 *          4.最低只限制到60%，避免保护动作本身直接导致坠机
 */
static void CurrentLimiter_Update(float current)
{
    // 防止ADC偏移或换算误差产生负电流
    if(current < 0.0f)
        current = 0.0f;

    // 第一次采样直接作为滤波初始值，避免0A缓慢爬升造成启动阶段读数失真
    if(!s_current_initialized)
    {
        s_current_filtered = current;
        s_current_initialized = true;
    }
    else
    {
        /**
         * 一阶低通滤波：
         * alpha = dt / (tau + dt)
         */
        const float alpha =
            TASK_PM_DT_S / (CURRENT_FILTER_TAU_S + TASK_PM_DT_S);

        s_current_filtered +=
            alpha * (current - s_current_filtered);
    }

    float target_scale = 1.0f;      // 默认不限制油门

    /**
     * 超过阈值时，根据“允许电流/实际电流”计算目标比例
     * 如测得55A：44/55 = 0.8，即目标油门比例为80%
     */
    if(s_current_filtered > CURRENT_LIMIT_A)
        target_scale = CURRENT_LIMIT_A / s_current_filtered;
    
    if(target_scale < CURRENT_LIMIT_MIN_SCALE)          // 保留最低60%的油门控制范围
        target_scale = CURRENT_LIMIT_MIN_SCALE;


    /**
     * 降低油门时使用较快的Attack；
     * 恢复油门时使用较慢的Release，防止电流在44A附近振荡
     */
    const float response_tau = (target_scale < s_current_scale)
                          ? CURRENT_ATTACK_TAU_S
                          : CURRENT_RELEASE_TAU_S;

    const float response_alpha = TASK_PM_DT_S / (response_tau + TASK_PM_DT_S);

    s_current_scale +=
        response_alpha * (target_scale - s_current_scale);

    /**
     * FlightCtrl只需要读取permile整数。
     * 在STM32F4上，对齐的16位整数读写是原子的，
     * 不需要为了这一项在高频控制循环里加Mutex。
     */
    g_power_health.current_limit_permille = (uint16_t)(s_current_scale * 1000.0f + 0.5f);

    // 低于99.5%时视为限流正在介入
    g_power_health.current_limiting = (g_power_health.current_limit_permille < 995U);
    // 保存滤波值
    g_power_health.current_filtered_a = s_current_filtered;
}

static void CurrentLimitIndicator_Update(void)
{
    if(!g_power_health.current_limiting)
    {
        s_current_limit_reported = false;
        return;
    }

    uint32_t now = osKernelGetTickCount();

    if(!s_current_limit_reported ||
        (now - s_current_limit_last_report_tick) >= CURRENT_LIMIT_REPOST_MS)
    {
        IndicatorEvent_t evt = EVT_CURRENT_LIMITING;

        if(osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0U, 0U) == osOK)
        {
            s_current_limit_reported = true;
            s_current_limit_last_report_tick = now;
        }
    }
}

void App_PowerMonit_Task(void *argument)
{
    (void)argument;
    BSP_Power_Init();

    uint32_t low_count = 0;
    PowerData_t pwr;

    for (;;)
    {
        BSP_Power_Read(&pwr);

        CurrentLimiter_Update(pwr.current);
        CurrentLimitIndicator_Update();

        g_heartbeat.powermonit_last_tick = osKernelGetTickCount();

        if(pwr.vbat <= VBAT_DISARM_THRESHOLD)
        {
            if(low_count < VBAT_FAULT_CONFIRM_COUNT)
                low_count++;
            
            if(low_count >= VBAT_FAULT_CONFIRM_COUNT)
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
