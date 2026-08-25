/**
 * @file    app_power_monitor.c
 * @brief   电池电压监测，故障确认及电流软限流实现。
 */

#include "app_power_monitor.h"
#include "app_shared_types.h"
#include "bsp_power.h"
#include "bsp_blackbox.h"
#include "cmsis_os2.h"

/*================================ Voltage Monitor ====================================*/

/*
 * 4S 电池 Critical Voltage 门限。
 * 持续低于该值会置起 voltage_fault，并由上层执行 Disarm 保护。
 */
#define VBAT_DISARM_THRESHOLD 13.2f 

#define VBAT_RECOVER_THRESHOLD 13.5f    // Critical Voltage 恢复门限，形成迟滞避免临界点反复切换。

/*
 * Critical Voltage 连续确认次数。
 * Power Monitor 周期 20 ms 时，25次约对应 500ms，
 * 用于过滤大机动造成的短时 Voltage Sag。
 */
#define VBAT_FAULT_CONFIRM_COUNT 25    

#define CRITICAL_BATTERY_REPOST_MS 1500U    // Critical Battery 持续故障提示重发周期，ms。

#define VBAT_LOW_WARNING_THRESHOLD 14.0f    // Low Battery 早起预警门限，只提示，不触发保护动作。

#define VBAT_LOW_WARNING_RECOVER 14.2f      // Low Battery 预警重新使能门限，用于形成告警迟滞。

/*================================ Current Limiter ====================================*/

/*
 * Current Limiter 总开关。
 * 当前阶段旁路限流，但仍保留 Current Filter 与 Blackbox 数据链。
 */
#define CURRENT_LIMITER_ENABLED 0U

#define CURRENT_LIMIT_A 44.0f           // 整机允许的持续电流目标上限，A。
                                
/*
 * Collective Throttle 最小缩放比例。
 * 即使严重过流也最多压缩到 60%，避免保护本身直接完全夺走升力。
 */
#define CURRENT_LIMIT_MIN_SCALE 0.60f   

#define CURRENT_FILTER_TAU_S 0.10f      // Current Measurement 一阶低通时间常数，s。
#define CURRENT_ATTACK_TAU_S 0.15f      // 限流介入时的响应时间常数，s。
#define CURRENT_RELEASE_TAU_S 1.00f     // 电流恢复后释放限流的响应时间常数，s。

#define CURRENT_LIMIT_REPOST_MS 2000U   // Current Limiting 持续提示重发周期，ms。

#define TASK_PM_PERIOD_MS 20U           // Power Monitor Task 周期，ms。

#define TASK_PM_DT_S 0.02f              // Power Monitor 标称执行周期，s。

extern osEventFlagsId_t SystemReadyEventGroupHandle;
extern osMessageQueueId_t IndicatorEventQueueHandle;

/* Low Battery Warning 是否已经在当前低压区间报告过。 */
static bool s_low_battery_warned = false;

/* 上一周期 Critical Battery Fault 是否处于 Active。 */
static bool s_critical_battery_was_active = false;

/* 最近一次 Critical Battery 提示成功投递时间。 */
static uint32_t s_critical_battery_last_repost_tick = 0;

/* 一阶低通后的整机电流，A。 */
static float s_current_filtered = 0.0f;

#if CURRENT_LIMITER_ENABLED != 0U
/* 当前 Collective Throttle 缩放比例，范围约为 0.60~1.00。 */
static float s_current_scale = 1.0f;
#endif

/* Current Filter 是否已经取得首个有效初值。 */
static bool s_current_initialized = false;

/*
 * 当前连续限流区间是否已经成功投递过 Indicator Event。
 * Queue 满导致投递失败时保持 false，下一周期继续尝试。
 */
static bool s_current_limit_reported = false;

/* 最近一次 Current Limiting 提示成功投递时间。 */
static uint32_t s_current_limit_last_report_tick = 0U;

/*
 * 根据整机总电流更新 Collective Throttle 限流比例。
 * 
 * 该限流器属于反馈式 Soft Limiter：
 * 不假设 DShot Command 与电流严格线性，而根据实际测得电流
 * 动态调整允许的 Collective Throttle Scale。
 * 
 * Roll/Pitch/Yaw Correction 不在此处缩放，
 * 优先保留姿态控制权限。 
 */
static void CurrentLimiter_Update(float current)
{
    // ADC Offset 或换算误差可能产生小负值，物理电流按 0 A 处理。
    if(current < 0.0f)
        current = 0.0f;

    /*
     * 第一帧直接建立 Filter State，
     * 避免滤波值从 0A 缓慢爬升造成启动阶段读数失真。
     */
    if(!s_current_initialized)
    {
        s_current_filtered = current;
        s_current_initialized = true;
    }
    else
    {
        /**
         * 一阶低通：
         * 
         * y += alpha * (x - y)
         * 
         * alpha = dt / (tau + dt)
         */
        const float alpha =
            TASK_PM_DT_S / (CURRENT_FILTER_TAU_S + TASK_PM_DT_S);

        s_current_filtered +=
            alpha * 
            (current - s_current_filtered);
    }

#if CURRENT_LIMITER_ENABLED == 0U

    /*
     * limiter Disabled 时仍维护滤波后的 Current，
     * 供 Blackbox 与 ADC Calibration 使用；
     * 但始终向 FlightCtrl 发布 100% Throttle 权限。
     */
    g_power_health.current_limit_permille = 1000U;
    g_power_health.current_limiting = false;
    g_power_health.current_filtered_a = s_current_filtered;

#else

    // 默认目标为不限幅 Collective Throttle。
    float target_scale = 1.0f;  

    /*
     * 电流超过门限后，以 I_limit / I_measured 生成目标缩放比例。
     * 
     * 如：
     * I_limit = 44 A
     * I       = 55 A
     * 
     * target_scale = 44 / 55 = 0.8
     * 
     * 这里只把它作为反馈方向和幅度估计，
     * 并不要求 Motor Throttle 与 Current 严格线性。
     */
    if(s_current_filtered > CURRENT_LIMIT_A)
        target_scale = CURRENT_LIMIT_A / s_current_filtered;
    
    if(target_scale < CURRENT_LIMIT_MIN_SCALE)
        target_scale = CURRENT_LIMIT_MIN_SCALE;


    /*
     * Attack 快，Release 慢：
     * 
     * - 过流时较快降低 Throttle：
     * - 电流恢复后缓慢释放限制。
     * 
     * 这样可以减少电流在 Limit 附近反复进入/退出限流造成的振荡。
     */
    const float response_tau = 
        (target_scale < s_current_scale)
            ? CURRENT_ATTACK_TAU_S
            : CURRENT_RELEASE_TAU_S;

    const float response_alpha = 
        TASK_PM_DT_S / (response_tau + TASK_PM_DT_S);

    s_current_scale +=
        response_alpha * (target_scale - s_current_scale);

    /*
     * FlightCtrl 只需要读取千分比整数 Scale。
     * 1000 表示 100%，600 表示 60%。
     */
    g_power_health.current_limit_permille = 
        (uint16_t)(s_current_scale * 1000.0f + 0.5f);

    /*
     * Scale 低于 99.5% 即认为 Current Limiter 已实际介入，
     * 避免微小浮点误差被当成有效限流。
     */
    g_power_health.current_limiting = 
        (g_power_health.current_limit_permille < 995U);
    
    g_power_health.current_filtered_a = s_current_filtered;
    
#endif
}

/*
 * 维护 Current Limiting Indicator Event。
 * 
 * 一次连续限流区间首次进入时立即提示；
 * 若限流持续存在，则按固定周期重新提示。
 */
static void CurrentLimitIndicator_Update(void)
{
    if(!g_power_health.current_limiting)
    {
        /*
         * 本轮限流已经解除。
         * 下次重新进入限流时允许立即发送新的边沿提示。
         */
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
            /*
             * 只有 Event 真正进入 Queue 后才记录 Reported。
             * Queue 满时下一周期继续尝试，不静默丢失首次提示。
             */
            s_current_limit_reported = true;
            s_current_limit_last_report_tick = now;
        }
    }
}

void App_PowerMonit_Task(void *argument)
{
    (void)argument;

    BSP_Power_Init();

    /* Critical Voltage 连续低压 Sample Count。 */
    uint32_t low_count = 0;

    PowerData_t pwr;

    for (;;)
    {
        BSP_Power_Read(&pwr);

        CurrentLimiter_Update(pwr.current);
        CurrentLimitIndicator_Update();

        g_heartbeat.powermonit_last_tick = osKernelGetTickCount();

        /*
         * Critical Voltage Fault：
         * 
         * 必须连续多周期低于门限才确认，
         * 避免瞬时大电流导致的 Voltage Sag 直接触发 Disarm。
         */
        if(pwr.vbat <= VBAT_DISARM_THRESHOLD)
        {
            if(low_count < VBAT_FAULT_CONFIRM_COUNT)
                low_count++;
            
            if(low_count >= VBAT_FAULT_CONFIRM_COUNT)
            {
                /*
                 * Voltage Fault 只在 0 -> 1 边沿记录一次 Blackbox Event。
                 */
                if (!g_power_health.voltage_fault)
                    BB_LogVoltageFault(osKernelGetTickCount());

                g_power_health.voltage_fault = true;
                osEventFlagsClear(SystemReadyEventGroupHandle, SYSREADY_BIT_VOLTAGE_OK);
            }
        }
        else
        {
            /*
             * 一旦离开 Fault Threshold，
             * 连续低压确认过程立即重新开始。
             */
            low_count = 0U;

            /*
             * 已经触发的 Voltage Fault 只有在电压恢复到更高的
             * Recover Threshold 后才解除，形成 13.2 / 13.5 V Hysteresis。
             */
            if(pwr.vbat >= VBAT_RECOVER_THRESHOLD)
            {
                g_power_health.voltage_fault = false;
                osEventFlagsSet(SystemReadyEventGroupHandle, SYSREADY_BIT_VOLTAGE_OK);
            }
        }

        /*
         * Critical Battery Fault 持续期间周期重发提示。
         * 首次进入 Fault 时立即发送。
         */
        if(g_power_health.voltage_fault)
        {
            const uint32_t now = osKernelGetTickCount();

            if(!s_critical_battery_was_active ||
                (now - s_critical_battery_last_repost_tick) >= CRITICAL_BATTERY_REPOST_MS)
            {
                IndicatorEvent_t evt = EVT_CRITICAL_BATTERY;

                osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0, 0);

                s_critical_battery_last_repost_tick = now;
            }
        }

        s_critical_battery_was_active = g_power_health.voltage_fault;

        /*
         * Low Battery Early Warning:
         * 只做提示，不修改 Voltage Fault，也不触发 Disarm。
         * 
         * 一次低压区间只提示一次，
         * 电压恢复到更高门限后重新允许下一次 Warning。
         */
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
