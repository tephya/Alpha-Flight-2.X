#include "app_indicator.h"
#include "app_shared_types.h"
#include "bsp_buzzer.h"
#include "bsp_led.h"
#include "cmsis_os2.h"
#include <stdbool.h>

extern osMessageQueueId_t IndicatorEventQueueHandle;

/*======= 低优先级分档：数值越大越优先，只有严格更高优先级的新事件才能打断当前正在播放的节拍 =======*/
#define INDICATOR_PRIO_NOTICE 1U // 纯提示：解锁/上锁/GPS定位成功
#define INDICATOR_PRIO_WARNING 2U // 需要关注但非当下失控：低电压
#define INDICATOR_PRIO_POWER_LIMIT 3U   // 过流保护
#define INDICATOR_PRIO_CRITICAL 4U      // 安全相关，必须立刻被听到：极低压/SD错误/IMU故障/失联
#define INDICATOR_PRIO_STATE_CHANGE 5U  // 解锁/上锁提示音，必须无条件优先于一切警告

typedef struct
{
    uint16_t on_ms;
    uint16_t off_ms;
} BeepStep_t;

typedef struct
{
    const BeepStep_t *steps;
    uint8_t step_count;
    uint8_t priority;
} IndicatorPattern_t;

/*====== 各事件的具体节拍。数值是经验起点，装机实测听感不好分别率就调这里，不用动架构 ======*/
static const BeepStep_t s_pat_armed[] = {{200, 0}};                 // 单声长鸣，锁确认
static const BeepStep_t s_pat_disarmed[] = {{80, 80}, {80, 0}};     // 两声短鸣
static const BeepStep_t s_pat_low_battery[] = {{300, 150}, {300, 150}, {300, 0}};   // 三声中等
static const BeepStep_t s_pat_critical_battery[] = {{100, 60}, {100, 60}, {100, 60}, {100, 60}, {100, 0}};      // 五连急促
static const BeepStep_t s_pat_gps_fix[] = {{50, 50}, {50, 50}, {50, 0}};            // 三声轻快短
static const BeepStep_t s_pat_sd_error[] = {{400, 200}, {400, 0}};                  // 两声长鸣，区别于低压警告
static const BeepStep_t s_pat_imu_fault[] = {{60, 60}, {60, 60}, {60, 60}, {60, 60}, {60, 60}, {60, 0}};        // 六连急促
static const BeepStep_t s_pat_rc_lost[] = {{600, 200}, {600, 200}, {600, 0}};       // 三声长鸣，最沉稳但最不能忽略
static const BeepStep_t s_pat_current_limiting[] = {{100, 80}, {100, 80}, {400, 80}};   // 三声，两短一长
static const BeepStep_t s_pat_rc_cal_started[] = {{70, 70}, {70, 0}};               // 两短
static const BeepStep_t s_pat_rc_cal_success[] = {{60, 60}, {60, 60}, {250, 0}};    // 两短+长
static const BeepStep_t s_pat_rc_cal_failed[] = {{350, 100}, {80, 0}};      // 一长一短
static const BeepStep_t s_pat_gyro_cal_success[] = {{50, 50}, {150, 0}};    // 短+中
static const BeepStep_t s_pat_level_trim_started[] = {{80, 80}, {80, 80}, {80, 0}};    // 三短
static const BeepStep_t s_pat_level_trim_success[] = {{80, 80}, {250, 0}};  // 短+长
static const BeepStep_t s_pat_level_trim_failed[] = {{400, 120}, {80, 120}, {80, 0}};   // 长+两短
static const BeepStep_t s_pat_mag_cal_started[] = {{70, 60}, {70, 60}, {70, 60}, {70, 0}};      // 四短
static const BeepStep_t s_pat_mag_cal_done[] = {{80, 80}, {80, 80}, {300, 0}};      // 两短+长
static const BeepStep_t s_pat_mag_cal_failed[] = {{450, 120}, {100, 0}};        // 长+中

/**
 * @brief   按事件类型返回对应节拍
 * @note    靠枚举名字(case)绑定，不靠数组位置，
 *          谁在IndicatorEvent_t里插入新值/调整顺序都不会导致这里错位，
 *          新增的枚举值如果忘了就在这里加case，走到default返回NULL，
 *          调用方会把它当异常值防御性丢弃，不会误播成别的事件的节拍。
 */
static const IndicatorPattern_t *Indicator_GetPattern(IndicatorEvent_t evt)
{
    switch (evt)
    {
    case EVT_ARMED:
    {
        static const IndicatorPattern_t pat = {s_pat_armed,
                                               1,
                                               INDICATOR_PRIO_STATE_CHANGE};
        return &pat;
    }
    case EVT_DISARMED:
    {
        static const IndicatorPattern_t pat = {s_pat_disarmed,
                                               2,
                                               INDICATOR_PRIO_STATE_CHANGE};
        return &pat;
    }
    case EVT_LOW_BATTERY:
    {
        static const IndicatorPattern_t pat = {s_pat_low_battery,
                                               3,
                                               INDICATOR_PRIO_WARNING};
        return &pat;
    }
    case EVT_CRITICAL_BATTERY:
    {
        static const IndicatorPattern_t pat = {s_pat_critical_battery,
                                               5,
                                               INDICATOR_PRIO_CRITICAL};
        return &pat;
    }
    case EVT_GPS_FIX_ACQUIRED:
    {
        static const IndicatorPattern_t pat = {s_pat_gps_fix,
                                               3,
                                               INDICATOR_PRIO_NOTICE};
        return &pat;
    }

    case EVT_SD_CARD_ERROR:
    {
        static const IndicatorPattern_t pat = {s_pat_sd_error,
                                               2,
                                               INDICATOR_PRIO_CRITICAL};
        return &pat;
    }
    case EVT_IMU_FAULT:
    {
        static const IndicatorPattern_t pat = {s_pat_imu_fault,
                                               6,
                                               INDICATOR_PRIO_CRITICAL};
        return &pat;
    }
    case EVT_RC_LOST:
    {
        static const IndicatorPattern_t pat = {s_pat_rc_lost,
                                               3,
                                               INDICATOR_PRIO_CRITICAL};
        return &pat;
    }
    case EVT_CURRENT_LIMITING:
    {
        static const IndicatorPattern_t pat = {s_pat_current_limiting,
                                             3,
                                             INDICATOR_PRIO_POWER_LIMIT};
        return &pat;
    }
    case EVT_RC_CALIB_STARTED:
    {
        static const IndicatorPattern_t pat = {s_pat_rc_cal_started,
                                               2,
                                               INDICATOR_PRIO_NOTICE};
        return &pat;
    }
    case EVT_RC_CALIB_SUCCESS:
    {
        static const IndicatorPattern_t pat = {s_pat_rc_cal_success,
                                               3,
                                               INDICATOR_PRIO_NOTICE};
        return &pat;
    }
    case EVT_RC_CALIB_FAILED:
    {
        static const IndicatorPattern_t pat = {s_pat_rc_cal_failed,
                                               2,
                                               INDICATOR_PRIO_WARNING};
        return &pat;
    }
    case EVT_GYRO_CALIB_SUCCESS:
    {
        static const IndicatorPattern_t pat = {s_pat_gyro_cal_success,
                                               2,
                                               INDICATOR_PRIO_NOTICE};
        return &pat;
    }
    case EVT_LEVEL_TRIM_STARTED:
    {
        static const IndicatorPattern_t pat = {s_pat_level_trim_started,
                                             3,
                                             INDICATOR_PRIO_NOTICE};
        return &pat;
    }
    case EVT_LEVEL_TRIM_SUCCESS:
    {
        static const IndicatorPattern_t pat = {s_pat_level_trim_success,
                                             2,
                                             INDICATOR_PRIO_NOTICE};
        return &pat;
    }
    case EVT_LEVEL_TRIM_FAILED:
    {
        static const IndicatorPattern_t pat = {s_pat_level_trim_failed,
                                             3,
                                             INDICATOR_PRIO_WARNING};
        return &pat;
    }
    case EVT_MAG_CAL_CAPTURE_STARTED:
    {
        static const IndicatorPattern_t pat = {s_pat_mag_cal_started,
                                               4,
                                               INDICATOR_PRIO_NOTICE};
        return &pat;
    }
    case EVT_MAG_CAL_CAPTURE_DONE:
    {
        static const IndicatorPattern_t pat = {s_pat_mag_cal_done,
                                               3,
                                               INDICATOR_PRIO_NOTICE};
        return &pat;
    }
    case EVT_MAG_CAL_CAPTURE_FAILED:
    {
        static const IndicatorPattern_t pat = {s_pat_mag_cal_failed,
                                               2,
                                               INDICATOR_PRIO_WARNING};
        return &pat;
    }
    default:
        return NULL;        // 不认识的事件值，调用方需检查NULL
    }
}


static bool Indicator_PlayPattern(const IndicatorPattern_t *pat, IndicatorEvent_t *out_preempt)
{
    for (uint8_t i = 0; i < pat->step_count; i++)
    {
        BSP_Buzzer_On();
        BSP_LED_On();
        osDelay(pat->steps[i].on_ms);

        BSP_Buzzer_Off();
        BSP_LED_Off();
        osDelay(pat->steps[i].off_ms);

        IndicatorEvent_t next_evt;
        if(osMessageQueueGet(IndicatorEventQueueHandle, &next_evt, NULL, 0) == osOK)
        {
            const IndicatorPattern_t *next_pat = Indicator_GetPattern(next_evt);
            if(next_pat != NULL && next_pat->priority > pat->priority)
            {
                *out_preempt = next_evt;
                return true;        // 优先级足够高，当前界牌提前终止，把新事件交回主循环播放
            }
            else
            {
                /*优先级不够高(或事件值异常越界，保守也放回去)，塞回队列尾部，
                 * 不丢弃这次通知，等当前这轮播完了自然轮到它*/
                osMessageQueuePut(IndicatorEventQueueHandle, &next_evt, 0, 0);
            }
        }
    }
    return false;
}

/**
 * @brief   Task_Indicator任务入口，由freertos.c的StartTask_Indicator转发调用
 * @note    消费IndicatorEventQueue，把事件翻译成蜂鸣器+LED同步闪烁
 *          高优先级事件可以打断正在播放的低优先级节拍
 */
void App_Indicator_Task(void *argument)
{
    (void)argument;

    BSP_Buzzer_Init();
    BSP_LED_Init();

    IndicatorEvent_t evt;

    for (;;)
    {
        if(osMessageQueueGet(IndicatorEventQueueHandle, &evt, NULL, osWaitForever) != osOK)
            continue;

        const IndicatorPattern_t *pat = Indicator_GetPattern(evt);
        if(pat == NULL)
            continue;       // 异常值，防御性丢弃，不该发生

        IndicatorEvent_t preempt_evt;
        while(Indicator_PlayPattern(pat, &preempt_evt))
        {
            evt = preempt_evt;  // 被打断，紧接着播放打断它的那个事件；如果那个又被更高优先级打断，继续循环
            pat = Indicator_GetPattern(evt);
            if(pat == NULL)
                break;          // GatPattern已经在PlayPatten内部过滤过NULL，防御性兜底
        }
    }
}
