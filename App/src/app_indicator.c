/**
 * @file    app_indicator.c
 * @brief   系统事件蜂鸣器与 LED 提示逻辑实现。
 * 
 * 每种 Indicator Event 映射到一个 Pattern；
 * Pattern 由若干 Beep Step 组成，并附带事件优先级。
 * 
 * Pattern 播放期间允许更高优先级事件在当前 Step 结束后抢占，
 * 低于或等于当前优先级的事件重新放回 Queue 等待后续处理。
 */

#include "app_indicator.h"
#include "app_shared_types.h"
#include "bsp_buzzer.h"
#include "bsp_led.h"
#include "cmsis_os2.h"
#include <stdbool.h>

extern osMessageQueueId_t IndicatorEventQueueHandle;

/*
 * Indicator Event 优先级。
 * 数值越大优先级越高，只有严格更高优先级的新事件
 * 才允许中断当前正在播放的 Pattern。
 */
#define INDICATOR_PRIO_NOTICE 1U        // 普通状态提示，如 GPS Fix，Calibration Success。
#define INDICATOR_PRIO_WARNING 2U       // 需要关注但非立即安全故障的警告。
#define INDICATOR_PRIO_POWER_LIMIT 3U   // Current Limiting 等实时保护状态。
#define INDICATOR_PRIO_CRITICAL 4U      // 电源，SD，IMU，RC 等关键故障。
#define INDICATOR_PRIO_STATE_CHANGE 5U  // Armed/Disarmed 状态变化，具有最高提示优先级。

/**
 * @brief   单个蜂鸣器/LED 节拍。
 */
typedef struct
{
    uint16_t on_ms;     /**< Buzzer 与 lED 保持 ON 的时间，ms。 */
    uint16_t off_ms;    /**< 当前 Step 后保持 OFF 的时间，ms。 */
} BeepStep_t;

/**
 * @brief   一个完整 Indicator Event 的播放 Pattern。
 */
typedef struct
{
    const BeepStep_t *steps;    /**< Pattern 的 Step 序列。 */
    uint8_t step_count;         /**< Step 数量。 */
    uint8_t priority;           /**< Pattern 抢占优先级。 */
} IndicatorPattern_t;

/*
 * 各 Indicator Event 对应的具体提示节拍。
 *
 * 节拍参数属于人机提示层配置，可根据装机后的实际辨识度调整，
 * 无需修改事件调度与抢占机制。
 */
static const BeepStep_t s_pat_armed[] = {
    {200U, 0U}}; /**< Armed：单声长鸣。 */

static const BeepStep_t s_pat_disarmed[] = {
    {80U, 80U},
    {80U, 0U}}; /**< Disarmed：两声短鸣。 */

static const BeepStep_t s_pat_low_battery[] = {
    {300U, 150U},
    {300U, 150U},
    {300U, 0U}}; /**< Low Battery：三声中长提示。 */

static const BeepStep_t s_pat_critical_battery[] = {
    {100U, 60U},
    {100U, 60U},
    {100U, 60U},
    {100U, 60U},
    {100U, 0U}}; /**< Critical Battery：五连急促提示。 */

static const BeepStep_t s_pat_gps_fix[] = {
    {50U, 50U},
    {50U, 50U},
    {50U, 0U}}; /**< GPS Fix：三声短提示。 */

static const BeepStep_t s_pat_sd_error[] = {
    {400U, 200U},
    {400U, 0U}}; /**< SD Error：两声长鸣。 */

static const BeepStep_t s_pat_imu_fault[] = {
    {60U, 60U},
    {60U, 60U},
    {60U, 60U},
    {60U, 60U},
    {60U, 60U},
    {60U, 0U}}; /**< IMU Fault：六连急促提示。 */

static const BeepStep_t s_pat_rc_lost[] = {
    {600U, 200U},
    {600U, 200U},
    {600U, 0U}}; /**< RC Lost：三声长鸣。 */

static const BeepStep_t s_pat_current_limiting[] = {
    {100U, 80U},
    {100U, 80U},
    {400U, 80U}}; /**< Current Limiting：两短一长。 */

static const BeepStep_t s_pat_rc_cal_started[] = {
    {70U, 70U},
    {70U, 0U}}; /**< RC Calibration Started：两声短鸣。 */

static const BeepStep_t s_pat_rc_cal_success[] = {
    {60U, 60U},
    {60U, 60U},
    {250U, 0U}}; /**< RC Calibration Success：两短一长。 */

static const BeepStep_t s_pat_rc_cal_failed[] = {
    {350U, 100U},
    {80U, 0U}}; /**< RC Calibration Failed：一长一短。 */

static const BeepStep_t s_pat_gyro_cal_success[] = {
    {50U, 50U},
    {150U, 0U}}; /**< Gyro Calibration Success：一短一中。 */

static const BeepStep_t s_pat_level_trim_started[] = {
    {80U, 80U},
    {80U, 80U},
    {80U, 0U}}; /**< Level Trim Started：三声短鸣。 */

static const BeepStep_t s_pat_level_trim_success[] = {
    {80U, 80U},
    {250U, 0U}}; /**< Level Trim Success：一短一长。 */

static const BeepStep_t s_pat_level_trim_failed[] = {
    {400U, 120U},
    {80U, 120U},
    {80U, 0U}}; /**< Level Trim Failed：一长两短。 */

static const BeepStep_t s_pat_mag_cal_started[] = {
    {70U, 60U},
    {70U, 60U},
    {70U, 60U},
    {70U, 0U}}; /**< Mag Calibration Started：四声短鸣。 */

static const BeepStep_t s_pat_mag_cal_done[] = {
    {80U, 80U},
    {80U, 80U},
    {300U, 0U}}; /**< Mag Calibration Done：两短一长。 */

static const BeepStep_t s_pat_mag_cal_failed[] = {
    {450U, 120U},
    {100U, 0U}}; /**< Mag Calibration Failed：一长一中。 */

/*
 * 根据 Indicator Event 获取对应 Pattern。
 *
 * 使用 switch 显式绑定 Event 与 Pattern，不依赖 Enum 数值作为数组索引，
 * 因此后续插入或调整 IndicatorEvent_t 枚举顺序不会造成映射错位。
 *
 * 未识别的 Event 返回 NULL，由调用方防御性丢弃。
 */
static const IndicatorPattern_t *Indicator_GetPattern(
    IndicatorEvent_t evt)
{
    switch (evt)
    {
    case EVT_ARMED:
    {
        static const IndicatorPattern_t pat = {
            s_pat_armed,
            1U,
            INDICATOR_PRIO_STATE_CHANGE};
        return &pat;
    }

    case EVT_DISARMED:
    {
        static const IndicatorPattern_t pat = {
            s_pat_disarmed,
            2U,
            INDICATOR_PRIO_STATE_CHANGE};
        return &pat;
    }

    case EVT_LOW_BATTERY:
    {
        static const IndicatorPattern_t pat = {
            s_pat_low_battery,
            3U,
            INDICATOR_PRIO_WARNING};
        return &pat;
    }

    case EVT_CRITICAL_BATTERY:
    {
        static const IndicatorPattern_t pat = {
            s_pat_critical_battery,
            5U,
            INDICATOR_PRIO_CRITICAL};
        return &pat;
    }

    case EVT_GPS_FIX_ACQUIRED:
    {
        static const IndicatorPattern_t pat = {
            s_pat_gps_fix,
            3U,
            INDICATOR_PRIO_NOTICE};
        return &pat;
    }

    case EVT_SD_CARD_ERROR:
    {
        static const IndicatorPattern_t pat = {
            s_pat_sd_error,
            2U,
            INDICATOR_PRIO_CRITICAL};
        return &pat;
    }

    case EVT_IMU_FAULT:
    {
        static const IndicatorPattern_t pat = {
            s_pat_imu_fault,
            6U,
            INDICATOR_PRIO_CRITICAL};
        return &pat;
    }

    case EVT_RC_LOST:
    {
        static const IndicatorPattern_t pat = {
            s_pat_rc_lost,
            3U,
            INDICATOR_PRIO_CRITICAL};
        return &pat;
    }

    case EVT_CURRENT_LIMITING:
    {
        static const IndicatorPattern_t pat = {
            s_pat_current_limiting,
            3U,
            INDICATOR_PRIO_POWER_LIMIT};
        return &pat;
    }

    case EVT_RC_CALIB_STARTED:
    {
        static const IndicatorPattern_t pat = {
            s_pat_rc_cal_started,
            2U,
            INDICATOR_PRIO_NOTICE};
        return &pat;
    }

    case EVT_RC_CALIB_SUCCESS:
    {
        static const IndicatorPattern_t pat = {
            s_pat_rc_cal_success,
            3U,
            INDICATOR_PRIO_NOTICE};
        return &pat;
    }

    case EVT_RC_CALIB_FAILED:
    {
        static const IndicatorPattern_t pat = {
            s_pat_rc_cal_failed,
            2U,
            INDICATOR_PRIO_WARNING};
        return &pat;
    }

    case EVT_GYRO_CALIB_SUCCESS:
    {
        static const IndicatorPattern_t pat = {
            s_pat_gyro_cal_success,
            2U,
            INDICATOR_PRIO_NOTICE};
        return &pat;
    }

    case EVT_LEVEL_TRIM_STARTED:
    {
        static const IndicatorPattern_t pat = {
            s_pat_level_trim_started,
            3U,
            INDICATOR_PRIO_NOTICE};
        return &pat;
    }

    case EVT_LEVEL_TRIM_SUCCESS:
    {
        static const IndicatorPattern_t pat = {
            s_pat_level_trim_success,
            2U,
            INDICATOR_PRIO_NOTICE};
        return &pat;
    }

    case EVT_LEVEL_TRIM_FAILED:
    {
        static const IndicatorPattern_t pat = {
            s_pat_level_trim_failed,
            3U,
            INDICATOR_PRIO_WARNING};
        return &pat;
    }

    case EVT_MAG_CAL_CAPTURE_STARTED:
    {
        static const IndicatorPattern_t pat = {
            s_pat_mag_cal_started,
            4U,
            INDICATOR_PRIO_NOTICE};
        return &pat;
    }

    case EVT_MAG_CAL_CAPTURE_DONE:
    {
        static const IndicatorPattern_t pat = {
            s_pat_mag_cal_done,
            3U,
            INDICATOR_PRIO_NOTICE};
        return &pat;
    }

    case EVT_MAG_CAL_CAPTURE_FAILED:
    {
        static const IndicatorPattern_t pat = {
            s_pat_mag_cal_failed,
            2U,
            INDICATOR_PRIO_WARNING};
        return &pat;
    }

    default:
        return NULL;
    }
}

/*
 * 播放一个完整 Indicator Pattern。
 * 
 * 每个 Step 完成 ON/OFF 节拍后非阻塞检查一次 Event Queue。
 * 若发现严格高于当前 Pattern 优先级的新事件，则提前结束当前 Pattern，
 * 并通过 out_preempt 将抢占事件交回主循环立即播放。
 * 
 * 低于或等于当前优先级的事件重新放回 Queue，
 * 保留通知但不允许打断当前 Pattern。
 */
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

        /*
         * 抢占粒度为一个完整 Beep Step。
         * 不在 ON/OFF 延时中间切断节拍，避免产生难以辨识的残缺提示音。
         */
        IndicatorEvent_t next_evt;

        if(osMessageQueueGet(IndicatorEventQueueHandle, &next_evt, NULL, 0) == osOK)
        {
            const IndicatorPattern_t *next_pat = Indicator_GetPattern(next_evt);
            if(next_pat != NULL && next_pat->priority > pat->priority)
            {
                *out_preempt = next_evt;
                return true;
            }

            /*
             * 优先级不足的合法事件暂不抢占：
             * 未知事件也保守放回 Queue，由主循环后续再次校验。
             */
            osMessageQueuePut(IndicatorEventQueueHandle, &next_evt, 0, 0);
        }
    }

    return false;
}

/*
 * Indicator 后台任务。
 * 
 * 阻塞等待系统 Indicator Event，将其转换为对应 Pattern 播放。
 * Pattern 播放期间若受到更高优先级事件，则立即切换到新 Pattern；
 * 新 Pattern 仍可继续被更高优先级事件再次抢占。
 */
void App_Indicator_Task(void *argument)
{
    (void)argument;

    BSP_Buzzer_Init();
    BSP_LED_Init();

    IndicatorEvent_t evt;

    for (;;)
    {
        /*
         * 无事件时永久阻塞，
         * Indicator Task 不通过周期轮询占用 CPU。
         */
        if(osMessageQueueGet(IndicatorEventQueueHandle, &evt, NULL, osWaitForever) != osOK)
            continue;

        const IndicatorPattern_t *pat = Indicator_GetPattern(evt);
        
        // 未知 Event 防御性丢弃。
        if(pat == NULL)
            continue;

        IndicatorEvent_t preempt_evt;

        /*
         * 若当前 Pattern 被抢占，立即转去播放抢占事件。
         * while 允许新的 Pattern 再次被更高优先级事件抢占。
         */
        while(Indicator_PlayPattern(pat, &preempt_evt))
        {
            evt = preempt_evt;
            pat = Indicator_GetPattern(evt);

            // PlayPattern 已检查过 Pattern，这里仅作防御性兜底。
            if(pat == NULL)
                break; 
        }
    }
}
