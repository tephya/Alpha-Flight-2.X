#include "app_rc_calibration.h"
#include "app_shared_types.h"
#include "app_mag_calibration.h"
#include "bsp_config_flash.h"
#include "cmsis_os2.h"
#include <limits.h>
#include <string.h>

extern osMessageQueueId_t IndicatorEventQueueHandle;

#define RC_CONFIG_RECORD_TYPE 0x0001U
#define RC_CONFIG_VERSION 1U

#define RC_CH_ROLL 0U
#define RC_CH_PITCH 1U
#define RC_CH_THROTTLE 2U
#define RC_CH_YAW 3U
#define RC_CH_ARM_SWITCH 4U
#define RC_CH_MODE_SWITCH 5U

#define RC_SWITCH_ON_THRESHOLD 1500U
#define RC_ARM_SWITCH_OFF_MAX 900U
#define RC_TRIGGER_LOW 300U
#define RC_TRIGGER_HIGH 1700U
#define RC_TRIGGER_HOLD_MS 2000U
#define RC_CAPTURE_MIN_MS 3000U
#define RC_CAPTURE_TIMEOUT_MS 30000U
#define RC_CENTER_HOLD_MS 1000U
#define RC_CENTER_TIMEOUT_MS 15000U

#define RC_MIN_FULL_SPAN 1400U
#define RC_MIN_HALF_SPAN 300U
#define RC_CENTER_WINDOW_COUNTS 200U
#define RC_CENTER_STABILITY_COUNTS 12U
#define RC_THROTTLE_LOW_MARGIN 60U
#define RC_NORMALIZED_MIN 172U
#define RC_NORMALIZED_MID 992U
#define RC_NORMALIZED_MAX 1811U
#define RC_NORMALIZED_DEADBAND 15U

typedef enum
{
    RC_CAL_STATE_IDLE = 0,
    RC_CAL_STATE_CAPTURE_RANGE,
    RC_CAL_STATE_CAPTURE_CENTER,
} RcCalibrationState_t;

static RcCalibration_t s_calibration;
static RcCalibration_t s_working;
static RcCalibrationState_t s_state = RC_CAL_STATE_IDLE;
static bool s_ready = false;

static bool s_entry_holding = false;
static uint32_t s_entry_start_tick = 0U;
static uint32_t s_state_start_tick = 0U;

static bool s_center_holding = false;
static uint32_t s_center_start_tick = 0U;
static uint32_t s_center_count = 0U;
static uint32_t s_center_sum[RC_CAL_CHANNEL_COUNT];
static uint16_t s_center_min[RC_CAL_CHANNEL_COUNT];
static uint16_t s_center_max[RC_CAL_CHANNEL_COUNT];

static void RcCalibration_PostEvent(IndicatorEvent_t evt)
{
    (void)osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0U, 0U);
}

static void RcCalibration_LoadDefaults(void)
{
    for (uint8_t i = 0U; i < RC_CAL_CHANNEL_COUNT; i++)
    {
        s_calibration.min[i] = RC_NORMALIZED_MIN;
        s_calibration.mid[i] = RC_NORMALIZED_MID;
        s_calibration.max[i] = RC_NORMALIZED_MAX;
    }

    s_calibration.deadband = RC_NORMALIZED_DEADBAND;
    s_calibration.reserved = 0U;
}

static bool RcCalibration_DataValid(const RcCalibration_t *cal)
{
    if(cal == NULL || cal->deadband > 100U)
        return false;

    for (uint8_t i = 0U; i < RC_CAL_CHANNEL_COUNT; i++)
    {
        if (cal->min[i] >= cal->mid[i] || cal->mid[i] >= cal->max[i])
            return false;

        if((uint16_t)(cal->max[i] - cal->min[i]) < RC_MIN_FULL_SPAN)
            return false;

        if((uint16_t)(cal->mid[i] - cal->min[i] < RC_MIN_HALF_SPAN ||
            (uint16_t)(cal->max[i] - cal->mid[i]) < RC_MIN_HALF_SPAN))
        {
            return false;
        }

        if(cal->max[i] > 2047U)
            return false;
    }

    return true;
}

static bool RcCalibration_EntryGasture(const RCChannelData_t *rc)
{
    return rc->link_ok &&
           rc->channels[RC_CH_ARM_SWITCH] <= RC_ARM_SWITCH_OFF_MAX &&       // SD未按下状态
           rc->channels[RC_CH_MODE_SWITCH] >= RC_SWITCH_ON_THRESHOLD &&     // SA按下状态
           rc->channels[RC_CH_THROTTLE] <= RC_TRIGGER_LOW &&
           rc->channels[RC_CH_ROLL] <= RC_TRIGGER_LOW &&
           rc->channels[RC_CH_PITCH] <= RC_TRIGGER_LOW &&
           rc->channels[RC_CH_YAW] <= RC_TRIGGER_LOW;   // 这里应该是LOW，才对应RC-YAW的左自旋方向——即向左拨动Yaw摇杆
}

static void RcCalibration_ResetCenterAccumulator(void)
{
    s_center_holding = false;
    s_center_start_tick = 0U;
    s_center_count = 0U;
    memset(s_center_sum, 0, sizeof(s_center_sum));

    for (uint8_t i = 0U; i < RC_CAL_CHANNEL_COUNT; i++)
    {
        s_center_min[i] = UINT16_MAX;
        s_center_max[i] = 0U;
    }
}

static void RcCalibration_Cancel(void)
{
    bool was_active = (s_state != RC_CAL_STATE_IDLE);

    s_state = RC_CAL_STATE_IDLE;
    s_entry_holding = false;
    RcCalibration_ResetCenterAccumulator();

    if(was_active)
        RcCalibration_PostEvent(EVT_RC_CALIB_FAILED);
}

static void RcCalibration_BeginCapture(const RCChannelData_t *rc, uint32_t now)
{
    memset(&s_working, 0, sizeof(s_working));

    for (uint8_t i = 0U; i < RC_CAL_CHANNEL_COUNT; i++)
    {
        s_working.min[i] = rc->channels[i];
        s_working.max[i] = rc->channels[i];
        s_working.mid[i] = RC_NORMALIZED_MID;
    }

    s_working.deadband = RC_NORMALIZED_DEADBAND;
    s_state = RC_CAL_STATE_CAPTURE_RANGE;
    s_state_start_tick = now;
    s_entry_holding = false;
    RcCalibration_ResetCenterAccumulator();
    RcCalibration_PostEvent(EVT_RC_CALIB_STARTED);
}

static void RcCalibration_UpdateRanges(const RCChannelData_t *rc)
{
    for (uint8_t i = 0U; i < RC_CAL_CHANNEL_COUNT; i++)
    {
        uint16_t value = rc->channels[i];
        if(value < s_working.min[i])
            s_working.min[i] = value;
        if(value > s_working.max[i])
            s_working.max[i] = value;
    }
}

static bool RcCalibration_RangesValid(void)
{
    for (uint8_t i = 0U; i < RC_CAL_CHANNEL_COUNT; i++)
    {
        if(s_working.max[i] <= s_working.min[i] ||
            (uint16_t)(s_working.max[i] - s_working.min[i]) < RC_MIN_FULL_SPAN)
        {
            return false;
        }
    }

    return true;
}

static bool RcCalibration_CenterPosePlausible(const RCChannelData_t *rc)
{
    if(rc->channels[RC_CH_THROTTLE] >
        (uint16_t)(s_working.min[RC_CH_THROTTLE] + RC_THROTTLE_LOW_MARGIN))
    {
        return false;
    }

    const uint8_t centered_axes[] = {RC_CH_ROLL, RC_CH_PITCH, RC_CH_YAW};

    for (uint8_t i = 0U; i < sizeof(centered_axes); i++)
    {
        uint8_t ch = centered_axes[i];
        uint16_t rough_mid = (uint16_t)(((uint32_t)s_working.min[ch] +
                                         (uint32_t)s_working.max[ch]) / 2U);
        int32_t error = (int32_t)rc->channels[ch] - (int32_t)rough_mid;
        
        if(error < -(int32_t)RC_CENTER_WINDOW_COUNTS || 
            error > (int32_t)RC_CENTER_WINDOW_COUNTS)
        {
            return false;
        }
    }

    return true;
}

static bool RcCalibration_CenterStillStable(const RCChannelData_t *rc)
{
    const uint8_t centered_axes[] = {RC_CH_ROLL, RC_CH_PITCH, RC_CH_YAW};

    for (uint8_t i = 0U; i < sizeof(centered_axes); i++)
    {
        uint8_t ch = centered_axes[i];
        uint16_t value = rc->channels[ch];

        if(value < s_center_min[ch])
            s_center_min[ch] = value;
        if(value > s_center_max[ch])
            s_center_max[ch] = value;

        if((uint16_t)(s_center_max[ch] - s_center_min[ch]) > RC_CENTER_STABILITY_COUNTS)
        {
            return false;
        }

        s_center_sum[ch] += value;
    }

    s_center_count++;
    return true;
}

static void RcCalibration_TryFinish(void)
{
    if(s_center_count == 0U)
    {
        RcCalibration_Cancel();
        return;
    }

    s_working.mid[RC_CH_ROLL] = (uint16_t)(s_center_sum[RC_CH_ROLL] / s_center_count);
    s_working.mid[RC_CH_PITCH] = (uint16_t)(s_center_sum[RC_CH_PITCH] / s_center_count);
    s_working.mid[RC_CH_YAW] = (uint16_t)(s_center_sum[RC_CH_YAW] / s_center_count);
    s_working.mid[RC_CH_THROTTLE] = (uint16_t)((((uint32_t)s_working.min[RC_CH_THROTTLE] +
                                                 (uint32_t)s_working.max[RC_CH_THROTTLE]) / 2U));

    s_working.deadband = RC_NORMALIZED_DEADBAND;
    s_working.reserved = 0U;

    if(!RcCalibration_DataValid(&s_working))
    {
        RcCalibration_Cancel();
        return;
    }

    ConfigFlashStatus_t result = BSP_ConfigFlash_Append(
        RC_CONFIG_RECORD_TYPE,
        RC_CONFIG_VERSION,
        &s_working,
        sizeof(s_working),
        NULL);
    
    if(result != CONFIG_FLASH_OK)
    {
        RcCalibration_Cancel();
        return;
    }

    s_calibration = s_working;
    s_ready = true;
    s_state = RC_CAL_STATE_IDLE;
    RcCalibration_ResetCenterAccumulator();
    RcCalibration_PostEvent(EVT_RC_CALIB_SUCCESS);
}

static uint16_t RcCalibration_MapAxis(uint16_t raw, uint16_t raw_min, uint16_t raw_mid, uint16_t raw_max)
{
    if(raw <= raw_min)
        return RC_NORMALIZED_MIN;
    if(raw >= raw_max)
        return RC_NORMALIZED_MAX;

    if(raw <= raw_mid)
    {
        uint32_t numerator = (uint32_t)(raw - raw_min) * (RC_NORMALIZED_MID - RC_NORMALIZED_MIN);
        uint32_t denominator = (uint32_t)(raw_mid - raw_min);
        return (uint16_t)(RC_NORMALIZED_MIN + numerator / denominator);
    }

    uint32_t numerator = (uint32_t)(raw - raw_mid) * (RC_NORMALIZED_MAX - RC_NORMALIZED_MID);
    uint32_t denominator = (uint32_t)(raw_max - raw_mid);
    return (uint16_t)(RC_NORMALIZED_MID + numerator / denominator);
}

void RcCalibration_Init(void)
{
    RcCalibration_LoadDefaults();
    s_ready = false;
    s_state = RC_CAL_STATE_IDLE;
    s_entry_holding = false;
    RcCalibration_ResetCenterAccumulator();

    RcCalibration_t stored;
    ConfigFlashStatus_t result = BSP_ConfigFlash_LoadLatest(
        RC_CONFIG_RECORD_TYPE,
        RC_CONFIG_VERSION,
        &stored,
        sizeof(stored),
        NULL);

    if(result == CONFIG_FLASH_OK && RcCalibration_DataValid(&stored))
    {
        s_calibration = stored;
        s_ready = true;
    }
}

void RcCalibration_Update(const RCChannelData_t *raw_rc)
{
    if(raw_rc == NULL)
        return;

    if(MagCalibration_IsActive())
    {
        /* Mag校准翻转机体期间不允许另一状态机并行运行 */
        RcCalibration_Cancel();
        return;
    }

    uint32_t now = osKernelGetTickCount();
    
    if(g_arm_state != ARM_STATE_DISARMED || !raw_rc->link_ok ||
        raw_rc->channels[RC_CH_ARM_SWITCH] > RC_ARM_SWITCH_OFF_MAX)
    {
        RcCalibration_Cancel();
        return;
    }

    switch (s_state)
    {
    case RC_CAL_STATE_IDLE:
        if(RcCalibration_EntryGasture(raw_rc))
        {
            if(!s_entry_holding)
            {
                s_entry_holding = true;
                s_entry_start_tick = now;
            }
            else if((now - s_entry_start_tick) >= RC_TRIGGER_HOLD_MS)
            {
                RcCalibration_BeginCapture(raw_rc, now);
            }
        }
        else
        {
            s_entry_holding = false;
        }
        break;
    
    case RC_CAL_STATE_CAPTURE_RANGE:
        RcCalibration_UpdateRanges(raw_rc);

        if((now - s_state_start_tick) >= RC_CAPTURE_TIMEOUT_MS)
        {
            RcCalibration_Cancel();
            break;
        }

        if((now - s_state_start_tick) >= RC_CAPTURE_MIN_MS &&
            raw_rc->channels[RC_CH_MODE_SWITCH] < RC_SWITCH_ON_THRESHOLD)
        {
            if(!RcCalibration_RangesValid())
            {
                RcCalibration_Cancel();
                return;
            }

            s_state = RC_CAL_STATE_CAPTURE_CENTER;
            s_state_start_tick = now;
            RcCalibration_ResetCenterAccumulator();
        }
        break;

    case RC_CAL_STATE_CAPTURE_CENTER:
        if((now - s_state_start_tick) >= RC_CENTER_TIMEOUT_MS)
        {
            RcCalibration_Cancel();
            break;
        }

        if(!RcCalibration_CenterPosePlausible(raw_rc))
        {
            RcCalibration_ResetCenterAccumulator();
            break;
        }

        if(!s_center_holding)
        {
            s_center_holding = true;
            s_center_start_tick = now;
        }

        if(!RcCalibration_CenterStillStable(raw_rc))
        {
            RcCalibration_ResetCenterAccumulator();
            break;
        }

        if((now - s_center_start_tick) >= RC_CENTER_HOLD_MS)
            RcCalibration_TryFinish();
        break;

    default:
        RcCalibration_Cancel();
        break;
    }
}

void RcCalibration_Apply(RCChannelData_t *rc)
{
    if(rc == NULL)
        return;

    for (uint8_t i = 0U; i < RC_CAL_CHANNEL_COUNT; i++)
    {
        rc->channels[i] = RcCalibration_MapAxis(rc->channels[i],
                                                s_calibration.min[i],
                                                s_calibration.mid[i],
                                                s_calibration.max[i]);
    }

    /* Roll/Pitch/Yaw的中心抖动统一压到精确中心值。
     * Throttle不设中心死区，避免破坏油门的连续性。 */
    const uint8_t centered_axes[] = {RC_CH_ROLL, RC_CH_PITCH, RC_CH_YAW};
    for (uint8_t i = 0U; i < (uint8_t)sizeof(centered_axes); i++)
    {
        uint8_t ch = centered_axes[i];
        int32_t center_error = (int32_t)rc->channels[ch] - (int32_t)RC_NORMALIZED_MID;

        if(center_error >= -(int32_t)s_calibration.deadband &&
            center_error <= (int32_t)s_calibration.deadband)
        {
            rc->channels[ch] = RC_NORMALIZED_MID;
        }
    }
}

bool RcCalibration_IsReady(void)
{
    return s_ready;
}

bool RcCalibration_IsActive(void)
{
    return s_state != RC_CAL_STATE_IDLE;
}

uint16_t RcCalibration_GetNormalizedDeadband(void)
{
    return s_calibration.deadband;
}
