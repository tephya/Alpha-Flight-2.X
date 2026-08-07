#include "app_mag_calibration.h"
#include "app_arm.h"
#include "app_blackbox.h"
#include "app_imu_calibration.h"
#include "app_level_trim.h"
#include "app_rc_calibration.h"
#include "app_shared_types.h"
#include "bsp_blackbox.h"
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"
#include <stddef.h>

extern osMessageQueueId_t IndicatorEventQueueHandle;

#define MAG_CAL_RC_CHANNEL_SB 7U
#define MAG_CAL_RC_CHANNEL_ARM 4U
#define MAG_CAL_RC_CHANNEL_THROTTLE 2U

#define MAG_CAL_SB_LOW_MAX 400U
#define MAG_CAL_SB_MIDDLE_MIN 700U
#define MAG_CAL_SB_MIDDLE_MAX 1300U
#define MAG_CAL_SB_HIGH_MIN 1600U

#define MAG_CAL_ARM_OFF_MAX 900U
#define MAG_CAL_THROTTLE_LOW_MAX 180U
#define MAG_CAL_TRIGGER_HOLD_MS 3000U

#define MAG_CAL_MIN_SAMPLE_COUNT 1500U  // 50Hz下约为30s
#define MAG_CAL_MAX_SAMPLE_COUNT 8000U  // 50Hz下约为160s

typedef struct 
{
    float bias[3];

    /* 对称Soft-Iron矩阵的存储顺序：
     * A00, A01, A02, A11, A12, A22 */
    float soft_matrix_sym[6];

    /* 室外标定环境中测得的磁场参考模长。
     * 当前保留用于诊断及后续磁场门限判断，
     * 本身不参与Mag向量校正计算 */
    float field_reference_gauss;
} MagCalibrationConfig_t;

static const MagCalibrationConfig_t s_mag_calibration =
{
    .bias = {
        0.777633692f,
        -1.131458540f,
        0.921689385f,
    },
    .soft_matrix_sym = {
        1.012112088f,
        0.024130066f,
        0.006713267f,
        0.985058770f,
        -0.005159549f,
        1.007059819f,
    },
    .field_reference_gauss = 0.458847875f,
};

typedef enum {
    MAG_CAL_STATE_IDLE = 0,
    MAG_CAL_STATE_WAIT_FILE_OPEN,
    MAG_CAL_STATE_CAPTURING,
    MAG_CAL_STATE_WAIT_FILE_CLOSE,
} MagCalibrationState_t;

static MagCalibrationState_t s_state = MAG_CAL_STATE_IDLE;
static bool s_trigger_holding;
static bool s_trigger_latched;
static bool s_abort_when_open;
static bool s_finish_candidate_success;
static uint32_t s_trigger_start_tick;

/* 由FlightCtrl写、Nav读；Cortex-M4对齐的32-bit读写是原子的 */
static volatile bool s_capture_enabled;
static volatile uint32_t s_sample_count;

static void MagCalibration_PostEvent(IndicatorEvent_t evt)
{
    (void)osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0U, 0U);
}

static bool MagCalibration_SbIsLow(uint16_t value)
{
    return value <= MAG_CAL_SB_LOW_MAX;
}

static bool MagCalibration_SbIsMiddle(uint16_t value)
{
    return value <= MAG_CAL_SB_MIDDLE_MAX &&
           value >= MAG_CAL_SB_MIDDLE_MIN;
}

static bool MagCalibration_SbIsHigh(uint16_t value)
{
    return value >= MAG_CAL_SB_HIGH_MIN;
}

static void MagCalibration_RequestFinish(bool success)
{
    /* 必须先停止Nav生产数据，再通知BlackBox Task flush并关闭文件 */
    s_capture_enabled = false;
    s_finish_candidate_success = success &&
                                 s_sample_count >= MAG_CAL_MIN_SAMPLE_COUNT &&
                                 BB_GetErrorFlags() == 0U;
    s_state = MAG_CAL_STATE_WAIT_FILE_CLOSE;
    BB_RequestClose();
}

void MagCalibration_Init(void)
{
    s_state = MAG_CAL_STATE_IDLE;
    s_trigger_holding = false;
    s_trigger_latched = false;
    s_abort_when_open = false;
    s_finish_candidate_success = false;
    s_trigger_start_tick = 0U;
    s_capture_enabled = false;
    s_sample_count = 0U;
}

void MagCalibration_HandleRc(const RCChannelData_t *rc)
{
    if(rc == NULL)
        return;

    const uint16_t sb = rc->channels[MAG_CAL_RC_CHANNEL_SB];
    const bool sb_low = MagCalibration_SbIsLow(sb);
    const bool sb_middle = MagCalibration_SbIsMiddle(sb);
    const bool sb_high = MagCalibration_SbIsHigh(sb);

    if(s_state == MAG_CAL_STATE_WAIT_FILE_OPEN)
    {
        /* 文件打开由低优先级Blackbox Task完成；等待期间仍然锁住Arm */
        if(!rc->link_ok || g_arm_state != ARM_STATE_DISARMED ||
            sb_low || sb_middle)
        {
            s_abort_when_open = true;
        }

        if(App_Blackbox_IsFileOpen())
        {
            if(s_abort_when_open)
            {
                MagCalibration_RequestFinish(false);
            }
            else
            {
                s_capture_enabled = true;
                s_state = MAG_CAL_STATE_CAPTURING;
                MagCalibration_PostEvent(EVT_MAG_CAL_CAPTURE_STARTED);
            }
        }
        return;
    }

    if(s_state == MAG_CAL_STATE_CAPTURING)
    {
        if(!rc->link_ok || g_arm_state != ARM_STATE_DISARMED || sb_low)
        {
            MagCalibration_RequestFinish(false);
        }
        else if(sb_middle)
        {
            MagCalibration_RequestFinish(true);
        }
        else if(s_sample_count >= MAG_CAL_MAX_SAMPLE_COUNT)
        {
            MagCalibration_RequestFinish(true);
        }
        else if(BB_GetErrorFlags() != 0U)
        {
            MagCalibration_RequestFinish(false);
        }
        return;
    }

    if(s_state == MAG_CAL_STATE_WAIT_FILE_CLOSE)
    {
        if(!App_Blackbox_IsFileOpen())
        {
            const bool success = s_finish_candidate_success &&
                                 App_Blackbox_LastCloseSucceeded();

            MagCalibration_PostEvent(success ?
                EVT_MAG_CAL_CAPTURE_DONE : EVT_MAG_CAL_CAPTURE_FAILED);

            s_state = MAG_CAL_STATE_IDLE;
            s_abort_when_open = false;
            s_finish_candidate_success = false;
            s_sample_count = 0U;
            /* s_trigger_latched保持true，必须回到SB中档后才能再次触发 */
        }
        return;
    }

    /* 以下仅处理IDLE状态，中档是唯一的重新待命位置 */
    if(sb_middle)
    {
        s_trigger_holding = false;
        s_trigger_latched = false;
        return;
    }

    if(!sb_high || s_trigger_latched)
    {
        s_trigger_holding = false;
        return;
    }

    const bool eligible =
        rc->link_ok &&
        g_arm_state == ARM_STATE_DISARMED &&
        rc->channels[MAG_CAL_RC_CHANNEL_ARM] <= MAG_CAL_ARM_OFF_MAX &&
        rc->channels[MAG_CAL_RC_CHANNEL_THROTTLE] <= MAG_CAL_THROTTLE_LOW_MAX &&
        ImuCalibration_IsReady() &&
        !RcCalibration_IsActive() &&
        !LevelTrim_IsActive() &&
        !App_Blackbox_IsFileOpen();
    
    if(!eligible)
    {
        s_trigger_holding = false;
        return;
    }

    const uint32_t now = osKernelGetTickCount();

    if(!s_trigger_holding)
    {
        s_trigger_holding = true;
        s_trigger_start_tick = now;
        return;
    }

    if((now - s_trigger_start_tick) >= MAG_CAL_TRIGGER_HOLD_MS)
    {
        s_trigger_holding = false;
        s_trigger_latched = true;
        s_abort_when_open = false;
        s_finish_candidate_success = false;
        s_sample_count = 0U;
        s_capture_enabled = false;
        s_state = MAG_CAL_STATE_WAIT_FILE_OPEN;
        BB_RequestNewFile();
    }
}

void MagCalibration_LogSample(const MagData_t *mag)
{
    if(!s_capture_enabled || mag == NULL || mag->ovfl)
        return;

    if(BB_LogMagCalibration(DWT->CYCCNT,
                            mag->MX,
                            mag->MY,
                            mag->MZ) == 0)
    {
		s_sample_count++;
    }
}

void MagCalibration_Apply(MagData_t *mag)
{
    if(mag == NULL || mag->ovfl)
        return;

    /* 先消除固定的Hard-Iron偏置。
     * 因为本函数会原地修改输入结构体，所以必须先保存三个中间量 */
    const float x = mag->MX - s_mag_calibration.bias[0];
    const float y = mag->MY - s_mag_calibration.bias[1];
    const float z = mag->MZ - s_mag_calibration.bias[2];

    const float *a = s_mag_calibration.soft_matrix_sym;

    /* corrected = A * (raw_ned - bias)
     * A为对称矩阵：
     * [ A00 A01 A02 ]
     * [ A01 A11 A12 ]
     * [ A02 A12 A22] */
    mag->MX = a[0] * x + a[1] * y + a[2] * z;
    mag->MY = a[1] * x + a[3] * y + a[4] * z;
    mag->MZ = a[2] * x + a[4] * y + a[5] * z;
}

bool MagCalibration_IsActive(void)
{
    return s_trigger_holding || s_state != MAG_CAL_STATE_IDLE;
}
