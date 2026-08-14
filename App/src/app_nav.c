#include "app_nav.h"
#include "app_mag_calibration.h"
#include "bsp_gps.h"
#include "bsp_qmc5883.h"
#include "app_shared_types.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <math.h>
#include <stdbool.h>

extern osMessageQueueId_t NavStateMailboxHandle;
extern osMessageQueueId_t MagDataMailboxHandle;
extern osMessageQueueId_t NavCommandQueueHandle;
extern osEventFlagsId_t SystemReadyEventGroupHandle;
extern osMessageQueueId_t IndicatorEventQueueHandle;

#define TASK_NAV_PERIOD_MS 20U
#define QMC_RECOVERY_FAILURE_COUNT 3U

#define GPS_KNOT_TO_MPS 0.514444f
#define GPS_DEG_TO_RAD 0.0174532925f
#define GPS_VELOCITY_MIN_SATELLITES 6U
#define GPS_VELOCITY_MAX_HDOP 2.5f
#define GPS_VELOCITY_MAX_RMC_PERIOD_MS 400U
#define GPS_VELOCITY_MAX_RMC_AGE_MS 600U
#define GPS_VELOCITY_MAX_GGA_AGE_MS 1500U
#define GPS_SAMPLE_MAX_RMC_AGE_MS 1500U
#define GPS_SAMPLE_MAX_GGA_AGE_MS 2500U
#define GPS_VELOCITY_MAX_SPEED_MPS 30.0f
#define GPS_POSITION_MIN_SATELLITES 9U
#define GPS_POSITION_MAX_HDOP 2.0f

#define GPS_EARTH_RADIUS_M 6378137.0f
#define GPS_DEG_E7_TO_RAD 1.745329252e-9f

#define GPS_POSITION_VELOCITY_HISTORY_SIZE 4U
#define GPS_POSITION_VELOCITY_MIN_SAMPLES 4U
#define GPS_POSITION_VELOCITY_MIN_WINDOW_MS 450U
#define GPS_POSITION_VELOCITY_MAX_WINDOW_MS 900U
#define GPS_POSITION_VELOCITY_MAX_SPEED_MPS 3.0f

typedef struct
{
    int32_t latitude_e7;
    int32_t longitude_e7;
    uint32_t tick_ms;
} NavGpsPositionSample_t;

static uint8_t s_home_valid = 0;        // 缓存值，只在处理NAV_CMD_SET_HOME时更新
static uint8_t s_qmc_failure_streak;    // qmc读取失败缓存值
static NavGpsPositionSample_t s_gps_position_history[GPS_POSITION_VELOCITY_HISTORY_SIZE];
static uint8_t s_gps_position_history_count;
static uint32_t s_gps_position_last_sequence;
static float s_gps_position_velocity_n_mps;
static float s_gps_position_velocity_e_mps;
static uint8_t s_gps_position_velocity_valid;

static void Nav_ResetGpsPositionVelocityHistory(void)
{
    s_gps_position_history_count = 0U;
    s_gps_position_velocity_n_mps = 0.0f;
    s_gps_position_velocity_e_mps = 0.0f;
    s_gps_position_velocity_valid = 0U;
}

static void Nav_InitGpsPositionVelocity(void)
{
    s_gps_position_last_sequence = 0U;
    Nav_ResetGpsPositionVelocityHistory();
}

/**
 * @brief   使用最近约1秒的GPS Position样本的线性回归斜率估算N/E速度。
 * @note    只在收到新的RMC sequence时更新；重复发布的NavState不会重复入窗。
 */
static void Nav_UpdateGpsPositionVelocity(const GPS_Data_t *gps,
                                            bool position_sample_valid)
{
    if(gps == NULL ||
        gps->rmc_sequence == 0U ||
        gps->rmc_sequence == s_gps_position_last_sequence)
    {
        return;
    }

    s_gps_position_last_sequence = gps->rmc_sequence;

    if(!position_sample_valid)
    {
        Nav_ResetGpsPositionVelocityHistory();
        return;
    }

    NavGpsPositionSample_t sample;
    sample.latitude_e7 = gps->gps_lat;
    sample.longitude_e7 = gps->gps_lon;
    sample.tick_ms = gps->rmc_last_update_ms;

    if(s_gps_position_history_count < GPS_POSITION_VELOCITY_HISTORY_SIZE)
    {
        s_gps_position_history[s_gps_position_history_count++] = sample;
    }
    else
    {
        for (uint8_t i = 1U; i < GPS_POSITION_VELOCITY_HISTORY_SIZE; i++)
            s_gps_position_history[i - 1U] = s_gps_position_history[i];

        s_gps_position_history[GPS_POSITION_VELOCITY_HISTORY_SIZE - 1U] = sample;
    }

    /* 当前新样本若无法形成可信窗口，不保留上一窗口的旧Velocity数值 */
    s_gps_position_velocity_n_mps = 0.0f;
    s_gps_position_velocity_e_mps = 0.0f;
    s_gps_position_velocity_valid = 0U;

    if(s_gps_position_history_count < GPS_POSITION_VELOCITY_MIN_SAMPLES)
        return;

    const NavGpsPositionSample_t *oldest = &s_gps_position_history[0];
    const NavGpsPositionSample_t *newest =
        &s_gps_position_history[s_gps_position_history_count - 1U];

    const uint32_t window_ms = newest->tick_ms - oldest->tick_ms;
    if(window_ms < GPS_POSITION_VELOCITY_MIN_WINDOW_MS ||
        window_ms > GPS_POSITION_VELOCITY_MAX_WINDOW_MS)
    {
        return;
    }

    const float reference_lat_rad =
        0.5f * ((float)oldest->latitude_e7 + (float)newest->latitude_e7) * GPS_DEG_E7_TO_RAD;
    const float meter_per_lat_e7 = GPS_EARTH_RADIUS_M * GPS_DEG_E7_TO_RAD;
    const float meter_per_lon_e7 = meter_per_lat_e7 * cosf(reference_lat_rad);

    float sum_t = 0.0f;
    float sum_n = 0.0f;
    float sum_e = 0.0f;

    for (uint8_t i = 0U; i < s_gps_position_history_count; i++)
    {
        const NavGpsPositionSample_t *p = &s_gps_position_history[i];
        const float t_s =
            (float)(uint32_t)(p->tick_ms - oldest->tick_ms) * 0.001f;
        const float n_m =
            (float)((int64_t)p->latitude_e7 - oldest->latitude_e7) * meter_per_lat_e7;
        const float e_m =
            (float)((int64_t)p->longitude_e7 - oldest->longitude_e7) * meter_per_lon_e7;

        sum_t += t_s;
        sum_n += n_m;
        sum_e += e_m;
    }

    const float inv_count = 1.0f / (float)s_gps_position_history_count;
    const float mean_t = sum_t * inv_count;
    const float mean_n = sum_n * inv_count;
    const float mean_e = sum_e * inv_count;

    float time_variance = 0.0f;
    float covariance_n = 0.0f;
    float covariance_e = 0.0f;

    for (uint8_t i = 0U; i < s_gps_position_history_count; i++)
    {
        const NavGpsPositionSample_t *p = &s_gps_position_history[i];
        const float t_s =
            (float)(uint32_t)(p->tick_ms - oldest->tick_ms) * 0.001f;
        const float n_m =
            (float)((int64_t)p->latitude_e7 - oldest->latitude_e7) * meter_per_lat_e7;
        const float e_m =
            (float)((int64_t)p->longitude_e7 - oldest->longitude_e7) * meter_per_lon_e7;

        const float centered_t = t_s - mean_t;
        time_variance += centered_t * centered_t;
        covariance_n += centered_t * (n_m - mean_n);
        covariance_e += centered_t * (e_m - mean_e);
    }

    if(time_variance <= 0.0001f)
        return;

    const float velocity_n_mps = covariance_n / time_variance;
    const float velocity_e_mps = covariance_e / time_variance;
    const float speed_mps = sqrtf(velocity_n_mps * velocity_n_mps +
                                 velocity_e_mps * velocity_e_mps);

    if(!(speed_mps >= 0.0f &&
        speed_mps <= GPS_POSITION_VELOCITY_MAX_SPEED_MPS))
    {
        return;
    }

    s_gps_position_velocity_n_mps = velocity_n_mps;
    s_gps_position_velocity_e_mps = velocity_e_mps;
    s_gps_position_velocity_valid = 1U;
}

static void Nav_BuildNavState(NavState_t *out)
{
    GPS_Data_t gps;
    GPS_CopyDataTo(&gps);

    out->gps_fix_type = gps.gps_fix_type;
    out->gps_satellites = gps.gps_satellites;
    out->valid = gps.valid;
    out->gps_lat = gps.gps_lat;
    out->gps_lon = gps.gps_lon;
    out->gps_altitude_m = gps.gps_altitude_m;
    out->gps_hdop = gps.gps_hdop;
    out->speed_knots = gps.speed_knots;
    out->course = gps.course;
    out->home_valid = s_home_valid;

    const float rmc_speed_mps = gps.speed_knots * GPS_KNOT_TO_MPS;
    const float course_rad = gps.course * GPS_DEG_TO_RAD;

    out->rmc_velocity_n_mps = rmc_speed_mps * cosf(course_rad);
    out->rmc_velocity_e_mps = rmc_speed_mps * sinf(course_rad);

    out->rmc_sequence = gps.rmc_sequence;
    out->rmc_last_update_ms = gps.rmc_last_update_ms;
    out->rmc_period_ms = gps.rmc_period_ms;

    const uint32_t now = HAL_GetTick();
    const uint32_t rmc_age =
        (gps.rmc_last_update_ms == 0U) ? 0xFFFFFFFFU : (uint32_t)(now - gps.rmc_last_update_ms);
    const uint32_t gga_age =
        (gps.gga_last_update_ms == 0U) ? 0xFFFFFFFFU : (uint32_t)(now - gps.gga_last_update_ms);

    out->rmc_age_ms = (rmc_age > 65535U) ? 65535 : (uint16_t)rmc_age;

    out->rmc_velocity_valid =
        (gps.valid == 'A') &&
        (gps.gps_fix_type >= 1U) &&
        (gps.gps_satellites >= GPS_VELOCITY_MIN_SATELLITES) &&
        (gps.gps_hdop > 0.0f) &&
        (gps.gps_hdop <= GPS_VELOCITY_MAX_HDOP) &&
        (rmc_age <= GPS_SAMPLE_MAX_RMC_AGE_MS) &&
        (gga_age <= GPS_SAMPLE_MAX_GGA_AGE_MS) &&
        (rmc_speed_mps >= 0.0f) &&
        (rmc_speed_mps <= GPS_VELOCITY_MAX_SPEED_MPS) &&
        (gps.course >= 0.0f) &&
        (gps.course <= 360.0f);

    out->gps_position_control_ready =
        (gps.valid == 'A') &&
        (gps.gps_fix_type >= 1U) &&
        (gps.gps_satellites >= GPS_POSITION_MIN_SATELLITES) &&
        (gps.gps_hdop > 0.0f) &&
        (gps.gps_hdop <= GPS_POSITION_MAX_HDOP) &&
        (gps.rmc_period_ms > 0U) &&
        (gps.rmc_period_ms <= GPS_VELOCITY_MAX_RMC_PERIOD_MS) &&
        (rmc_age <= GPS_VELOCITY_MAX_RMC_AGE_MS) &&
        (gga_age <= GPS_VELOCITY_MAX_GGA_AGE_MS) &&
        (gps.gps_lat >= -900000000) &&
        (gps.gps_lat <= 900000000) &&
        (gps.gps_lon >= -1800000000) &&
        (gps.gps_lon <= 1800000000);

    if(out->gps_position_control_ready == 0U)
        Nav_ResetGpsPositionVelocityHistory();

    Nav_UpdateGpsPositionVelocity(&gps, out->gps_position_control_ready != 0U);

    out->gps_position_velocity_n_mps = s_gps_position_velocity_n_mps;
    out->gps_position_velocity_e_mps = s_gps_position_velocity_e_mps;
    out->gps_position_velocity_valid = (s_gps_position_velocity_valid != 0U) &&
                                       (out->gps_position_control_ready != 0U);

    /*
     * 控制观测固定使用RMC ground speed/course。
     * POSITION_WINDOW与RMC来自同一颗GPS，并不是独立传感器；在二者之间
     * 切换会把估计方法差异变成真实Velocity阶跃。窗口Velocity继续记录，
     * 但不再进入Horizontal Estimator。
     */
    out->gps_velocity_n_mps = out->rmc_velocity_n_mps;
    out->gps_velocity_e_mps = out->rmc_velocity_e_mps;
    out->gps_velocity_valid = out->rmc_velocity_valid;
    out->gps_velocity_source = NAV_GPS_VELOCITY_SOURCE_RMC;

    out->gps_velocity_control_ready =
        (out->gps_velocity_valid != 0U) &&
        (gps.rmc_period_ms > 0U) &&
        (gps.rmc_period_ms <= GPS_VELOCITY_MAX_RMC_PERIOD_MS) &&
        (rmc_age <= GPS_VELOCITY_MAX_RMC_AGE_MS) &&
        (gga_age <= GPS_VELOCITY_MAX_GGA_AGE_MS);
}

void App_Nav_Task(void *argument)
{
    GPS_Init();
    QMC_Init();
    Nav_InitGpsPositionVelocity();

    for (;;)
    {
        GPS_Poll(); // 消费DMA缓冲区，解析NMEA

        uint8_t pre_home_valid = s_home_valid;  // 记录本轮循环开始前的值，两处更新点后统一判断跳变

        NavCommand_t cmd;
        if(osMessageQueueGet(NavCommandQueueHandle, &cmd, NULL, 0) == osOK){
            switch (cmd)
            {
            case NAV_CMD_SET_HOME:
                s_home_valid = GPS_SetHome() ? 1 : 0;
                break;
            
            default:
                break;
            }
        }

        /* 开机后持续自动重试，直到成功为止，维持旧架构“上电即自动搜星”的语义；
         * 一旦成功就不再重试，除非以后有外部命令显式触发 */
        if(!s_home_valid)
        {
            s_home_valid = GPS_SetHome() ? 1 : 0;
        }

        // 检测HOME_OK_BIT
        if(s_home_valid)
            osEventFlagsSet(SystemReadyEventGroupHandle, SYSREADY_BIT_HOME_VALID);
        else
            osEventFlagsClear(SystemReadyEventGroupHandle, SYSREADY_BIT_HOME_VALID);

        // 只在0->1跳变沿发一次
        if(!pre_home_valid && s_home_valid)
        {
            IndicatorEvent_t evt = EVT_GPS_FIX_ACQUIRED;
            osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0, 0);
        }

        NavState_t nav;
        Nav_BuildNavState(&nav);
        if (osMessageQueueGetSpace(NavStateMailboxHandle) == 0)
        {
            NavState_t discard;
            osMessageQueueGet(NavStateMailboxHandle, &discard, NULL, 0);
        }
        osMessageQueuePut(NavStateMailboxHandle, &nav, 0, 0);

        MagData_t mag;
        HAL_StatusTypeDef mag_read_status = QMC_ReadData(); // I2C1读磁力计，内部完成Raw2Gauss
        // 检查读取Mag状态
        if(mag_read_status == HAL_OK)
        {
            s_qmc_failure_streak = 0U;

            /* 只有本轮I2C读取成功，才复制并发布新的Mag数据。
             * OVFL样本仍发布给YawEstimator，由其记录明确的拒绝原因，
             * 但SYSREADY_BIT_MAG_OK必须保持清除。 */
            QMC_CopyTo(&mag);
            // 标定采集需在校正前
            MagCalibration_LogSample(&mag);
            // 校正Mag NED向量
            MagCalibration_Apply(&mag);

            if(!mag.ovfl)
                osEventFlagsSet(SystemReadyEventGroupHandle, SYSREADY_BIT_MAG_OK);
            else
                osEventFlagsClear(SystemReadyEventGroupHandle, SYSREADY_BIT_MAG_OK);

            if (osMessageQueueGetSpace(MagDataMailboxHandle) == 0U)
            {
                MagData_t discard;
                (void)osMessageQueueGet(MagDataMailboxHandle, &discard, NULL, 0U);
            }
            (void)osMessageQueuePut(MagDataMailboxHandle, &mag, 0U, 0U);
        }
        else
        {
            /* 读取失败时不能重新发布mag_data内部缓存的上一帧，
             * 否则FlightCtrl会把旧数据误认为持续到达的新样本 */
            osEventFlagsClear(SystemReadyEventGroupHandle, SYSREADY_BIT_MAG_OK);

            if(s_qmc_failure_streak < QMC_RECOVERY_FAILURE_COUNT)
                s_qmc_failure_streak++;

            if(s_qmc_failure_streak >= QMC_RECOVERY_FAILURE_COUNT)
            {
                s_qmc_failure_streak = 0U;

                /* 先恢复I2C Bus，再重新配置QMC的量程、ODR和Continuous Mode，
                 * 即使恢复成功，本轮也不置MAG_OK，必须等待下一轮真实读取成功。 */
                if(IIC_RecoverBus() == HAL_OK)
                {
                    (void)QMC_Init();
                }
                
            }
        }

        osDelay(TASK_NAV_PERIOD_MS);
    }
}
