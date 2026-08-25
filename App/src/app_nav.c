/**
 * @file    app_nav.c
 * @brief   GPS Navigation 状态构建与 Mag 数据采集任务实现。
 */

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

#define TASK_NAV_PERIOD_MS 20U          // Navigation Task 周期，ms：20ms 对应标称 50 Hz。

#define QMC_RECOVERY_FAILURE_COUNT 3U   // 连续 QMC 读取失败达到该次数后尝试恢复 I2C Bus 并重新初始化 QMC。

/* GPS 基础换算参数。 */
#define GPS_KNOT_TO_MPS 0.514444f
#define GPS_DEG_TO_RAD 0.0174532925f

/*
 * RMC Velocity Sample 的基础有效性门限。
 * 这组条件用于判断单个 GPS Velocity Observation 本身是否可用。
 */
#define GPS_VELOCITY_MIN_SATELLITES 8U
#define GPS_VELOCITY_MAX_HDOP 2.5f
#define GPS_VELOCITY_MAX_RMC_PERIOD_MS 400U
#define GPS_VELOCITY_MAX_RMC_AGE_MS 600U
#define GPS_VELOCITY_MAX_GGA_AGE_MS 1500U

/*
 * 相对宽松的单 Sample 新鲜度门限。
 * Sample Valid 与 Control Ready 分离：
 * 数据可以被认为“有效”，但未必足够新鲜，稳定到允许闭环控制。
 */
#define GPS_SAMPLE_MAX_RMC_AGE_MS 1500U
#define GPS_SAMPLE_MAX_GGA_AGE_MS 2500U

#define GPS_VELOCITY_MAX_SPEED_MPS 30.0f

/* Position Hold 对 GPS Fix Quality 使用更严格门限。 */
#define GPS_POSITION_MIN_SATELLITES 12U
#define GPS_POSITION_MAX_HDOP 2.0f

/* WGS-84 赤道半径及 E7 经纬度到弧度的换算系数。 */
#define GPS_EARTH_RADIUS_M 6378137.0f
#define GPS_DEG_E7_TO_RAD 1.745329252e-9f

/*
 * GPS Position-window Velocity 配置。
 * 
 * 使用最近约 0.4~0.7 s 的位置样本做线性回归，
 * 作为独立于 RMC Course 计算方法的 Position-derived Velocity Observation。
 */
#define GPS_POSITION_VELOCITY_HISTORY_SIZE 8U
#define GPS_POSITION_VELOCITY_MIN_SAMPLES 3U
#define GPS_POSITION_VELOCITY_MIN_WINDOW_MS 400U
#define GPS_POSITION_VELOCITY_MAX_WINDOW_MS 700U
#define GPS_POSITION_VELOCITY_MAX_SPEED_MPS 3.0f

/**
 * @brief   GPS Position-window 中的单个位置样本。
 */
typedef struct
{
    int32_t latitude_e7;    /**< 纬度，deg × 1e7。 */
    int32_t longitude_e7;   /**< 经度，deg × 1e7。 */
    uint32_t tick_ms;       /**< 对应 GPS Sample 更新时间，ms。 */
} NavGpsPositionSample_t;

static uint8_t s_home_valid = 0;        // Home 是否已经成功建立；仅由 Navigation Task 修改。
static uint8_t s_qmc_failure_streak;    // QMC 连续读取失败计数。

/* GPS Position-window Velocity 历史及当前结果。 */
static NavGpsPositionSample_t 
    s_gps_position_history[GPS_POSITION_VELOCITY_HISTORY_SIZE];

static uint8_t s_gps_position_history_count;
static uint32_t s_gps_position_last_sequence;

static float s_gps_position_velocity_n_mps;
static float s_gps_position_velocity_e_mps;
static uint8_t s_gps_position_velocity_valid;

/*
 * 清空 Position-window Velocity 历史。
 * 
 * GPS Position Quality 一旦失效，旧窗口整体作废，
 * 防止恢复后将失效前后的 Postition Sample 拼接成错误速度。
 */
static void Nav_ResetGpsPositionVelocityHistory(void)
{
    s_gps_position_history_count = 0U;

    s_gps_position_velocity_n_mps = 0.0f;
    s_gps_position_velocity_e_mps = 0.0f;
    s_gps_position_velocity_valid = 0U;
}

/* 初始化 GPS Position-derived Velocity 状态。 */
static void Nav_InitGpsPositionVelocity(void)
{
    s_gps_position_last_sequence = 0U;
    Nav_ResetGpsPositionVelocityHistory();
}

/*
 * 使用最近约 0.4s~0.7s GPS Position Sample 的线性回归斜率
 * 估算 North/East Velocity。
 * 
 * 只在 RMC Sequence 真正推进时加入一个新样本。
 * 避免 Nav Task 以 50Hz 重复发布同一 GPS Sample 是重复入窗。
 */
static void Nav_UpdateGpsPositionVelocity(
    const GPS_Data_t *gps,
    bool position_sample_valid)
{
    /*
     * Sequence == 0 表示尚未取得优秀奥 RMC Sample；
     * Sequence 未变化表示本轮仍是上一份 GPS 数据。
     */
    if(gps == NULL ||
        gps->rmc_sequence == 0U ||
        gps->rmc_sequence == s_gps_position_last_sequence)
    {
        return;
    }

    s_gps_position_last_sequence = gps->rmc_sequence;

    /*
     * 新 GPS Sample 的 Position Quality 不满足要求时，
     * 旧窗口立即失效，恢复后必须重新积累连续有效样本。
     */
    if(!position_sample_valid)
    {
        Nav_ResetGpsPositionVelocityHistory();
        return;
    }

    NavGpsPositionSample_t sample;

    sample.latitude_e7 = gps->gps_lat;
    sample.longitude_e7 = gps->gps_lon;
    sample.tick_ms = gps->rmc_last_update_ms;

    /*
     * History 未满时直接追加：
     * 满后丢弃最老样本，保持固定长度滑动窗口。
     */
    if(s_gps_position_history_count < 
        GPS_POSITION_VELOCITY_HISTORY_SIZE)
    {
        s_gps_position_history[s_gps_position_history_count++] = sample;
    }
    else
    {
        for (uint8_t i = 1U; i < GPS_POSITION_VELOCITY_HISTORY_SIZE; i++)
            s_gps_position_history[i - 1U] = s_gps_position_history[i];

        s_gps_position_history[GPS_POSITION_VELOCITY_HISTORY_SIZE - 1U] = sample;
    }

    /*
     * 每次有新 GPS Position Sample 时都重新判定当前窗口。
     * 如果新窗口无法形成可信 Velocity，
     * 不继续保留上一窗口已经过时的结果。
     */
    s_gps_position_velocity_n_mps = 0.0f;
    s_gps_position_velocity_e_mps = 0.0f;
    s_gps_position_velocity_valid = 0U;

    if(s_gps_position_history_count < GPS_POSITION_VELOCITY_MIN_SAMPLES)
        return;

    const uint8_t newest_index = s_gps_position_history_count - 1U;

    const NavGpsPositionSample_t *newest =
        &s_gps_position_history[newest_index];

    /*
     * 从历史中由旧到新寻找合适的起点。
     * 
     * 优先选择当前 History 汇总跨度最大的 400~700ms 子窗口，
     * 因而 GPS 为 5 Hz 或 10 Hz 时都不依赖固定 Frame Count。
     */
    uint8_t oldest_index = s_gps_position_history_count;

    for (uint8_t i = 0U; 
        (uint8_t)(s_gps_position_history_count - i) >= GPS_POSITION_VELOCITY_MIN_SAMPLES; 
        i++)
    {
        const uint32_t window_ms = 
            newest->tick_ms - 
            s_gps_position_history[i].tick_ms;

        // 当前起点太老，继续向后缩短窗口。
        if(window_ms > GPS_POSITION_VELOCITY_MAX_WINDOW_MS)
            continue;

        /*
         * 当前起点已经使窗口短于最小时间。
         * 后面的样本只会使窗口更短，因此可以直接结束搜索。
         */
        if(window_ms < GPS_POSITION_VELOCITY_MIN_WINDOW_MS)
            break;

        oldest_index = i;
        break;
    }

    if(oldest_index >= s_gps_position_history_count)
        return;
    
    const NavGpsPositionSample_t *oldest = 
        &s_gps_position_history[oldest_index];

    const uint8_t regression_sample_count =
        s_gps_position_history_count - oldest_index;

    /*
     * 在当前小范围运动内采用局部切平面近似：
     *
     * North ≈ ΔLatitude × Earth Radius
     * East  ≈ ΔLongitude × Earth Radius × cos(latitude)
     *
     * 经度方向每个 E7 对应的实际距离随纬度变化，
     * 因此使用窗口首尾平均纬度计算比例。
     */
    const float reference_lat_rad =
        0.5f * 
        ((float)oldest->latitude_e7 + (float)newest->latitude_e7) * 
        GPS_DEG_E7_TO_RAD;

    const float meter_per_lat_e7 = 
        GPS_EARTH_RADIUS_M * GPS_DEG_E7_TO_RAD;
    const float meter_per_lon_e7 = 
        meter_per_lat_e7 * cosf(reference_lat_rad);

    float sum_t = 0.0f;
    float sum_n = 0.0f;
    float sum_e = 0.0f;

    /*
     * 第一遍计算所有 Sample 的平均时间及平均 N/E Position，
     * 为后续最小二乘线性回归做中心化。
     */
    for (uint8_t i = oldest_index; i < s_gps_position_history_count; i++)
    {
        const NavGpsPositionSample_t *p = &s_gps_position_history[i];

        const float t_s =
            (float)(uint32_t)(p->tick_ms - oldest->tick_ms) * 0.001f;

        const float n_m =
            (float)((int64_t)p->latitude_e7 - 
                    oldest->latitude_e7) * 
            meter_per_lat_e7;

        const float e_m =
            (float)((int64_t)p->longitude_e7 -
                    oldest->longitude_e7) *
            meter_per_lon_e7;

        sum_t += t_s;
        sum_n += n_m;
        sum_e += e_m;
    }

    const float inv_count = 1.0f / (float)regression_sample_count;

    const float mean_t = sum_t * inv_count;
    const float mean_n = sum_n * inv_count;
    const float mean_e = sum_e * inv_count;

    float time_variance = 0.0f;
    float covariance_n = 0.0f;
    float covariance_e = 0.0f;

    /*
     * 对 Position = Position0 + Velocity × Time 做一阶最小二乘拟合。
     *
     * 回归斜率：
     *
     * velocity = Σ[(t-mean_t)(p-mean_p)]
     *            ------------------------
     *                  Σ(t-mean_t)^2
     *
     * 分别对 North 和 East 计算，即得到 N/E Velocity。
     */
    for (uint8_t i = oldest_index; i < s_gps_position_history_count; i++)
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

    /*
     * 时间轴跨度过小时回归斜率可能没有可靠意义，
     * 同时避免除以接近零的数。
     */
    if(time_variance <= 0.0001f)
        return;

    const float velocity_n_mps = covariance_n / time_variance;
    const float velocity_e_mps = covariance_e / time_variance;
    const float speed_mps = sqrtf(velocity_n_mps * velocity_n_mps +
                                 velocity_e_mps * velocity_e_mps);

    /*
     * Position-window Velocity 只服务于低速 Position Hold 辅助判断。
     * 
     * `speed >= 0` 与 上限联合判断同时排除 NaN；
     * 超出合理低速范围的结果直接认为窗口估计不可信。
     */
    if(!(speed_mps >= 0.0f &&
        speed_mps <= GPS_POSITION_VELOCITY_MAX_SPEED_MPS))
    {
        return;
    }

    s_gps_position_velocity_n_mps = velocity_n_mps;
    s_gps_position_velocity_e_mps = velocity_e_mps;
    s_gps_position_velocity_valid = 1U;
}

/*
 * 根据 GPS Driver 当前快照构建一次完整 NavState。
 * 
 * NavState 区分：
 * - Sample Valid：单份观测本身是否合法；
 * - Control Ready：数据质量和新鲜度是否足够进入闭环控制。
 */
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

    /*
     * RMC Ground Speed + Course 转换为 N/E Velocity。
     *
     * N = speed × cos(course)
     * E = speed × sin(course)
     * 
     * NMEA Course 为相对 True North 顺时针角度。
     */
    const float rmc_speed_mps = gps.speed_knots * GPS_KNOT_TO_MPS;
    const float course_rad = gps.course * GPS_DEG_TO_RAD;

    out->rmc_velocity_n_mps = rmc_speed_mps * cosf(course_rad);
    out->rmc_velocity_e_mps = rmc_speed_mps * sinf(course_rad);

    out->rmc_sequence = gps.rmc_sequence;
    out->rmc_last_update_ms = gps.rmc_last_update_ms;
    out->rmc_period_ms = gps.rmc_period_ms;

    const uint32_t now = HAL_GetTick();

    /*
     * 尚未收到对应 NMEA Sentence 时使用 UINT32_MAX 表示无效 Age，
     * 防止启动阶段 timestamp == 0 被误认为新鲜数据。
     */
    const uint32_t rmc_age =
        (gps.rmc_last_update_ms == 0U) 
            ? 0xFFFFFFFFU 
            : (uint32_t)(now - gps.rmc_last_update_ms);
    const uint32_t gga_age =
        (gps.gga_last_update_ms == 0U) 
            ? 0xFFFFFFFFU 
            : (uint32_t)(now - gps.gga_last_update_ms);

    /*
     * NavState 只保留 uint16_t Age，
     * 超过可表示范围时饱和到 65535 ms。
     */
    out->rmc_age_ms = 
        (rmc_age > 65535U) 
            ? 65535 
            : (uint16_t)rmc_age;

    /*
     * RMC Velocity Sample Valid：
     * 判断当前 Ground Speed / Course Observation 本身是否可信。
     * 
     * 这里使用较宽松的 Sample Age；
     * 是否允许闭环控制由后面的 gps_velocity_control_ready
     * 使用哽咽条件再次判断。
     */
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

    /*
     * Position Hold 需要比 Velocity Observation 更严格的 GPS Quality，
     * 包括更多 Satellite，更低 HDOP，更严格 Sentence Age
     * 以及合法的经纬度范围。
     */
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

    /*
     * Positoin Quality 一旦失效，Position-derived Velocity
     * 的历史窗口必须立即失效，恢复后重新建立。
     */
    if(out->gps_position_control_ready == 0U)
        Nav_ResetGpsPositionVelocityHistory();

    Nav_UpdateGpsPositionVelocity(&gps, out->gps_position_control_ready != 0U);

    out->gps_position_velocity_n_mps = s_gps_position_velocity_n_mps;
    out->gps_position_velocity_e_mps = s_gps_position_velocity_e_mps;
    out->gps_position_velocity_valid = (s_gps_position_velocity_valid != 0U) &&
                                       (out->gps_position_control_ready != 0U);

    /*
     * Horizontal Estimator 的主 GPS Velocity Observation
     * 固定使用 RMC Ground Speed + Course。
     * 
     * Position-window Velocity 和 RMC 都来自同一颗 GPS，
     * 并不是两个独立 Sensor。运行时在两种算法之间切换，
     * 会将两种估计方法的系统差异表现成真实 Velocity Step。
     * 
     * 因此 Position-window Velocity 仅保留用于
     * Position Hold 一致性检查，Braking 判断和诊断，
     * 不直接作为 Estimator 的 Velocity Correction Source。
     */
    out->gps_velocity_n_mps = out->rmc_velocity_n_mps;
    out->gps_velocity_e_mps = out->rmc_velocity_e_mps;
    out->gps_velocity_valid = out->rmc_velocity_valid;
    out->gps_velocity_source = NAV_GPS_VELOCITY_SOURCE_RMC;

    /*
     * Velocity Control Ready 在 Sample Valid 基础上进一步要求：
     * RMC 周期稳定 且 RMC/GGA 都足够新鲜。
     */
    out->gps_velocity_control_ready =
        (out->gps_velocity_valid != 0U) &&
        (gps.rmc_period_ms > 0U) &&
        (gps.rmc_period_ms <= GPS_VELOCITY_MAX_RMC_PERIOD_MS) &&
        (rmc_age <= GPS_VELOCITY_MAX_RMC_AGE_MS) &&
        (gga_age <= GPS_VELOCITY_MAX_GGA_AGE_MS);
}

void App_Nav_Task(void *argument)
{
    (void)argument;

    GPS_Init();
    QMC_Init();

    Nav_InitGpsPositionVelocity();

    for (;;)
    {
        /*
         * 从 UART DMA Ring Buffer 消费新字节并解析 NMEA，
         * 随后执行 GPS Driver 的运行器维护。
         */
        GPS_Poll();

        GPS_RuntimeService(g_arm_state == ARM_STATE_DISARMED);

        /*
         * 保存本轮循环开始前的 Home 状态，
         * 用于统一检测后续所有路径产生的 0 -> 1 跳变。
         */
        const uint8_t pre_home_valid = s_home_valid;

        /*
         * 非阻塞处理 Navigation Command。
         * 当前仅支持显示重新请求设置 Home。
         */
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

        /*
         * 启动后持续尝试建立 Home，直到首次成功。
         * 成功后停止自动重试，除非后续受到显式 SET_HOME Command。
         */
        if(!s_home_valid)
        {
            s_home_valid = GPS_SetHome() ? 1 : 0;
        }

        /*
         * Home Valid 同步发布到 System Ready Event Group，
         * 供其他模块快速判断 Navigation 基准是否已经建立。
         */
        if(s_home_valid)
        {
            osEventFlagsSet(SystemReadyEventGroupHandle, SYSREADY_BIT_HOME_VALID);
        }
        else
        {
            osEventFlagsClear(SystemReadyEventGroupHandle, SYSREADY_BIT_HOME_VALID);
        }

        // 仅在 Home 首次由  Invalid -> Valid 时发布一次 GPS Fix 提示。
        if(!pre_home_valid && s_home_valid)
        {
            IndicatorEvent_t evt = EVT_GPS_FIX_ACQUIRED;
            osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0, 0);
        }

        /*
         * 构建最新 NavState，并按 Mailbox 语义发布。
         * 
         * Queue 满时先丢弃旧状态，再写入当前最新状态，
         * 因此消费者只关心最新 Snapshot，不积压历史数据。
         */
        NavState_t nav;
        Nav_BuildNavState(&nav);

        if (osMessageQueueGetSpace(NavStateMailboxHandle) == 0)
        {
            NavState_t discard;
            osMessageQueueGet(NavStateMailboxHandle, &discard, NULL, 0);
        }

        (void)osMessageQueuePut(NavStateMailboxHandle, &nav, 0, 0);

        /*
         * QMC 数据链：
         * 
         * I2C Read
         * -> Raw-to-Gauss / Body-axis Conversion
         * -> Calibration Dateset Logging
         * -> Hard/Soft-Iron Correction
         * -> Mag Mailbox
         */
        MagData_t mag;

        HAL_StatusTypeDef mag_read_status = QMC_ReadData();
        
        if(mag_read_status == HAL_OK)
        {
            s_qmc_failure_streak = 0U;

            /*
             * 只有本轮真实 I2C Read 成功后，
             * 才复制并发布一份新的 Mag Sample。
             * 
             * Overflow Sample 仍发布给 Yaw Estimator，
             * 使其能够记录明确的 Reject Reson；
             * 但不能将 MAG_OK 标记为 Ready。
             */
            QMC_CopyTo(&mag);
            
            /*
             * Mag Calibration Dataset 必须记录尚未执行
             * Hard/Soft-Iron Correction 的原始 NED Vector。
             */
            MagCalibration_LogSample(&mag);
            
            // 对正常运行使用的 Mag Vector 应用 Hard/Soft-Iron 校正。
            MagCalibration_Apply(&mag);

            if(!mag.ovfl)
                osEventFlagsSet(SystemReadyEventGroupHandle, SYSREADY_BIT_MAG_OK);
            else
                osEventFlagsClear(SystemReadyEventGroupHandle, SYSREADY_BIT_MAG_OK);

            /*
             * Mag Mailbox 同样采用“只保留最新值”语义。
             * FlightCtrl 不需要逐帧消费历史 Mag 数据。
             */
            if (osMessageQueueGetSpace(MagDataMailboxHandle) == 0U)
            {
                MagData_t discard;
                (void)osMessageQueueGet(MagDataMailboxHandle, &discard, NULL, 0U);
            }

            (void)osMessageQueuePut(MagDataMailboxHandle, &mag, 0U, 0U);
        }
        else
        {
            /*
             * 本轮 QMC Read 失败时不能重新发布 Driver Cache 中上一帧数据，
             * 否则 FlightCtrl 会把旧 Sample 错误识别为持续到达的新观测。
             */
            osEventFlagsClear(SystemReadyEventGroupHandle, SYSREADY_BIT_MAG_OK);

            if(s_qmc_failure_streak < QMC_RECOVERY_FAILURE_COUNT)
                s_qmc_failure_streak++;

            /*
             * 连续多次读取失败后尝试主动恢复 I2C Bus，
             * 再重新配置 QMC 的 Range，ODR 和 Continous Mode。
             */
            if(s_qmc_failure_streak >= QMC_RECOVERY_FAILURE_COUNT)
            {
                s_qmc_failure_streak = 0U;

                /*
                 * 即使 Bus Recovery 和 QMC Init 成功，
                 * 本轮仍然不重新置 MAG_OK。
                 * 
                 * 必须等下一周期真正读到一帧新 QMC 数据，
                 * 才重新认为 Mag Date Path 健康。
                 */
                if(IIC_RecoverBus() == HAL_OK)
                {
                    (void)QMC_Init();
                }
                
            }
        }

        osDelay(TASK_NAV_PERIOD_MS);
    }
}
