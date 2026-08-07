#include "app_nav.h"
#include "app_mag_calibration.h"
#include "bsp_gps.h"
#include "bsp_qmc5883.h"
#include "app_shared_types.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"

extern osMessageQueueId_t NavStateMailboxHandle;
extern osMessageQueueId_t MagDataMailboxHandle;
extern osMessageQueueId_t NavCommandQueueHandle;
extern osEventFlagsId_t SystemReadyEventGroupHandle;
extern osMessageQueueId_t IndicatorEventQueueHandle;

#define TASK_NAV_PERIOD_MS 20U
#define QMC_RECOVERY_FAILURE_COUNT 3U

static uint8_t s_home_valid = 0;        // 缓存值，只在处理NAV_CMD_SET_HOME时更新
static uint8_t s_qmc_failure_streak;    // qmc读取失败缓存值

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
}

void App_Nav_Task(void *argument)
{
    GPS_Init();
    QMC_Init();

    for (;;)
    {
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

        GPS_Poll();     // 消费DMA缓冲区，解析NMEA

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
