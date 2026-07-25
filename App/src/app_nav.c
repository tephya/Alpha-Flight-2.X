#include "app_nav.h"
#include "bsp_gps.h"
#include "bsp_qmc5883.h"
#include "app_shared_types.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"

extern osMessageQueueId_t NavStateMailboxHandle;
extern osMessageQueueId_t MagDataMailboxHandle;
extern osMessageQueueId_t NavCommandQueueHandle;

#define TASK_NAV_PERIOD_MS 20

static uint8_t s_home_valid = 0;        // 缓存值，只在处理NAV_CMD_SET_HOME时更新

static void Nav_BuildNavState(NavState_t *out)
{
    GPS_Data_t gps;
    GPS_CopyDataTo(&gps);

    out->gps_fix_type = gps.gps_fix_type;
    out->gps_satellites = gps.gps_satellites;
    out->valid = gps.valid;
    out->gps_lat = (int32_t)(gps.gps_lat * 1e7f);
    out->gps_lon = (int32_t)(gps.gps_lon * 1e7f);
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
        QMC_ReadData(); // I2C1读磁力计，内部完成Raw2Gauss
        QMC_CopyTo(&mag);

        if (osMessageQueueGetSpace(MagDataMailboxHandle) == 0)
        {
            MagData_t discard;
            osMessageQueueGet(MagDataMailboxHandle, &discard, NULL, 0);
        }
        osMessageQueuePut(MagDataMailboxHandle, &mag, 0, 0);

        osDelay(TASK_NAV_PERIOD_MS);
    }
}
