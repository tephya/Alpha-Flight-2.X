#include "app_nav.h"
#include "bsp_gps.h"
#include "bsp_qmc5883.h"
#include "app_shared_types.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"

extern osMessageQueueId_t NavStateMailboxHandle;
extern osMessageQueueId_t MagDataMailboxHandle;

#define TASK_NAV_PERIOD_MS 20

static void Nav_BuildNavState(NavState_t *out)
{
    GPS_Data_t gps;
    GPS_Data_CopyTo(&gps);

    out->gps_fix_type = gps.gps_fix_type;
    out->gps_satellites = gps.gps_satellites;
    out->valid = gps.valid;
    out->gps_lat = (int32_t)(gps.gps_lat * 1e7f);
    out->gps_lon = (int32_t)(gps.gps_lon * 1e7f);
    out->gps_altitude_m = gps.gps_altitude_m;
    out->gps_hdop = gps.gps_hdop;
    out->speed_knots = gps.speed_knots;
    out->course = gps.course;
}

void App_Nav_Task(void *argument)
{
    GPS_Init();
    QMC_Init();

    for (;;)
    {
        GPS_Poll();     // 消费DMA缓冲区，解析NMEA
        QMC_ReadData(); // I2C1读磁力计，内部完成Raw2Gauss

        NavState_t nav;
        Nav_BuildNavState(&nav);
        if (osMessageQueueGetSpace(NavStateMailboxHandle) == 0)
        {
            NavState_t discard;
            osMessageQueueGet(NavStateMailboxHandle, &discard, NULL, 0);
        }
        osMessageQueuePut(NavStateMailboxHandle, &nav, 0, 0);

        MAG_Data_t full_mag;
        QMC_CopyTo(&full_mag);

        MagData_t mag = {
            .MX = full_mag.MX,
            .MY = full_mag.MY,
            .MZ = full_mag.MZ};

        if (osMessageQueueGetSpace(MagDataMailboxHandle) == 0)
        {
            MagData_t discard;
            osMessageQueueGet(MagDataMailboxHandle, &discard, NULL, 0);
        }
        osMessageQueuePut(MagDataMailboxHandle, &mag, 0, 0);

        osDelay(TASK_NAV_PERIOD_MS);
    }
}
