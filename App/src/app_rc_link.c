#include "app_rc_link.h"
#include "app_shared_types.h"
#include "bsp_elrs.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"

extern osMessageQueueId_t RCChannelMailboxHandle;
extern osMessageQueueId_t SystemReadyEventGroupHandle;

/**
 * EdgeTx Packet Rate = 150Hz, 周期≈6.63ms；
 * DMA_Buf_Size = 128 B；
 * 因此设置轮询周期为4ms相当于留了三倍裕量，来防止丢数据
 */
#define TASK_RC_LINK_PERIOD_MS 4U

void App_RcLink_Task(void *argument)
{
    (void)argument;

    ELRS_Init();

    for (;;)
    {
        ELRS_Poll();

        RCChannelData_t rc;
        ELRS_CopyTo(&rc);

        if(osMessageQueueGetSpace(RCChannelMailboxHandle) == 0)
        {
            RCChannelData_t discard;
            osMessageQueueGet(RCChannelMailboxHandle, &discard, NULL, 0);
        }
        osMessageQueuePut(RCChannelMailboxHandle, &rc, 0, 0);

        // 检测RC_LINK_OK_BIT
        if(rc.link_ok)
            osEventFlagsSet(SystemReadyEventGroupHandle, SYSREADY_BIT_RC_LINK_OK);
        else
            osEventFlagsClear(SystemReadyEventGroupHandle, SYSREADY_BIT_RC_LINK_OK);

        osDelay(TASK_RC_LINK_PERIOD_MS);
    }
}
