#include "app_rc_link.h"
#include "app_shared_types.h"
#include "bsp_elrs.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"

extern osMessageQueueId_t RCChannelMailboxHandle;
extern osMessageQueueId_t SystemReadyEventGroupHandle;
extern osMessageQueueId_t IndicatorEventQueueHandle;

/* EdgeTx Packet Rate = 150Hz, 周期≈6.63ms；
 * DMA_Buf_Size = 128 B；
 * 因此设置轮询周期为4ms相当于留了三倍裕量，来防止丢数据 */
#define TASK_RC_LINK_PERIOD_MS 4U
#define RC_LOST_REPORT_MS 2500U // 持续报警重发间隔，需大于EVT_RC_LOST节拍自身播放时长(约2200ms)

/* 上电默认false：语义是“曾经连上过、现在断了才算失联事件”
 * 不能在还没绑定过遥控器的开机瞬间就先报一次“失联”，那不是真的丢失 */
static bool s_last_link_ok = false;
static uint32_t s_rc_lost_last_report_tick = 0;

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

        // 遥控失联持续报警
        if(!rc.link_ok)
        {
            if(s_last_link_ok || 
                (osKernelGetTickCount() - s_rc_lost_last_report_tick) >= RC_LOST_REPORT_MS)
            {
                IndicatorEvent_t evt = EVT_RC_LOST;
                osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0, 0);
                s_rc_lost_last_report_tick = osKernelGetTickCount();
            }
        }
        s_last_link_ok = rc.link_ok;

        osDelay(TASK_RC_LINK_PERIOD_MS);
    }
}
