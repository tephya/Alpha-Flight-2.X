/**
 * @file    app_rc_link.c
 * @brief   ELRS 数据处理，RC Calibration，Mailbox 发布及失联提示实现。
 */

#include "app_rc_link.h"
#include "app_rc_calibration.h"
#include "app_shared_types.h"
#include "bsp_elrs.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"

extern osMessageQueueId_t RCChannelMailboxHandle;
extern osEventFlagsId_t SystemReadyEventGroupHandle;
extern osMessageQueueId_t IndicatorEventQueueHandle;

/* 
 * EdgeTx Packet Rate = 150 Hz, 对应周期约 6.67 ms；
 * RC Link Task 以 4 ms 周期消费 UART DMA Buffer，
 * 使软件消费频率高于遥控数据到达频率，并留出一定调度裕量。
 */
#define TASK_RC_LINK_PERIOD_MS 4U

/** RC Lost 持续状态提示重发周期，ms。 */
#define RC_LOST_REPORT_MS 2500U

/*
 * RC Link 状态历史。
 * 
 * 只有系统曾经建立过 RC Link，之后再发生断连，
 * 才视为真正的 RC Lost Event。
 * 上电后尚未首次连接的阶段不主动报告失联。
 */
static bool s_last_link_ok = false;
static bool s_ever_linked = false;

/** 最近一次 RC Lost 提示投递时间。 */
static uint32_t s_rc_lost_last_report_tick = 0;

void App_RcLink_Task(void *argument)
{
    (void)argument;

    ELRS_Init();
    RcCalibration_Init();

    for (;;)
    {
        /*
         * 消费 UART DMA Buffer 中的新 ELRS 数据，
         * 更新 Driver 内部最新 RC Snapshot。
         */
        ELRS_Poll();

        RCChannelData_t rc;
        ELRS_CopyTo(&rc);

        /*
         * Calibration 状态机必须读取尚未归一化的原始 CRSF 通道值，
         * 因为 Min/Mid/Max 本身就是针对实际遥控器原始输出进行采集。
         * 
         * Calibration Update 完成后，再将主控制通道归一化，
         * 然后发布给 FlightCtrl。
         */
        RcCalibration_Update(&rc);
        RcCalibration_Apply(&rc);

        /*
         * RCChanelMailbox 采用“只保留最新状态”的 Mailbox 语义。
         * Queue 已满时先丢弃旧 Snapshot，再写入当前最新 RC 数据。
         */
        if(osMessageQueueGetSpace(RCChannelMailboxHandle) == 0)
        {
            RCChannelData_t discard;
            osMessageQueueGet(RCChannelMailboxHandle, &discard, NULL, 0);
        }

        (void)osMessageQueuePut(RCChannelMailboxHandle, &rc, 0, 0);

        /*
         * RC link Ready 直接反应当前 ELRS Link 状态。
         * 失联时立即清除 Ready Bit，由 Arm/FlightCtrl 上层决定保护动作。
         */
        if (rc.link_ok)
        {
            osEventFlagsSet(
                SystemReadyEventGroupHandle,
                SYSREADY_BIT_RC_LINK_OK);
        }
        else
        {
            osEventFlagsClear(
                SystemReadyEventGroupHandle,
                SYSREADY_BIT_RC_LINK_OK);
        }

        /*
         * RC Calibration 只有在已经存在有效参数
         * 且当前没有执行新的 Calibration 时才视为 Ready。
         */
        if (RcCalibration_IsReady() &&
            !RcCalibration_IsActive())
        {
            osEventFlagsSet(
                SystemReadyEventGroupHandle,
                SYSREADY_BIT_RC_CALIB_OK);
        }
        else
        {
            osEventFlagsClear(
                SystemReadyEventGroupHandle,
                SYSREADY_BIT_RC_CALIB_OK);
        }

        uint32_t now = osKernelGetTickCount();

        /*
         * 首次成功建立 RC Link 只记录历史状态，不播放提示。
         * 之后只有从 Connected 进入 Lost，才认为发生真正的失联事件。
         */
        if(rc.link_ok)
        {
            s_ever_linked = true;
        }
        else if(s_ever_linked)   
        {
            /*
             * RC Lost 首次发生时立即提示；
             * 若失联持续存在，则按固定周期重新报告。
             */
            if (s_last_link_ok ||
                (uint32_t)(now - s_rc_lost_last_report_tick) >= RC_LOST_REPORT_MS)
            {
                IndicatorEvent_t evt = EVT_RC_LOST;

                (void)osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0U, 0U);
                
                s_rc_lost_last_report_tick = now;
            }
        }
        s_last_link_ok = rc.link_ok;

        osDelay(TASK_RC_LINK_PERIOD_MS);
    }
}
