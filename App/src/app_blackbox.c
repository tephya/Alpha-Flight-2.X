/**
 * @file    app_blackbox.c
 * @brief   Blackbox 后台文件管理与 SD 写入任务实现。
 */

#include "app_blackbox.h"
#include "app_shared_types.h"
#include "bsp_blackbox.h"
#include "cmsis_os2.h"
#include "string.h"
#include "bsp_sdcard.h"

extern osMessageQueueId_t IndicatorEventQueueHandle;

/** 当前是否存在已打开的 Blackbox 日志文件。 */
static volatile bool s_file_open;

/** 最近一次日志文件关闭操作是否成功。 */
static volatile bool s_last_close_succeeded;

bool App_Blackbox_IsFileOpen(void)
{
    return s_file_open;
}

bool App_Blackbox_LastCloseSucceeded(void)
{
    return s_last_close_succeeded;
}

void App_Blackbox_Task(void *argument)
{
    (void)argument;

    // 上电后不主动创建日志文件，等待首次 NEWFILE 请求。
    s_file_open = false;
    s_last_close_succeeded = false;

    // 用于检测 Blackbox Error Flag 从 0 -> 非 0 的条件。
    uint8_t last_err_flags = 0U;

    /*
     * 必须先创建 Blackbox 内部控制同步对象。
     * 非则后续 NEWFILE/CLOSE 请求可能发送到尚未初始化的对象。
     */
    BB_ControlInit();           

    for (;;)
    {  
        if(!s_file_open)
        {
            /*
             * 无日志文件时阻塞等待控制请求，
             * 避免 Blackbox Task 在空闲状态持续占用 CPU。
             */
            uint32_t ctrl = BB_PollControlRequest(osWaitForever);

            if((int32_t)ctrl >= 0 && 
                (ctrl & BB_CTRL_NEWFILE_REQ))
            {
                /*
                 * SD 挂载或文件创建失败时周期性重试。
                 * Blackbox Task 自身阻塞等待，不影响其他 RTOS Task 运行。
                 */
                while(BB_Init() != 0)
                {
					osDelay(500); 
				}
                BB_BufferInit();

                s_file_open = true;
                s_last_close_succeeded = false;
            }
            continue;
        }

        /*
         * 正常情况下，Blackbox 缓冲区写满后会通过同步信号提前唤醒 Task。
         * 50 ms 超时只作为轮询上限，保证 CLOSE/NEWFILE 控制请求
         * 不会因长期没有 READY Buffer 而无限延迟。
         */
        if (BB_WaitReady(50) == 0)
        {
            // 一次唤醒后尽量排空所有 READY Buffer，减少待写数据积压。
            while (BB_Process())
                ;
        }

        /*
         * 数据处理完成后非阻塞检查控制请求，
         * 避免文件控制流程破坏正常 Buffer 写入节奏。
         */
        uint32_t ctrl = BB_PollControlRequest(0);

        if((int32_t)ctrl >= 0)
        {
            if(ctrl & BB_CTRL_CLOSE_REQ)
            {
                /*
                 * 关闭前先写完所有 READY Buffer，
                 * BB_Close() 再负责 Flush 尾部数据并真正关闭文件。
                 */
                while(BB_Process())
                    ;

                s_last_close_succeeded = (BB_Close() == 0);

                // 回到无文件状态，等待下一次 Armed 后的新建请求。
                s_file_open = false;
            }

            if (ctrl & BB_CTRL_NEWFILE_REQ)
            {
                /*
                 * 防御异常重复 NEWFILE 请求。
                 * 若文件仍处于打开状态，必须先完整关闭后再创建新文件。
                 */
                if(s_file_open)
                {
                    while(BB_Process())
                        ;
                    s_last_close_succeeded = (BB_Close() == 0);
                    s_file_open = false;
                }

                while (BB_Init() != 0)
                {
                    osDelay(500);
                }
                BB_BufferInit();

                s_last_close_succeeded = false;
                s_file_open = true;
            }
        }

        /*
         * Buffer Overflow 与 SD Write Error 都意味着 Blackbox 数据未正常落盘，
         * 因此同一上报 EVT_SD_CARD_ERROR。
         * 
         * 仅在 Error Flag 从 0 -> 非 0 时上报一次，避免每轮循环重复发送事件。
         * BB_BufferInit() 在新建日志文件时会清除底层 Error Flag，
         * Last_err_flags 会在后续循环中自然重新同步。
         */
        uint8_t err_flags = BB_GetErrorFlags();
        if(err_flags != 0 && last_err_flags == 0)
        {
            IndicatorEvent_t evt = EVT_SD_CARD_ERROR;
            osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0, 0);
        }
        last_err_flags = err_flags;
    }
}
