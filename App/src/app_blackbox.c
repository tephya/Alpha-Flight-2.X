#include "app_blackbox.h"
#include "app_shared_types.h"
#include "bsp_blackbox.h"
#include "cmsis_os2.h"
#include "string.h"
#include "bsp_sdcard.h"

extern osMessageQueueId_t IndicatorEventQueueHandle;

/**
 * @brief   Task_Blackbox任务入口，由freertos.c的Start_Blackbox转发调用
 * @note    内部流程：BB_Init(挂载SD卡+建文件) ->
 *                   BB_BufferInit ->
 *                   for(;;){BB_WaitReady(osWaitForever); BB_Process();}
 */
void App_Blackbox_Task(void *argument)
{
    (void)argument;

    bool file_open = false;     // 还没有任何文件被打开过，开机不自动建文件
    uint8_t last_err_flags = 0; // BB_GetErrorFlags()跳边检测用

    /* 必须在第一次等待/置位控制请求之前调用，
     *否则BB_RequestNewFile的信号会丢进一个还不存在的对象里 */
    BB_ControlInit();           

    for (;;)
    {  
        if(!file_open)
        {
            uint32_t ctrl = BB_PollControlRequest(osWaitForever);
            if((int32_t)ctrl >= 0 && (ctrl & BB_CTRL_NEWFILE_REQ))
            {
                while(BB_Init() != 0)
                {
					osDelay(500);   // 挂载失败(卡未插好/供电未稳)，定期重试而非直接卡死整个飞控
				}
                BB_BufferInit();
                file_open = true;
            }
            continue;
        }

        /* 50ms只是轮询周期上限，正常情况数据就绪信号量会在缓冲写满时(约每30ms@800Hz)
         * 提前唤醒，这个超时只是保证控制请求(关闭/开新文件)不会被无限期拖延 */
        if (BB_WaitReady(50) == 0)
        {
            while (BB_Process())
                ; // 一次性排空两块缓冲区里的所有READY，不留到下一轮
        }

        uint32_t ctrl = BB_PollControlRequest(0);   // 非阻塞peek，不影响上面数据处理的节奏
        if((int32_t)ctrl >= 0)
        {
            if(ctrl & BB_CTRL_CLOSE_REQ)
            {
                while(BB_Process())
                    ;       // 关闭前先把剩余READY数据先落盘，再flush尾巴+真正关闭文件
                BB_Close();
                file_open = false;  // 回到“没有文件”状态，等下一次解锁再开新文件
            }

            if (ctrl & BB_CTRL_NEWFILE_REQ)
            {
                while (BB_Init() != 0)
                    osDelay(500); // 挂载失败，定期重试而非直接卡死
                BB_BufferInit();
            }
        }

        /* BB_GetErrorFlags()的bit0(缓冲溢出)/bit1(SD写入失败)都归为EVT_SD_CARD_ERROR，
         * 没有单独区分的必要，两者都代表“数据没能正常落盘”。只在0->非0跳变沿报一次，
         * 不会每轮循环重复刷。BB_BufferInit每次开新文件都会把这两个flag清零，
         * 所以这里不需要额外充值last_error_flags，自然会随下一轮读数同步 */
        uint8_t err_flags = BB_GetErrorFlags();
        if(err_flags != 0 && last_err_flags == 0)
        {
            IndicatorEvent_t evt = EVT_SD_CARD_ERROR;
            osMessageQueuePut(IndicatorEventQueueHandle, &evt, 0, 0);
        }
        last_err_flags = err_flags;
    }
}
