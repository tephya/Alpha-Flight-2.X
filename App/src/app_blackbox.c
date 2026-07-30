#include "app_blackbox.h"
#include "app_shared_types.h"
#include "bsp_blackbox.h"
#include "cmsis_os2.h"

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

    // BB_Init()内部含SD卡上电时序(阻塞若干ms)，必须在调度器启动后的Task上下文里跑
    while(BB_Init() != 0)
    {
        osDelay(500);   // 挂载失败(卡未插好/供电未稳)，定期重试而非直接卡死整个飞控
    }

    BB_BufferInit();

    uint8_t last_err_flags = 0;     // BB_GetErrorFlags()跳边检测用

    for (;;)
    {   
        if(BB_WaitReady(50) == 0)
        {
            while(BB_Process())
                ;   // 一次性排空两块缓冲区里的所有READY，不留到下一轮
        }

        uint32_t ctrl = BB_PollControlRequest(0);   // 非阻塞peek，不影响上面数据处理的节奏
        if((int32_t)ctrl >= 0)
        {
            if(ctrl && BB_CTRL_CLOSE_REQ)
            {
                while(BB_Process())
                    ;       // 关闭前先把剩余READY数据先落盘，再flush尾巴+真正关闭文件
                BB_Close();
            }

            if (ctrl && BB_CTRL_NEWFILE_REQ)
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
