#include "app_blackbox.h"
#include "bsp_blackbox.h"
#include "cmsis_os2.h"

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

    for (;;)
    {
        if(BB_WaitReady(osWaitForever) == 0)
        {
            BB_Process();
        }

        // TODO: BB_GetErrorFlags()非0时，应该往IndicatorEventQueue发一条EVT_SD_CARD_ERROR
    }
}
