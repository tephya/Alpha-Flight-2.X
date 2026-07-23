/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_shared_types.h"
#include "app_nav.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Task_FlightCtrl */
osThreadId_t Task_FlightCtrlHandle;
const osThreadAttr_t Task_FlightCtrl_attributes = {
  .name = "Task_FlightCtrl",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for Task_IMU2_RDD */
osThreadId_t Task_IMU2_RDDHandle;
const osThreadAttr_t Task_IMU2_RDD_attributes = {
  .name = "Task_IMU2_RDD",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityRealtime6,
};
/* Definitions for Task_RC_LINK */
osThreadId_t Task_RC_LINKHandle;
const osThreadAttr_t Task_RC_LINK_attributes = {
  .name = "Task_RC_LINK",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime6,
};
/* Definitions for Task_Nav */
osThreadId_t Task_NavHandle;
const osThreadAttr_t Task_Nav_attributes = {
  .name = "Task_Nav",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityRealtime5,
};
/* Definitions for Task_Blackbox */
osThreadId_t Task_BlackboxHandle;
const osThreadAttr_t Task_Blackbox_attributes = {
  .name = "Task_Blackbox",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime4,
};
/* Definitions for Task_Telemetry */
osThreadId_t Task_TelemetryHandle;
const osThreadAttr_t Task_Telemetry_attributes = {
  .name = "Task_Telemetry",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime4,
};
/* Definitions for Task_PowerMonit */
osThreadId_t Task_PowerMonitHandle;
const osThreadAttr_t Task_PowerMonit_attributes = {
  .name = "Task_PowerMonit",
  .stack_size = 160 * 4,
  .priority = (osPriority_t) osPriorityRealtime4,
};
/* Definitions for Task_Indicator */
osThreadId_t Task_IndicatorHandle;
const osThreadAttr_t Task_Indicator_attributes = {
  .name = "Task_Indicator",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime3,
};
/* Definitions for Task_IWDG_Feed */
osThreadId_t Task_IWDG_FeedHandle;
const osThreadAttr_t Task_IWDG_Feed_attributes = {
  .name = "Task_IWDG_Feed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime4,
};
/* Definitions for BlackboxLogQueue */
osMessageQueueId_t BlackboxLogQueueHandle;
uint8_t BlackboxLogQueueBuffer[ 24 * sizeof( Blackbox_Frame_t ) ];
osStaticMessageQDef_t BlackboxLogQueueCB;
const osMessageQueueAttr_t BlackboxLogQueue_attributes = {
  .name = "BlackboxLogQueue",
  .cb_mem = &BlackboxLogQueueCB,
  .cb_size = sizeof(BlackboxLogQueueCB),
  .mq_mem = &BlackboxLogQueueBuffer,
  .mq_size = sizeof(BlackboxLogQueueBuffer)
};
/* Definitions for RCChannelMailbox */
osMessageQueueId_t RCChannelMailboxHandle;
const osMessageQueueAttr_t RCChannelMailbox_attributes = {
  .name = "RCChannelMailbox"
};
/* Definitions for NavStateMailbox */
osMessageQueueId_t NavStateMailboxHandle;
const osMessageQueueAttr_t NavStateMailbox_attributes = {
  .name = "NavStateMailbox"
};
/* Definitions for IndicatorEventQueue */
osMessageQueueId_t IndicatorEventQueueHandle;
const osMessageQueueAttr_t IndicatorEventQueue_attributes = {
  .name = "IndicatorEventQueue"
};
/* Definitions for MagDataMailbox */
osMessageQueueId_t MagDataMailboxHandle;
const osMessageQueueAttr_t MagDataMailbox_attributes = {
  .name = "MagDataMailbox"
};
/* Definitions for NavCommandQueue */
osMessageQueueId_t NavCommandQueueHandle;
const osMessageQueueAttr_t NavCommandQueue_attributes = {
  .name = "NavCommandQueue"
};
/* Definitions for RC_FailsafeTimer */
osTimerId_t RC_FailsafeTimerHandle;
const osTimerAttr_t RC_FailsafeTimer_attributes = {
  .name = "RC_FailsafeTimer"
};
/* Definitions for SystemReadyEventGroup */
osEventFlagsId_t SystemReadyEventGroupHandle;
const osEventFlagsAttr_t SystemReadyEventGroup_attributes = {
  .name = "SystemReadyEventGroup"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask_FlightControl(void *argument);
void StartTask_IMU2_RDD(void *argument);
void StartTask_RC_LINK(void *argument);
void StartTask_Nav(void *argument);
void StartTask_Blackbox(void *argument);
void StartTask_Telemetry(void *argument);
void StartTask_PowerMonitor(void *argument);
void StartTask_Indicator(void *argument);
void StartTask_IWDG(void *argument);
void RC_FailsafeTimeout_Callback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of RC_FailsafeTimer */
  RC_FailsafeTimerHandle = osTimerNew(RC_FailsafeTimeout_Callback, osTimerOnce, NULL, &RC_FailsafeTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of BlackboxLogQueue */
  BlackboxLogQueueHandle = osMessageQueueNew (24, sizeof(Blackbox_Frame_t), &BlackboxLogQueue_attributes);

  /* creation of RCChannelMailbox */
  RCChannelMailboxHandle = osMessageQueueNew (1, sizeof(RCChannelData_t), &RCChannelMailbox_attributes);

  /* creation of NavStateMailbox */
  NavStateMailboxHandle = osMessageQueueNew (1, sizeof(NavState_t), &NavStateMailbox_attributes);

  /* creation of IndicatorEventQueue */
  IndicatorEventQueueHandle = osMessageQueueNew (8, sizeof(IndicatorEvent_t), &IndicatorEventQueue_attributes);

  /* creation of MagDataMailbox */
  MagDataMailboxHandle = osMessageQueueNew (1, sizeof(MagData_t), &MagDataMailbox_attributes);

  /* creation of NavCommandQueue */
  NavCommandQueueHandle = osMessageQueueNew (4, sizeof(NavCommand_t), &NavCommandQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_FlightCtrl */
  Task_FlightCtrlHandle = osThreadNew(StartTask_FlightControl, NULL, &Task_FlightCtrl_attributes);

  /* creation of Task_IMU2_RDD */
  Task_IMU2_RDDHandle = osThreadNew(StartTask_IMU2_RDD, NULL, &Task_IMU2_RDD_attributes);

  /* creation of Task_RC_LINK */
  Task_RC_LINKHandle = osThreadNew(StartTask_RC_LINK, NULL, &Task_RC_LINK_attributes);

  /* creation of Task_Nav */
  Task_NavHandle = osThreadNew(StartTask_Nav, NULL, &Task_Nav_attributes);

  /* creation of Task_Blackbox */
  Task_BlackboxHandle = osThreadNew(StartTask_Blackbox, NULL, &Task_Blackbox_attributes);

  /* creation of Task_Telemetry */
  Task_TelemetryHandle = osThreadNew(StartTask_Telemetry, NULL, &Task_Telemetry_attributes);

  /* creation of Task_PowerMonit */
  Task_PowerMonitHandle = osThreadNew(StartTask_PowerMonitor, NULL, &Task_PowerMonit_attributes);

  /* creation of Task_Indicator */
  Task_IndicatorHandle = osThreadNew(StartTask_Indicator, NULL, &Task_Indicator_attributes);

  /* creation of Task_IWDG_Feed */
  Task_IWDG_FeedHandle = osThreadNew(StartTask_IWDG, NULL, &Task_IWDG_Feed_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of SystemReadyEventGroup */
  SystemReadyEventGroupHandle = osEventFlagsNew(&SystemReadyEventGroup_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTask_FlightControl */
/**
  * @brief  Function implementing the Task_FlightCtrl thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask_FlightControl */
void StartTask_FlightControl(void *argument)
{
  /* USER CODE BEGIN StartTask_FlightControl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask_FlightControl */
}

/* USER CODE BEGIN Header_StartTask_IMU2_RDD */
/**
* @brief Function implementing the Task_IMU2_RDD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_IMU2_RDD */
void StartTask_IMU2_RDD(void *argument)
{
  /* USER CODE BEGIN StartTask_IMU2_RDD */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask_IMU2_RDD */
}

/* USER CODE BEGIN Header_StartTask_RC_LINK */
/**
* @brief Function implementing the Task_RC_LINK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_RC_LINK */
void StartTask_RC_LINK(void *argument)
{
  /* USER CODE BEGIN StartTask_RC_LINK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask_RC_LINK */
}

/* USER CODE BEGIN Header_StartTask_Nav */
/**
* @brief Function implementing the Task_Nav thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_Nav */
void StartTask_Nav(void *argument)
{
  /* USER CODE BEGIN StartTask_Nav */
  /* Infinite loop */
  App_Nav_Task(argument);
  /* USER CODE END StartTask_Nav */
}

/* USER CODE BEGIN Header_StartTask_Blackbox */
/**
* @brief Function implementing the Task_Blackbox thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_Blackbox */
void StartTask_Blackbox(void *argument)
{
  /* USER CODE BEGIN StartTask_Blackbox */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask_Blackbox */
}

/* USER CODE BEGIN Header_StartTask_Telemetry */
/**
* @brief Function implementing the Task_Telemetry thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_Telemetry */
void StartTask_Telemetry(void *argument)
{
  /* USER CODE BEGIN StartTask_Telemetry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask_Telemetry */
}

/* USER CODE BEGIN Header_StartTask_PowerMonitor */
/**
* @brief Function implementing the Task_PowerMonit thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_PowerMonitor */
void StartTask_PowerMonitor(void *argument)
{
  /* USER CODE BEGIN StartTask_PowerMonitor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask_PowerMonitor */
}

/* USER CODE BEGIN Header_StartTask_Indicator */
/**
* @brief Function implementing the Task_Indicator thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_Indicator */
void StartTask_Indicator(void *argument)
{
  /* USER CODE BEGIN StartTask_Indicator */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask_Indicator */
}

/* USER CODE BEGIN Header_StartTask_IWDG */
/**
* @brief Function implementing the Task_IWDG_Feed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_IWDG */
void StartTask_IWDG(void *argument)
{
  /* USER CODE BEGIN StartTask_IWDG */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask_IWDG */
}

/* RC_FailsafeTimeout_Callback function */
void RC_FailsafeTimeout_Callback(void *argument)
{
  /* USER CODE BEGIN RC_FailsafeTimeout_Callback */

  /* USER CODE END RC_FailsafeTimeout_Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
