/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "insTask.h"
//#include "RM_remote.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
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
osThreadId motorMoveCtrlHandle;
osThreadId insCalHandle;
osThreadId chassisRctrlHandle;
osThreadId chassisLctrlHandle;
osThreadId remoteCtrlHandle;
osThreadId observeCtrlHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
osSemaphoreId imuBinarySem01Handle;
osStaticSemaphoreDef_t imuBinarySemControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void motorMoveTask(void const * argument);
void insTask(void const * argument);
void chassisRtask(void const * argument);
void chassisLtask(void const * argument);
void remoteTask(void const * argument);
void observeTask(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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

  /* Create the semaphores(s) */
  /* definition and creation of imuBinarySem01 */
  osSemaphoreStaticDef(imuBinarySem01, &imuBinarySemControlBlock);
  imuBinarySem01Handle = osSemaphoreCreate(osSemaphore(imuBinarySem01), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(motorMoveCtrl, motorMoveTask, osPriorityHigh, 0, 512);
  motorMoveCtrlHandle = osThreadCreate(osThread(motorMoveCtrl), NULL);
  osThreadDef(insCal, insTask, osPriorityRealtime, 0, 512);
  insCalHandle = osThreadCreate(osThread(insCal), NULL);
  osThreadDef(chassisRctrl, chassisRtask, osPriorityRealtime, 0, 1024);
  chassisRctrlHandle = osThreadCreate(osThread(chassisRctrl), NULL);
  osThreadDef(chassisLctrl, chassisLtask, osPriorityRealtime, 0, 1024);
  chassisLctrlHandle = osThreadCreate(osThread(chassisLctrl), NULL);
  osThreadDef(remoteCtrl, remoteTask, osPriorityHigh, 0, 512);
  remoteCtrlHandle = osThreadCreate(osThread(remoteCtrl), NULL);
  osThreadDef(observeCtrl, observeTask, osPriorityHigh, 0, 512);
  observeCtrlHandle = osThreadCreate(osThread(observeCtrl), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
__weak void motorMoveTask(void const * argument)
{
  for(;;)
  {
    osDelay(1);
  }
}

__weak void insTask(void const * argument)
{
  for(;;)
  {
    INS_Task();
  }
}

__weak void chassisRtask(void const * argument)
{
  for(;;)
  {
    osDelay(1);
  }
}

__weak void chassisLtask(void const * argument)
{
  for(;;)
  {
    osDelay(1);
  }
}

__weak void remoteTask(void const * argument)
{
  for(;;)
  {
    osDelay(1);
  }
}

__weak void observeTask(void const * argument)
{
  for(;;)
  {
    osDelay(1);
  }
}
/* USER CODE END Application */
