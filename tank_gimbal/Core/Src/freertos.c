/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "can_send_task.hpp"
#include "gimbal_task.hpp"
#include "remote_control_task.hpp"
#include "gantry_task.hpp"
#include "usb_device.h"
#include "watchdog_heartbeat.h"
#include "iwdg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/***************************
 * 看门狗管理任务约每 30 ms 检查一次。
管理任务开始运行后，最多给关键任务 3 秒启动时间。
进入正常监控后，任一心跳连续约 200 ms 没有推进，就判定异常。
 */
#define WATCHDOG_CHECK_PERIOD_MS 30U
#define WATCHDOG_STARTUP_TIMEOUT_MS 3000U
#define WATCHDOG_HEARTBEAT_TIMEOUT_MS 200U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile uint32_t g_watchdog_heartbeats[WATCHDOG_TASK_COUNT] = {0};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
TaskHandle_t xRemoteHandle;
TaskHandle_t xCanSendHandle;
TaskHandle_t xGimbalHandle;
TaskHandle_t xGantryHandle;

static void watchdog_task(void *argument);

extern void MX_USB_DEVICE_Init(void);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USB is initialized by defaultTask after the scheduler starts. */
  if (defaultTaskHandle == NULL ||
      xTaskCreate(can_send_task, "CAN_Send_Task", 2048, NULL, osPriorityAboveNormal+2, &xCanSendHandle) != pdPASS ||
      xTaskCreate(gimbal_task, "Gimbal_Task", 1024, NULL, osPriorityAboveNormal+1, &xGimbalHandle) != pdPASS ||
      xTaskCreate(remote_control_task, "Remote_Control_Task", 512, NULL, osPriorityAboveNormal, &xRemoteHandle) != pdPASS ||
      xTaskCreate(gantry_task, "Gantry_Task", 128, NULL, osPriorityAboveNormal, &xGantryHandle) != pdPASS ||
      xTaskCreate(watchdog_task, "Watchdog_Task", 128, NULL, osPriorityAboveNormal+3, NULL) != pdPASS)
  {
    Error_Handler();
  }


  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osThreadExit();
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Watchdog_RecordHeartbeat(WatchdogTask task)
{
  taskENTER_CRITICAL();
  ++g_watchdog_heartbeats[task];
  taskEXIT_CRITICAL();
}

static void watchdog_task(void *argument)
{
  const TickType_t start = xTaskGetTickCount();
  uint32_t previous[WATCHDOG_TASK_COUNT] = {0};
  TickType_t last_change[WATCHDOG_TASK_COUNT] = {0};
  uint32_t seen = 0;
  uint8_t running = 0;
  uint8_t failed = 0;
  (void)argument;
  for (;;)
  {
    const TickType_t now = xTaskGetTickCount();
    uint32_t snapshot[WATCHDOG_TASK_COUNT];
    uint8_t healthy = 1;
    taskENTER_CRITICAL();
    for (unsigned i = 0; i < WATCHDOG_TASK_COUNT; ++i)
      snapshot[i] = g_watchdog_heartbeats[i];
    taskEXIT_CRITICAL();

    for (unsigned i = 0; i < WATCHDOG_TASK_COUNT; ++i)
    {
      if (snapshot[i] != previous[i])
      {
        previous[i] = snapshot[i];
        last_change[i] = now;
        seen |= (1UL << i);
      }
      if ((seen & (1UL << i)) == 0U ||
          (TickType_t)(now - last_change[i]) >= pdMS_TO_TICKS(WATCHDOG_HEARTBEAT_TIMEOUT_MS))
        healthy = 0;
    }

    if (!failed)
    {
      if (!running)
      {
        if ((TickType_t)(now - start) >= pdMS_TO_TICKS(WATCHDOG_STARTUP_TIMEOUT_MS))
          failed = 1;
        else if (healthy)
          running = 1;
      }
      else if (!healthy)
        failed = 1;

      if (!failed && HAL_IWDG_Refresh(&hiwdg1) != HAL_OK)
        failed = 1;
    }
    vTaskDelay(pdMS_TO_TICKS(WATCHDOG_CHECK_PERIOD_MS));
  }
}

/* USER CODE END Application */

