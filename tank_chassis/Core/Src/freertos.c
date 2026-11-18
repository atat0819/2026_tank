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
#include "usb_device.h"
#include "iwdg.h"
#include "watchdog_heartbeat.h"





/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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
extern void gantry_task(void *argument);
BaseType_t start_remote_control;
BaseType_t start_can_send;
BaseType_t start_up_stair;

TaskHandle_t xUpStairHandle;
TaskHandle_t xRemoteHandle;
TaskHandle_t xCanSendHandle;
TaskHandle_t xUIHandle;
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
  	start_can_send = xTaskCreate(can_send_task, "CAN_Send_Task", 1024, NULL, osPriorityAboveNormal, &xCanSendHandle);
  start_remote_control = xTaskCreate(remote_task, "Remote_Control_Task", 256, NULL, osPriorityAboveNormal, &xRemoteHandle);
  xTaskCreate(ui_task, "UI_Task", 384, NULL, osPriorityNormal, &xUIHandle);
  start_up_stair = xTaskCreate(up_stair_task, "Up_Stair_Task", 512, NULL, osPriorityRealtime, &xUpStairHandle);
  const BaseType_t start_gantry = xTaskCreate(gantry_task, "Gantry_Task", 512, NULL, osPriorityRealtime, &xGantryHandle);
  const BaseType_t start_watchdog = xTaskCreate(watchdog_task, "Watchdog_Task", 128, NULL, osPriorityRealtime+1, NULL);
  if (start_can_send != pdPASS || start_remote_control != pdPASS ||
      start_up_stair != pdPASS || start_gantry != pdPASS || start_watchdog != pdPASS)
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

