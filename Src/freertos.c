/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "socket_receive.h"
#include "water_level_task.h"
#include "tem_hum_task.h"
#include "socket_send.h"
#include "socket_receive.h"
#include "pwm_task.h"
#include "rov_movation_task.h"
#include "parse_task.h"
#include "detect_task.h"
#include "debug_info.h"
#include "task_time_count.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//osThreadId water_level_handle;
osThreadId tem_hum_handle;
osThreadId socket_send_handle;
osThreadId socket_receive_handle;
osThreadId pwm_task_handle;
osThreadId movation_task_handle;
osThreadId parse_task_handle;
osThreadId water_level_handle;
osThreadId debug_info_handle;
osThreadId task_time_count_handle;
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
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Pre/Post sleep processing prototypes */
void PreSleepProcessing(uint32_t *ulExpectedIdleTime);
void PostSleepProcessing(uint32_t *ulExpectedIdleTime);

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
	
}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN PREPOSTSLEEP */
__weak void PreSleepProcessing(uint32_t *ulExpectedIdleTime)
{
/* place for user code */ 
	__HAL_RCC_GPIOA_CLK_DISABLE(); 
	__HAL_RCC_GPIOB_CLK_DISABLE(); 
	__HAL_RCC_GPIOC_CLK_DISABLE(); 
	__HAL_RCC_GPIOD_CLK_DISABLE(); 
	__HAL_RCC_GPIOE_CLK_DISABLE(); 
	__HAL_RCC_GPIOF_CLK_DISABLE();
	__HAL_RCC_GPIOG_CLK_DISABLE();
}

__weak void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{
/* place for user code */
	__HAL_RCC_GPIOA_CLK_ENABLE(); 
	__HAL_RCC_GPIOB_CLK_ENABLE(); 
	__HAL_RCC_GPIOC_CLK_ENABLE(); 
	__HAL_RCC_GPIOD_CLK_ENABLE(); 
	__HAL_RCC_GPIOE_CLK_ENABLE(); 
	__HAL_RCC_GPIOF_CLK_ENABLE(); 
	__HAL_RCC_GPIOG_CLK_ENABLE();
}
/* USER CODE END PREPOSTSLEEP */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	osThreadDef(socket_receive, server_receive_task, osPriorityAboveNormal, 0, 512);
    socket_receive_handle = osThreadCreate(osThread(socket_receive), NULL);
	
	osThreadDef(socket_send, server_send_task, osPriorityAboveNormal, 0, 512);
    socket_send_handle = osThreadCreate(osThread(socket_send), NULL);

	osThreadDef(parse, parse_task, osPriorityNormal, 0, 128);
    parse_task_handle = osThreadCreate(osThread(parse), NULL);
	
	osThreadDef(pwm, pwm_task, osPriorityNormal, 0, 128);
    pwm_task_handle = osThreadCreate(osThread(pwm), NULL);
	
	osThreadDef(WATER_LEVEL, water_level_task, osPriorityNormal, 0, 128);
    water_level_handle = osThreadCreate(osThread(WATER_LEVEL), NULL);
	
	osThreadDef(TEM_HUM, tem_hum_task, osPriorityNormal, 0, 128);
    tem_hum_handle = osThreadCreate(osThread(TEM_HUM), NULL);
	
	osThreadDef(ROV_MOVATION, rov_movation_task, osPriorityAboveNormal, 0, 512);
    movation_task_handle = osThreadCreate(osThread(ROV_MOVATION), NULL);
	
    osThreadDef(debug_info, debug_info_task, osPriorityNormal, 0, 256);
    debug_info_handle = osThreadCreate(osThread(debug_info), NULL);
	
	osThreadDef(task_time_count, task_time_count_task, osPriorityAboveNormal, 0, 256);
    task_time_count_handle = osThreadCreate(osThread(task_time_count), NULL);
	
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
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
