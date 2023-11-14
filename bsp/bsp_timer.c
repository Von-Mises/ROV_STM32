/**
  ****************************(C) COPYRIGHT 2023 ZJU****************************
  * @file       bsp_timer.c/h
  * @brief      初始化任务统计定时器，为FreeRTOS任务统计提供时基（该时基频率应为系统始终节拍的10~100倍）
  *             
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-8-2023      Qiqi Li(李琪琪) 1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 ZJU****************************
  */
#include "bsp_timer.h"
#include "lwip.h"

volatile unsigned long FreeRTOSRunTimeTicks = 0;
extern TIM_HandleTypeDef htim5;

void Timer_Init(void)
{
	HAL_TIM_Base_Start_IT(&htim5);
}

void TIM5_IRQHandler(void)
{
	//清中断，否则会一直触发中断无法执行其它任务
	__HAL_TIM_CLEAR_FLAG(&htim5,TIM_FLAG_UPDATE);
	//计数值自增一
	FreeRTOSRunTimeTicks++;
}
