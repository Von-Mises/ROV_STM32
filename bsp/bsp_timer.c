/**
  ****************************(C) COPYRIGHT 2023 ZJU****************************
  * @file       bsp_timer.c/h
  * @brief      ��ʼ������ͳ�ƶ�ʱ����ΪFreeRTOS����ͳ���ṩʱ������ʱ��Ƶ��ӦΪϵͳʼ�ս��ĵ�10~100����
  *             
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-8-2023      Qiqi Li(������) 1. done
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
	//���жϣ������һֱ�����ж��޷�ִ����������
	__HAL_TIM_CLEAR_FLAG(&htim5,TIM_FLAG_UPDATE);
	//����ֵ����һ
	FreeRTOSRunTimeTicks++;
}
