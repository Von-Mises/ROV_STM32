#include "bsp_pulse.h"
#include "main.h"
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

uint16_t crawler_motor_1;
uint16_t crawler_motor_2;

void Pulse_Fb_Init()
{
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim4);
}

/*
* ÿ��5ms����һ���ж�,�Ĵ����תһȦΪ12*71������
*����rpm�ļ���ΪCNT/��6*71��*12000
*/
void TIM2_IRQHandler(void)
{
	__HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_UPDATE);									   //����жϱ�־λ
	crawler_motor_1 = __HAL_TIM_GET_COUNTER(&htim3)/(12*71.0)*12000;                   //��ȡ��λʱ���ڼ������Ƶ�CNTֵ
	crawler_motor_2 = __HAL_TIM_GET_COUNTER(&htim4)/(12*71.0)*12000;                   //��ȡ��λʱ���ڼ������Ƶ�CNTֵ
	//��ʱ������
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	__HAL_TIM_SET_COUNTER(&htim4, 0);
}

void get_crawler_motor_speed(uint16_t* pcrawler_motor_1,uint16_t* pcrawler_motor_2)
{
	*pcrawler_motor_1 = crawler_motor_1;
	*pcrawler_motor_2 = crawler_motor_2;
}
