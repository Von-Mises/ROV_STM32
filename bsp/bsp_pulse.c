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
* 每隔5ms进入一次中断,履带电机转一圈为12*71个脉冲
*所以rpm的计算为CNT/（6*71）*12000
*/
void TIM2_IRQHandler(void)
{
	__HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_UPDATE);									   //清除中断标志位
	crawler_motor_1 = __HAL_TIM_GET_COUNTER(&htim3)/(12*71.0)*12000;                   //读取单位时间内计数器计的CNT值
	crawler_motor_2 = __HAL_TIM_GET_COUNTER(&htim4)/(12*71.0)*12000;                   //读取单位时间内计数器计的CNT值
	//定时器清零
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	__HAL_TIM_SET_COUNTER(&htim4, 0);
}

void get_crawler_motor_speed(uint16_t* pcrawler_motor_1,uint16_t* pcrawler_motor_2)
{
	*pcrawler_motor_1 = crawler_motor_1;
	*pcrawler_motor_2 = crawler_motor_2;
}
