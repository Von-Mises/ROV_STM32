/**
Ƶ��(f)
ʱ��(F)
Ԥ��Ƶ(Prescaler)
��������(Counter Period)
f = timer_clock  / (Prescaler+1) / (Counter_Period+1)  //timer�ڼ����ʱ����Զ���ֵ+1
�������£�
Prescaler = 800-1
Counter_Period = 1000-1
��ʱ��������APB2�����ϣ�Ƶ��Ϊ200MHZ
f = 240Mhz / (800-1+1) / (1000-1+1) = 250hz
T = 1/250 = 4ms
**/

#include "bsp_pwm.h"
#include "main.h"

#define CONSTRAIN(amt, low, high)  ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

extern TIM_HandleTypeDef htim8;

void pwm_init(void)
{
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 275);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 125);
	servo_pwm_set(SERVO_MID_VAL);
}
/**
 * D30������źŷ�ΧΪ500us-2500us
 * ������װ��ֵӦΪ 125-625
 * ʵ���޷�ֵӦ���ݻ�е��װλ�þ���
**/
void servo_pwm_set(uint16_t pwm)
{
	 CONSTRAIN(pwm, 125, 625);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm);
}

/**
 * ��Դ���źŷ�ΧΪ1100us-1900us
 * ������װ��ֵӦΪ 275-475
**/
void bright_pwm_set(uint16_t pwm)
{
	CONSTRAIN(pwm, 275, 475);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm);
}
