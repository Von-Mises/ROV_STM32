/**
频率(f)
时间(F)
预分频(Prescaler)
计数周期(Counter Period)
f = timer_clock  / (Prescaler+1) / (Counter_Period+1)  //timer在计算的时候会自动将值+1
计算如下：
Prescaler = 800-1
Counter_Period = 1000-1
定时器挂载在APB2总线上，频率为200MHZ
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
 * D30舵机的信号范围为500us-2500us
 * 所以重装载值应为 125-625
 * 实际限幅值应根据机械安装位置决定
**/
void servo_pwm_set(uint16_t pwm)
{
	 CONSTRAIN(pwm, 125, 625);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm);
}

/**
 * 光源的信号范围为1100us-1900us
 * 所以重装载值应为 275-475
**/
void bright_pwm_set(uint16_t pwm)
{
	CONSTRAIN(pwm, 275, 475);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm);
}
