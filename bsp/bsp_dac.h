#ifndef BSP_DAC_H
#define BSP_DAC_H
#include "struct_typedef.h"

#define Motor1_Dir_PIN                   GPIO_PIN_1	
#define Motor2_Dir_PIN                   GPIO_PIN_0
#define Motor_Dir_GPIO_PORT            	 GPIOB	
			


/**
  * @brief          Output Analog voltage
  * @param[in]      L_vol: voltage of left track motor
  * @param[in]      R_vol: voltage of right track motor
  * @retval         none
  */
/**
  * @brief          输出DAC
  * @param[in]      L_vol: 左轮履带电机输入电压值（0~3300）
  * @param[in]      R_vol: 右轮履带电机输入电压值（0~3300）
  * @retval         none
  */
extern void Track_Motor_Ctrl(int16_t L_vol,int16_t R_vol);

extern void Track_Motor_Init(void);

#endif
