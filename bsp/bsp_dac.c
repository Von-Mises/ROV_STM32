#include "bsp_dac.h"
#include "main.h"

extern DAC_HandleTypeDef hdac1;

#define max 					4095	
#define VDDA 					2.5f
#define _Limit(var)       (var = (((var) < (max)) ? (var) : (max)))


/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			{p->BSRR=i;}			  								//����Ϊ�ߵ�ƽ		
#define digitalLo(p,i)			{p->BSRR=(uint32_t)i << 16;}				//����͵�ƽ

/*���Ƶ������GPIO�궨��*/
#define Motor1_CW()     				digitalLo(Motor_Dir_GPIO_PORT,Motor1_Dir_PIN)		//�����͵�ƽ��ת
#define Motor1_CCW()     				digitalHi(Motor_Dir_GPIO_PORT,Motor1_Dir_PIN)		//�����ߵ�ƽ��ת

#define Motor2_CW()     				digitalLo(Motor_Dir_GPIO_PORT,Motor2_Dir_PIN)		//�ҵ���͵�ƽ��ת
#define Motor2_CCW()     				digitalHi(Motor_Dir_GPIO_PORT,Motor2_Dir_PIN)		//�ҵ���ߵ�ƽ��ת

uint32_t Motor_L;
uint32_t Motor_R;

void Track_Motor_Init()
{
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,&Motor_L,1,DAC_ALIGN_12B_R);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2,&Motor_R,1,DAC_ALIGN_12B_R);
}


/**
  * @brief          Output Analog voltage
  * @param[in]      L_vol: voltage of left track motor
  * @param[in]      R_vol: voltage of right track motor
  * @retval         none
  */
/**
  * @brief          ���DAC
  * @param[in]      L_vol: �����Ĵ���������ѹֵ��-2500~2500��
  * @param[in]      R_vol: �����Ĵ���������ѹֵ��-2500~2500��
  * @retval         none
  */
void Track_Motor_Ctrl(int16_t L_vol,int16_t R_vol)
{
	 float temp_l=L_vol;
	 float temp_r=R_vol;
	if(temp_l>=0)
	{
		Motor1_CW();
	}
	else
	{
		temp_l = -temp_l;
		Motor1_CCW();
	}
	
	temp_l = temp_l/1000.0f; 
	Motor_L=(uint32_t)(temp_l*max/VDDA);
	_Limit(Motor_L);
	
	if(temp_r>=0)
	{
		Motor2_CCW();
	}
	else
	{
		temp_r = -temp_r;
		
		Motor2_CW();
	}
	
	temp_r= temp_r/1000.0f; 
	Motor_R=(uint32_t)(temp_r*max/VDDA); 
	_Limit(Motor_R);
	
	//12λ�Ҷ������ݸ�ʽ����DACֵ
	HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1, DAC_ALIGN_12B_R,Motor_L);
	HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_2, DAC_ALIGN_12B_R,Motor_R); 	
}
