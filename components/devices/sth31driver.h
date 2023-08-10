/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       sth31driver.c/h
  * @brief      ������STH31��ʪ��ģ�����������������оƬ�ĳ�ʼ�������üĴ���
	*							����ɷ��ͺͶ�ȡ��
  * @note       STH1ֻ֧��IIC��ȡ
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-12-2022     HaoLion(������)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#ifndef STH31DRIVER_H
#define STH31DRIVER_H

#include "struct_typedef.h"

#define SHT31_ADDR_WRITE 0x88  //ADDR (pin 2)connected to VDD
#define SHT31_ADDR_READ 0x89  
#define STH31_I2C    I2C2

/* ----------------------- Data Struct ------------------------------------- */
typedef enum
{
    /* �����λ���� */

    SOFT_RESET_CMD = 0x30A2,	
    /*
    ���β���ģʽ
    ������ʽ��Repeatability_CS_CMD
    CS�� Clock stretching
    */
    HIGH_ENABLED_CMD    = 0x2C06,
    MEDIUM_ENABLED_CMD  = 0x2C0D,
    LOW_ENABLED_CMD     = 0x2C10,
    HIGH_DISABLED_CMD   = 0x2400,
    MEDIUM_DISABLED_CMD = 0x240B,
    LOW_DISABLED_CMD    = 0x2416,

    /*
    ���ڲ���ģʽ
    ������ʽ��Repeatability_MPS_CMD
    MPS��measurement per second
    */
    HIGH_0_5_CMD   = 0x2032,
    MEDIUM_0_5_CMD = 0x2024,
    LOW_0_5_CMD    = 0x202F,
    HIGH_1_CMD     = 0x2130,
    MEDIUM_1_CMD   = 0x2126,
    LOW_1_CMD      = 0x212D,
    HIGH_2_CMD     = 0x2236,
    MEDIUM_2_CMD   = 0x2220,
    LOW_2_CMD      = 0x222B,
    HIGH_4_CMD     = 0x2334,
    MEDIUM_4_CMD   = 0x2322,
    LOW_4_CMD      = 0x2329,
    HIGH_10_CMD    = 0x2737,
    MEDIUM_10_CMD  = 0x2721,
    LOW_10_CMD     = 0x272A,
	/* ���ڲ���ģʽ��ȡ�������� */
	READOUT_FOR_PERIODIC_MODE = 0xE000,
} SHT31_CMD;

/* ----------------------- Extern Function ----------------------------------- */
/**
 * @brief	��ʼ��SHT30
 * @param	none
 * @retval	none
 * @note	���ڲ���ģʽ
*/
extern void SHT31_Init(void);

/**
 * @brief	��SHT31��ȡһ������
 * @param	dat ���� �洢��ȡ���ݵĵ�ַ��6���ֽ����飩
 * @retval	��ȡ�ɹ�  ���� ����1
 * 			��ȡʧ��  ���� ����0���������¶�ֵ��ʪ��ֵΪ0
*/
extern bool_t SHT31_Read_Dat(float* temperature, float* humidity);


#endif
