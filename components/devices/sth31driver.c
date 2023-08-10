/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       sth31driver.c/h
  * @brief      ������STH31��ʪ��ģ�����������������оƬ�ĳ�ʼ�������üĴ���
	*							����ɷ��ͺͶ�ȡ��
  * @note       STH31ֻ֧��IIC��ȡ
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
#include "main.h"
#include "sth31driver.h"
#include "bsp_i2c.h"
#include "config.h"





/**
 * @brief	��SHT31����һ��ָ��(16bit)
 * @param	cmd ���� SHT31ָ���SHT31_MODE��ö�ٶ��壩
 * @retval	none
*/
static void	SHT31_Send_Cmd(SHT31_CMD cmd)
{
    uint8_t cmd_buffer[2];
    cmd_buffer[0] = cmd >> 8;
    cmd_buffer[1] = cmd;
    bsp_I2C_master_transmit(STH31_I2C, SHT31_ADDR_WRITE, cmd_buffer, 2);
}


/**
 * @brief	��λSHT31
 * @param	none
 * @retval	none
*/
void SHT31_reset(void)
{
    SHT31_Send_Cmd(SOFT_RESET_CMD);
    ROV_Delay(20);
}

/**
 * @brief	��ʼ��SHT30
 * @param	none
 * @retval	none
 * @note	���ڲ���ģʽ
*/
void SHT31_Init(void)
{
		SHT31_reset();
		SHT31_Send_Cmd(MEDIUM_2_CMD);
}



#define CRC8_POLYNOMIAL 0x31
static uint8_t CheckCrc8(uint8_t* const message, uint8_t initial_value)
{
    uint8_t  remainder;	    //����
    uint8_t  i = 0, j = 0;  //ѭ������

    /* ��ʼ�� */
    remainder = initial_value;

    for(j = 0; j < 2;j++)
    {
        remainder ^= message[j];

        /* �����λ��ʼ���μ���  */
        for (i = 0; i < 8; i++)
        {
            if (remainder & 0x80)
            {
                remainder = (remainder << 1)^CRC8_POLYNOMIAL;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
    }

    /* ���ؼ����CRC�� */
    return remainder;
}

/**
 * @brief	��SHT31���յ�6���ֽ����ݽ���CRCУ�飬��ת��Ϊ�¶�ֵ��ʪ��ֵ
 * @param	dat  ���� �洢�������ݵĵ�ַ��6���ֽ����飩
 * @retval	У��ɹ�  ���� ����1
 * 			У��ʧ��  ���� ����0���������¶�ֵ��ʪ��ֵΪ0
*/
uint8_t SHT31_Dat_To_Float(uint8_t* const dat, float* temperature, float* humidity)
{
	uint16_t recv_temperature = 0;
	uint16_t recv_humidity = 0;
	
	/* У���¶����ݺ�ʪ�������Ƿ������ȷ */
	if(CheckCrc8(dat, 0xFF) != dat[2] || CheckCrc8(&dat[3], 0xFF) != dat[5])
		return 0;
	
	/* ת���¶����� */
	recv_temperature = ((uint16_t)dat[0]<<8)|dat[1];
	*temperature = -45 + 175*((float)recv_temperature/65535);
	
	/* ת��ʪ������ */
	recv_humidity = ((uint16_t)dat[3]<<8)|dat[4];
	*humidity = 100 * ((float)recv_humidity / 65535);
	
	return 1;
}

/**
 * @brief	��SHT31��ȡһ������
 * @param	dat ���� �洢��ȡ���ݵĵ�ַ��6���ֽ����飩
 * @retval	��ȡ�ɹ�  ���� ����0
 * 			��ȡʧ��  ���� ����1���������¶�ֵ��ʪ��ֵΪ0
*/
bool_t SHT31_Read_Dat(float* temperature, float* humidity)
{
	uint8_t dat[6];
	SHT31_Send_Cmd(MEDIUM_1_CMD);
	/*ʹ��Ӳ��IICʱ�����﷢������Ҫ��ʱһ��ʱ�䣬�������������*/
	ROV_Delay(5);
	bsp_I2C_master_receive(STH31_I2C, SHT31_ADDR_READ, dat, 6);
	return SHT31_Dat_To_Float(dat,temperature,humidity);
}

