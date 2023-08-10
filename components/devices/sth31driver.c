/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       sth31driver.c/h
  * @brief      这里是STH31温湿度模块的驱动函数，包括芯片的初始化，配置寄存器
	*							，完成发送和读取等
  * @note       STH31只支持IIC读取
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-12-2022     HaoLion(郝亮亮)    1. done
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
 * @brief	向SHT31发送一条指令(16bit)
 * @param	cmd —— SHT31指令（在SHT31_MODE中枚举定义）
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
 * @brief	复位SHT31
 * @param	none
 * @retval	none
*/
void SHT31_reset(void)
{
    SHT31_Send_Cmd(SOFT_RESET_CMD);
    ROV_Delay(20);
}

/**
 * @brief	初始化SHT30
 * @param	none
 * @retval	none
 * @note	周期测量模式
*/
void SHT31_Init(void)
{
		SHT31_reset();
		SHT31_Send_Cmd(MEDIUM_2_CMD);
}



#define CRC8_POLYNOMIAL 0x31
static uint8_t CheckCrc8(uint8_t* const message, uint8_t initial_value)
{
    uint8_t  remainder;	    //余数
    uint8_t  i = 0, j = 0;  //循环变量

    /* 初始化 */
    remainder = initial_value;

    for(j = 0; j < 2;j++)
    {
        remainder ^= message[j];

        /* 从最高位开始依次计算  */
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

    /* 返回计算的CRC码 */
    return remainder;
}

/**
 * @brief	将SHT31接收的6个字节数据进行CRC校验，并转换为温度值和湿度值
 * @param	dat  —— 存储接收数据的地址（6个字节数组）
 * @retval	校验成功  —— 返回1
 * 			校验失败  —— 返回0，并设置温度值和湿度值为0
*/
uint8_t SHT31_Dat_To_Float(uint8_t* const dat, float* temperature, float* humidity)
{
	uint16_t recv_temperature = 0;
	uint16_t recv_humidity = 0;
	
	/* 校验温度数据和湿度数据是否接收正确 */
	if(CheckCrc8(dat, 0xFF) != dat[2] || CheckCrc8(&dat[3], 0xFF) != dat[5])
		return 0;
	
	/* 转换温度数据 */
	recv_temperature = ((uint16_t)dat[0]<<8)|dat[1];
	*temperature = -45 + 175*((float)recv_temperature/65535);
	
	/* 转换湿度数据 */
	recv_humidity = ((uint16_t)dat[3]<<8)|dat[4];
	*humidity = 100 * ((float)recv_humidity / 65535);
	
	return 1;
}

/**
 * @brief	从SHT31读取一次数据
 * @param	dat —— 存储读取数据的地址（6个字节数组）
 * @retval	读取成功  —— 返回0
 * 			读取失败  —— 返回1，并设置温度值和湿度值为0
*/
bool_t SHT31_Read_Dat(float* temperature, float* humidity)
{
	uint8_t dat[6];
	SHT31_Send_Cmd(MEDIUM_1_CMD);
	/*使用硬件IIC时，这里发送完需要延时一段时间，否则读数据乱码*/
	ROV_Delay(5);
	bsp_I2C_master_receive(STH31_I2C, SHT31_ADDR_READ, dat, 6);
	return SHT31_Dat_To_Float(dat,temperature,humidity);
}

