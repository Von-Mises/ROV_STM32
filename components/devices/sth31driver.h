/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       sth31driver.c/h
  * @brief      这里是STH31温湿度模块的驱动函数，包括芯片的初始化，配置寄存器
	*							，完成发送和读取等
  * @note       STH1只支持IIC读取
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

#ifndef STH31DRIVER_H
#define STH31DRIVER_H

#include "struct_typedef.h"

#define SHT31_ADDR_WRITE 0x88  //ADDR (pin 2)connected to VDD
#define SHT31_ADDR_READ 0x89  
#define STH31_I2C    I2C2

/* ----------------------- Data Struct ------------------------------------- */
typedef enum
{
    /* 软件复位命令 */

    SOFT_RESET_CMD = 0x30A2,	
    /*
    单次测量模式
    命名格式：Repeatability_CS_CMD
    CS： Clock stretching
    */
    HIGH_ENABLED_CMD    = 0x2C06,
    MEDIUM_ENABLED_CMD  = 0x2C0D,
    LOW_ENABLED_CMD     = 0x2C10,
    HIGH_DISABLED_CMD   = 0x2400,
    MEDIUM_DISABLED_CMD = 0x240B,
    LOW_DISABLED_CMD    = 0x2416,

    /*
    周期测量模式
    命名格式：Repeatability_MPS_CMD
    MPS：measurement per second
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
	/* 周期测量模式读取数据命令 */
	READOUT_FOR_PERIODIC_MODE = 0xE000,
} SHT31_CMD;

/* ----------------------- Extern Function ----------------------------------- */
/**
 * @brief	初始化SHT30
 * @param	none
 * @retval	none
 * @note	周期测量模式
*/
extern void SHT31_Init(void);

/**
 * @brief	从SHT31读取一次数据
 * @param	dat —— 存储读取数据的地址（6个字节数组）
 * @retval	读取成功  —— 返回1
 * 			读取失败  —— 返回0，并设置温度值和湿度值为0
*/
extern bool_t SHT31_Read_Dat(float* temperature, float* humidity);


#endif
