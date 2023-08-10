/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       IMU_receive.c/h
  * @brief      解析IMU发送过来的数据,利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-23-2022     HaoLion(郝亮亮)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#ifndef IMU_RECEIVE_H
#define IMU_RECEIVE_H

#include "struct_typedef.h"


#define IMU_HEADER_SOF 				0xA55A
#define DATA_EOF					0xAA

#define IMU_CMD_LENGTH 		6
#define IMU_FRAME_LENGTH 	80
#define IMU_DATA_LENGTH 	40


/* ----------------------- Data Struct ------------------------------------- */

typedef __packed struct
{
	float yaw;					//姿态角，单位为°
	float pitch;
	float roll;
	float ax;						//加速度数据，单位为g
	float ay;
	float az;
	float gx;						//角速度数据，单位为°/s
	float gy;
	float gz;
	float mx;						//磁力计数据，单位为Gs
	float my;
	float mz;
	int32_t press;			//气压计，缺省
	float temperate;		//温度，单位为℃
	int32_t timeSample;	//时间戳，缺省
	uint8_t m_flag;			//磁状态标志，缺省
} IMU_data_t;	

typedef enum
{
    START_CMD_ID                 		= 0x01,
    STOP_CMD_ID                			= 0x02,
    CALIBRATION_CMD_ID             	= 0xE3,
    SAVE_CALIBRATION_CMD_ID         = 0xE1,
    OPEN_MAGNETIC_CORRECT_CMD_ID   	= 0xE2,
    CLOSE_MAGNETIC_CORRECT_CMD_ID  	= 0xE4,
}imu_cmd_id_e;

/* ----------------------- extern Function ----------------------------------- */
extern void IMU_Set_Cmd(imu_cmd_id_e imu_cmd_id);
extern const IMU_data_t *get_imu_data_point(void);
extern void IMU_init(void);

#endif
