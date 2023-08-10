/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       IMU_receive.c/h
  * @brief      ����IMU���͹���������,����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж�������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-23-2022     HaoLion(������)    1. done
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
	float yaw;					//��̬�ǣ���λΪ��
	float pitch;
	float roll;
	float ax;						//���ٶ����ݣ���λΪg
	float ay;
	float az;
	float gx;						//���ٶ����ݣ���λΪ��/s
	float gy;
	float gz;
	float mx;						//���������ݣ���λΪGs
	float my;
	float mz;
	int32_t press;			//��ѹ�ƣ�ȱʡ
	float temperate;		//�¶ȣ���λΪ��
	int32_t timeSample;	//ʱ�����ȱʡ
	uint8_t m_flag;			//��״̬��־��ȱʡ
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
