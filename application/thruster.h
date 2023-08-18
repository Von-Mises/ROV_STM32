/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       thruster.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ����������ƽ�������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-11-2022     Qiqi Li(������)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#ifndef __THRUSTER_H__
#define __THRUSTER_H__

#include "struct_typedef.h"


/* thruster status */
typedef enum
{
		THRUSTER_OK=0,			//�ƽ����޹���
		THRUSTER_FAULT=1,		//�ƽ�������
} motor_flaut_status_e;

/* CAN send and receive ID */
typedef enum
{
    CAN_M1_ID=0x202,										//����֡
		CAN_M2_ID=0x203,										//����֡
		CAN_M3_ID=0x204,										//����֡
		CAN_M4_ID=0x205,										//����֡
		CAN_M5_ID=0x206,										//����֡
		CAN_M6_ID=0x207,										//����֡
		CAN_RETURN_ID=0x18,									//ȷ�Ϸ���֡
        CAN_MOTOR_PARAM_REQUEST_ID=0x30,		//�����������֡
		CAN_MOTOR_PARAM_ID=0x38,						//�������֡
		CAN_CONTROL_PARAM_ID=0x60,					//���Ʋ�������֡	
} can_msg_id_e;

//motor data
typedef struct
{
    uint16_t speed_rpm;								//���ת��
		float given_voltage;						//ĸ�ߵ�ѹ
    float given_current;						//ĸ�ߵ���
    uint8_t controller_temperate;			//�������¶�
    uint8_t motor_temperate;					//����¶�
		uint8_t motor_status;							//���״̬
} motor_param_t;

/* Controller parameters Index */
typedef enum
{
    MODE=0x0001,									//���ÿ���ģʽ��00Ϊģ��������
		POLE_COUPLE=0x0003,						//���õ���ļ�������Ĭ��Ϊ3���������
    TEMPERATE_PROTECT=0x0008,			//������±�����Ĭ��Ϊ70��
		Speed_Kp=0x000D,							//�ٶȻ�Kp��Ĭ��3000
		Speed_Ki=0x000E,							//�ٶȻ�Ki��Ĭ��20	
		Current_Kp=0x000F,						//������Kp��Ĭ��100
		Current_Ki=0x0010,						//������Ki��Ĭ��20
		CAN_Buadrate=0x0014,					//CANͨѶ���������� 4:1M 8:500K 16:250K
		CAN_Upload=0x0015,						//CAN�ϴ���� Ĭ��250ms
		CAN_Timeout=0x0016,						//CANͨѶ��ʱʱ�� Ĭ��3000ms
		Init_Angle=0x0020,						//��ʼ��ѧϰ
		Factory_Reset=0xEEEE,					//�ָ�Ĭ�ϳ�������
		Save_Param=0xFFFF,						//������������ò���
} controller_param_set_e;



/**
  * @brief          Ԥ�ȿ������е����������ܵ��µ�һ���·�����ʱ������������ӳ����
	* @param[in]      none
  * @retval         none
  */
extern void Thruster_Init(void);
/**
  * @brief          �����ƽ�������֡
	* @param[in]      Motor_ID: �ƽ���ID
  * @param[in]      velocity: �����ƽ���ת��, ��Χ [-3500,3500]
  * @param[in]      ACK_MODE: ���ûش�ģʽ  0X00��250ms�Զ��ϴ�     0xAA���յ�ָ��ش�
  * @retval         none
  */
extern void CAN_cmd_control(uint32_t Motor_ID,int16_t velocity, uint8_t ACK_MODE);

/**
  * @brief          ���ÿ���������
  * @param[in]      _parm: ��������������
  * @param[in]      value: ��������ֵ
  * @retval         none
  */
extern void CAN_set_controler(controller_param_set_e _param, uint16_t value);

/**
  * @brief          Ӧ��ģʽ�������������ش�
  * @param[in]      none
  * @retval         none
  */
extern void CAN_param_request(uint8_t Motor_ID);


/**
  * @brief          �����ƽ�������ָ��
  * @param[in]      i: ������,��Χ[0,5]
  * @retval         �������ָ��
  */
extern const motor_param_t *get_thruster_measure_point(uint8_t i);

#endif
