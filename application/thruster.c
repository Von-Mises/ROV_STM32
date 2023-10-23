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

#include "thruster.h"
#include <string.h>
#include "main.h"
#include "config.h"
#include <stdio.h>


extern FDCAN_HandleTypeDef hfdcan1;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->speed_rpm = (uint16_t)((data)[0] << 8 | (data)[1]);      \
        (ptr)->given_voltage = ((float)((data)[2] << 8 | (data)[3])/10);  \
        (ptr)->given_current = ((float)((data)[4] << 8 | (data)[5])/10);  \
        (ptr)->controller_temperate = ((data)[6]-50);                        \
		(ptr)->motor_temperate = ((data)[7]-50); 														\
    }

#define max 2400		
#define _Limit(var)       (var = (((var) < (max)) ? (var) : (max)))
		
/*
* 
*/
static FDCAN_TxHeaderTypeDef  thruster_tx_message;
static motor_param_t 				motor_param[6];
static uint8_t              thruster_can_send_data[8];

		
/**
  * @brief          Ԥ�ȿ������е����������ܵ��µ�һ���·�����ʱ������������ӳ����
	* @param[in]    none
  * @retval         none
  */
void Thruster_Init(void)
{
		for (int i = 0; i < 6; i++)
		{
			CAN_cmd_control(CAN_M1_ID+i,0,0xAA);	
			HAL_Delay(1);
		}
		
}
	
/**
  * @brief          �����ƽ�������֡
	* @param[in]      Motor_ID: �ƽ���ID
  * @param[in]      velocity: �����ƽ���ת��, ��Χ [-3500,3500]
  * @param[in]      ACK_MODE: ���ûش�ģʽ  0X00��200ms�Զ��ϴ�     0xAA���յ�ָ��ش�
  * @retval         none
  */
void CAN_cmd_control(uint32_t Motor_ID,int16_t velocity, uint8_t ACK_MODE)
{
	int16_t speed;
    thruster_tx_message.Identifier = Motor_ID;											//����֡ID	
    thruster_tx_message.IdType = FDCAN_STANDARD_ID;									//��׼ID
    thruster_tx_message.TxFrameType = FDCAN_DATA_FRAME;							//����֡
    thruster_tx_message.DataLength = FDCAN_DLC_BYTES_8;							//���ݳ���
    thruster_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    thruster_tx_message.BitRateSwitch = FDCAN_BRS_OFF;							//�ر����ݲ������л�
    thruster_tx_message.FDFormat=FDCAN_CLASSIC_CAN;                 //��ͳ��CANģʽ
    thruster_tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;      //�޷����¼�
    thruster_tx_message.MessageMarker=0;
	memset(thruster_can_send_data,0,sizeof(thruster_can_send_data));
	if(velocity>0)
	{
		speed = velocity;
		_Limit(speed);										 //�޷�
		thruster_can_send_data[0] = 0x01;  //�������
		thruster_can_send_data[1] = 0x00;	 //�����ת
	}
	else if(velocity==0)
	{
		speed = velocity;
		thruster_can_send_data[0] = 0x01;  //�������
		thruster_can_send_data[1] = 0x00;		
	}
	else if(velocity<0)
	{
		speed = -velocity;
		_Limit(speed);
		thruster_can_send_data[0] = 0x01;  //�������
		thruster_can_send_data[1] = 0x01;		//�����ת
	}
    thruster_can_send_data[2] = (speed >> 8);
	thruster_can_send_data[3] = speed;
	thruster_can_send_data[7] = 0x00;
  	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&thruster_tx_message,thruster_can_send_data);
}


uint8_t can_type;
uint8_t thruster_id;
uint8_t rx_data[8];
/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs&FDCAN_IT_RX_FIFO0_NEW_MESSAGE)!= RESET)
	{
		FDCAN_RxHeaderTypeDef  rx_header;
		
		uint16_t Motor_Status;
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
		//HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
		/*��ȡ��������*/
		can_type = rx_header.Identifier >> 4;
		/*��ȡ�ƽ������*/
		thruster_id = rx_header.Identifier & 0x0f;
		/*�ƽ��������±�,�����ƽ����Ǵ�0x02��ʼ���*/
		uint8_t thruster_index = thruster_id - 0x02;
		if(thruster_index > 6) return;
		switch(can_type)
		{
			case CAN_RETURN_ID:
			{
				Motor_Status = rx_data[0]<<8 | rx_data[1];
				if(Motor_Status == 0x0000)
				{
					motor_param[thruster_index].motor_status = THRUSTER_OK;
				}
				else
				{
					motor_param[thruster_index].motor_status = THRUSTER_FAULT;
				}
				break;
			}
			case CAN_MOTOR_PARAM_ID:
			{
				get_motor_measure(&motor_param[thruster_index],rx_data);
				printf("ID:7 current: %f",motor_param[5].given_current);
				break;
			}
			default :
			{
				break;
			}
		}
	}
}


/**
  * @brief          ���ÿ���������
  * @param[in]      _parm: ��������������
  * @param[in]      value: ��������ֵ
  * @retval         none
  */
void CAN_set_controler(controller_param_set_e _param, uint16_t value)
{
    thruster_tx_message.Identifier = CAN_CONTROL_PARAM_ID;					//���Ʋ�������֡ID	
    thruster_tx_message.IdType = FDCAN_STANDARD_ID;									//��׼ID
    thruster_tx_message.TxFrameType = FDCAN_DATA_FRAME;							//����֡
    thruster_tx_message.DataLength = FDCAN_DLC_BYTES_4;							//	���ݳ���
	thruster_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	thruster_tx_message.BitRateSwitch = FDCAN_BRS_OFF;							//�ر����ݲ������л�
	thruster_tx_message.FDFormat=FDCAN_CLASSIC_CAN;                //��ͳ��CANģʽ
    thruster_tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //�޷����¼�
    thruster_tx_message.MessageMarker=0;
    thruster_can_send_data[0] = (_param >> 8);
	thruster_can_send_data[1] = _param;
	thruster_can_send_data[2] = (value >> 8);
	thruster_can_send_data[3] = value;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&thruster_tx_message,thruster_can_send_data);
}

/**
  * @brief          Ӧ��ģʽ�������������ش�
  * @param[in]      none
  * @retval         none
  */
void CAN_param_request(uint8_t Motor_ID)
{
    thruster_tx_message.Identifier = (CAN_MOTOR_PARAM_REQUEST_ID << 4) + (Motor_ID + 2);		//�����������֡ID	
    thruster_tx_message.IdType = FDCAN_STANDARD_ID;									//��׼ID
    thruster_tx_message.TxFrameType = FDCAN_DATA_FRAME;							//����֡
    thruster_tx_message.DataLength = FDCAN_DLC_BYTES_2;							//	���ݳ���
	thruster_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	thruster_tx_message.BitRateSwitch = FDCAN_BRS_OFF;							//�ر����ݲ������л�
	thruster_tx_message.FDFormat=FDCAN_CLASSIC_CAN;                //��ͳ��CANģʽ
    thruster_tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //�޷����¼�
    thruster_tx_message.MessageMarker=0;
    thruster_can_send_data[0] = 0x00;
	thruster_can_send_data[1] = 0x00;
	uint16_t i = 0;
	while(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&thruster_tx_message,thruster_can_send_data)!=HAL_OK)
	{
		i++;
		/*��ֹ����*/
		if(i > 0xffff) break;
	}
}


/**
  * @brief          return the thruster motor data point
  * @param[in]      i: motor number,range [0,5]
  * @retval         motor data point
  */
/**
  * @brief          �����ƽ�������ָ��
  * @param[in]      i: ������,��Χ[0,5]
  * @retval         �������ָ��
  */
const motor_param_t *get_thruster_measure_point(uint8_t i)
{
    return &motor_param[(i % 6)];
}

