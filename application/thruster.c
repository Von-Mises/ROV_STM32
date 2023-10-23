/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       thruster.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收推进器数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-11-2022     Qiqi Li(李琪琪)    1. done
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
  * @brief          预先开启所有电机，否则可能导致第一次下发命令时各个电机出现延迟情况
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
  * @brief          发送推进器控制帧
	* @param[in]      Motor_ID: 推进器ID
  * @param[in]      velocity: 控制推进器转速, 范围 [-3500,3500]
  * @param[in]      ACK_MODE: 设置回传模式  0X00：200ms自动上传     0xAA：收到指令回传
  * @retval         none
  */
void CAN_cmd_control(uint32_t Motor_ID,int16_t velocity, uint8_t ACK_MODE)
{
	int16_t speed;
    thruster_tx_message.Identifier = Motor_ID;											//控制帧ID	
    thruster_tx_message.IdType = FDCAN_STANDARD_ID;									//标准ID
    thruster_tx_message.TxFrameType = FDCAN_DATA_FRAME;							//数据帧
    thruster_tx_message.DataLength = FDCAN_DLC_BYTES_8;							//数据长度
    thruster_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    thruster_tx_message.BitRateSwitch = FDCAN_BRS_OFF;							//关闭数据波特率切换
    thruster_tx_message.FDFormat=FDCAN_CLASSIC_CAN;                 //传统的CAN模式
    thruster_tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;      //无发送事件
    thruster_tx_message.MessageMarker=0;
	memset(thruster_can_send_data,0,sizeof(thruster_can_send_data));
	if(velocity>0)
	{
		speed = velocity;
		_Limit(speed);										 //限幅
		thruster_can_send_data[0] = 0x01;  //开启电机
		thruster_can_send_data[1] = 0x00;	 //电机正转
	}
	else if(velocity==0)
	{
		speed = velocity;
		thruster_can_send_data[0] = 0x01;  //开启电机
		thruster_can_send_data[1] = 0x00;		
	}
	else if(velocity<0)
	{
		speed = -velocity;
		_Limit(speed);
		thruster_can_send_data[0] = 0x01;  //开启电机
		thruster_can_send_data[1] = 0x01;		//电机反转
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
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
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
		/*提取命令类型*/
		can_type = rx_header.Identifier >> 4;
		/*提取推进器编号*/
		thruster_id = rx_header.Identifier & 0x0f;
		/*推进器数据下标,由于推进器是从0x02开始编号*/
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
  * @brief          配置控制器参数
  * @param[in]      _parm: 控制器参数索引
  * @param[in]      value: 参数配置值
  * @retval         none
  */
void CAN_set_controler(controller_param_set_e _param, uint16_t value)
{
    thruster_tx_message.Identifier = CAN_CONTROL_PARAM_ID;					//控制参数配置帧ID	
    thruster_tx_message.IdType = FDCAN_STANDARD_ID;									//标准ID
    thruster_tx_message.TxFrameType = FDCAN_DATA_FRAME;							//数据帧
    thruster_tx_message.DataLength = FDCAN_DLC_BYTES_4;							//	数据长度
	thruster_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	thruster_tx_message.BitRateSwitch = FDCAN_BRS_OFF;							//关闭数据波特率切换
	thruster_tx_message.FDFormat=FDCAN_CLASSIC_CAN;                //传统的CAN模式
    thruster_tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //无发送事件
    thruster_tx_message.MessageMarker=0;
    thruster_can_send_data[0] = (_param >> 8);
	thruster_can_send_data[1] = _param;
	thruster_can_send_data[2] = (value >> 8);
	thruster_can_send_data[3] = value;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&thruster_tx_message,thruster_can_send_data);
}

/**
  * @brief          应答模式下请求电机参数回传
  * @param[in]      none
  * @retval         none
  */
void CAN_param_request(uint8_t Motor_ID)
{
    thruster_tx_message.Identifier = (CAN_MOTOR_PARAM_REQUEST_ID << 4) + (Motor_ID + 2);		//电机参数请求帧ID	
    thruster_tx_message.IdType = FDCAN_STANDARD_ID;									//标准ID
    thruster_tx_message.TxFrameType = FDCAN_DATA_FRAME;							//数据帧
    thruster_tx_message.DataLength = FDCAN_DLC_BYTES_2;							//	数据长度
	thruster_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	thruster_tx_message.BitRateSwitch = FDCAN_BRS_OFF;							//关闭数据波特率切换
	thruster_tx_message.FDFormat=FDCAN_CLASSIC_CAN;                //传统的CAN模式
    thruster_tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //无发送事件
    thruster_tx_message.MessageMarker=0;
    thruster_can_send_data[0] = 0x00;
	thruster_can_send_data[1] = 0x00;
	uint16_t i = 0;
	while(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&thruster_tx_message,thruster_can_send_data)!=HAL_OK)
	{
		i++;
		/*防止卡死*/
		if(i > 0xffff) break;
	}
}


/**
  * @brief          return the thruster motor data point
  * @param[in]      i: motor number,range [0,5]
  * @retval         motor data point
  */
/**
  * @brief          返回推进器数据指针
  * @param[in]      i: 电机编号,范围[0,5]
  * @retval         电机数据指针
  */
const motor_param_t *get_thruster_measure_point(uint8_t i)
{
    return &motor_param[(i % 6)];
}

