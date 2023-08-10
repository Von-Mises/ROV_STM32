 /**
 * ***************************@copyright (C) Copyright  2023  ZJU***************************
 * @file   DebugInfo.c/h
 * @brief  ����״̬��Ϣ��ʾ
 * @author Qiqi Li(������)
 * @version 1.0
 * @date 2023-08-3
 * 
 * @verbatim:
 * ==============================================================================
 *                                                                               
 * ==============================================================================
 * @endverbatim
 * ***************************@copyright (C) Copyright  2023  ZJU***************************
 */
 //ʹ��freertos��hal���дһ��������������CAN����
#include "CAN_receive.h"
#include <string.h>
#include "main.h"
#include "cmsis_os.h"

//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->speed_rpm = (uint16_t)((data)[0] << 8 | (data)[1]);      \
        (ptr)->given_voltage = ((uint16_t)((data)[2] << 8 | (data)[3])/10);  \
        (ptr)->given_current = ((uint16_t)((data)[4] << 8 | (data)[5])/10);  \
        (ptr)->controller_temperate = ((data)[6]-50);                        \
				(ptr)->motor_temperate = ((data)[7]-50); 														\
    }

extern FDCAN_HandleTypeDef hfdcan1;
static motor_param_t 				motor_param[6];

/**
 * @brief Function implementing the can_receive thread.
 * @param  argument         none
 */
void can_receive_task(void const * argument)
{
//  /* USER CODE BEGIN can_receive */
//  FDCAN_RxHeaderTypeDef  rx_header;
//  uint8_t rx_data[8];
//  uint16_t Motor_Status;
//  uint8_t can_type;
//  uint8_t thruster_id;
//  /* Infinite loop */
//  for(;;)
//  {
//    //�ж�CAN�Ľ���FIFO�Ƿ�Ϊ��
//    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
//    {
//      /* code */
//      HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, rx_data);
//      /*��ȡ��������*/
//      can_type = rx_header.Identifier >> 4;
//      /*��ȡ�ƽ������*/
//      thruster_id = rx_header.Identifier & 0x0f;
//      /*�ƽ��������±�,�����ƽ����Ǵ�0x02��ʼ���*/
//      uint8_t thruster_index = thruster_id - 0x02;
//      if(thruster_index > 6)
//        return;
//      switch(can_type)
//      {
//        case CAN_RETURN_ID:
//        {
//          Motor_Status = rx_data[0]<<8 | rx_data[1];
//          if(Motor_Status == 0x0000)
//          {
//            motor_param[thruster_index].motor_status = THRUSTER_OK;
//          }
//          else
//          {
//            motor_param[thruster_index].motor_status = THRUSTER_FAULT;
//          }
//          break;
//        }
//        case CAN_MOTOR_PARAM_ID:
//        {
//          get_motor_measure(&motor_param[thruster_index],rx_data);
//          break;
//        }
//        default :
//        {
//          break;
//        }
//      }

//    }
//    osDelay(200);
//  }
  /* USER CODE END can_receive */
}
