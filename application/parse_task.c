/**
  ****************************(C) COPYRIGHT 2023 ZJU****************************
  * @file       parse_task.c/h
  * @brief      socket receive data slove.
  *             上位机数据下发处理线程.        
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-5-2023     Qiqi Li(李琪琪)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 ZJU****************************
  */
	
#include "parse_task.h"
#include "main.h"
#include "cmsis_os.h"

#include "detect_task.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "comunication.h"
	
fifo_s_t tcp_fifo;
uint8_t tcp_fifo_buf[TCP_FIFO_BUF_LENGTH];
unpack_data_t tcp_unpack_obj;
	
	/**
  * @brief          parse task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          接收数据解析任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void parse_task(void const * argument)
{
	init_recieve_struct_data();
    fifo_s_init(&tcp_fifo, tcp_fifo_buf, TCP_FIFO_BUF_LENGTH);
	while(1)
	{
		tcp_unpack_fifo_data();
		osDelay(10);
	}
}



/**
  * @brief          tcp data upacked 
  * @param[in]      data	data of frame
  * @param[in]      length of data
  * @retval         none
  */
/**
  * @brief          数据解包
  * @param[in]      data 帧原始数据
  * @param[in]      framelen 帧长度
  * @retval         none
  */
uint8_t byte = 0;
void tcp_unpack_fifo_data()
{
	
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &tcp_unpack_obj;

  while ( fifo_s_used(&tcp_fifo) )
  {
    byte = fifo_s_get(&tcp_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_MSG_TYPE;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
			
	  case STEP_MSG_TYPE:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_LOW;
      }break;
			
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < REC_TEXT_MAX_SIZE)
        {
          p_obj->unpack_step = STEP_HEADER_CRC8;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REC_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REC_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REC_HEADER_CRC_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REC_HEADER_CRC_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REC_HEADER_CRC_LEN + p_obj->data_len))
          {
            receive_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}
