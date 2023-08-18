#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#include "struct_typedef.h"


#define REC_TEXT_MAX_SIZE         					249											//数据段最大的长度
#define REC_PROTOCOL_HEADER_SIZE    sizeof(frame_header_struct_t)		//帧头长度
#define HEADER_SOF 													0xA5										//固定起始字符
#define REC_PROTOCOL_FRAME_MAX_SIZE          256    								//上传数据最大的长度									  									
#define REC_PROTOCOL_CRC16_SIZE          			2     								//CRC16校验	
#define REC_HEADER_CRC_LEN										7											//头部和CRC16的长度

#pragma pack(push, 1)

typedef enum
{
	ROV_STATUS_ID = 0X01,						//下位机上传时代表ROV运动状态
	TEXT_MSG_ID = 0X02,						//发送文本消息便于调试
}msg_type_t;

typedef enum
{
	MOV_CMD_ID = 0X01,						//运动控制命令
	SET_PID_ID = 0X02,						//设置PID命令
	SET_ATT_ID = 0X03,
}cmd_type_t;

typedef  struct
{
  uint8_t SOF;
  uint8_t msg_type;
  uint16_t data_length;
  uint8_t CRC8;
} frame_header_struct_t;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_MSG_TYPE	   = 1,
  STEP_LENGTH_LOW  = 2,
  STEP_LENGTH_HIGH = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_tcp_step_e;


typedef struct
{
  frame_header_struct_t *p_header;                             //协议帧头结构体
  uint16_t       data_len;									   //已解析数据长度
  uint8_t        protocol_packet[REC_PROTOCOL_FRAME_MAX_SIZE]; //接收数据帧最大长度
  unpack_tcp_step_e  unpack_step;                              //解包状态标志
  uint16_t       index;			                               //解包数据所存数组下标								
} unpack_data_t;



#pragma pack(pop)

#endif //_PROTOCOL_H
