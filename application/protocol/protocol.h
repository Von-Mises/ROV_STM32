#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#include "struct_typedef.h"


#define REC_TEXT_MAX_SIZE         					249											//���ݶ����ĳ���
#define REC_PROTOCOL_HEADER_SIZE    sizeof(frame_header_struct_t)		//֡ͷ����
#define HEADER_SOF 													0xA5										//�̶���ʼ�ַ�
#define REC_PROTOCOL_FRAME_MAX_SIZE          256    								//�ϴ��������ĳ���									  									
#define REC_PROTOCOL_CRC16_SIZE          			2     								//CRC16У��	
#define REC_HEADER_CRC_LEN										7											//ͷ����CRC16�ĳ���

#pragma pack(push, 1)

typedef enum
{
	ROV_STATUS_ID = 0X01,						//��λ���ϴ�ʱ����ROV�˶�״̬
	TEXT_MSG_ID = 0X02,						//�����ı���Ϣ���ڵ���
}msg_type_t;

typedef enum
{
	MOV_CMD_ID = 0X01,						//�˶���������
	SET_PID_ID = 0X02,						//����PID����
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
  frame_header_struct_t *p_header;                             //Э��֡ͷ�ṹ��
  uint16_t       data_len;									   //�ѽ������ݳ���
  uint8_t        protocol_packet[REC_PROTOCOL_FRAME_MAX_SIZE]; //��������֡��󳤶�
  unpack_tcp_step_e  unpack_step;                              //���״̬��־
  uint16_t       index;			                               //����������������±�								
} unpack_data_t;



#pragma pack(pop)

#endif //_PROTOCOL_H
