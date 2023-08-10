/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       comunication.c/h
  * @brief      ����λ��ͨѶ���ݽ���         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-12-2022     HaoLion(������)    1. done
  *															ZhangYujio(���ھ�)
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#include "comunication.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"


frame_header_struct_t rov_recieve_header;
frame_header_struct_t rov_send_header;

ext_rov_status_t rov_status;
ext_control_cmd_t control_cmd;
ext_control_pos_t pos_control_cmd;
ext_PIDs_set_t PIDs_set;
uint8_t text_buffer[REC_TEXT_MAX_SIZE];


void init_recieve_struct_data(void)
{
    memset(&rov_recieve_header, 0, sizeof(frame_header_struct_t));
	
	memset(&control_cmd, 0, sizeof(ext_control_cmd_t));
	
	memset(&pos_control_cmd, 0, sizeof(pos_control_cmd));

	memset(&PIDs_set, 0, sizeof(ext_PIDs_set_t));
}

void init_send_struct_data(void)
{
    memset(&rov_send_header, 0, sizeof(frame_header_struct_t));

    memset(&rov_status, 0, sizeof(ext_rov_status_t));
	
	memset(text_buffer, 0, sizeof(REC_TEXT_MAX_SIZE));
}


const uint8_t* get_text_buffer(void)
{
	return text_buffer;
}


/**
  * @brief          ��������
  * @param[in]      frame 	����������֡
  * @retval         none
  */
void receive_data_solve(uint8_t *frame)
{
    uint16_t msg_type = 0;
    uint8_t index = 0;

    memcpy(&rov_recieve_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    msg_type = rov_recieve_header.msg_type;
	
    switch (msg_type)
    {
        case MOV_CMD_ID:
        {
            memcpy(&control_cmd, frame + index, sizeof(ext_control_cmd_t));
        }
        break;
        case SET_PID_ID:
        {
            memcpy(&PIDs_set, frame + index, sizeof(ext_PIDs_set_t));
        }
        break;
		case SET_POS_ID:
		{
			memcpy(&pos_control_cmd, frame + index, sizeof(ext_control_pos_t));
		}
		break;
        default:
        {
            break;
        }
    }
}

/**
  * @brief          ����Ҫ�޸ĵ�pidֵ
  * @param[in]      pid   Ҫ��ȡ��pid
  * @param[in]      flag   ��һ����pid  
  * @retval         none
  */
void get_pids(pid_type_def* pid, uint8_t flag)
{
	PID_set_t pid_set;
	switch (flag)
    {
        case 0:
        {
            pid_set = PIDs_set.depth_loop_pid;
			break;
        }
        case 1:
        {
			pid_set = PIDs_set.track_speed_pid;
			break;
        } 
		case 2:
        {
			pid_set = PIDs_set.track_turn_loop_pid;
			break;
        }
		case 3:
        {
			pid_set = PIDs_set.yaw_angle_pid;
			break;
        }
		case 4:
		{
			pid_set = PIDs_set.yaw_angular_velocity_pid;
			break;
		}
		case 5:
        {
			pid_set = PIDs_set.pitch_angle_pid;
			break;
        }
		case 6:
		{
			pid_set = PIDs_set.pitch_angular_velocity_pid;
			break;
		}
		case 7:
        {
			pid_set = PIDs_set.roll_angle_pid;
			break;
        }
		case 8:
		{
			pid_set = PIDs_set.roll_angular_velocity_pid;
			break;
		}
        default:
        {
            break;					
        }
    }
	
	pid->Kp = pid_set.P_set;
	pid->Ki = pid_set.I_set;
	pid->Kd = pid_set.D_set;
}


/**
  * @brief          �õ���������
  * @param[in]      none  
  * @retval         ctrl_cmd	Ҫ��ȡ���˶�����
  */
ext_control_cmd_t*  get_ctrl_cmd(void)
{
	return &control_cmd;
}

/**
  * @brief          �õ�λ�˿�������
  * @param[in]      none  
  * @retval         pos_control_cmd	Ҫ��ȡ���˶�����
  */
ext_control_pos_t*  get_pos_ctrl_cmd(void)
{
	return &pos_control_cmd;
}

/**
  * @brief          �õ�ROV״̬���
  * @param[in]      none
  * @retval         rov_statusָ��
  */
ext_rov_status_t* get_rov_status()
{
	return &rov_status;
}



/**
* @brief     �������ݴ��
* @param[in] cmd_type:  ��������ID
* @param[in] *p_data: ���ݶ�
* @param[in] len:     ���ݶγ���
* @retval			����Ҫ���͵����ݴ�С
*/
	
uint16_t send_data_pack(uint8_t cmd_type, uint8_t *p_data, uint16_t len)
{
	/*�ı����ݴ�С����*/
	if(len > REC_TEXT_MAX_SIZE)
	{
		len = REC_TEXT_MAX_SIZE;
	}
	uint16_t frame_length = REC_PROTOCOL_HEADER_SIZE  + len + REC_PROTOCOL_CRC16_SIZE;   //����֡����	

	memset(text_buffer,0,frame_length);  //�洢���ݵ���������
	
	/*****֡ͷ���*****/
	text_buffer[0] = HEADER_SOF;//����֡��ʼ�ֽ�
	text_buffer[1] = cmd_type;//������������
	memcpy(&text_buffer[2],(uint8_t*)&len,2);//����֡��data�ĳ���
	append_CRC8_check_sum(text_buffer,REC_PROTOCOL_HEADER_SIZE);  //֡ͷУ��CRC8

	/*****���ݴ��*****/
	memcpy(&text_buffer[REC_PROTOCOL_HEADER_SIZE], p_data, len);
	append_CRC16_check_sum(text_buffer,frame_length);  //һ֡����У��CRC16	
	return REC_HEADER_CRC_LEN+len;
}



