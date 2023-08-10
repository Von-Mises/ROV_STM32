/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       comunication.c/h
  * @brief      上下位机通讯数据解析         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-12-2022     HaoLion(郝亮亮)    1. done
  *															ZhangYujio(张钰炯)
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
  * @brief          解析数据
  * @param[in]      frame 	解析的数据帧
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
  * @brief          返回要修改的pid值
  * @param[in]      pid   要获取的pid
  * @param[in]      flag   哪一环的pid  
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
  * @brief          得到控制命令
  * @param[in]      none  
  * @retval         ctrl_cmd	要获取的运动命令
  */
ext_control_cmd_t*  get_ctrl_cmd(void)
{
	return &control_cmd;
}

/**
  * @brief          得到位姿控制命令
  * @param[in]      none  
  * @retval         pos_control_cmd	要获取的运动命令
  */
ext_control_pos_t*  get_pos_ctrl_cmd(void)
{
	return &pos_control_cmd;
}

/**
  * @brief          得到ROV状态句柄
  * @param[in]      none
  * @retval         rov_status指针
  */
ext_rov_status_t* get_rov_status()
{
	return &rov_status;
}



/**
* @brief     发送内容打包
* @param[in] cmd_type:  命令内容ID
* @param[in] *p_data: 数据段
* @param[in] len:     数据段长度
* @retval			返回要发送的数据大小
*/
	
uint16_t send_data_pack(uint8_t cmd_type, uint8_t *p_data, uint16_t len)
{
	/*文本数据大小限制*/
	if(len > REC_TEXT_MAX_SIZE)
	{
		len = REC_TEXT_MAX_SIZE;
	}
	uint16_t frame_length = REC_PROTOCOL_HEADER_SIZE  + len + REC_PROTOCOL_CRC16_SIZE;   //数据帧长度	

	memset(text_buffer,0,frame_length);  //存储数据的数组清零
	
	/*****帧头打包*****/
	text_buffer[0] = HEADER_SOF;//数据帧起始字节
	text_buffer[1] = cmd_type;//发送数据类型
	memcpy(&text_buffer[2],(uint8_t*)&len,2);//数据帧中data的长度
	append_CRC8_check_sum(text_buffer,REC_PROTOCOL_HEADER_SIZE);  //帧头校验CRC8

	/*****数据打包*****/
	memcpy(&text_buffer[REC_PROTOCOL_HEADER_SIZE], p_data, len);
	append_CRC16_check_sum(text_buffer,frame_length);  //一帧数据校验CRC16	
	return REC_HEADER_CRC_LEN+len;
}



