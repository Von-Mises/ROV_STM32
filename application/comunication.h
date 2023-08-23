/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       comunication.c/h
  * @brief      ����λ��ͨѶ���ݽ���         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-12-2022     Qiqi Li(������)    1. done
  *															ZhangYujio(���ھ�)
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#ifndef __COMUNICATION_H__
#define __COMUNICATION_H__


#include "struct_typedef.h"
#include "IMU_receive.h"
#include "pid.h"



/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct 
{
	int16_t thruster_speed[6];  //�ƽ����ٶȣ���0-6�ֱ�Ϊ���ϡ����ϡ����¡����¡���ࡢ�Ҳ�
    int16_t track_speed[2];			//�����Ĵ�������ٶ�
}motors_status_t;


typedef __packed struct 
{
    float cabin_temperature;//�����¶�
    float cabin_humidity;//����ʪ��
    float cabin_water_level;//����ˮλ
    float cpu_temperature;//CPU�¶�
}cabin_status_t;


typedef __packed struct 
{
    motors_status_t motors_status;	//����˶�״̬,16�ֽ�
    IMU_data_t IMU_data;        		//IMU��Ϣ,61�ֽ�
    float ROV_depth;                //�����Ϣ��4�ֽ�
	float ROV_height;								//�߶���Ϣ��4�ֽ�
    cabin_status_t cabin_status;  	//������Ϣ��16�ֽ�
    uint8_t run_mode;          			//�˶�ģʽ��1�ֽ�
}ext_rov_status_t;




typedef __packed struct 
{
    int P_set;
    int I_set;
    int D_set;
}PID_set_t;

typedef __packed struct 
{
    PID_set_t depth_loop_pid;											//���PID
    PID_set_t track_speed_pid;										//�Ĵ�����ջ�PID
    PID_set_t track_turn_loop_pid;								//�Ĵ�ģʽת��PID
    PID_set_t yaw_angle_pid;              				//yaw angle PID.Yaw�Ƕ�pid
	PID_set_t yaw_angular_velocity_pid;          	//yaw angular velocity PID.Yaw���ٶ�pid
	PID_set_t pitch_angle_pid;              			//pitch angle PID.Pitch�Ƕ�pid
	PID_set_t pitch_angular_velocity_pid;        	//pitch angular velocity PID.Pitch���ٶ�pid
	PID_set_t roll_angle_pid;              				//roll angle PID.Roll�Ƕ�pid
	PID_set_t roll_angular_velocity_pid;        	//roll angular velocity PID.Roll���ٶ�pid
}ext_PIDs_set_t;


typedef __packed struct 
{
    int YAW;
    int ROL;
    int PIT;
    int VF;
	int VY;
    int VZ;
	int8_t steering_gear_angle;//����Ƕȣ���ʼΪ0����Χ��-90~90
	uint8_t brightness;//���ȷ�Χ��0~100
	uint8_t Mode;//ROVģʽ
}ext_control_cmd_t;

typedef __packed struct
{
	float Depth;
	float Yaw;
//	float Roll;
//	float Pitch;
} ext_control_pos_t;

/* ----------------------- Extern Function ----------------------------------- */
/**
  * @brief          ���ݽ��
  * @param[in]      frame 	����������֡
  * @retval         none
  */
extern void receive_data_solve(uint8_t *frame);

/**
* @brief     �������ݴ��
* @param[in] cmd_type:  ��������ID
* @param[in] *p_data: ���ݶ�
* @param[in] len:     ���ݶγ���
* @retval			����Ҫ���͵����ݴ�С
*/
	
extern uint16_t send_data_pack(uint8_t cmd_type, uint8_t *p_data, uint16_t len);

/**
  * @brief          ����Ҫ�޸ĵ�pidֵ
  * @param[in]      pid   Ҫ��ȡ��pid
  * @param[in]      flag   ��һ����pid  0������PID����  1�������⻷PID����  2�������ڻ�PID����  3����תPID����
  * @retval         none
  */
extern void get_pids(pid_type_def* pid, uint8_t flag);


/**
  * @brief          �õ���������
  * @param[in]      none  
  * @retval         ctrl_cmd	Ҫ��ȡ���˶�����
  */
extern ext_control_cmd_t*  get_ctrl_cmd(void);

/**
  * @brief          �õ�λ�˿�������
  * @param[in]      none  
  * @retval         pos_control_cmd	Ҫ��ȡ���˶�����
  */
extern ext_control_pos_t*  get_pos_ctrl_cmd(void);

/**
  * @brief          �õ�ROV״̬���
  * @param[in]      none
  * @retval         rov_statusָ��
  */
extern ext_rov_status_t* get_rov_status(void);

extern const uint8_t* get_text_buffer(void);
extern void init_recieve_struct_data(void);
extern void init_send_struct_data(void);
#endif
