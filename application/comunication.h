/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       comunication.c/h
  * @brief      上下位机通讯数据解析         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-12-2022     Qiqi Li(李琪琪)    1. done
  *															ZhangYujio(张钰炯)
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
	int16_t thruster_speed[6];  //推进器速度，从0-6分别为左上、右上、左下、右下、左侧、右侧
    int16_t track_speed[2];			//左、右履带电机的速度
}motors_status_t;


typedef __packed struct 
{
    float cabin_temperature;//舱内温度
    float cabin_humidity;//舱内湿度
    float cabin_water_level;//舱内水位
    float cpu_temperature;//CPU温度
}cabin_status_t;


typedef __packed struct 
{
    motors_status_t motors_status;	//电机运动状态,16字节
    IMU_data_t IMU_data;        		//IMU信息,61字节
    float ROV_depth;                //深度信息，4字节
	float ROV_height;								//高度信息，4字节
    cabin_status_t cabin_status;  	//舱内信息，16字节
    uint8_t run_mode;          			//运动模式，1字节
}ext_rov_status_t;




typedef __packed struct 
{
    int P_set;
    int I_set;
    int D_set;
}PID_set_t;

typedef __packed struct 
{
    PID_set_t depth_loop_pid;											//定深环PID
    PID_set_t track_speed_pid;										//履带电机闭环PID
    PID_set_t track_turn_loop_pid;								//履带模式转向环PID
    PID_set_t yaw_angle_pid;              				//yaw angle PID.Yaw角度pid
	PID_set_t yaw_angular_velocity_pid;          	//yaw angular velocity PID.Yaw角速度pid
	PID_set_t pitch_angle_pid;              			//pitch angle PID.Pitch角度pid
	PID_set_t pitch_angular_velocity_pid;        	//pitch angular velocity PID.Pitch角速度pid
	PID_set_t roll_angle_pid;              				//roll angle PID.Roll角度pid
	PID_set_t roll_angular_velocity_pid;        	//roll angular velocity PID.Roll角速度pid
}ext_PIDs_set_t;


typedef __packed struct 
{
    int YAW;
    int ROL;
    int PIT;
    int VF;
	int VY;
    int VZ;
	int8_t steering_gear_angle;//舵机角度，初始为0，范围：-90~90
	uint8_t brightness;//亮度范围：0~100
	uint8_t Mode;//ROV模式
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
  * @brief          数据解包
  * @param[in]      frame 	解析的数据帧
  * @retval         none
  */
extern void receive_data_solve(uint8_t *frame);

/**
* @brief     发送内容打包
* @param[in] cmd_type:  命令内容ID
* @param[in] *p_data: 数据段
* @param[in] len:     数据段长度
* @retval			返回要发送的数据大小
*/
	
extern uint16_t send_data_pack(uint8_t cmd_type, uint8_t *p_data, uint16_t len);

/**
  * @brief          返回要修改的pid值
  * @param[in]      pid   要获取的pid
  * @param[in]      flag   哪一环的pid  0：定深PID设置  1：稳姿外环PID设置  2：稳姿内环PID设置  3：翻转PID设置
  * @retval         none
  */
extern void get_pids(pid_type_def* pid, uint8_t flag);


/**
  * @brief          得到控制命令
  * @param[in]      none  
  * @retval         ctrl_cmd	要获取的运动命令
  */
extern ext_control_cmd_t*  get_ctrl_cmd(void);

/**
  * @brief          得到位姿控制命令
  * @param[in]      none  
  * @retval         pos_control_cmd	要获取的运动命令
  */
extern ext_control_pos_t*  get_pos_ctrl_cmd(void);

/**
  * @brief          得到ROV状态句柄
  * @param[in]      none
  * @retval         rov_status指针
  */
extern ext_rov_status_t* get_rov_status(void);

extern const uint8_t* get_text_buffer(void);
extern void init_recieve_struct_data(void);
extern void init_send_struct_data(void);
#endif
