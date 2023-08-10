/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       rov_movation_task.c/h
  * @brief      控制ROV运动状态主任务         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-14-2022     HaoLion(郝亮亮)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "comunication.h"
#include "user_lib.h"


//in the beginning of task ,wait a time
//任务开始空闲一段时间
#define ROV_TASK_INIT_TIME 357
//rov movation task control time  2ms
//ROV运动任务控制间隔 5ms
#define ROV_CONTROL_TIME_MS 5

//运动任务控制间隔 0.002s
#define ROV_CONTROL_TIME 0.002f

#define ROV_ACCEL_YAW_NUM 0.1666666667f
#define ROV_ACCEL_ROL_NUM 0.3333333333f


//rocker value deadline
//摇杆死区
#define ROV_RC_DEADLINE 5

//履带模式下，遥控器的yaw遥杆转变为旋转速度的比例
#define ROV_ANGLE_TO_WZ_SEN  5//0.002f



//rov forward or back max speed
//rov运动过程最大航行速度
#define NORMAL_MAX_ROV_SPEED_F 2.0f
//rov rise or sink max speed
//rov运动过程最大沉浮速度
#define NORMAL_MAX_ROV_SPEED_Z 1.5f

//水平推进器到ROV中心的虚拟距离 单位m
#define THRUSTER_DISTANCE_TO_CENTER  0.3f

//履带到ROV中心的虚拟距离 单位m
#define TRACK_DISTANCE_TO_CENTER  0.2f

//depth hold loop PID
//定深环pid值
#define ROV_DEPTH_PID_KP 2000.0f
#define ROV_DEPTH_PID_KI 0.0f
#define ROV_DEPTH_PID_KD 0.0f
#define ROV_DEPTH_PID_MAX_OUT 500.0f
#define ROV_DEPTH_PID_MAX_IOUT 0.2f

//track mode turn loop PID
//履带模式下转向环PID
#define TRACK_WZ_PID_KP 0.0f
#define TRACK_WZ_PID_KI 0.0f
#define TRACK_WZ_PID_KD 0.0f
#define TRACK_WZ_PID_MAX_OUT  0.0f
#define TRACK_WZ_PID_MAX_IOUT 0.0f

//track motor velocity loop PID
//履带电机速度环PID
#define TRACK_MOTOR_SPEED_PID_KP 0.0f
#define TRACK_MOTOR_SPEED_PID_KI 0.0f
#define TRACK_MOTOR_SPEED_PID_KD 0.0f
#define TRACK_MOTOR_SPEED_PID_MAX_OUT  0.0f
#define TRACK_MOTOR_SPEED_PID_MAX_IOUT 0.0f

//yaw angle PID
//Yaw角度pid
#define ROV_YAW_ANGLE_PID_KP 200.0f
#define ROV_YAW_ANGLE_PID_KI 0.0f
#define ROV_YAW_ANGLE_PID_KD 0.0f
#define ROV_YAW_ANGLE_PID_MAX_OUT 500.0f
#define ROV_YAW_ANGLE_PID_MAX_IOUT 0.2f

//yaw angular velocity PID
//Yaw角速度pid
#define ROV_YAW_ANGULAR_VELOCITY_PID_KP 200.0f
#define ROV_YAW_ANGULAR_VELOCITY_PID_KI 0.0f
#define ROV_YAW_ANGULAR_VELOCITY_PID_KD 0.0f
#define ROV_YAW_ANGULAR_VELOCITY_PID_MAX_OUT 500.0f
#define ROV_YAW_ANGULAR_VELOCITY_PID_MAX_IOUT 0.2f 

//pitch angle PID
//Pitch角度pid
#define ROV_PITCH_ANGLE_PID_KP 0.0f
#define ROV_PITCH_ANGLE_PID_KI 0.0f
#define ROV_PITCH_ANGLE_PID_KD 0.0f
#define ROV_PITCH_ANGLE_PID_MAX_OUT 0.0f
#define ROV_PITCH_ANGLE_PID_MAX_IOUT 0.0f

//pitch angular velocity PID
//Pitch角速度pid
#define ROV_PITCH_ANGULAR_VELOCITY_PID_KP 0.0f
#define ROV_PITCH_ANGULAR_VELOCITY_PID_KI 0.0f
#define ROV_PITCH_ANGULAR_VELOCITY_PID_KD 0.0f
#define ROV_PITCH_ANGULAR_VELOCITY_PID_MAX_OUT 0.0f
#define ROV_PITCH_ANGULAR_VELOCITY_PID_MAX_IOUT 0.0f

//roll angle PID
//Roll角度pid
#define ROV_ROLL_ANGLE_PID_KP 0.0f
#define ROV_ROLL_ANGLE_PID_KI 0.0f
#define ROV_ROLL_ANGLE_PID_KD 0.0f
#define ROV_ROLL_ANGLE_PID_MAX_OUT 0.0f
#define ROV_ROLL_ANGLE_PID_MAX_IOUT 0.0f

//roll angular velocity PID
//Roll角速度pid
#define ROV_ROLL_ANGULAR_VELOCITY_PID_KP 0.0f
#define ROV_ROLL_ANGULAR_VELOCITY_PID_KI 0.0f
#define ROV_ROLL_ANGULAR_VELOCITY_PID_KD 0.0f
#define ROV_ROLL_ANGULAR_VELOCITY_PID_MAX_OUT 0.0f
#define ROV_ROLL_ANGULAR_VELOCITY_PID_MAX_IOUT 0.0f


/* ----------------------- Data Struct ------------------------------------- */

typedef enum
{
	Raw_Mode,                 			//速度直接开环发送
	Swiming_Mode,										//ROV正常浮游模式，不用履带
    Crawing_Mode,										//履带爬行模式，此时主要控制vf和yaw
    No_Move_Mode,										//停止模式，此时机器人稳定当前状态，进行作业（暂不开发）
} rov_mode_e;


typedef struct
{
    const ext_control_cmd_t *rov_Ctrl;             	//ROV使用的控制指针, the point to remote control
    const ext_control_pos_t *rov_Pos_Ctrl;                 //ROV使用的位姿控制指针， the point to pos control
    const IMU_data_t *IMU_data;             				//the point to the euler angle of gyro sensor.获取陀螺仪解算出的欧拉角指针
    rov_mode_e rov_mode;               							//state machine. ROV控制状态机
    rov_mode_e last_rov_mode;          							//last state machine.ROV上次控制状态机
    motors_status_t motor_rov;          						//rov motor data.ROV电机数据
	
	int16_t thruster_speed_set[6];									//设定的推进器转速
	int16_t track_voltage_set[2];										//设定的履带电机电压

	fp32 depth;																			//深度数据
	fp32 depth_set;																	//设定的深度值
	fp32 yaw_angle_set;																//设定的定艏角度
	pid_type_def depth_loop_pid;										//定深环PID
	
	pid_type_def track_speed_pid[2];								//履带电机闭环PID
	pid_type_def track_turn_loop_pid;								//履带模式转向环PID
	
	pid_type_def yaw_angle_pid;              				//yaw angle PID.Yaw角度pid
	pid_type_def yaw_angular_velocity_pid;          //yaw angular velocity PID.Yaw角速度pid
	pid_type_def pitch_angle_pid;              			//pitch angle PID.Pitch角度pid
	pid_type_def pitch_angular_velocity_pid;        //pitch angular velocity PID.Pitch角速度pid
	pid_type_def roll_angle_pid;              			//roll angle PID.Roll角度pid
	pid_type_def roll_angular_velocity_pid;         //roll angular velocity PID.Roll角速度pid
	
    first_order_filter_type_t rov_cmd_slow_set_yaw;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
    first_order_filter_type_t rov_cmd_slow_set_roll;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
	
	
	fp32 vf_set;                      //rov forward speed, positive means forward,unit m/s. rov游动速度 航行方向 前为正
	fp32 vz_set;                      //rov vertical speed, positive means forward,unit m/s. rov游动速度 深度方向 上浮为正
	fp32 vf_crawl_set;								//track forward speed ,positive means forward,unit m/s. rov履带速度  前为正
	fp32 wz_set;											//rov rotation speed in crawl mode, positive means counterclockwise,unit rad/s.ROV履带模式下旋转角速度，逆时针为正 单位 rad/s
	fp32 yaw_set;                     //rov set Euler angle,unit °.rov的yaw轴设定角度，单位 °
	fp32 pitch_set;                   //rov set Euler angle,unit °.rov的pitch轴设定角度，单位 °
	fp32 roll_set;                    //rov set Euler angle,unit °.rov的roll轴设定角度，单位 °
	

  fp32 vf_max_speed;  //max forward speed, unit m/s.航行方向最大速度 单位m/s
  fp32 vf_min_speed;  //max backward speed, unit m/s.后退方向最大速度 单位m/s
  fp32 vz_max_speed;  //max letf speed, unit m/s.上升方向最大速度 单位m/s
  fp32 vz_min_speed;  //max right speed, unit m/s.下沉方向最大速度 单位m/s

} rov_move_t;



/* ----------------------- Extern Function ----------------------------------- */

/**
  * @brief          根据上位机下发的遥控器数据，修正角度和速度值
  *                 
  * @param[in]      yaw_set yaw轴稳定的角度
  * @param[in]      roll_set roll轴稳定的角度
  * @param[out]     rov_move_transit:"rov_move"变量指针.
  * @retval         none
  */
extern void rov_rc_to_control_vector(fp32 *yaw_set, fp32 *roll_set, rov_move_t *rov_move_rc_to_vector);

/**
  * @brief          返回ROV控制模式
  * @param[in]     none
  * @retval         uint8_t
  */
extern uint8_t get_rov_mode(void);


/**
  * @brief          rov movation task, osDelay ROV_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          ROV运动任务，间隔 ROV_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
extern void rov_movation_task(void const *pvParameters);


#endif
