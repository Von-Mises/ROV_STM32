/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       rov_movation_task.c/h
  * @brief      ����ROV�˶�״̬������         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-14-2022     HaoLion(������)    1. done
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
//����ʼ����һ��ʱ��
#define ROV_TASK_INIT_TIME 357
//rov movation task control time  2ms
//ROV�˶�������Ƽ�� 5ms
#define ROV_CONTROL_TIME_MS 5

//�˶�������Ƽ�� 0.002s
#define ROV_CONTROL_TIME 0.002f

#define ROV_ACCEL_YAW_NUM 0.1666666667f
#define ROV_ACCEL_ROL_NUM 0.3333333333f


//rocker value deadline
//ҡ������
#define ROV_RC_DEADLINE 5

//�Ĵ�ģʽ�£�ң������yawң��ת��Ϊ��ת�ٶȵı���
#define ROV_ANGLE_TO_WZ_SEN  5//0.002f



//rov forward or back max speed
//rov�˶�����������ٶ�
#define NORMAL_MAX_ROV_SPEED_F 2.0f
//rov rise or sink max speed
//rov�˶������������ٶ�
#define NORMAL_MAX_ROV_SPEED_Z 1.5f

//ˮƽ�ƽ�����ROV���ĵ�������� ��λm
#define THRUSTER_DISTANCE_TO_CENTER  0.3f

//�Ĵ���ROV���ĵ�������� ��λm
#define TRACK_DISTANCE_TO_CENTER  0.2f

//depth hold loop PID
//���pidֵ
#define ROV_DEPTH_PID_KP 2000.0f
#define ROV_DEPTH_PID_KI 0.0f
#define ROV_DEPTH_PID_KD 0.0f
#define ROV_DEPTH_PID_MAX_OUT 500.0f
#define ROV_DEPTH_PID_MAX_IOUT 0.2f

//track mode turn loop PID
//�Ĵ�ģʽ��ת��PID
#define TRACK_WZ_PID_KP 0.0f
#define TRACK_WZ_PID_KI 0.0f
#define TRACK_WZ_PID_KD 0.0f
#define TRACK_WZ_PID_MAX_OUT  0.0f
#define TRACK_WZ_PID_MAX_IOUT 0.0f

//track motor velocity loop PID
//�Ĵ�����ٶȻ�PID
#define TRACK_MOTOR_SPEED_PID_KP 0.0f
#define TRACK_MOTOR_SPEED_PID_KI 0.0f
#define TRACK_MOTOR_SPEED_PID_KD 0.0f
#define TRACK_MOTOR_SPEED_PID_MAX_OUT  0.0f
#define TRACK_MOTOR_SPEED_PID_MAX_IOUT 0.0f

//yaw angle PID
//Yaw�Ƕ�pid
#define ROV_YAW_ANGLE_PID_KP 200.0f
#define ROV_YAW_ANGLE_PID_KI 0.0f
#define ROV_YAW_ANGLE_PID_KD 0.0f
#define ROV_YAW_ANGLE_PID_MAX_OUT 500.0f
#define ROV_YAW_ANGLE_PID_MAX_IOUT 0.2f

//yaw angular velocity PID
//Yaw���ٶ�pid
#define ROV_YAW_ANGULAR_VELOCITY_PID_KP 200.0f
#define ROV_YAW_ANGULAR_VELOCITY_PID_KI 0.0f
#define ROV_YAW_ANGULAR_VELOCITY_PID_KD 0.0f
#define ROV_YAW_ANGULAR_VELOCITY_PID_MAX_OUT 500.0f
#define ROV_YAW_ANGULAR_VELOCITY_PID_MAX_IOUT 0.2f 

//pitch angle PID
//Pitch�Ƕ�pid
#define ROV_PITCH_ANGLE_PID_KP 0.0f
#define ROV_PITCH_ANGLE_PID_KI 0.0f
#define ROV_PITCH_ANGLE_PID_KD 0.0f
#define ROV_PITCH_ANGLE_PID_MAX_OUT 0.0f
#define ROV_PITCH_ANGLE_PID_MAX_IOUT 0.0f

//pitch angular velocity PID
//Pitch���ٶ�pid
#define ROV_PITCH_ANGULAR_VELOCITY_PID_KP 0.0f
#define ROV_PITCH_ANGULAR_VELOCITY_PID_KI 0.0f
#define ROV_PITCH_ANGULAR_VELOCITY_PID_KD 0.0f
#define ROV_PITCH_ANGULAR_VELOCITY_PID_MAX_OUT 0.0f
#define ROV_PITCH_ANGULAR_VELOCITY_PID_MAX_IOUT 0.0f

//roll angle PID
//Roll�Ƕ�pid
#define ROV_ROLL_ANGLE_PID_KP 0.0f
#define ROV_ROLL_ANGLE_PID_KI 0.0f
#define ROV_ROLL_ANGLE_PID_KD 0.0f
#define ROV_ROLL_ANGLE_PID_MAX_OUT 0.0f
#define ROV_ROLL_ANGLE_PID_MAX_IOUT 0.0f

//roll angular velocity PID
//Roll���ٶ�pid
#define ROV_ROLL_ANGULAR_VELOCITY_PID_KP 0.0f
#define ROV_ROLL_ANGULAR_VELOCITY_PID_KI 0.0f
#define ROV_ROLL_ANGULAR_VELOCITY_PID_KD 0.0f
#define ROV_ROLL_ANGULAR_VELOCITY_PID_MAX_OUT 0.0f
#define ROV_ROLL_ANGULAR_VELOCITY_PID_MAX_IOUT 0.0f


/* ----------------------- Data Struct ------------------------------------- */

typedef enum
{
	Raw_Mode,                 			//�ٶ�ֱ�ӿ�������
	Swiming_Mode,										//ROV��������ģʽ�������Ĵ�
    Crawing_Mode,										//�Ĵ�����ģʽ����ʱ��Ҫ����vf��yaw
    No_Move_Mode,										//ֹͣģʽ����ʱ�������ȶ���ǰ״̬��������ҵ���ݲ�������
} rov_mode_e;


typedef struct
{
    const ext_control_cmd_t *rov_Ctrl;             	//ROVʹ�õĿ���ָ��, the point to remote control
    const ext_control_pos_t *rov_Pos_Ctrl;                 //ROVʹ�õ�λ�˿���ָ�룬 the point to pos control
    const IMU_data_t *IMU_data;             				//the point to the euler angle of gyro sensor.��ȡ�����ǽ������ŷ����ָ��
    rov_mode_e rov_mode;               							//state machine. ROV����״̬��
    rov_mode_e last_rov_mode;          							//last state machine.ROV�ϴο���״̬��
    motors_status_t motor_rov;          						//rov motor data.ROV�������
	
	int16_t thruster_speed_set[6];									//�趨���ƽ���ת��
	int16_t track_voltage_set[2];										//�趨���Ĵ������ѹ

	fp32 depth;																			//�������
	fp32 depth_set;																	//�趨�����ֵ
	fp32 yaw_angle_set;																//�趨�Ķ����Ƕ�
	pid_type_def depth_loop_pid;										//���PID
	
	pid_type_def track_speed_pid[2];								//�Ĵ�����ջ�PID
	pid_type_def track_turn_loop_pid;								//�Ĵ�ģʽת��PID
	
	pid_type_def yaw_angle_pid;              				//yaw angle PID.Yaw�Ƕ�pid
	pid_type_def yaw_angular_velocity_pid;          //yaw angular velocity PID.Yaw���ٶ�pid
	pid_type_def pitch_angle_pid;              			//pitch angle PID.Pitch�Ƕ�pid
	pid_type_def pitch_angular_velocity_pid;        //pitch angular velocity PID.Pitch���ٶ�pid
	pid_type_def roll_angle_pid;              			//roll angle PID.Roll�Ƕ�pid
	pid_type_def roll_angular_velocity_pid;         //roll angular velocity PID.Roll���ٶ�pid
	
    first_order_filter_type_t rov_cmd_slow_set_yaw;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ
    first_order_filter_type_t rov_cmd_slow_set_roll;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ
	
	
	fp32 vf_set;                      //rov forward speed, positive means forward,unit m/s. rov�ζ��ٶ� ���з��� ǰΪ��
	fp32 vz_set;                      //rov vertical speed, positive means forward,unit m/s. rov�ζ��ٶ� ��ȷ��� �ϸ�Ϊ��
	fp32 vf_crawl_set;								//track forward speed ,positive means forward,unit m/s. rov�Ĵ��ٶ�  ǰΪ��
	fp32 wz_set;											//rov rotation speed in crawl mode, positive means counterclockwise,unit rad/s.ROV�Ĵ�ģʽ����ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
	fp32 yaw_set;                     //rov set Euler angle,unit ��.rov��yaw���趨�Ƕȣ���λ ��
	fp32 pitch_set;                   //rov set Euler angle,unit ��.rov��pitch���趨�Ƕȣ���λ ��
	fp32 roll_set;                    //rov set Euler angle,unit ��.rov��roll���趨�Ƕȣ���λ ��
	

  fp32 vf_max_speed;  //max forward speed, unit m/s.���з�������ٶ� ��λm/s
  fp32 vf_min_speed;  //max backward speed, unit m/s.���˷�������ٶ� ��λm/s
  fp32 vz_max_speed;  //max letf speed, unit m/s.������������ٶ� ��λm/s
  fp32 vz_min_speed;  //max right speed, unit m/s.�³���������ٶ� ��λm/s

} rov_move_t;



/* ----------------------- Extern Function ----------------------------------- */

/**
  * @brief          ������λ���·���ң�������ݣ������ǶȺ��ٶ�ֵ
  *                 
  * @param[in]      yaw_set yaw���ȶ��ĽǶ�
  * @param[in]      roll_set roll���ȶ��ĽǶ�
  * @param[out]     rov_move_transit:"rov_move"����ָ��.
  * @retval         none
  */
extern void rov_rc_to_control_vector(fp32 *yaw_set, fp32 *roll_set, rov_move_t *rov_move_rc_to_vector);

/**
  * @brief          ����ROV����ģʽ
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
  * @brief          ROV�˶����񣬼�� ROV_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
extern void rov_movation_task(void const *pvParameters);


#endif