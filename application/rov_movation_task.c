/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       rov_movation_task.c/h
  * @brief      ����ROV�˶�״̬������         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     DEC-14-2022     Qiqi Li(������)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */
#include "rov_movation_task.h"
#include "rov_behaviour.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "pid.h"
#include "socket_receive.h"
#include "socket_send.h"
#include "bsp_pulse.h"
#include "thruster.h"
#include "bsp_dac.h"
#include "depth_meter_receive.h"
#include "detect_task.h"

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

//ROV�˶�����
rov_move_t rov_move;

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
void rov_movation_task(void const *pvParameters)
{
	//wait a time 
    //����һ��ʱ��
    vTaskDelay(ROV_TASK_INIT_TIME);
    //rov movation init
    //ROV�˶����Ƴ�ʼ��
    rov_init(&rov_move);
	
	uint8_t i = 0;

    while (1)
    {
        //set rov control mode
        //����ROV����ģʽ
        rov_set_mode(&rov_move);
        //rov movation data update
        //ROV�˶����ݸ���
        rov_feedback_update(&rov_move);
		//set rov pid parameters
		//����ROV����PIDֵ
		rov_set_pids(&rov_move);
        //set rov control set-point 
        //ROV����������
        rov_set_contorl(&rov_move);
        //rov every loop pid calculate
        //ROV����PID����
        rov_control_loop(&rov_move);
			
        for (i = 0; i < 6; i++)
		{
			//�����ƽ����ٶ���CAN����
			osDelay(1);
			CAN_cmd_control(CAN_M1_ID+i,rov_move.thruster_speed_set[i],0xAA);	
		} 
		Track_Motor_Ctrl(rov_move.track_voltage_set[0], rov_move.track_voltage_set[1]);
		
		//os delay
        //ϵͳ��ʱ
        osDelay(ROV_CONTROL_TIME_MS);
		
		//�������ģʽ�£��������ö�����ȺͶ����Ƕ�
//		if(rov_move.rov_mode == ROV_ONLY_ALTHOLD || rov_move.rov_mode == ROV_ONLY_ATTHOLD || rov_move.rov_mode == ROV_NORMAL)
//		{
//			rov_move.depth_set = get_depth_data();
//			rov_move.yaw_angle_set = get_imu_data_point()->yaw;
//		}
    }
}


/**
  * @brief          ����ROV����ģʽ
  * @param[in]     none
  * @retval         uint8_t
  */
uint8_t get_rov_mode(void)
{
	return rov_move.rov_mode;
}

/**
  * @brief          set rov pid parameters
  * @param[out]     none
  * @retval         none
  */
/**
  * @brief          rov����PID��������
  * @param[out]     none
  * @retval         none
  */
static void rov_set_pids(rov_move_t *rov_move_pids)
{
	if(rov_move_pids == NULL) return;
	if(rov_move_pids->pid_change == 1)
	{
		get_pids(&rov_move_pids->depth_loop_pid, 0); 			//���PID
		
		get_pids(&rov_move_pids->track_speed_pid[0], 1);		//�Ĵ�����ջ�PID
		get_pids(&rov_move_pids->track_speed_pid[1], 1);		//�Ĵ�����ջ�PID
		get_pids(&rov_move_pids->track_turn_loop_pid, 2);		//�Ĵ�ģʽת��PID								                
		
		get_pids(&rov_move_pids->yaw_angle_pid, 3);              	//yaw angle PID.Yaw�Ƕ�pid
		get_pids(&rov_move_pids->yaw_angular_velocity_pid, 4);      //yaw angular velocity PID.Yaw���ٶ�pid
	//	get_pids(&rov_move_pids->pitch_angle_pid, 5);              	//pitch angle PID.Pitch�Ƕ�pid
	//	get_pids(&rov_move_pids->pitch_angular_velocity_pid, 6);    //pitch angular velocity PID.Pitch���ٶ�pid
		get_pids(&rov_move_pids->roll_angle_pid, 7);              	//roll angle PID.Roll�Ƕ�pid
		get_pids(&rov_move_pids->roll_angular_velocity_pid, 8);     //roll angular velocity PID.Roll���ٶ�pid
		
		rov_move_pids->pid_change = 0;
	}
	else return;
	}


/**
  * @brief          rov motor speed data updata
  * @param[out]     rov_move_updata: "rov_move_t" valiable point
  * @retval         none
  */
/**
  * @brief          ROV����ٶȸ���
  * @param[out]     rov_move_updata:"rov_move_t"����ָ��.
  * @retval         none
  */
static void rov_feedback_update(rov_move_t *rov_move_updata)
{
	uint16_t motor_speed_1,motor_speed_2;
	get_crawler_motor_speed(&motor_speed_1, &motor_speed_2);
	rov_move_updata->motor_rov.track_speed[0] = motor_speed_1;
	rov_move_updata->motor_rov.track_speed[1] = motor_speed_2;
	for(int i=0;i<6;i++)
	{
		rov_move_updata->motor_rov.thruster_speed[i] = get_thruster_measure_point(i)->speed_rpm;
	}
	//��ȡ���
	rov_move_updata->depth = get_depth_data();
}

/**
  * @brief          "rov_move" valiable initialization, include pid initialization, remote control data point initialization, 
  *                  and IMU data point initialization.
  * @param[out]     rov_move_init: "rov_move_t" valiable point
  * @retval         none
  */
/**
  * @brief          ��ʼ��"rov_move"����������pid��ʼ���� ң����ָ���ʼ����IMU����ָ���ʼ��
  * @param[out]     rov_move_init:"rov_move_t"����ָ��.
  * @retval         none
  */
static void rov_init(rov_move_t *rov_move_init)
{
    if (rov_move_init == NULL)
    {
        return;
    }
	uint8_t i;
	//depth hold loop PID
	//���pidֵ
    const static fp32 rov_depth_pid[3] = {ROV_DEPTH_PID_KP, ROV_DEPTH_PID_KI, ROV_DEPTH_PID_KD};
	//track mode turn loop PID
	//�Ĵ�ģʽ��ת��PID
	const static fp32 track_wz_pid[3] = {TRACK_WZ_PID_KP, TRACK_WZ_PID_KI, TRACK_WZ_PID_KD};
	//track motor velocity loop PID
	//�Ĵ�����ٶȻ�PID
	const static fp32 track_speed_pid[3] = {TRACK_MOTOR_SPEED_PID_KP, TRACK_MOTOR_SPEED_PID_KI, TRACK_MOTOR_SPEED_PID_KD};
	//yaw angular velocity PID
	//Yaw���ٶ�pid
	const static fp32 rov_yaw_w_pid[3] = {ROV_YAW_ANGULAR_VELOCITY_PID_KP, ROV_YAW_ANGULAR_VELOCITY_PID_KI, ROV_YAW_ANGULAR_VELOCITY_PID_KD};
	//yaw angle PID
	//Yaw�Ƕ�pid
	const static fp32 rov_yaw_pid[3] = {ROV_YAW_ANGLE_PID_KP, ROV_YAW_ANGLE_PID_KI, ROV_YAW_ANGLE_PID_KD};
	//pitch angular velocity PID
	//Pitch���ٶ�pid
	const static fp32 rov_pitch_w_pid[3] = {ROV_PITCH_ANGULAR_VELOCITY_PID_KP, ROV_PITCH_ANGULAR_VELOCITY_PID_KI, ROV_PITCH_ANGULAR_VELOCITY_PID_KD};
	//pitch angle PID
	//Pitch�Ƕ�pid
	const static fp32 rov_pitch_pid[3] = {ROV_PITCH_ANGLE_PID_KP, ROV_PITCH_ANGLE_PID_KI, ROV_PITCH_ANGLE_PID_KD};
	//roll angular velocity PID
	//Roll���ٶ�pid
	const static fp32 rov_roll_w_pid[3] = {ROV_ROLL_ANGULAR_VELOCITY_PID_KP, ROV_ROLL_ANGULAR_VELOCITY_PID_KI, ROV_ROLL_ANGULAR_VELOCITY_PID_KD};
	//roll angle PID
	//Roll�Ƕ�pid
	const static fp32 rov_roll_pid[3] = {ROV_ROLL_ANGLE_PID_KP, ROV_ROLL_ANGLE_PID_KI, ROV_ROLL_ANGLE_PID_KD};
	//initialize PID
    //��ʼ��PID
	for (i = 0; i < 2; i++)
    {
        PID_init(&rov_move_init->track_speed_pid[i], PID_POSITION, track_speed_pid, TRACK_MOTOR_SPEED_PID_MAX_OUT, TRACK_MOTOR_SPEED_PID_MAX_IOUT);
    }
	//���PID
    PID_init(&rov_move_init->depth_loop_pid, PID_POSITION, rov_depth_pid, ROV_DEPTH_PID_MAX_OUT, ROV_DEPTH_PID_MAX_IOUT);
	//�Ĵ�ģʽ��ת��PID
	PID_init(&rov_move_init->track_turn_loop_pid, PID_POSITION, track_wz_pid, TRACK_MOTOR_SPEED_PID_MAX_OUT, TRACK_MOTOR_SPEED_PID_MAX_IOUT);
	//yaw
	//���ٶ���Ҫ����PI����������ʽPID
	PID_init(&rov_move_init->yaw_angular_velocity_pid, PID_DELTA, rov_yaw_w_pid, ROV_YAW_ANGULAR_VELOCITY_PID_MAX_OUT, ROV_YAW_ANGULAR_VELOCITY_PID_MAX_IOUT);
	PID_init(&rov_move_init->yaw_angle_pid, PID_POSITION, rov_yaw_pid, ROV_YAW_ANGLE_PID_MAX_OUT, ROV_YAW_ANGLE_PID_MAX_IOUT);
	//pitch
    PID_init(&rov_move_init->pitch_angular_velocity_pid, PID_DELTA, rov_pitch_w_pid, ROV_PITCH_ANGULAR_VELOCITY_PID_MAX_OUT, ROV_PITCH_ANGULAR_VELOCITY_PID_MAX_IOUT);
	PID_init(&rov_move_init->pitch_angle_pid, PID_POSITION, rov_pitch_pid, ROV_PITCH_ANGLE_PID_MAX_OUT, ROV_PITCH_ANGLE_PID_MAX_IOUT);
	//roll
	PID_init(&rov_move_init->roll_angular_velocity_pid, PID_DELTA, rov_roll_w_pid, ROV_ROLL_ANGULAR_VELOCITY_PID_MAX_OUT, ROV_ROLL_ANGULAR_VELOCITY_PID_MAX_IOUT);
	PID_init(&rov_move_init->roll_angle_pid, PID_POSITION, rov_roll_pid, ROV_ROLL_ANGLE_PID_MAX_OUT, ROV_ROLL_ANGLE_PID_MAX_IOUT);

    //in beginning�� rov mode is normal 
    //rov����״̬Ϊ����
    rov_move_init->rov_mode = ROV_ZERO_FORCE;
	
	//set pid change state to zero
	//��ʼ��pid���ı�־λΪ0
	rov_move_init->pid_change = 0;
	
    //get remote control point
    //��ȡ��������ָ��
    rov_move_init->rov_Ctrl = get_ctrl_cmd();
	
	//get remote pos control point
    //��ȡλ�˿�������ָ��
    rov_move_init->rov_Pos_Ctrl = get_pos_ctrl_cmd();
	
    //get gyro sensor euler angle point
    //��ȡ��������̬��ָ��
    rov_move_init->IMU_data = get_imu_data_point();

    //update data
    //����һ������
    rov_feedback_update(rov_move_init);
}

/**
  * @brief          set rov control mode, mainly call 'rov_behaviour_mode_set' function
  * @param[out]     rov_move_mode: "rov_move_t" valiable point
  * @retval         none
  */
/**
  * @brief          ����ROV����ģʽ����Ҫ��'rov_behaviour_mode_set'�����иı�
  * @param[out]     rov_move_mode:"rov_move_t"����ָ��.
  * @retval         none
  */
static void rov_set_mode(rov_move_t *rov_move_mode)
{
    if (rov_move_mode == NULL)
    {
        return;
    }
    rov_move_mode->rov_mode = rov_move_mode->rov_Ctrl->Mode;
}

/**
  * @brief          set rov control set-point,  movement control value is set by "rov_behaviour_control_set".
  * @param[out]     rov_move_control: "rov_move_t" valiable point
  * @retval         none
  */
/**
  * @brief          ����ROV��������ֵ, �˶�����ֵ��ͨ��rov_behaviour_control_set�������õ�
  * @param[out]     rov_move_control:"rov_move_t"����ָ��.
  * @retval         none
  */
static void rov_set_contorl(rov_move_t *rov_move_control)
{

    if (rov_move_control == NULL)
    {
        return;
    }
	rov_move_control->depth_set = rov_move_control->rov_Pos_Ctrl->Depth;
	rov_move_control->yaw_angle_set = rov_move_control->rov_Pos_Ctrl->Yaw;

    //���ݿ���ģʽ����ROV����ʵ���ٶ�
    if (rov_move_control->rov_mode == ROV_OPEN)
    {
		rov_move_control->vf_set = rov_move_control->rov_Ctrl->VF;
		rov_move_control->vz_set = rov_move_control->rov_Ctrl->VZ;
		rov_move_control->yaw_set = rov_move_control->rov_Ctrl->YAW;
		rov_move_control->roll_set = rov_move_control->rov_Ctrl->ROL;
		rov_move_control->pitch_set = 0;//rov_move_control->rov_Ctrl->PIT;
		rov_move_control->vf_crawl_set = 0.0f;
		rov_move_control->wz_set = 0.0f;
				
    }
    else if (rov_move_control->rov_mode == ROV_NORMAL || rov_move_control->rov_mode == ROV_ONLY_ALTHOLD
		     || rov_move_control->rov_mode == ROV_ONLY_ATTHOLD)
    {
		rov_move_control->vf_set = rov_move_control->rov_Ctrl->VF;
		rov_move_control->vz_set = rov_move_control->rov_Ctrl->VZ;
		rov_move_control->yaw_set = rov_move_control->rov_Ctrl->YAW;
		rov_move_control->roll_set = rov_move_control->rov_Ctrl->ROL;
		rov_move_control->pitch_set = 0;//rov_move_control->rov_Ctrl->PIT;
		rov_move_control->vf_crawl_set = 0.0f;
		rov_move_control->wz_set = 0.0f;
    }
    else if (rov_move_control->rov_mode == ROV_CRAWLING || rov_move_control->rov_mode == ROV_STICK_WALL)
    {
		rov_move_control->vf_set = 0.0f;
		rov_move_control->vz_set = 0.0f;
		if(rov_move_control->rov_mode == ROV_STICK_WALL)
		{
			rov_move_control->vz_set = 150;
		}
		rov_move_control->yaw_set = 0.0f;
		rov_move_control->roll_set = 0.0f;
		rov_move_control->pitch_set = 0.0f;
		rov_move_control->vf_crawl_set = rov_move_control->rov_Ctrl->VF;
		//�ֱ��·��ĽǶ�ֵ��Ϊ�Ĵ���ת���ٶ�
		rov_move_control->wz_set = rov_move_control->rov_Ctrl->YAW;
    }
    else if (rov_move_control->rov_mode == ROV_NO_MOVE || rov_move_control->rov_mode == ROV_ZERO_FORCE)
    {
		rov_move_control->vf_set = 0.0f;
		rov_move_control->vz_set = 0.0f;
		rov_move_control->yaw_set = 0.0f;
		rov_move_control->roll_set = 0.0f;
		rov_move_control->pitch_set = 0.0f;
		rov_move_control->vf_crawl_set = 0.0f;
		rov_move_control->wz_set = 0.0f;
    }
}


/**
  * @brief          ���ݿ��ƽ�������ƽ�������ٶ�
  * @param[in]      yaw_control_out,: yaw�������
  * @param[in]      pitch_control_out,: pitch�������
  * @param[in]      roll_control_out,: roll�������
  * @param[in]      depth_control_out,: ���������
	* @param[in]      vf_control_out,: ���������
  * @param[out]     thruster_speed: �����ƽ����ٶ� ��0-5�ֱ�Ϊ���¡����ϡ����ϡ����¡��Ҳࡢ���
  * @retval         none
  */
static void rov_vector_to_thruster_speed(const fp32 yaw_control_out, const fp32 pitch_control_out, const fp32 roll_control_out,const fp32 depth_control_out,const fp32 vf_control_out , fp32 thruster_speed[6])
{
	//��ֱ�ƽ���
	thruster_speed[0] = depth_control_out + roll_control_out - pitch_control_out; //����
	thruster_speed[1] = depth_control_out - roll_control_out + pitch_control_out; //����
	thruster_speed[2] = depth_control_out + roll_control_out + pitch_control_out; //����
	thruster_speed[3] = depth_control_out - roll_control_out - pitch_control_out; //����
	//ˮƽ�ƽ���
	thruster_speed[4] = vf_control_out - yaw_control_out; //��
	thruster_speed[5] = vf_control_out + yaw_control_out; //��
}

/**
  * @brief          ���ݿ��ƽ��������Ĵ��������ٶ�
  * @param[in]      track_wz_out,: ��ת�ٶȿ�����
  * @param[in]      track_vf_control_out,: ǰ��������
  * @param[out]     track_speed: �Ĵ��ٶ� �ֱ�Ϊ��ࡢ�Ҳ�
  * @retval         none
  */
static void rov_vector_to_track_speed(const fp32 track_wz_out,const fp32 track_vf_control_out , fp32 track_speed[2])
{
	track_speed[0] = -track_vf_control_out - TRACK_DISTANCE_TO_CENTER*track_wz_out;
	track_speed[1] = -track_vf_control_out + TRACK_DISTANCE_TO_CENTER*track_wz_out;
}

/**
  * @brief          ����ѭ�������ݿ����趨ֵ�����������������ٶȣ����п���
  * @param[out]     rov_move_control_loop:"rov_move"����ָ��.
  * @retval         none
  */
static void rov_control_loop(rov_move_t *rov_move_control_loop)
{
	fp32 yaw_control_out, pitch_control_out, roll_control_out, depth_control_out, vf_control_out, track_turn_control_out, track_vf_control_out;
    fp32 thruster_speed[6] = {0.0f, 0.0f, 0.0f, 0.0f , 0.0f , 0.0f};
	fp32 track_speed[2] = {0.0f, 0.0f};
	uint8_t i = 0;
	//����������������
	rov_angle_loop_calc(&yaw_control_out, &pitch_control_out, &roll_control_out, rov_move_control_loop);
	rov_depth_loop_calc(&depth_control_out, rov_move_control_loop);
	track_turn_loop_calc(&track_turn_control_out, rov_move_control_loop);
	rov_vf_control_calc(&vf_control_out, &track_vf_control_out, rov_move_control_loop);
	//���ݿ������������˶��ֽ����
	rov_vector_to_track_speed(track_turn_control_out,track_vf_control_out,track_speed);
	rov_vector_to_thruster_speed(yaw_control_out, pitch_control_out, roll_control_out, depth_control_out, vf_control_out , thruster_speed);
		
//		//ֱ�ӷ����Ĵ������ѹֵ
//		if(rov_move_control_loop->rov_mode == Raw_Mode)
//		{
//			rov_move_control_loop->track_voltage_set[0] = (int16_t)track_speed[0];
//			rov_move_control_loop->track_voltage_set[1] = (int16_t)track_speed[1];
//		}
//		else
//		{
//			//calculate pid
//			//����pid
//			for (i = 0; i < 2; i++)
//			{
//					PID_calc(&rov_move_control_loop->track_speed_pid[i], rov_move_control_loop->motor_rov.track_speed[i], track_speed[i]);
//			}
//			rov_move_control_loop->track_voltage_set[0] = (int16_t)rov_move_control_loop->track_speed_pid[0].out;
//			rov_move_control_loop->track_voltage_set[1] = (int16_t)rov_move_control_loop->track_speed_pid[1].out;
//		}
	//���Կ�������Ҫɾ��
	rov_move_control_loop->track_voltage_set[0] = (int16_t)track_speed[0];
	rov_move_control_loop->track_voltage_set[1] = (int16_t)track_speed[1];
	for(i = 0; i < 6; i++)
	{
		rov_move_control_loop->thruster_speed_set[i] = (int16_t)thruster_speed[i];
	}
}




