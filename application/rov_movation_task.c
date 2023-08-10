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

/**
  * @brief          "rov_move" valiable initialization, include pid initialization, remote control data point initialization, 
  *                  and IMU data point initialization.
  * @param[out]     rov_move_init: "rov_move_t" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"rov_move"变量，包括pid初始化， 遥控器指针初始化，IMU数据指针初始化
  * @param[out]     rov_move_init:"rov_move_t"变量指针.
  * @retval         none
  */
static void rov_init(rov_move_t *rov_move_init);

/**
  * @brief          set rov control mode, mainly call 'rov_behaviour_mode_set' function
  * @param[out]     rov_move_mode: "rov_move_t" valiable point
  * @retval         none
  */
/**
  * @brief          设置ROV控制模式，主要在'rov_behaviour_mode_set'函数中改变
  * @param[out]     rov_move_mode:"rov_move_t"变量指针.
  * @retval         none
  */
static void rov_set_mode(rov_move_t *rov_move_mode);

/**
  * @brief          rov motor speed data updata
  * @param[out]     rov_move_updata: "rov_move_t" valiable point
  * @retval         none
  */
/**
  * @brief          ROV电机速度更新
  * @param[out]     rov_move_updata:"rov_move_t"变量指针.
  * @retval         none
  */
static void rov_feedback_update(rov_move_t *rov_move_updata);

/**
  * @brief          when rov mode change, some param should be changed
  * @param[out]     rov_move_transit: "rov_move" valiable point
  * @retval         none
  */
/**
  * @brief          rov模式改变，有些参数需要改变，
  * @param[out]     rov_move_transit:"rov_move"变量指针.
  * @retval         none
  */
static void rov_mode_change_control_transit(rov_move_t *rov_move_transit);

/**
  * @brief          set rov pid parameters
  * @param[out]     none
  * @retval         none
  */
/**
  * @brief          rov各环PID参数设置
  * @param[out]     none
  * @retval         none
  */
static void rov_set_pids(rov_move_t *rov_move_pids);

/**
  * @brief          控制循环，根据控制设定值，计算各环并且输出速度，进行控制
  * @param[out]     rov_move_control_loop:"rov_move"变量指针.
  * @retval         none
  */
static void rov_control_loop(rov_move_t *rov_move_control_loop);


/**
  * @brief          set rov control set-point,  movement control value is set by "rov_behaviour_control_set".
  * @param[out]     rov_move_control: "rov_move_t" valiable point
  * @retval         none
  */
/**
  * @brief          设置ROV控制设置值, 运动控制值是通过rov_behaviour_control_set函数设置的
  * @param[out]     rov_move_control:"rov_move_t"变量指针.
  * @retval         none
  */
static void rov_set_contorl(rov_move_t *rov_move_control);







//ROV运动数据
rov_move_t rov_move;

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
void rov_movation_task(void const *pvParameters)
{
	//wait a time 
    //空闲一段时间
    vTaskDelay(ROV_TASK_INIT_TIME);
    //rov movation init
    //ROV运动控制初始化
    rov_init(&rov_move);
	
	uint8_t i = 0;

    while (1)
    {
        //set rov control mode
        //设置ROV控制模式
        rov_set_mode(&rov_move);
        //when mode changes, some data save
        //模式切换数据保存
        rov_mode_change_control_transit(&rov_move);
        //rov movation data update
        //ROV运动数据更新
        rov_feedback_update(&rov_move);
		//set rov pid parameters
		//设置ROV各环PID值
		rov_set_pids(&rov_move);
        //set rov control set-point 
        //ROV控制量设置
        rov_set_contorl(&rov_move);
        //rov every loop pid calculate
        //ROV各环PID计算
        rov_control_loop(&rov_move);
			
        for (i = 0; i < 6; i++)
		{
			//发送推进器速度至CAN总线
			osDelay(1);
			CAN_cmd_control(CAN_M1_ID+i,rov_move.thruster_speed_set[i],0xAA);	
		} 
		Track_Motor_Ctrl(rov_move.track_voltage_set[0], rov_move.track_voltage_set[1]);
		
		//os delay
        //系统延时
        osDelay(ROV_CONTROL_TIME_MS);
    }
}


/**
  * @brief          返回ROV控制模式
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
  * @brief          rov各环PID参数设置
  * @param[out]     none
  * @retval         none
  */
static void rov_set_pids(rov_move_t *rov_move_pids)
{
	if(rov_move_pids == NULL) return;
	get_pids(&rov_move_pids->depth_loop_pid, 0); 			//定深环PID
	
	get_pids(&rov_move_pids->track_speed_pid[0], 1);		//履带电机闭环PID
	get_pids(&rov_move_pids->track_speed_pid[1], 1);		//履带电机闭环PID
	get_pids(&rov_move_pids->track_turn_loop_pid, 2);		//履带模式转向环PID								                
	
	get_pids(&rov_move_pids->yaw_angle_pid, 3);              	//yaw angle PID.Yaw角度pid
	get_pids(&rov_move_pids->yaw_angular_velocity_pid, 4);      //yaw angular velocity PID.Yaw角速度pid
//	get_pids(&rov_move_pids->pitch_angle_pid, 5);              	//pitch angle PID.Pitch角度pid
//	get_pids(&rov_move_pids->pitch_angular_velocity_pid, 6);    //pitch angular velocity PID.Pitch角速度pid
	get_pids(&rov_move_pids->roll_angle_pid, 7);              	//roll angle PID.Roll角度pid
	get_pids(&rov_move_pids->roll_angular_velocity_pid, 8);     //roll angular velocity PID.Roll角速度pid
}

/**
  * @brief          rov motor speed data updata
  * @param[out]     rov_move_updata: "rov_move_t" valiable point
  * @retval         none
  */
/**
  * @brief          ROV电机速度更新
  * @param[out]     rov_move_updata:"rov_move_t"变量指针.
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
	//获取深度
	rov_move_updata->depth = get_depth_data();

}

/**
  * @brief          "rov_move" valiable initialization, include pid initialization, remote control data point initialization, 
  *                  and IMU data point initialization.
  * @param[out]     rov_move_init: "rov_move_t" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"rov_move"变量，包括pid初始化， 遥控器指针初始化，IMU数据指针初始化
  * @param[out]     rov_move_init:"rov_move_t"变量指针.
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
	//定深环pid值
    const static fp32 rov_depth_pid[3] = {ROV_DEPTH_PID_KP, ROV_DEPTH_PID_KI, ROV_DEPTH_PID_KD};
	//track mode turn loop PID
	//履带模式下转向环PID
	const static fp32 track_wz_pid[3] = {TRACK_WZ_PID_KP, TRACK_WZ_PID_KI, TRACK_WZ_PID_KD};
	//track motor velocity loop PID
	//履带电机速度环PID
	const static fp32 track_speed_pid[3] = {TRACK_MOTOR_SPEED_PID_KP, TRACK_MOTOR_SPEED_PID_KI, TRACK_MOTOR_SPEED_PID_KD};
	//yaw angular velocity PID
	//Yaw角速度pid
	const static fp32 rov_yaw_w_pid[3] = {ROV_YAW_ANGULAR_VELOCITY_PID_KP, ROV_YAW_ANGULAR_VELOCITY_PID_KI, ROV_YAW_ANGULAR_VELOCITY_PID_KD};
	//yaw angle PID
	//Yaw角度pid
	const static fp32 rov_yaw_pid[3] = {ROV_YAW_ANGLE_PID_KP, ROV_YAW_ANGLE_PID_KI, ROV_YAW_ANGLE_PID_KD};
	//pitch angular velocity PID
	//Pitch角速度pid
	const static fp32 rov_pitch_w_pid[3] = {ROV_PITCH_ANGULAR_VELOCITY_PID_KP, ROV_PITCH_ANGULAR_VELOCITY_PID_KI, ROV_PITCH_ANGULAR_VELOCITY_PID_KD};
	//pitch angle PID
	//Pitch角度pid
	const static fp32 rov_pitch_pid[3] = {ROV_PITCH_ANGLE_PID_KP, ROV_PITCH_ANGLE_PID_KI, ROV_PITCH_ANGLE_PID_KD};
	//roll angular velocity PID
	//Roll角速度pid
	const static fp32 rov_roll_w_pid[3] = {ROV_ROLL_ANGULAR_VELOCITY_PID_KP, ROV_ROLL_ANGULAR_VELOCITY_PID_KI, ROV_ROLL_ANGULAR_VELOCITY_PID_KD};
	//roll angle PID
	//Roll角度pid
	const static fp32 rov_roll_pid[3] = {ROV_ROLL_ANGLE_PID_KP, ROV_ROLL_ANGLE_PID_KI, ROV_ROLL_ANGLE_PID_KD};
		//initialize PID
    //初始化PID
	for (i = 0; i < 2; i++)
    {
        PID_init(&rov_move_init->track_speed_pid[i], PID_POSITION, track_speed_pid, TRACK_MOTOR_SPEED_PID_MAX_OUT, TRACK_MOTOR_SPEED_PID_MAX_IOUT);
    }
	//定深环PID
    PID_init(&rov_move_init->depth_loop_pid, PID_POSITION, rov_depth_pid, ROV_DEPTH_PID_MAX_OUT, ROV_DEPTH_PID_MAX_IOUT);
	//履带模式下转向环PID
	PID_init(&rov_move_init->track_turn_loop_pid, PID_POSITION, track_wz_pid, TRACK_MOTOR_SPEED_PID_MAX_OUT, TRACK_MOTOR_SPEED_PID_MAX_IOUT);
	//yaw
	//角速度主要考虑PI，采用增量式PID
	PID_init(&rov_move_init->yaw_angular_velocity_pid, PID_DELTA, rov_yaw_w_pid, ROV_YAW_ANGULAR_VELOCITY_PID_MAX_OUT, ROV_YAW_ANGULAR_VELOCITY_PID_MAX_IOUT);
	PID_init(&rov_move_init->yaw_angle_pid, PID_POSITION, rov_yaw_pid, ROV_YAW_ANGLE_PID_MAX_OUT, ROV_YAW_ANGLE_PID_MAX_IOUT);
	//pitch
    PID_init(&rov_move_init->pitch_angular_velocity_pid, PID_DELTA, rov_pitch_w_pid, ROV_PITCH_ANGULAR_VELOCITY_PID_MAX_OUT, ROV_PITCH_ANGULAR_VELOCITY_PID_MAX_IOUT);
	PID_init(&rov_move_init->pitch_angle_pid, PID_POSITION, rov_pitch_pid, ROV_PITCH_ANGLE_PID_MAX_OUT, ROV_PITCH_ANGLE_PID_MAX_IOUT);
	//roll
	PID_init(&rov_move_init->roll_angular_velocity_pid, PID_DELTA, rov_roll_w_pid, ROV_ROLL_ANGULAR_VELOCITY_PID_MAX_OUT, ROV_ROLL_ANGULAR_VELOCITY_PID_MAX_IOUT);
	PID_init(&rov_move_init->roll_angle_pid, PID_POSITION, rov_roll_pid, ROV_ROLL_ANGLE_PID_MAX_OUT, ROV_ROLL_ANGLE_PID_MAX_IOUT);
		
    const static fp32 rov_yaw_order_filter[1] = {ROV_ACCEL_YAW_NUM};
    const static fp32 rov_roll_order_filter[1] = {ROV_ACCEL_ROL_NUM};

    //in beginning， rov mode is normal 
    //rov开机状态为正常
    rov_move_init->rov_mode = Raw_Mode;
    //get remote control point
    //获取控制数据指针
    rov_move_init->rov_Ctrl = get_ctrl_cmd();
	
	//get remote pos control point
    //获取位姿控制数据指针
    rov_move_init->rov_Pos_Ctrl = get_pos_ctrl_cmd();
	
    //get gyro sensor euler angle point
    //获取陀螺仪姿态角指针
    rov_move_init->IMU_data = get_imu_data_point();
		
    //first order low-pass filter  replace ramp function
    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&rov_move_init->rov_cmd_slow_set_yaw, ROV_CONTROL_TIME, rov_yaw_order_filter);
    first_order_filter_init(&rov_move_init->rov_cmd_slow_set_roll, ROV_CONTROL_TIME, rov_roll_order_filter);

    //max and min speed
    //最大 最小速度
    rov_move_init->vf_max_speed = NORMAL_MAX_ROV_SPEED_F;
    rov_move_init->vf_min_speed = -NORMAL_MAX_ROV_SPEED_F;

    rov_move_init->vz_max_speed = NORMAL_MAX_ROV_SPEED_Z;
    rov_move_init->vz_min_speed = -NORMAL_MAX_ROV_SPEED_Z;

    //update data
    //更新一下数据
    rov_feedback_update(rov_move_init);
}

/**
  * @brief          set rov control mode, mainly call 'rov_behaviour_mode_set' function
  * @param[out]     rov_move_mode: "rov_move_t" valiable point
  * @retval         none
  */
/**
  * @brief          设置ROV控制模式，主要在'rov_behaviour_mode_set'函数中改变
  * @param[out]     rov_move_mode:"rov_move_t"变量指针.
  * @retval         none
  */
static void rov_set_mode(rov_move_t *rov_move_mode)
{
    if (rov_move_mode == NULL)
    {
        return;
    }
    //in file "rov_behaviour.c"
    rov_behaviour_mode_set(rov_move_mode);
}

/**
  * @brief          when rov mode change, some param should be changed
  * @param[out]     rov_move_transit: "rov_move" valiable point
  * @retval         none
  */
/**
  * @brief          rov模式改变，有些参数需要改变，
  * @param[out]     rov_move_transit:"rov_move"变量指针.
  * @retval         none
  */
static void rov_mode_change_control_transit(rov_move_t *rov_move_transit)
{
    if (rov_move_transit == NULL)
    {
        return;
    }

    if (rov_move_transit->last_rov_mode == rov_move_transit->rov_mode)
    {
        return;
    }
	
    rov_move_transit->last_rov_mode = rov_move_transit->rov_mode;
}


/**
  * @brief          根据上位机下发的遥控器数据，修正角度和速度值
  *                 
  * @param[in]      yaw_set yaw轴稳定的角度
  * @param[in]      roll_set roll轴稳定的角度
  * @param[out]     rov_move_transit:"rov_move"变量指针.
  * @retval         none
  */
void rov_rc_to_control_vector(fp32 *yaw_set, fp32 *roll_set, rov_move_t *rov_move_rc_to_vector)
{
    if (rov_move_rc_to_vector == NULL || yaw_set == NULL || roll_set == NULL)
    {
        return;
    }
    
    fp32 yaw_channel, roll_channel;
    //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(rov_move_rc_to_vector->rov_Ctrl->YAW, yaw_channel, ROV_RC_DEADLINE);
    rc_deadband_limit(rov_move_rc_to_vector->rov_Ctrl->ROL, roll_channel, ROV_RC_DEADLINE);


    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&rov_move_rc_to_vector->rov_cmd_slow_set_yaw, yaw_channel);
    first_order_filter_cali(&rov_move_rc_to_vector->rov_cmd_slow_set_roll, roll_channel);
    
    *yaw_set = rov_move_rc_to_vector->rov_cmd_slow_set_yaw.out;
    *roll_set = rov_move_rc_to_vector->rov_cmd_slow_set_roll.out;
}


/**
  * @brief          set rov control set-point,  movement control value is set by "rov_behaviour_control_set".
  * @param[out]     rov_move_control: "rov_move_t" valiable point
  * @retval         none
  */
/**
  * @brief          设置ROV控制设置值, 运动控制值是通过rov_behaviour_control_set函数设置的
  * @param[out]     rov_move_control:"rov_move_t"变量指针.
  * @retval         none
  */
static void rov_set_contorl(rov_move_t *rov_move_control)
{

    if (rov_move_control == NULL)
    {
        return;
    }


    fp32 vf_set = 0.0f, vz_set = 0.0f, yaw_set = 0.0f, pitch_set = 0.0f, roll_set = 0.0f;
    //获取三个控制设置值
    rov_behaviour_control_set(&vf_set, &vz_set, &yaw_set, &pitch_set, &roll_set, rov_move_control);

    //根据控制模式，给ROV配置实际速度
    if (rov_move_control->rov_mode == Raw_Mode)
    {
		rov_move_control->vf_set = vf_set;
		rov_move_control->vz_set = vz_set;
		rov_move_control->yaw_set = yaw_set;
		rov_move_control->roll_set = roll_set;
		rov_move_control->pitch_set = pitch_set;
		rov_move_control->vf_crawl_set = 0.0f;
		rov_move_control->wz_set = 0.0f;
				
    }
    else if (rov_move_control->rov_mode == Swiming_Mode)
    {
		rov_move_control->vf_set = vf_set;
		rov_move_control->vz_set = vz_set;
		rov_move_control->yaw_set = yaw_set;
		rov_move_control->roll_set = roll_set;
		rov_move_control->pitch_set = pitch_set;
		rov_move_control->vf_crawl_set = 0.0f;
		rov_move_control->wz_set = 0.0f;
    }
    else if (rov_move_control->rov_mode == Crawing_Mode)
    {
		rov_move_control->vf_set = 0.0f;
		rov_move_control->vz_set = vz_set;
		rov_move_control->yaw_set = 0.0f;
		rov_move_control->roll_set = 0.0f;
		rov_move_control->pitch_set = 0.0f;
		rov_move_control->vf_crawl_set = vf_set;
		//手柄下发的角度值变为履带旋转角速度
		rov_move_control->wz_set = yaw_set*ROV_ANGLE_TO_WZ_SEN;
    }
    else if (rov_move_control->rov_mode == No_Move_Mode)
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
  * @brief          根据控制解算各个推进器输出速度
  * @param[in]      yaw_control_out,: yaw轴控制量
  * @param[in]      pitch_control_out,: pitch轴控制量
  * @param[in]      roll_control_out,: roll轴控制量
  * @param[in]      depth_control_out,: 定深环控制量
	* @param[in]      vf_control_out,: 艏向控制量
  * @param[out]     thruster_speed: 六个推进器速度 从0-5分别为左上、右上、左下、右下、左侧、右侧
  * @retval         none
  */
static void rov_vector_to_thruster_speed(const fp32 yaw_control_out, const fp32 pitch_control_out, const fp32 roll_control_out,const fp32 depth_control_out,const fp32 vf_control_out , fp32 thruster_speed[6])
{
	//垂直推进器
	thruster_speed[0] = depth_control_out + roll_control_out - pitch_control_out; //右下
	thruster_speed[1] = depth_control_out - roll_control_out + pitch_control_out; //左上
	thruster_speed[2] = depth_control_out + roll_control_out + pitch_control_out; //右上
	thruster_speed[3] = depth_control_out - roll_control_out - pitch_control_out; //左下
	//水平推进器
	thruster_speed[4] = vf_control_out - THRUSTER_DISTANCE_TO_CENTER*yaw_control_out; //右
	thruster_speed[5] = vf_control_out + THRUSTER_DISTANCE_TO_CENTER*yaw_control_out; //左
//	if(rov_move.IMU_data->roll > 80)
//	{
//		thruster_speed[1] = 0.5 * depth_control_out - roll_control_out + pitch_control_out; //左上
//		thruster_speed[3] = 0.5 * depth_control_out - roll_control_out - pitch_control_out; //左下
//	}else if(rov_move.IMU_data->roll <-80)
//	{
//		thruster_speed[0] = 0.5 * depth_control_out + roll_control_out - pitch_control_out; //右下
//		thruster_speed[2] = 0.5 * depth_control_out + roll_control_out + pitch_control_out; //右上
//	}

}

/**
  * @brief          根据控制解算两个履带电机输出速度
  * @param[in]      track_wz_out,: 旋转速度控制量
	* @param[in]      track_vf_control_out,: 前进控制量
  * @param[out]     track_speed: 履带速度 分别为左侧、右侧
  * @retval         none
  */
static void rov_vector_to_track_speed(const fp32 track_wz_out,const fp32 track_vf_control_out , fp32 track_speed[2])
{
	track_speed[0] = -track_vf_control_out - TRACK_DISTANCE_TO_CENTER*track_wz_out;
	track_speed[1] = -track_vf_control_out + TRACK_DISTANCE_TO_CENTER*track_wz_out;
}

/**
  * @brief          控制循环，根据控制设定值，计算各环并且输出速度，进行控制
  * @param[out]     rov_move_control_loop:"rov_move"变量指针.
  * @retval         none
  */
static void rov_control_loop(rov_move_t *rov_move_control_loop)
{
	fp32 yaw_control_out, pitch_control_out, roll_control_out, depth_control_out, vf_control_out, track_turn_control_out, track_vf_control_out;
    fp32 thruster_speed[6] = {0.0f, 0.0f, 0.0f, 0.0f , 0.0f , 0.0f};
	fp32 track_speed[2] = {0.0f, 0.0f};
	uint8_t i = 0;
	//计算各环控制输出量
	rov_angle_loop_calc(&yaw_control_out, &pitch_control_out, &roll_control_out, rov_move_control_loop);
	rov_depth_loop_calc(&depth_control_out, rov_move_control_loop);
	track_turn_loop_calc(&track_turn_control_out, rov_move_control_loop);
	rov_vf_control_calc(&vf_control_out, &track_vf_control_out, rov_move_control_loop);
	//根据控制量，进行运动分解计算
	rov_vector_to_track_speed(track_turn_control_out,track_vf_control_out,track_speed);
	rov_vector_to_thruster_speed(yaw_control_out, pitch_control_out, roll_control_out, depth_control_out, vf_control_out , thruster_speed);
		
//		//直接发送履带电机电压值
//		if(rov_move_control_loop->rov_mode == Raw_Mode)
//		{
//			rov_move_control_loop->track_voltage_set[0] = (int16_t)track_speed[0];
//			rov_move_control_loop->track_voltage_set[1] = (int16_t)track_speed[1];
//		}
//		else
//		{
//			//calculate pid
//			//计算pid
//			for (i = 0; i < 2; i++)
//			{
//					PID_calc(&rov_move_control_loop->track_speed_pid[i], rov_move_control_loop->motor_rov.track_speed[i], track_speed[i]);
//			}
//			rov_move_control_loop->track_voltage_set[0] = (int16_t)rov_move_control_loop->track_speed_pid[0].out;
//			rov_move_control_loop->track_voltage_set[1] = (int16_t)rov_move_control_loop->track_speed_pid[1].out;
//		}
	//测试开环，需要删除
	rov_move_control_loop->track_voltage_set[0] = (int16_t)track_speed[0];
	rov_move_control_loop->track_voltage_set[1] = (int16_t)track_speed[1];
	for(i = 0; i < 6; i++)
	{
		rov_move_control_loop->thruster_speed_set[i] = (int16_t)thruster_speed[i];
	}
}




