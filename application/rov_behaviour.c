/**
  ****************************(C) COPYRIGHT 2023 ZJU****************************
  * @file       rov_behaviour.c/h
  * @brief      ROV的多模工作实现，可以继续添加其他模式...         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     FEB-7-2023     Qiqi Li(李琪琪)    1. done
  *
  @verbatim
  ==============================================================================
	
	如果要添加一个新的行为模式
    1.首先，在rov_behaviour.h文件中， 添加一个新行为名字在 rov_behaviour_e
    erum
    {  
        ...
        ...
        ROV_XXX_XXX, // 新添加的
    }rov_behaviour_e,

    2. 实现一个新的函数 rov_xxx_xxx_control(fp32 *vf, fp32 *vz, fp32 *yaw, fp32 *pitch, fp32 *roll ,rov_move_t * rov )
        "vf,vz,yaw,pitch,roll" 参数是ROV运动控制输入量
        第一个参数: 'vf_set' 通常控制艏向移动,正值 前进， 负值 后退
        第二个参数: 'vz_set' 通常控制深度方向移动,正值 上浮, 负值 下潜
        第三个参数: 'yaw_set' yaw轴角度控制
				第四个参数: 'pitch_set' pitch轴角度控制
				第五个参数: 'roll_set' roll轴角度控制
        在这个新的函数, 你能给 "vf_set","vy_set", "yaw_set","pitch_set" and "roll_set" 赋值想要的速度参数
    3.  在"rov_behaviour_mode_set"这个函数中，添加新的逻辑判断，给rov_behaviour_mode赋值成ROV_XXX_XXX
        在函数最后，添加"else if(rov_behaviour_mode == ROV_XXX_XXX)" ,然后选择一种ROV控制模式
        4种:
        Swiming_Mode : ROV正常浮游模式，不用履带
        Crawling_Mode : 履带爬行模式，此时主要控制vf和yaw
        No_Move_Mode : 停止模式，此时机器人稳定当前状态，进行作业（暂不开发）
        Raw_Mode : 使用"vf_set","vy_set", "yaw_set","pitch_set" and "roll_set"直接发送到can总线
    4.  在"rov_behaviour_control_set" 函数的最后，添加
        else if(rov_behaviour_mode == ROV_XXX_XXX)
        {
            rov_xxx_xxx_control(fp32 *vf, fp32 *vz, fp32 *yaw, fp32 *pitch, fp32 *roll ,rov_move_t * rov )
        }

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 ZJU****************************
  */
	
#include "rov_movation_task.h"
#include "rov_behaviour.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "socket_send.h"
#include "depth_meter_receive.h"

//highlight, the variable rov control 
//留意，这个rov控制变量
extern rov_move_t rov_move;
rov_mode_t *rov_mode = &rov_move.rov_mode;

/**
  * @brief          返回ROV行为
  * @param[in]     none
  * @retval         uint8_t
  */
uint8_t get_rov_behaviour(void)
{
	return *rov_mode;
}

/**
  * @brief          ROV角度环控制输出，采用串级PID控制
  * @param[in]      yaw_control_out yaw轴控制量输出
  * @param[in]      pitch_control_out pitch轴控制量输出
  * @param[in]      roll_control_out roll轴控制量输出
  * @param[in]      rov_loop_calc ROV数据
  * @retval         none
  */

void rov_angle_loop_calc(fp32 *yaw_control_out, fp32 *pitch_control_out, fp32 *roll_control_out, rov_move_t * rov_loop_calc)
{
	fp32 cur_yaw_set = rov_loop_calc->yaw_set;
	fp32 cur_roll_set = rov_loop_calc->roll_set;
	
	if(rov_loop_calc->rov_mode == ROV_ONLY_ATTHOLD || rov_loop_calc->rov_mode == ROV_NORMAL)
	{
		//获取当前值
		fp32 yaw_cur = rov_loop_calc->IMU_data->yaw;
		//获取目标值
		fp32 yaw_tar = rov_loop_calc->yaw_angle_set;
		//如果推杆速度过小，则进入定艏环计算，否则开环发送
		if(cur_yaw_set <= ROV_YAW_HOLD_DEADLINE && cur_yaw_set >= -ROV_YAW_HOLD_DEADLINE)
		{
			if(yaw_tar == 360) yaw_cur = 360;
			else
			{
				if(yaw_tar - yaw_cur > 180) yaw_cur += 360;
				else if(yaw_tar - yaw_cur < -180) yaw_cur -= 360;
			}
			*yaw_control_out = PID_calc(&rov_loop_calc->depth_loop_pid, yaw_cur, yaw_tar);
		}
		//当推杆速度超过死区时，重新调整目标艏向
		else
		{
			*yaw_control_out = cur_yaw_set*ROV_OPEN_YAW_SCALE;
			rov_loop_calc->last_rov_mode = ROV_ZERO_FORCE;
		}
	


//		if(yaw_tar == 360) yaw_cur = 360;
//		else
//		{
//			if(yaw_tar - yaw_cur > 180) yaw_cur += 360;
//			else if(yaw_tar - yaw_cur < -180) yaw_cur -= 360;
//		}
//		//计算角度环（外环）
//		*yaw_control_out = PID_calc(&rov_loop_calc->yaw_angle_pid, yaw_cur, yaw_tar);
	}
	else
	{
//		*yaw_control_out = rov_loop_calc->yaw_set*ROV_OPEN_ANGLE_SCALE;
//		*roll_control_out = rov_loop_calc->roll_set*ROV_OPEN_ANGLE_SCALE;
		*pitch_control_out = rov_loop_calc->pitch_set*ROV_OPEN_PIT_SCALE;
		
		if(cur_roll_set > (ROV_MAX_THR_ROLL_SET / 2))
		{
			*roll_control_out = cur_roll_set * ROV_OPEN_ROL_SCALE;
		}
		else if(cur_roll_set >= - (ROV_MAX_THR_ROLL_SET / 2) && cur_roll_set <= (ROV_MAX_THR_ROLL_SET / 2))
		{
			*roll_control_out = 0;
		}
		else if(cur_roll_set < -(ROV_MAX_THR_ROLL_SET / 2))
		{
			*roll_control_out = cur_roll_set * ROV_OPEN_ROL_SCALE;
		}
		
		if(cur_yaw_set > (ROV_MAX_THR_YAW_SET / 2))
		{
			*yaw_control_out = cur_yaw_set * ROV_OPEN_YAW_SCALE;
		}
		else if(cur_yaw_set >= - (ROV_MAX_THR_YAW_SET / 2) && cur_yaw_set <= (ROV_MAX_THR_YAW_SET / 2))
		{
			*yaw_control_out = 0;
		}
		else if(cur_yaw_set < -(ROV_MAX_THR_YAW_SET / 2))
		{
			*yaw_control_out = cur_yaw_set * ROV_OPEN_YAW_SCALE;
		}
	}
	
	
}

/**
	* @brief        ROV定深控制输出
  * @param[in]      depth_control_out 定深控制量输出
  * @param[in]      rov_loop_calc ROV数据
  * @retval         none
  */

void rov_depth_loop_calc(fp32 *depth_control_out, rov_move_t * rov_loop_calc)
{
	fp32 cur_vz_set = rov_loop_calc->vz_set;
	if(rov_loop_calc->rov_mode == ROV_ONLY_ALTHOLD || rov_loop_calc->rov_mode == ROV_NORMAL)
	{
		//获取当前值
		fp32 depth_cur = *rov_loop_calc->depth;
		//获取目标值
		fp32 depth_tar = rov_loop_calc->depth_set;
		//如果推杆速度过小，则进入定深环计算，否则开环发送
		if(cur_vz_set <= ROV_DEPTH_HOLD_DEADLINE && cur_vz_set >= -ROV_DEPTH_HOLD_DEADLINE)
		{
			*depth_control_out = PID_calc(&rov_loop_calc->depth_loop_pid, depth_cur, depth_tar);
		}
		//当推杆速度超过死区时，重新调整目标深度
		else
		{
			*depth_control_out = cur_vz_set*ROV_OPEN_HEAVE_SCALE;
			rov_loop_calc->last_rov_mode = ROV_ZERO_FORCE;
		}
		
		
		
//		//获取当前值
//		fp32 depth_cur = *rov_loop_calc->depth;
//		//获取目标值
//		fp32 depth_tar = rov_loop_calc->depth_set;
//		//计算深度环
//		*depth_control_out = PID_calc(&rov_loop_calc->depth_loop_pid, depth_cur, depth_tar);
	}
	else
	{
//		*depth_control_out = cur_vz_set*ROV_OPEN_VELOCITY_SCALE;
		if(rov_loop_calc->rov_mode == ROV_STICK_WALL)
		{
			*depth_control_out = 2400;
		}
		else
		{
			if(cur_vz_set> ROV_MAX_THR_HEAVE_SET / 2)
			{
				*depth_control_out = cur_vz_set * ROV_OPEN_HEAVE_SCALE;
			}
			else if(cur_vz_set >= -(ROV_MAX_THR_HEAVE_SET / 2) && cur_vz_set <= (ROV_MAX_THR_HEAVE_SET / 2))
			{
				*depth_control_out = 0;
			}
			else if(cur_vz_set < -(ROV_MAX_THR_HEAVE_SET / 2))
			{
				*depth_control_out = cur_vz_set * ROV_OPEN_HEAVE_SCALE;
			}
		}
	}
}

/**
	* @brief          ROV履带控制输出
  * @param[in]      track_wz_out 履带旋转控制量输出
  * @param[in]      rov_loop_calc ROV数据
  * @retval         none
  */
void track_turn_loop_calc(fp32 *track_wz_out, rov_move_t * rov_loop_calc)
{
//	if(rov_behaviour_mode == ROV_CRAWLING || rov_behaviour_mode == ROV_STICK_WALL)
//	{
////		*track_wz_out = PID_calc(&rov_loop_calc->track_turn_loop_pid, rov_loop_calc->IMU_data->gz, rov_loop_calc->wz_set);
//			*track_wz_out = rov_loop_calc->wz_set * TRACK_OPEN_WZ_SCALE;//调试
//	}
//	else
//	{
//		*track_wz_out = rov_loop_calc->wz_set * TRACK_OPEN_WZ_SCALE;
//	}
	uint16_t half_max_yaw_set = ROV_MAX_YAW_SET / 2;
	if(rov_loop_calc->wz_set > half_max_yaw_set)
	{
		*track_wz_out = 2 * half_max_yaw_set * TRACK_OPEN_WZ_SCALE;
	}
	else if(rov_loop_calc->wz_set >= -half_max_yaw_set && rov_loop_calc->wz_set <= half_max_yaw_set)
	{
		*track_wz_out = 0;
	}
	else if(rov_loop_calc->wz_set < -half_max_yaw_set)
	{
		*track_wz_out = -2 * half_max_yaw_set * TRACK_OPEN_WZ_SCALE;
	}
}

/**
	* @brief        ROV艏向速度控制输出（开环）
  * @param[in]      vf_control_out 艏向速度控制输出
  * @param[in]      track_vf_control_out 履带模式下前进速度控制输出
  * @param[in]      rov_loop_calc ROV数据
  * @retval         none
  */
void rov_vf_control_calc(fp32 *vf_control_out, fp32 *track_vf_control_out, rov_move_t * rov_loop_calc)
{
//	*track_vf_control_out = rov_loop_calc->vf_crawl_set * TRACK_OPEN_VELOCITY_SCALE;
	fp32 cur_vf_crawl_set = rov_loop_calc->vf_crawl_set;
	fp32 cur_vf_set = rov_loop_calc->vf_set;
	
	if(cur_vf_crawl_set > (ROV_MAX_TRA_FORWARD_SET / 2))
	{
		*track_vf_control_out = cur_vf_crawl_set * TRACK_OPEN_VELOCITY_SCALE;
	}
	else if(cur_vf_crawl_set >= -(ROV_MAX_TRA_FORWARD_SET / 2) && cur_vf_crawl_set <= (ROV_MAX_TRA_FORWARD_SET / 2))
	{
		*track_vf_control_out = 0;
	}
	else if(cur_vf_crawl_set < -(ROV_MAX_TRA_FORWARD_SET / 2))
	{
		*track_vf_control_out = cur_vf_crawl_set * TRACK_OPEN_VELOCITY_SCALE;
	}
	
//	*vf_control_out = rov_loop_calc->vf_set * ROV_OPEN_VELOCITY_SCALE;
	if(cur_vf_set > ROV_MAX_THR_FORWARD_SET / 2)
	{
		*vf_control_out = cur_vf_set * ROV_OPEN_FORWAD_SCALE;
	}
	else if(cur_vf_set >= -(ROV_MAX_THR_FORWARD_SET / 2)  && cur_vf_set <= (ROV_MAX_THR_FORWARD_SET / 2))
	{
		*vf_control_out = 0;
	}
	else if(cur_vf_set < -(ROV_MAX_THR_FORWARD_SET / 2))
	{
		*vf_control_out = cur_vf_set * ROV_OPEN_FORWAD_SCALE;
	}
	
}
	




