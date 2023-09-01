/**
  ****************************(C) COPYRIGHT 2023 ZJU****************************
  * @file       rov_behaviour.c/h
  * @brief      ROV�Ķ�ģ����ʵ�֣����Լ����������ģʽ...         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     FEB-7-2023     Qiqi Li(������)    1. done
  *
  @verbatim
  ==============================================================================
	
	���Ҫ���һ���µ���Ϊģʽ
    1.���ȣ���rov_behaviour.h�ļ��У� ���һ������Ϊ������ rov_behaviour_e
    erum
    {  
        ...
        ...
        ROV_XXX_XXX, // ����ӵ�
    }rov_behaviour_e,

    2. ʵ��һ���µĺ��� rov_xxx_xxx_control(fp32 *vf, fp32 *vz, fp32 *yaw, fp32 *pitch, fp32 *roll ,rov_move_t * rov )
        "vf,vz,yaw,pitch,roll" ������ROV�˶�����������
        ��һ������: 'vf_set' ͨ�����������ƶ�,��ֵ ǰ���� ��ֵ ����
        �ڶ�������: 'vz_set' ͨ��������ȷ����ƶ�,��ֵ �ϸ�, ��ֵ ��Ǳ
        ����������: 'yaw_set' yaw��Ƕȿ���
				���ĸ�����: 'pitch_set' pitch��Ƕȿ���
				���������: 'roll_set' roll��Ƕȿ���
        ������µĺ���, ���ܸ� "vf_set","vy_set", "yaw_set","pitch_set" and "roll_set" ��ֵ��Ҫ���ٶȲ���
    3.  ��"rov_behaviour_mode_set"��������У�����µ��߼��жϣ���rov_behaviour_mode��ֵ��ROV_XXX_XXX
        �ں���������"else if(rov_behaviour_mode == ROV_XXX_XXX)" ,Ȼ��ѡ��һ��ROV����ģʽ
        4��:
        Swiming_Mode : ROV��������ģʽ�������Ĵ�
        Crawling_Mode : �Ĵ�����ģʽ����ʱ��Ҫ����vf��yaw
        No_Move_Mode : ֹͣģʽ����ʱ�������ȶ���ǰ״̬��������ҵ���ݲ�������
        Raw_Mode : ʹ��"vf_set","vy_set", "yaw_set","pitch_set" and "roll_set"ֱ�ӷ��͵�can����
    4.  ��"rov_behaviour_control_set" ������������
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
//���⣬���rov���Ʊ���
extern rov_move_t rov_move;
rov_mode_t *rov_mode = &rov_move.rov_mode;

/**
  * @brief          ����ROV��Ϊ
  * @param[in]     none
  * @retval         uint8_t
  */
uint8_t get_rov_behaviour(void)
{
	return *rov_mode;
}

/**
  * @brief          ROV�ǶȻ�������������ô���PID����
  * @param[in]      yaw_control_out yaw����������
  * @param[in]      pitch_control_out pitch����������
  * @param[in]      roll_control_out roll����������
  * @param[in]      rov_loop_calc ROV����
  * @retval         none
  */

void rov_angle_loop_calc(fp32 *yaw_control_out, fp32 *pitch_control_out, fp32 *roll_control_out, rov_move_t * rov_loop_calc)
{
	fp32 cur_yaw_set = rov_loop_calc->yaw_set;
	fp32 cur_roll_set = rov_loop_calc->roll_set;
	
	if(rov_loop_calc->rov_mode == ROV_ONLY_ATTHOLD || rov_loop_calc->rov_mode == ROV_NORMAL)
	{
		//��ȡ��ǰֵ
		fp32 yaw_cur = rov_loop_calc->IMU_data->yaw;
		//��ȡĿ��ֵ
		fp32 yaw_tar = rov_loop_calc->yaw_angle_set;
		//����Ƹ��ٶȹ�С������붨�������㣬���򿪻�����
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
		//���Ƹ��ٶȳ�������ʱ�����µ���Ŀ������
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
//		//����ǶȻ����⻷��
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
	* @brief        ROV����������
  * @param[in]      depth_control_out ������������
  * @param[in]      rov_loop_calc ROV����
  * @retval         none
  */

void rov_depth_loop_calc(fp32 *depth_control_out, rov_move_t * rov_loop_calc)
{
	fp32 cur_vz_set = rov_loop_calc->vz_set;
	if(rov_loop_calc->rov_mode == ROV_ONLY_ALTHOLD || rov_loop_calc->rov_mode == ROV_NORMAL)
	{
		//��ȡ��ǰֵ
		fp32 depth_cur = *rov_loop_calc->depth;
		//��ȡĿ��ֵ
		fp32 depth_tar = rov_loop_calc->depth_set;
		//����Ƹ��ٶȹ�С������붨����㣬���򿪻�����
		if(cur_vz_set <= ROV_DEPTH_HOLD_DEADLINE && cur_vz_set >= -ROV_DEPTH_HOLD_DEADLINE)
		{
			*depth_control_out = PID_calc(&rov_loop_calc->depth_loop_pid, depth_cur, depth_tar);
		}
		//���Ƹ��ٶȳ�������ʱ�����µ���Ŀ�����
		else
		{
			*depth_control_out = cur_vz_set*ROV_OPEN_HEAVE_SCALE;
			rov_loop_calc->last_rov_mode = ROV_ZERO_FORCE;
		}
		
		
		
//		//��ȡ��ǰֵ
//		fp32 depth_cur = *rov_loop_calc->depth;
//		//��ȡĿ��ֵ
//		fp32 depth_tar = rov_loop_calc->depth_set;
//		//������Ȼ�
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
	* @brief          ROV�Ĵ��������
  * @param[in]      track_wz_out �Ĵ���ת���������
  * @param[in]      rov_loop_calc ROV����
  * @retval         none
  */
void track_turn_loop_calc(fp32 *track_wz_out, rov_move_t * rov_loop_calc)
{
//	if(rov_behaviour_mode == ROV_CRAWLING || rov_behaviour_mode == ROV_STICK_WALL)
//	{
////		*track_wz_out = PID_calc(&rov_loop_calc->track_turn_loop_pid, rov_loop_calc->IMU_data->gz, rov_loop_calc->wz_set);
//			*track_wz_out = rov_loop_calc->wz_set * TRACK_OPEN_WZ_SCALE;//����
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
	* @brief        ROV�����ٶȿ��������������
  * @param[in]      vf_control_out �����ٶȿ������
  * @param[in]      track_vf_control_out �Ĵ�ģʽ��ǰ���ٶȿ������
  * @param[in]      rov_loop_calc ROV����
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
	




