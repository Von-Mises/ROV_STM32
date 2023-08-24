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
#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "struct_typedef.h"
#include "rov_movation_task.h"

#define ROV_OPEN_YAW_SCALE        10 					// 开环模式下YAW角比例系数
#define ROV_OPEN_ROL_SCALE        14 					// 开环模式下YAW角比例系数
#define ROV_OPEN_PIT_SCALE        14				    // 开环模式下YAW角比例系数
#define ROV_OPEN_FORWAD_SCALE     20 			        // 开环模式下前后速度的比例系数
#define ROV_OPEN_HEAVE_SCALE      15                    // 开环模式下升沉速度的比例系数
#define TRACK_OPEN_WZ_SCALE       50 					// 开环模式下履带旋转的比例系数
#define TRACK_OPEN_VELOCITY_SCALE 20 				 	// 开环模式下履带直线运动的比例系数
#define ROV_DEPTH_HOLD_DEADLINE   50 					// 定深模式下，让ROV保持定深模式的死区速度，大于这个速度将开环发送给垂直推进器
#define ROV_YAW_HOLD_DEADLINE     90					// 定艏模式下，让ROV保持定艏模式的死区速度，大于这个速度将开环发送给垂直推进器
#define ROV_MAX_YAW_SET           180                   // 爬行转向控制下发幅值
#define ROV_MAX_TRA_FORWARD_SET   100					// 爬行前进控制下发幅值
#define ROV_MAX_THR_FORWARD_SET   100					// 推进前进控制下发幅值
#define ROV_MAX_THR_YAW_SET       180         // 推进转向控制下发幅值
#define ROV_MAX_THR_ROLL_SET      180         // 推进横滚控制下发幅值
#define ROV_MAX_THR_HEAVE_SET     100         // 推进升沉控制下发幅值

/* ----------------------- Extern Function ----------------------------------- */

/**
	* @brief          ROV无力的行为状态机下，控制模式是raw，故而设定值会直接发送到can总线上或PWM开环发送故而将设定值都设置为0
	* @param[in]      vf_set前进的速度 设定值将开环发送
  * @param[in]      vz_set左右的速度 设定值将开环发送
  * @param[in]      yaw_set yaw轴稳定的角度 设定值将开环发送
  * @param[in]      pitch_set pitch轴稳定的角度 设定值将开环发送
  * @param[in]      roll_set roll轴稳定的角度 设定值将开环发送
  * @param[in]      rov_set_to_vector ROV数据
  * @retval         none
  */

static void rov_zero_force_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector);

/**
	* @brief          ROV开环模式
	* @param[in]      vf_set前进的速度 设定值将开环发送
  * @param[in]      vz_set左右的速度 设定值将开环发送
  * @param[in]      yaw_set yaw轴稳定的角度 设定值将开环发送
  * @param[in]      pitch_set pitch轴稳定的角度 设定值将开环发送
  * @param[in]      roll_set roll轴稳定的角度 设定值将开环发送
  * @param[in]      rov_set_to_vector ROV数据
  * @retval         none
  */

static void rov_open_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector);

/**
	* @brief          ROV只开启定深模式
	* @param[in]      vf_set前进的速度 设定值将开环发送
  * @param[in]      vz_set左右的速度 设定值将开环发送
  * @param[in]      yaw_set yaw轴稳定的角度 设定值将开环发送
  * @param[in]      pitch_set pitch轴稳定的角度
  * @param[in]      roll_set roll轴稳定的角度
  * @param[in]      rov_set_to_vector ROV数据
  * @retval         none
  */

static void rov_only_althold_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector);

/**
	* @brief          ROV只开启稳姿
	* @param[in]      vf_set前进的速度 
  * @param[in]      vz_set左右的速度 
  * @param[in]      yaw_set yaw轴稳定的角度 
  * @param[in]      pitch_set pitch轴稳定的角度 
  * @param[in]      roll_set roll轴稳定的角度
  * @param[in]      rov_set_to_vector ROV数据
  * @retval         none
  */

static void rov_only_atthold_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector);


/**
	* @brief          ROV正常浮游状态机下的
	* @param[in]      vf_set前进的速度 设定值将开环发送
  * @param[in]      vz_set左右的速度 设定值将开环发送
  * @param[in]      yaw_set yaw轴稳定的角度 设定值将开环发送
  * @param[in]      pitch_set pitch轴稳定的角度 设定值将开环发送
  * @param[in]      roll_set roll轴稳定的角度 设定值将开环发送
  * @param[in]      rov_set_to_vector ROV数据
  * @retval         none
  */

static void rov_normal_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector);

/**
	* @brief          ROV陆地上前进，故推进器全部关闭
	* @param[in]      vf_set前进的速度
  * @param[in]      vz_set左右的速度 设定值将开环发送
  * @param[in]      yaw_set yaw轴稳定的角度 设定值将开环发送
  * @param[in]      pitch_set pitch轴稳定的角度 设定值将开环发送
  * @param[in]      roll_set roll轴稳定的角度 设定值将开环发送
  * @param[in]      rov_set_to_vector ROV数据
  * @retval         none
  */

static void rov_crawling_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector);


/**
	* @brief          ROV贴壁状态下，四个垂直推进器将开环发送推力，履带电机闭环
	* @param[in]      vf_set前进的速度 
  * @param[in]      vz_set左右的速度 设定值将开环发送
  * @param[in]      yaw_set yaw轴稳定的角度 设定值将开环发送
  * @param[in]      pitch_set pitch轴稳定的角度 设定值将开环发送
  * @param[in]      roll_set roll轴稳定的角度 设定值将开环发送
  * @param[in]      rov_set_to_vector ROV数据
  * @retval         none
  */

static void rov_stick_wall_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector);



/**
	* @brief          ROV停止模式，此时机器人稳定当前状态，进行作业（暂不开发）
	* @param[in]      vf_set前进的速度 设定值将开环发送
  * @param[in]      vz_set左右的速度 设定值将开环发送
  * @param[in]      yaw_set yaw轴稳定的角度 设定值将开环发送
  * @param[in]      pitch_set pitch轴稳定的角度 设定值将开环发送
  * @param[in]      roll_set roll轴稳定的角度 设定值将开环发送
  * @param[in]      rov_set_to_vector ROV数据
  * @retval         none
  */

static void rov_no_move_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector);

/**
  * @brief          通过逻辑判断，赋值"rov_behaviour_mode"成哪种模式
  * @param[in]      rov_move_mode: ROV数据
  * @retval         none
  */
void rov_behaviour_mode_set(rov_move_t *rov_move_mode);


/**
  * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
	* @param[in]      vf_set前进的速度 设定值将开环发送
  * @param[in]      vz_set左右的速度 设定值将开环发送
  * @param[in]      yaw_set yaw轴稳定的角度 设定值将开环发送
  * @param[in]      pitch_set pitch轴稳定的角度 设定值将开环发送
  * @param[in]      roll_set roll轴稳定的角度 设定值将开环发送
  * @param[in]      rov_set_to_vector ROV数据
  * @retval         none
  */

extern void rov_behaviour_control_set(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector);

/**
	* @brief          ROV角度环控制输出，采用串级PID控制
  * @param[in]      yaw_control_out yaw轴控制量输出
  * @param[in]      pitch_control_out pitch轴控制量输出
  * @param[in]      roll_control_out roll轴控制量输出
  * @param[in]      rov_loop_calc ROV数据
  * @retval         none
  */

extern void rov_angle_loop_calc(fp32 *yaw_control_out, fp32 *pitch_control_out, fp32 *roll_control_out, rov_move_t * rov_loop_calc);


/**
	* @brief          ROV定深控制输出
  * @param[in]      depth_control_out 定深控制量输出
  * @param[in]      rov_loop_calc ROV数据
  * @retval         none
  */

extern void rov_depth_loop_calc(fp32 *depth_control_out, rov_move_t * rov_loop_calc);

/**
	* @brief          ROV履带控制输出
  * @param[in]      track_wz_out 履带旋转控制量输出
  * @param[in]      rov_loop_calc ROV数据
  * @retval         none
  */
extern void track_turn_loop_calc(fp32 *track_wz_out, rov_move_t * rov_loop_calc);

/**
	* @brief          ROV艏向速度控制输出（开环）
  * @param[in]      vf_control_out 艏向速度控制输出
  * @param[in]      track_vf_control_out 履带模式下前进速度控制输出
  * @param[in]      rov_loop_calc ROV数据
  * @retval         none
  */
extern void rov_vf_control_calc(fp32 *vf_control_out, fp32 *track_vf_control_out, rov_move_t * rov_loop_calc);

/**
  * @brief          返回ROV行为
  * @param[in]     none
  * @retval         uint8_t
  */
extern uint8_t get_rov_behaviour(void);

#endif
