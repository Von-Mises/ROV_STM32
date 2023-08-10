/**
  ****************************(C) COPYRIGHT 2023 ZJU****************************
  * @file       rov_behaviour.c/h
  * @brief      ROV的多模工作实现，可以继续添加其他模式...         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     FEB-7-2023     HaoLion(郝亮亮)    1. done
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
        Crawing_Mode : 履带爬行模式，此时主要控制vf和yaw
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


//highlight, the variable rov behaviour mode 
//留意，这个底盘行为模式变量
rov_behaviour_e rov_behaviour_mode = ROV_ZERO_FORCE;


#define Over_Zero_Handlle(tar, cur, Half_T)        			 \
    {                                                    \
        if ((tar)-(cur) > (2)*(Half_T)) 								 \
        {                                                \
            (cur) += (Half_T);                           \
        }                                                \
        else if ((tar)-(cur) < (-2)*(Half_T))            \
        {                                                \
            (cur) -= (Half_T);                           \
        }                                                \
    }




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
  * @brief          返回ROV行为
  * @param[in]     none
  * @retval         uint8_t
  */
uint8_t get_rov_behaviour(void)
{
	return rov_behaviour_mode;
}

/**
  * @brief          通过逻辑判断，赋值"rov_behaviour_mode"成哪种模式
  * @param[in]      rov_move_mode: ROV数据
  * @retval         none
  */
void rov_behaviour_mode_set(rov_move_t *rov_move_mode)
{
    if (rov_move_mode == NULL)
    {
        return;
    }

    //根据控制命令设置模式
    if (rov_move_mode->rov_Ctrl->Mode == ROV_OPEN)
    {
        //开环模式
        rov_behaviour_mode = ROV_OPEN;
    }
	
    else if (rov_move_mode->rov_Ctrl->Mode == ROV_ONLY_ALTHOLD)
    {
		//只开启定深
		rov_behaviour_mode = ROV_ONLY_ALTHOLD;
    }
	
	else if (rov_move_mode->rov_Ctrl->Mode == ROV_ONLY_ATTHOLD)
	{
		//只开启定姿
		rov_behaviour_mode = ROV_ONLY_ATTHOLD;
		rov_move_mode->yaw_angle_set = rov_move_mode->IMU_data->yaw;
	}
	
    else if (rov_move_mode->rov_Ctrl->Mode == ROV_NORMAL)
    { 
		//ROV正常浮游模式，开启了定深和定艏
		rov_behaviour_mode =  ROV_NORMAL;
    }
	
	else if (rov_move_mode->rov_Ctrl->Mode == ROV_CRAWLING)
	{
		//ROV陆地爬行模式
		rov_behaviour_mode = ROV_CRAWLING;
	}
	
	else if (rov_move_mode->rov_Ctrl->Mode == ROV_STICK_WALL)
	{
		//ROV贴壁模式
		rov_behaviour_mode = ROV_STICK_WALL;
	}
	
	else if (rov_move_mode->rov_Ctrl->Mode == ROV_ZERO_FORCE)
	{
		//ROV无力模式，即所有电机转速开环为0
		rov_behaviour_mode = ROV_ZERO_FORCE;
	}
	
	else if (rov_move_mode->rov_Ctrl->Mode == ROV_NO_MOVE)
	{
		//停止模式，此时机器人稳定当前状态，进行作业（暂不开发）
		rov_behaviour_mode = ROV_NO_MOVE;
	}

    //add your own logic to enter the new mode
    //添加自己的逻辑判断进入新模式


    //accord to beheviour mode, choose rov control mode
    //根据行为模式选择一个ROV控制模式
    if (rov_behaviour_mode == ROV_OPEN)
    {
        rov_move_mode->rov_mode = Raw_Mode; 
    }
    else if (rov_behaviour_mode == ROV_ONLY_ALTHOLD)
    {
        rov_move_mode->rov_mode = Swiming_Mode;
    }
		else if (rov_behaviour_mode == ROV_ONLY_ATTHOLD)
    {
        rov_move_mode->rov_mode = Swiming_Mode;
    }
    else if (rov_behaviour_mode == ROV_NORMAL)
    {
        rov_move_mode->rov_mode = Swiming_Mode;
    }
    else if (rov_behaviour_mode == ROV_CRAWLING)
    {
        rov_move_mode->rov_mode = Crawing_Mode;
    }
    else if (rov_behaviour_mode == ROV_STICK_WALL)
    {
        rov_move_mode->rov_mode = Crawing_Mode;
    }
    else if (rov_behaviour_mode == ROV_ZERO_FORCE)
    {
        rov_move_mode->rov_mode = Raw_Mode;
    }
    else if (rov_behaviour_mode == ROV_NO_MOVE)
    {
		rov_move_mode->rov_mode = No_Move_Mode;
    }
}

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

void rov_behaviour_control_set(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector)
{

	if (vf_set == NULL || vz_set == NULL || yaw_set == NULL || pitch_set == NULL || roll_set == NULL || rov_set_to_vector == NULL)
    {
        return;
    }

    if (rov_behaviour_mode == ROV_ZERO_FORCE)
    {
        rov_zero_force_control(vf_set, vz_set, yaw_set, pitch_set, roll_set ,rov_set_to_vector);
    }
    else if (rov_behaviour_mode == ROV_OPEN)
    {
        rov_open_control(vf_set, vz_set, yaw_set, pitch_set, roll_set ,rov_set_to_vector);
    }
		else if (rov_behaviour_mode == ROV_ONLY_ALTHOLD)
    {
        rov_only_althold_control(vf_set, vz_set, yaw_set, pitch_set, roll_set ,rov_set_to_vector);
    }
		else if (rov_behaviour_mode == ROV_ONLY_ATTHOLD)
    {
        rov_only_atthold_control(vf_set, vz_set, yaw_set, pitch_set, roll_set ,rov_set_to_vector);
    }
		else if (rov_behaviour_mode == ROV_NORMAL)
    {
        rov_normal_control(vf_set, vz_set, yaw_set, pitch_set, roll_set ,rov_set_to_vector);
    }
		else if (rov_behaviour_mode == ROV_CRAWLING)
    {
        rov_crawling_control(vf_set, vz_set, yaw_set, pitch_set, roll_set ,rov_set_to_vector);
    }
		else if (rov_behaviour_mode == ROV_STICK_WALL)
    {
        rov_stick_wall_control(vf_set, vz_set, yaw_set, pitch_set, roll_set ,rov_set_to_vector);
    }
		else if (rov_behaviour_mode == ROV_NO_MOVE)
    {
        rov_no_move_control(vf_set, vz_set, yaw_set, pitch_set, roll_set ,rov_set_to_vector);
    }
}


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

static void rov_zero_force_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector)
{
    if (vf_set == NULL || vz_set == NULL || yaw_set == NULL || pitch_set == NULL || roll_set == NULL || rov_set_to_vector == NULL)
    {
        return;
    }
    *vf_set = 0.0f;
    *vz_set = 0.0f;
	*yaw_set = 0.0f;
    *pitch_set = 0.0f;
	*roll_set = 0.0f;
}



/**
	* @brief          ROV正常浮游状态机下的
	* @param[in]      vf_set前进的速度 
  * @param[in]      vz_set左右的速度 
  * @param[in]      yaw_set yaw轴稳定的角度 
  * @param[in]      pitch_set pitch轴稳定的角度 
  * @param[in]      roll_set roll轴稳定的角度
  * @param[in]      rov_set_to_vector ROV数据
  * @retval         none
  */

static void rov_normal_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector)
{
    if (vf_set == NULL || vz_set == NULL || yaw_set == NULL || pitch_set == NULL || roll_set == NULL || rov_set_to_vector == NULL)
    {
        return;
    }
		
		//低通滤波防止下发角度变化太快
	rov_rc_to_control_vector(yaw_set, roll_set, rov_set_to_vector);
	
	*vf_set = rov_set_to_vector->rov_Ctrl->VF;
    *vz_set = rov_set_to_vector->rov_Ctrl->VZ;
    *pitch_set = 0.0f;
    
}

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

static void rov_crawling_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector)
{
    if (vf_set == NULL || vz_set == NULL || yaw_set == NULL || pitch_set == NULL || roll_set == NULL || rov_set_to_vector == NULL)
    {
        return;
    }
		
	*vf_set = rov_set_to_vector->rov_Ctrl->VF;
    *vz_set = 0.0f;
	*yaw_set = rov_set_to_vector->rov_Ctrl->YAW;
		
    *pitch_set = 0.0f;
	*roll_set = 0.0f;
    
}

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

static void rov_stick_wall_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector)
{
    if (vf_set == NULL || vz_set == NULL || yaw_set == NULL || pitch_set == NULL || roll_set == NULL || rov_set_to_vector == NULL)
    {
        return;
    }
		
	*vf_set = rov_set_to_vector->rov_Ctrl->VF;
    *vz_set = 100.0f;
	*yaw_set = rov_set_to_vector->rov_Ctrl->YAW;
    *pitch_set = 0.0f;
	*roll_set = 0.0f;
    
}

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

static void rov_only_althold_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector)
{
    if (vf_set == NULL || vz_set == NULL || yaw_set == NULL || pitch_set == NULL || roll_set == NULL || rov_set_to_vector == NULL)
    {
        return;
    }
		
	*vf_set = rov_set_to_vector->rov_Ctrl->VF;
    *vz_set = rov_set_to_vector->rov_Ctrl->VZ;
	//开环发送
	*yaw_set = rov_set_to_vector->rov_Ctrl->YAW;
    *pitch_set = 0.0f;
	*roll_set = 0.0f;
    
}

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

static void rov_only_atthold_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector)
{
    if (vf_set == NULL || vz_set == NULL || yaw_set == NULL || pitch_set == NULL || roll_set == NULL || rov_set_to_vector == NULL)
    {
        return;
    }
		
	//低通滤波防止下发角度变化太快
	rov_rc_to_control_vector(yaw_set, roll_set, rov_set_to_vector);
	
	*vf_set = rov_set_to_vector->rov_Ctrl->VF;
    *vz_set = rov_set_to_vector->rov_Ctrl->VZ;
    *pitch_set = 0.0f;
    
}

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

static void rov_open_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector)
{
    if (vf_set == NULL || vz_set == NULL || yaw_set == NULL || pitch_set == NULL || roll_set == NULL || rov_set_to_vector == NULL)
    {
        return;
    }
	//低通滤波防止下发角度变化太快
//		rov_rc_to_control_vector(yaw_set, roll_set, rov_set_to_vector);
	*yaw_set = rov_set_to_vector->rov_Ctrl->YAW;
	*roll_set = rov_set_to_vector->rov_Ctrl->ROL;
		
	*vf_set = rov_set_to_vector->rov_Ctrl->VF;
    *vz_set = rov_set_to_vector->rov_Ctrl->VZ;
    *pitch_set = 0.0f;
    
}

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

static void rov_no_move_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector)
{
    if (vf_set == NULL || vz_set == NULL || yaw_set == NULL || pitch_set == NULL || roll_set == NULL || rov_set_to_vector == NULL)
    {
        return;
    }
		//暂不开发
		*vf_set = 0.0f;
		*vz_set = 0.0f;
		*yaw_set = 0.0f;
		*pitch_set = 0.0f;
		*roll_set = 0.0f;
    
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
	if(rov_behaviour_mode == ROV_NORMAL || rov_behaviour_mode == ROV_ONLY_ATTHOLD)
	{
		fp32 yaw_cur = 0.0f, pitch_cur = 0.0f, roll_cur = 0.0f;
		fp32 yaw_tar= 0.0f, pitch_tar = 0.0f, roll_tar = 0.0f;
		//获取当前值
		yaw_cur = rov_loop_calc->IMU_data->yaw;
		pitch_cur = rov_loop_calc->IMU_data->pitch;
		roll_cur = rov_loop_calc->IMU_data->roll;
		//获取目标值
		yaw_tar = rov_loop_calc->yaw_angle_set;
//		yaw_tar = rov_loop_calc->IMU_data->yaw;
//		pitch_tar = rov_loop_calc->IMU_data->yaw;
//		roll_tar = rov_loop_calc->IMU_data->yaw;
		//过零处理
		//Over_Zero_Handlle(yaw_tar,yaw_cur,180);
		//计算角度环（外环）
		PID_calc(&rov_loop_calc->yaw_angle_pid, yaw_cur, yaw_tar);
//		PID_calc(&rov_loop_calc->pitch_angle_pid, pitch_cur, pitch_tar);
//		PID_calc(&rov_loop_calc->roll_angle_pid, roll_cur, roll_tar);
		//计算角速度环（内环）
		*yaw_control_out = PID_calc(&rov_loop_calc->yaw_angular_velocity_pid, rov_loop_calc->IMU_data->gz, rov_loop_calc->yaw_angle_pid.out);
//		*pitch_control_out = PID_calc(&rov_loop_calc->pitch_angular_velocity_pid, rov_loop_calc->IMU_data->gx, rov_loop_calc->pitch_angle_pid.out);
//		*roll_control_out = PID_calc(&rov_loop_calc->roll_angular_velocity_pid, rov_loop_calc->IMU_data->gy, rov_loop_calc->roll_angle_pid.out);
	}
	else
	{
//		*yaw_control_out = rov_loop_calc->yaw_set*ROV_OPEN_ANGLE_SCALE;
//		*roll_control_out = rov_loop_calc->roll_set*ROV_OPEN_ANGLE_SCALE;
		*pitch_control_out = rov_loop_calc->pitch_set*ROV_OPEN_ANGLE_SCALE;
		
		uint16_t half_max_thr_roll_set = ROV_MAX_THR_ROLL_SET * ROV_OPEN_ANGLE_SCALE / 2;
		uint16_t half_max_roll_set = ROV_MAX_THR_ROLL_SET / 2;
		if(rov_loop_calc->roll_set > half_max_roll_set)
		{
			*roll_control_out = 2 * half_max_thr_roll_set;
		}
		else if(rov_loop_calc->roll_set >= -half_max_roll_set && rov_loop_calc->roll_set <= half_max_roll_set)
		{
			*roll_control_out = 0;
		}
		else if(rov_loop_calc->roll_set < -half_max_roll_set)
		{
			*roll_control_out = -2 * half_max_thr_roll_set;
		}
		
		uint16_t half_max_thr_yaw_set = ROV_MAX_THR_YAW_SET * ROV_OPEN_ANGLE_SCALE / 2;
		uint16_t half_max_yaw_set = ROV_MAX_THR_YAW_SET / 2;
		if(rov_loop_calc->yaw_set > half_max_yaw_set)
		{
			*yaw_control_out = 2 * half_max_thr_yaw_set;
		}
		else if(rov_loop_calc->yaw_set >= -half_max_yaw_set && rov_loop_calc->yaw_set <= half_max_yaw_set)
		{
			*yaw_control_out = 0;
		}
		else if(rov_loop_calc->yaw_set < -half_max_yaw_set)
		{
			*yaw_control_out = -2 * half_max_thr_yaw_set;
		}
	}
	
	
}

/**
	* @brief          ROV定深控制输出
  * @param[in]      depth_control_out 定深控制量输出
  * @param[in]      rov_loop_calc ROV数据
  * @retval         none
  */

void rov_depth_loop_calc(fp32 *depth_control_out, rov_move_t * rov_loop_calc)
{
	static fp32 last_vz_set = 0; 
	fp32 cur_vz_set = rov_loop_calc->vz_set;
	if(rov_behaviour_mode == ROV_ONLY_ALTHOLD || rov_behaviour_mode == ROV_NORMAL)
	{
		//当推杆进入稳定区时候，重新设置定深值
		if((last_vz_set > ROV_DEPTH_HOLD_DEADLINE && cur_vz_set <= ROV_DEPTH_HOLD_DEADLINE ) 
			|| (last_vz_set < -ROV_DEPTH_HOLD_DEADLINE && cur_vz_set >= -ROV_DEPTH_HOLD_DEADLINE))
		{
			rov_loop_calc->depth_set = rov_loop_calc->depth;
		}
		//如果推杆速度过小，则进入定深环计算，否则开环发送
		if(cur_vz_set <= ROV_DEPTH_HOLD_DEADLINE && cur_vz_set >= -ROV_DEPTH_HOLD_DEADLINE)
		{
			*depth_control_out = PID_calc(&rov_loop_calc->depth_loop_pid, rov_loop_calc->depth, rov_loop_calc->depth_set);
		}
		else
		{
			*depth_control_out = cur_vz_set*ROV_OPEN_VELOCITY_SCALE;
		}
	}
	else
	{
//		*depth_control_out = cur_vz_set*ROV_OPEN_VELOCITY_SCALE;
		
		uint16_t half_max_thr_heave_set = ROV_MAX_THR_HEAVE_SET * 25 / 2;
		uint16_t half_max_heave_set = ROV_MAX_THR_HEAVE_SET / 2;
		if(cur_vz_set> half_max_heave_set)
		{
			*depth_control_out = 2 * half_max_thr_heave_set;
		}
		else if(cur_vz_set >= -half_max_heave_set && rov_loop_calc->yaw_set <= half_max_heave_set)
		{
			*depth_control_out = 0;
		}
		else if(cur_vz_set < -half_max_heave_set)
		{
			*depth_control_out = -2 * half_max_thr_heave_set;
		}
		

	}
		last_vz_set = cur_vz_set;
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
	uint16_t half_max_yaw_set = ROV_MAX_YAW_SET * ROV_ANGLE_TO_WZ_SEN / 2;
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
	* @brief          ROV艏向速度控制输出（开环）
  * @param[in]      vf_control_out 艏向速度控制输出
  * @param[in]      track_vf_control_out 履带模式下前进速度控制输出
  * @param[in]      rov_loop_calc ROV数据
  * @retval         none
  */
void rov_vf_control_calc(fp32 *vf_control_out, fp32 *track_vf_control_out, rov_move_t * rov_loop_calc)
{
//	*track_vf_control_out = rov_loop_calc->vf_crawl_set * TRACK_OPEN_VELOCITY_SCALE;
	uint16_t half_tra_max_forward_set = ROV_MAX_TRA_FORWARD_SET * TRACK_OPEN_VELOCITY_SCALE / 2;
	if(rov_loop_calc->vf_crawl_set > (ROV_MAX_TRA_FORWARD_SET / 2))
	{
		*track_vf_control_out = 2 * half_tra_max_forward_set;
	}
	else if(rov_loop_calc->vf_crawl_set >= -(ROV_MAX_TRA_FORWARD_SET / 2) && rov_loop_calc->wz_set <= (ROV_MAX_TRA_FORWARD_SET / 2))
	{
		*track_vf_control_out = 0;
	}
	else if(rov_loop_calc->vf_crawl_set < -(ROV_MAX_TRA_FORWARD_SET / 2))
	{
		*track_vf_control_out = -2 * half_tra_max_forward_set;
	}
	
//	*vf_control_out = rov_loop_calc->vf_set * ROV_OPEN_VELOCITY_SCALE;
	uint16_t half_thr_max_forward_set = ROV_MAX_THR_FORWARD_SET * ROV_OPEN_VELOCITY_SCALE / 2;
	if(rov_loop_calc->vf_set > ROV_MAX_THR_FORWARD_SET / 2)
	{
		*vf_control_out = 2 * half_thr_max_forward_set;
	}
	else if(rov_loop_calc->vf_set >= -(ROV_MAX_THR_FORWARD_SET / 2)  && rov_loop_calc->vf_set <= (ROV_MAX_THR_FORWARD_SET / 2))
	{
		*vf_control_out = 0;
	}
	else if(rov_loop_calc->vf_set < -(ROV_MAX_THR_FORWARD_SET / 2))
	{
		*vf_control_out = -2 * half_thr_max_forward_set;
	}
	
}
	




