/**
  ****************************(C) COPYRIGHT 2023 ZJU****************************
  * @file       rov_behaviour.c/h
  * @brief      ROV�Ķ�ģ����ʵ�֣����Լ����������ģʽ...         
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     FEB-7-2023     HaoLion(������)    1. done
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
        Crawing_Mode : �Ĵ�����ģʽ����ʱ��Ҫ����vf��yaw
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


//highlight, the variable rov behaviour mode 
//���⣬���������Ϊģʽ����
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
	* @brief          ROV��������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����ϻ�PWM�������͹ʶ����趨ֵ������Ϊ0
	* @param[in]      vf_setǰ�����ٶ� �趨ֵ����������
  * @param[in]      vz_set���ҵ��ٶ� �趨ֵ����������
  * @param[in]      yaw_set yaw���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      pitch_set pitch���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      roll_set roll���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      rov_set_to_vector ROV����
  * @retval         none
  */

static void rov_zero_force_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector);

/**
	* @brief          ROV����ģʽ
	* @param[in]      vf_setǰ�����ٶ� �趨ֵ����������
  * @param[in]      vz_set���ҵ��ٶ� �趨ֵ����������
  * @param[in]      yaw_set yaw���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      pitch_set pitch���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      roll_set roll���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      rov_set_to_vector ROV����
  * @retval         none
  */

static void rov_open_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector);

/**
	* @brief          ROVֻ��������ģʽ
	* @param[in]      vf_setǰ�����ٶ� �趨ֵ����������
  * @param[in]      vz_set���ҵ��ٶ� �趨ֵ����������
  * @param[in]      yaw_set yaw���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      pitch_set pitch���ȶ��ĽǶ�
  * @param[in]      roll_set roll���ȶ��ĽǶ�
  * @param[in]      rov_set_to_vector ROV����
  * @retval         none
  */

static void rov_only_althold_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector);

/**
	* @brief          ROVֻ��������
	* @param[in]      vf_setǰ�����ٶ� 
  * @param[in]      vz_set���ҵ��ٶ� 
  * @param[in]      yaw_set yaw���ȶ��ĽǶ� 
  * @param[in]      pitch_set pitch���ȶ��ĽǶ� 
  * @param[in]      roll_set roll���ȶ��ĽǶ�
  * @param[in]      rov_set_to_vector ROV����
  * @retval         none
  */

static void rov_only_atthold_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector);


/**
	* @brief          ROV��������״̬���µ�
	* @param[in]      vf_setǰ�����ٶ� �趨ֵ����������
  * @param[in]      vz_set���ҵ��ٶ� �趨ֵ����������
  * @param[in]      yaw_set yaw���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      pitch_set pitch���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      roll_set roll���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      rov_set_to_vector ROV����
  * @retval         none
  */

static void rov_normal_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector);

/**
	* @brief          ROV½����ǰ�������ƽ���ȫ���ر�
	* @param[in]      vf_setǰ�����ٶ�
  * @param[in]      vz_set���ҵ��ٶ� �趨ֵ����������
  * @param[in]      yaw_set yaw���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      pitch_set pitch���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      roll_set roll���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      rov_set_to_vector ROV����
  * @retval         none
  */

static void rov_crawling_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector);


/**
	* @brief          ROV����״̬�£��ĸ���ֱ�ƽ��������������������Ĵ�����ջ�
	* @param[in]      vf_setǰ�����ٶ� 
  * @param[in]      vz_set���ҵ��ٶ� �趨ֵ����������
  * @param[in]      yaw_set yaw���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      pitch_set pitch���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      roll_set roll���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      rov_set_to_vector ROV����
  * @retval         none
  */

static void rov_stick_wall_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector);



/**
	* @brief          ROVֹͣģʽ����ʱ�������ȶ���ǰ״̬��������ҵ���ݲ�������
	* @param[in]      vf_setǰ�����ٶ� �趨ֵ����������
  * @param[in]      vz_set���ҵ��ٶ� �趨ֵ����������
  * @param[in]      yaw_set yaw���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      pitch_set pitch���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      roll_set roll���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      rov_set_to_vector ROV����
  * @retval         none
  */

static void rov_no_move_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector);


/**
  * @brief          ����ROV��Ϊ
  * @param[in]     none
  * @retval         uint8_t
  */
uint8_t get_rov_behaviour(void)
{
	return rov_behaviour_mode;
}

/**
  * @brief          ͨ���߼��жϣ���ֵ"rov_behaviour_mode"������ģʽ
  * @param[in]      rov_move_mode: ROV����
  * @retval         none
  */
void rov_behaviour_mode_set(rov_move_t *rov_move_mode)
{
    if (rov_move_mode == NULL)
    {
        return;
    }

    //���ݿ�����������ģʽ
    if (rov_move_mode->rov_Ctrl->Mode == ROV_OPEN)
    {
        //����ģʽ
        rov_behaviour_mode = ROV_OPEN;
    }
	
    else if (rov_move_mode->rov_Ctrl->Mode == ROV_ONLY_ALTHOLD)
    {
		//ֻ��������
		rov_behaviour_mode = ROV_ONLY_ALTHOLD;
    }
	
	else if (rov_move_mode->rov_Ctrl->Mode == ROV_ONLY_ATTHOLD)
	{
		//ֻ��������
		rov_behaviour_mode = ROV_ONLY_ATTHOLD;
		rov_move_mode->yaw_angle_set = rov_move_mode->IMU_data->yaw;
	}
	
    else if (rov_move_mode->rov_Ctrl->Mode == ROV_NORMAL)
    { 
		//ROV��������ģʽ�������˶���Ͷ���
		rov_behaviour_mode =  ROV_NORMAL;
    }
	
	else if (rov_move_mode->rov_Ctrl->Mode == ROV_CRAWLING)
	{
		//ROV½������ģʽ
		rov_behaviour_mode = ROV_CRAWLING;
	}
	
	else if (rov_move_mode->rov_Ctrl->Mode == ROV_STICK_WALL)
	{
		//ROV����ģʽ
		rov_behaviour_mode = ROV_STICK_WALL;
	}
	
	else if (rov_move_mode->rov_Ctrl->Mode == ROV_ZERO_FORCE)
	{
		//ROV����ģʽ�������е��ת�ٿ���Ϊ0
		rov_behaviour_mode = ROV_ZERO_FORCE;
	}
	
	else if (rov_move_mode->rov_Ctrl->Mode == ROV_NO_MOVE)
	{
		//ֹͣģʽ����ʱ�������ȶ���ǰ״̬��������ҵ���ݲ�������
		rov_behaviour_mode = ROV_NO_MOVE;
	}

    //add your own logic to enter the new mode
    //����Լ����߼��жϽ�����ģʽ


    //accord to beheviour mode, choose rov control mode
    //������Ϊģʽѡ��һ��ROV����ģʽ
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
  * @brief          ���ÿ�����.���ݲ�ͬ���̿���ģʽ��������������Ʋ�ͬ�˶�.������������棬����ò�ͬ�Ŀ��ƺ���.
	* @param[in]      vf_setǰ�����ٶ� �趨ֵ����������
  * @param[in]      vz_set���ҵ��ٶ� �趨ֵ����������
  * @param[in]      yaw_set yaw���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      pitch_set pitch���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      roll_set roll���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      rov_set_to_vector ROV����
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
	* @brief          ROV��������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����ϻ�PWM�������͹ʶ����趨ֵ������Ϊ0
	* @param[in]      vf_setǰ�����ٶ� �趨ֵ����������
  * @param[in]      vz_set���ҵ��ٶ� �趨ֵ����������
  * @param[in]      yaw_set yaw���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      pitch_set pitch���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      roll_set roll���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      rov_set_to_vector ROV����
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
	* @brief          ROV��������״̬���µ�
	* @param[in]      vf_setǰ�����ٶ� 
  * @param[in]      vz_set���ҵ��ٶ� 
  * @param[in]      yaw_set yaw���ȶ��ĽǶ� 
  * @param[in]      pitch_set pitch���ȶ��ĽǶ� 
  * @param[in]      roll_set roll���ȶ��ĽǶ�
  * @param[in]      rov_set_to_vector ROV����
  * @retval         none
  */

static void rov_normal_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector)
{
    if (vf_set == NULL || vz_set == NULL || yaw_set == NULL || pitch_set == NULL || roll_set == NULL || rov_set_to_vector == NULL)
    {
        return;
    }
		
		//��ͨ�˲���ֹ�·��Ƕȱ仯̫��
	rov_rc_to_control_vector(yaw_set, roll_set, rov_set_to_vector);
	
	*vf_set = rov_set_to_vector->rov_Ctrl->VF;
    *vz_set = rov_set_to_vector->rov_Ctrl->VZ;
    *pitch_set = 0.0f;
    
}

/**
	* @brief          ROV½����ǰ�������ƽ���ȫ���ر�
	* @param[in]      vf_setǰ�����ٶ�
  * @param[in]      vz_set���ҵ��ٶ� �趨ֵ����������
  * @param[in]      yaw_set yaw���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      pitch_set pitch���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      roll_set roll���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      rov_set_to_vector ROV����
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
* @brief          ROV����״̬�£��ĸ���ֱ�ƽ��������������������Ĵ�����ջ�
	* @param[in]      vf_setǰ�����ٶ� 
  * @param[in]      vz_set���ҵ��ٶ� �趨ֵ����������
  * @param[in]      yaw_set yaw���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      pitch_set pitch���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      roll_set roll���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      rov_set_to_vector ROV����
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
	* @brief          ROVֻ��������ģʽ
	* @param[in]      vf_setǰ�����ٶ� �趨ֵ����������
  * @param[in]      vz_set���ҵ��ٶ� �趨ֵ����������
  * @param[in]      yaw_set yaw���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      pitch_set pitch���ȶ��ĽǶ�
  * @param[in]      roll_set roll���ȶ��ĽǶ�
  * @param[in]      rov_set_to_vector ROV����
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
	//��������
	*yaw_set = rov_set_to_vector->rov_Ctrl->YAW;
    *pitch_set = 0.0f;
	*roll_set = 0.0f;
    
}

/**
	* @brief          ROVֻ��������
	* @param[in]      vf_setǰ�����ٶ� 
  * @param[in]      vz_set���ҵ��ٶ� 
  * @param[in]      yaw_set yaw���ȶ��ĽǶ� 
  * @param[in]      pitch_set pitch���ȶ��ĽǶ� 
  * @param[in]      roll_set roll���ȶ��ĽǶ�
  * @param[in]      rov_set_to_vector ROV����
  * @retval         none
  */

static void rov_only_atthold_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector)
{
    if (vf_set == NULL || vz_set == NULL || yaw_set == NULL || pitch_set == NULL || roll_set == NULL || rov_set_to_vector == NULL)
    {
        return;
    }
		
	//��ͨ�˲���ֹ�·��Ƕȱ仯̫��
	rov_rc_to_control_vector(yaw_set, roll_set, rov_set_to_vector);
	
	*vf_set = rov_set_to_vector->rov_Ctrl->VF;
    *vz_set = rov_set_to_vector->rov_Ctrl->VZ;
    *pitch_set = 0.0f;
    
}

/**
	* @brief          ROV����ģʽ
	* @param[in]      vf_setǰ�����ٶ� �趨ֵ����������
  * @param[in]      vz_set���ҵ��ٶ� �趨ֵ����������
  * @param[in]      yaw_set yaw���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      pitch_set pitch���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      roll_set roll���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      rov_set_to_vector ROV����
  * @retval         none
  */

static void rov_open_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector)
{
    if (vf_set == NULL || vz_set == NULL || yaw_set == NULL || pitch_set == NULL || roll_set == NULL || rov_set_to_vector == NULL)
    {
        return;
    }
	//��ͨ�˲���ֹ�·��Ƕȱ仯̫��
//		rov_rc_to_control_vector(yaw_set, roll_set, rov_set_to_vector);
	*yaw_set = rov_set_to_vector->rov_Ctrl->YAW;
	*roll_set = rov_set_to_vector->rov_Ctrl->ROL;
		
	*vf_set = rov_set_to_vector->rov_Ctrl->VF;
    *vz_set = rov_set_to_vector->rov_Ctrl->VZ;
    *pitch_set = 0.0f;
    
}

/**
  * @brief          ROVֹͣģʽ����ʱ�������ȶ���ǰ״̬��������ҵ���ݲ�������
  * @param[in]      vf_setǰ�����ٶ� �趨ֵ����������
  * @param[in]      vz_set���ҵ��ٶ� �趨ֵ����������
  * @param[in]      yaw_set yaw���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      pitch_set pitch���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      roll_set roll���ȶ��ĽǶ� �趨ֵ����������
  * @param[in]      rov_set_to_vector ROV����
  * @retval         none
  */

static void rov_no_move_control(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector)
{
    if (vf_set == NULL || vz_set == NULL || yaw_set == NULL || pitch_set == NULL || roll_set == NULL || rov_set_to_vector == NULL)
    {
        return;
    }
		//�ݲ�����
		*vf_set = 0.0f;
		*vz_set = 0.0f;
		*yaw_set = 0.0f;
		*pitch_set = 0.0f;
		*roll_set = 0.0f;
    
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
	if(rov_behaviour_mode == ROV_NORMAL || rov_behaviour_mode == ROV_ONLY_ATTHOLD)
	{
		fp32 yaw_cur = 0.0f, pitch_cur = 0.0f, roll_cur = 0.0f;
		fp32 yaw_tar= 0.0f, pitch_tar = 0.0f, roll_tar = 0.0f;
		//��ȡ��ǰֵ
		yaw_cur = rov_loop_calc->IMU_data->yaw;
		pitch_cur = rov_loop_calc->IMU_data->pitch;
		roll_cur = rov_loop_calc->IMU_data->roll;
		//��ȡĿ��ֵ
		yaw_tar = rov_loop_calc->yaw_angle_set;
//		yaw_tar = rov_loop_calc->IMU_data->yaw;
//		pitch_tar = rov_loop_calc->IMU_data->yaw;
//		roll_tar = rov_loop_calc->IMU_data->yaw;
		//���㴦��
		//Over_Zero_Handlle(yaw_tar,yaw_cur,180);
		//����ǶȻ����⻷��
		PID_calc(&rov_loop_calc->yaw_angle_pid, yaw_cur, yaw_tar);
//		PID_calc(&rov_loop_calc->pitch_angle_pid, pitch_cur, pitch_tar);
//		PID_calc(&rov_loop_calc->roll_angle_pid, roll_cur, roll_tar);
		//������ٶȻ����ڻ���
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
	* @brief          ROV����������
  * @param[in]      depth_control_out ������������
  * @param[in]      rov_loop_calc ROV����
  * @retval         none
  */

void rov_depth_loop_calc(fp32 *depth_control_out, rov_move_t * rov_loop_calc)
{
	static fp32 last_vz_set = 0; 
	fp32 cur_vz_set = rov_loop_calc->vz_set;
	if(rov_behaviour_mode == ROV_ONLY_ALTHOLD || rov_behaviour_mode == ROV_NORMAL)
	{
		//���Ƹ˽����ȶ���ʱ���������ö���ֵ
		if((last_vz_set > ROV_DEPTH_HOLD_DEADLINE && cur_vz_set <= ROV_DEPTH_HOLD_DEADLINE ) 
			|| (last_vz_set < -ROV_DEPTH_HOLD_DEADLINE && cur_vz_set >= -ROV_DEPTH_HOLD_DEADLINE))
		{
			rov_loop_calc->depth_set = rov_loop_calc->depth;
		}
		//����Ƹ��ٶȹ�С������붨����㣬���򿪻�����
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
	* @brief          ROV�����ٶȿ��������������
  * @param[in]      vf_control_out �����ٶȿ������
  * @param[in]      track_vf_control_out �Ĵ�ģʽ��ǰ���ٶȿ������
  * @param[in]      rov_loop_calc ROV����
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
	




