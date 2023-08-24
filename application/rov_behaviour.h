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
#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "struct_typedef.h"
#include "rov_movation_task.h"

#define ROV_OPEN_YAW_SCALE        10 					// ����ģʽ��YAW�Ǳ���ϵ��
#define ROV_OPEN_ROL_SCALE        14 					// ����ģʽ��YAW�Ǳ���ϵ��
#define ROV_OPEN_PIT_SCALE        14				    // ����ģʽ��YAW�Ǳ���ϵ��
#define ROV_OPEN_FORWAD_SCALE     20 			        // ����ģʽ��ǰ���ٶȵı���ϵ��
#define ROV_OPEN_HEAVE_SCALE      15                    // ����ģʽ�������ٶȵı���ϵ��
#define TRACK_OPEN_WZ_SCALE       50 					// ����ģʽ���Ĵ���ת�ı���ϵ��
#define TRACK_OPEN_VELOCITY_SCALE 20 				 	// ����ģʽ���Ĵ�ֱ���˶��ı���ϵ��
#define ROV_DEPTH_HOLD_DEADLINE   50 					// ����ģʽ�£���ROV���ֶ���ģʽ�������ٶȣ���������ٶȽ��������͸���ֱ�ƽ���
#define ROV_YAW_HOLD_DEADLINE     90					// ����ģʽ�£���ROV���ֶ���ģʽ�������ٶȣ���������ٶȽ��������͸���ֱ�ƽ���
#define ROV_MAX_YAW_SET           180                   // ����ת������·���ֵ
#define ROV_MAX_TRA_FORWARD_SET   100					// ����ǰ�������·���ֵ
#define ROV_MAX_THR_FORWARD_SET   100					// �ƽ�ǰ�������·���ֵ
#define ROV_MAX_THR_YAW_SET       180         // �ƽ�ת������·���ֵ
#define ROV_MAX_THR_ROLL_SET      180         // �ƽ���������·���ֵ
#define ROV_MAX_THR_HEAVE_SET     100         // �ƽ����������·���ֵ

/* ----------------------- Extern Function ----------------------------------- */

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
  * @brief          ͨ���߼��жϣ���ֵ"rov_behaviour_mode"������ģʽ
  * @param[in]      rov_move_mode: ROV����
  * @retval         none
  */
void rov_behaviour_mode_set(rov_move_t *rov_move_mode);


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

extern void rov_behaviour_control_set(fp32 *vf_set, fp32 *vz_set, fp32 *yaw_set, fp32 *pitch_set, fp32 *roll_set ,rov_move_t * rov_set_to_vector);

/**
	* @brief          ROV�ǶȻ�������������ô���PID����
  * @param[in]      yaw_control_out yaw����������
  * @param[in]      pitch_control_out pitch����������
  * @param[in]      roll_control_out roll����������
  * @param[in]      rov_loop_calc ROV����
  * @retval         none
  */

extern void rov_angle_loop_calc(fp32 *yaw_control_out, fp32 *pitch_control_out, fp32 *roll_control_out, rov_move_t * rov_loop_calc);


/**
	* @brief          ROV����������
  * @param[in]      depth_control_out ������������
  * @param[in]      rov_loop_calc ROV����
  * @retval         none
  */

extern void rov_depth_loop_calc(fp32 *depth_control_out, rov_move_t * rov_loop_calc);

/**
	* @brief          ROV�Ĵ��������
  * @param[in]      track_wz_out �Ĵ���ת���������
  * @param[in]      rov_loop_calc ROV����
  * @retval         none
  */
extern void track_turn_loop_calc(fp32 *track_wz_out, rov_move_t * rov_loop_calc);

/**
	* @brief          ROV�����ٶȿ��������������
  * @param[in]      vf_control_out �����ٶȿ������
  * @param[in]      track_vf_control_out �Ĵ�ģʽ��ǰ���ٶȿ������
  * @param[in]      rov_loop_calc ROV����
  * @retval         none
  */
extern void rov_vf_control_calc(fp32 *vf_control_out, fp32 *track_vf_control_out, rov_move_t * rov_loop_calc);

/**
  * @brief          ����ROV��Ϊ
  * @param[in]     none
  * @retval         uint8_t
  */
extern uint8_t get_rov_behaviour(void);

#endif
