/**
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  * @file       thruster.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收推进器数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-11-2022     Qiqi Li(李琪琪)    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 ZJU****************************
  */

#ifndef __THRUSTER_H__
#define __THRUSTER_H__

#include "struct_typedef.h"


/* thruster status */
typedef enum
{
		THRUSTER_OK=0,			//推进器无故障
		THRUSTER_FAULT=1,		//推进器故障
} motor_flaut_status_e;

/* CAN send and receive ID */
typedef enum
{
    CAN_M1_ID=0x202,										//控制帧
		CAN_M2_ID=0x203,										//控制帧
		CAN_M3_ID=0x204,										//控制帧
		CAN_M4_ID=0x205,										//控制帧
		CAN_M5_ID=0x206,										//控制帧
		CAN_M6_ID=0x207,										//控制帧
		CAN_RETURN_ID=0x18,									//确认返回帧
        CAN_MOTOR_PARAM_REQUEST_ID=0x30,		//电机参数请求帧
		CAN_MOTOR_PARAM_ID=0x38,						//电机参数帧
		CAN_CONTROL_PARAM_ID=0x60,					//控制参数配置帧	
} can_msg_id_e;

//motor data
typedef struct
{
    uint16_t speed_rpm;								//电机转速
		float given_voltage;						//母线电压
    float given_current;						//母线电流
    uint8_t controller_temperate;			//控制器温度
    uint8_t motor_temperate;					//电机温度
		uint8_t motor_status;							//电机状态
} motor_param_t;

/* Controller parameters Index */
typedef enum
{
    MODE=0x0001,									//设置控制模式，00为模拟量控制
		POLE_COUPLE=0x0003,						//设置电机的极对数，默认为3，无需更改
    TEMPERATE_PROTECT=0x0008,			//电机过温保护，默认为70℃
		Speed_Kp=0x000D,							//速度环Kp，默认3000
		Speed_Ki=0x000E,							//速度环Ki，默认20	
		Current_Kp=0x000F,						//电流环Kp，默认100
		Current_Ki=0x0010,						//电流环Ki，默认20
		CAN_Buadrate=0x0014,					//CAN通讯波特率设置 4:1M 8:500K 16:250K
		CAN_Upload=0x0015,						//CAN上传间隔 默认250ms
		CAN_Timeout=0x0016,						//CAN通讯超时时间 默认3000ms
		Init_Angle=0x0020,						//初始角学习
		Factory_Reset=0xEEEE,					//恢复默认出厂设置
		Save_Param=0xFFFF,						//保存控制器配置参数
} controller_param_set_e;



/**
  * @brief          预先开启所有电机，否则可能导致第一次下发命令时各个电机出现延迟情况
	* @param[in]      none
  * @retval         none
  */
extern void Thruster_Init(void);
/**
  * @brief          发送推进器控制帧
	* @param[in]      Motor_ID: 推进器ID
  * @param[in]      velocity: 控制推进器转速, 范围 [-3500,3500]
  * @param[in]      ACK_MODE: 设置回传模式  0X00：250ms自动上传     0xAA：收到指令回传
  * @retval         none
  */
extern void CAN_cmd_control(uint32_t Motor_ID,int16_t velocity, uint8_t ACK_MODE);

/**
  * @brief          配置控制器参数
  * @param[in]      _parm: 控制器参数索引
  * @param[in]      value: 参数配置值
  * @retval         none
  */
extern void CAN_set_controler(controller_param_set_e _param, uint16_t value);

/**
  * @brief          应答模式下请求电机参数回传
  * @param[in]      none
  * @retval         none
  */
extern void CAN_param_request(uint8_t Motor_ID);


/**
  * @brief          返回推进器数据指针
  * @param[in]      i: 电机编号,范围[0,5]
  * @retval         电机数据指针
  */
extern const motor_param_t *get_thruster_measure_point(uint8_t i);

#endif
