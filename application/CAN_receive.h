 /**
 * ***************************@copyright (C) Copyright  2023  ZJU***************************
 * @file   CAN_receive.c/h
 * @brief  有关CAN数据接收任务在这里
 * @author Hao Lion(郝亮亮)    Email:(haolion_zju@foxmail.com)
 * @version 1.0
 * @date 2023-04-17
 * 
 * @verbatim:
 * ==============================================================================
 *                                                                               
 * ==============================================================================
 * @endverbatim
 * ***************************@copyright (C) Copyright  2023  ZJU***************************
 */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "thruster.h"

#include "struct_typedef.h"

/**
 * @brief Function implementing the can_receive thread.
 * @param  argument         none
 */
extern void can_receive_task(void const * argument);


#endif
