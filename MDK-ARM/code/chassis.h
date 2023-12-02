#ifndef __CHASSIS_H
#define __CHASSIS_H	 

#include "pid.h"
#include "bsp_can.h"
#include "chassis_t.h"
#include "key_task.h"
#include "function_lib.h"
#include "Remote_Control.h"
#include "all_task.h"


void chassis_task(void);
void chassis_Init(void);/*底盘初始化*/
void chassis_follow(void);
void chassis_top(void);
void chassis_motor_diversion(uint8_t mode);  //ж��

#endif
