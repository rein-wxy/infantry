#ifndef __ALL_TASK_H
#define __ALL_TASK_H	 

#include "stm32f4xx_hal.h"
#include "bsp_can.h"
#include "chassis.h"
#include "bsp_usart.h"
#include "gimbal.h"
#include "vision.h"
#include "ui.h"
#include "imu.h"
#include "monitor.h"

#define mode_car (1)//1新上供步兵---2哨兵上云台

void task_Init(void);
/*初始化*/
void base_Init(void);
/*底盘任务*/
void chassis_all_task(void);
/*底盘任务*/
void gimbal_all_task(void);

void key_all_task(void);

/*陀螺仪任务*/
void imu_all_task(void);

void referee_all_task(void);


#endif

