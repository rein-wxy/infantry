#ifndef __GIMBAL_H
#define __GIMBAL_H	 

#include "chassis.h"
#include "kalman.h"
#include "vision.h"
#include "imu.h"
//限位

#if mode_car == 1
/*新上供*/
#define pitch_up_angle 4300
#define pitch_down_angle 2800
#endif

#if mode_car == 2
/*哨兵上云台*/
#define pitch_up_angle 3210
#define pitch_down_angle 2245
#endif

#define pitch_up_imu 4670
#define pitch_down_imu 3570


/*云台结构体变量*/
typedef struct
{
	uint8_t yaw_imu_angle_flag;          //yaw轴记录陀螺仪角度标志位
	double set_yaw_angle;                //yaw轴目标角
	double set_yaw_speed;                //yaw轴目标速度
	
	uint8_t pitch_imu_angle_flag;			//pitch轴记录陀螺仪角标志位
	double set_pitch_angle;						//pitch轴目标角度
	double set_pitch_speed;						//pitch轴目标速度
	double last_set_pitch_angle;
	uint32_t Wpower;					//克服云台重力常数
	
	
	uint8_t pid_choice_flag[2];          //云台pid切换标志
	uint8_t yaw_turn_round_flag;         //一键掉头标志  0/1 ->开/关
	uint32_t yaw_turn_round_time;        //yaw掉头时间
	float yaw_turn_round_step;           //yaw掉头步进值->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	uint32_t shoot_stop_time;			//无力摩擦轮停转
	
	//普通
	float mouse_x_db;									//鼠标X轴灵敏度
	float RC_x_db;										//遥控器X轴灵敏度
	float mouse_y_db;									//鼠标X轴灵敏度
	float RC_y_db;										//遥控器y轴灵敏度
	//自瞄
	float vis_mouse_x_db;									//鼠标X轴灵敏度
	float vis_RC_x_db;										//遥控器X轴灵敏度
	float vis_mouse_y_db;									//鼠标X轴灵敏度
	float vis_RC_y_db;										//遥控器X轴灵敏度
	
	float vis_x_db;
	float vis_y_db;
}gimbal_variable_t;






extern gimbal_variable_t varible_gimbal;

void W_power(void);
void gimbal_key_control(void); //键鼠模式 xy 滤波处理
void PID_CIMBAL_CHOICE(void);   //自瞄pid切换

void yaw_turn_round(uint8_t flag);        /*yaw轴一键掉头*/
void YAW_MOTOR(uint8_t flag);
void PITCH_MOTOR(uint8_t flag);
void disability_gimbal_motor(uint8_t mode);
void gimbal_Init(void);
void gimbal_task(void);

#endif
