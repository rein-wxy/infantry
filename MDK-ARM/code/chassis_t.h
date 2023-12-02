#ifndef __CHASSIS_T_H
#define __CHASSIS_T_H	 

#include "stm32f4xx_hal.h"
#include "pid.h"
#include "chassis.h"
#include "referee_limit.h"
#include "kalman.h"
#include "imu.h"
#include "math.h"

#define chassis_stop_if  (0)  //底盘停转  -> 没太明白  调试用
#define expansion_degree  0.1f    //零度误差
#define chassis_middle     4037
#define PI 3.1415926f


/*结构体底盘键鼠移动*/
typedef  struct 
{
	float Va;
	float Vd;
	float Vw;
	float Vs;

}chassis_key_t;

typedef struct
{
	uint8_t pid_choice_flag_degree_follow[3];									//底盘跟随切换清楚标志
	uint8_t chassis_follow_pid_change[3];										//底盘跟随pid切换
	float degree_1,get_degree_1;									//处理前角度，扩大后角度
	float degree_2,get_degree_2;									//处理前角度，扩大后角度
	float gyro_x,gyro_y;											//云台坐标系下的X,Y
	int16_t set_chassis_speed[4],send_chassis_speed[4];				//底盘目标速度，底盘发送
	uint16_t wait_follow_flag;															//等待找头标志
	int8_t top_direction;																		//小陀螺方向
	uint8_t top_direction_flag;															//小陀螺方向标志
	float total_torque;																//总转矩
	float top_s_c;   //变速陀螺系数
	float temp1;			//new找头
	float angle1;
}chassis_variable_t;


extern chassis_variable_t varible_chassis;
extern chassis_key_t chassis_key_mode;

void chassis_power_torque(void);   //转矩*速度*系数*0，954=输出功率
void CHASSIS_follow_pid_change(void);
void chassis_assist_task(void);

void find_degree(void);//new 跟随角度
void chassis_follow_degree(void);
void chassis_caculate(int16_t x,int16_t y,int16_t z);//麦轮电机解算
void Top_direction(void);                            //小陀螺方向
void CHASSIS_KEY_MODE(void);
float degree_upstep(float degree);                    //角度扩大计算
void Relative_degree(void);
void gyroscope_remote_resolve(float channel1,float channel2,float re_angle);//运动解算
void Top_change(void);						//变速陀螺


#endif
