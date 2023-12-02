#ifndef __SEND_H
#define __SEND_H	 

#include "chassis.h"
#include "referee_limit.h"
#include "tim.h"
#include "Remote_Control.h"

#define speed_new (1) //0———old 1--new
#define continuous_stir_time_trigger 1000		//连发触发时间
typedef struct
{
	int32_t set_send_speed;					//拨弹盘目标速度
	int32_t set_send_angle;					//拨弹盘目标位置
	uint8_t send_flag;                      //防止拨弹盘乱转---->有用吗？
	uint8_t switch_send_flag;				//发射开关标志
	
	int16_t set_shoot_speed[2]; 		//摩擦轮速度
	uint8_t speed_flag[3];				//摩擦轮速度设置标志---->逐级
	float speed_speed;				//摩擦轮速度设置
	uint8_t fric_send_flag;             //摩擦轮真的转起来了（0，1）
	

	uint8_t send_data_update;           //射速更新
	uint8_t send_data_limit;            //新版热量控制
	uint8_t send_mode_flag;				//发射模式，0单点，1单点结束，2连发
	uint32_t send_mode_time;				//发射时间
	uint32_t send_time_err;   //
	
	
	uint16_t speed_limit;                	//弹速上限
	uint8_t send_freq;						//连发射频 ——每秒射频（）
	uint8_t lock_pill;						//卡弹（0，1堵）
	uint8_t lock_pid_clear_flag[2];			//堵转pid清理
	uint16_t lock_flag;						//堵转清理时间标志位
	
	
	float low_speed;				//测试用
	float hight_speed;
	//
	uint8_t heart_data_update;
}varible_send_t;

extern varible_send_t varible_send;


void send_Init(void);			  /*发射初始化*/    
void shoot_task(void);			  /*摩擦轮任务*/
void pill_depot_task(void);       /*弹舱开关*/
void locked_rotor_tesk(void);     /*堵转检测*/
void locked_task(void);			  /*堵转处理*/
void stir_task(void);			  /*拨弹盘任务*/
void send_task(void);  	    /*发射任务*/
void send_mo(void);       //鼠标
void send_rc(void);       //遥控
float speed_offest(float speed,float limit);//弹速补偿
		
#endif
