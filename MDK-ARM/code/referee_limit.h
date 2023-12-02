#ifndef __REFEREE_LIMIT_H
#define __REFEREE_LIMIT_H	 

#include "chassis.h"
#include "send.h"
#include "referee.h"

typedef struct{                          /*有待研究*/
	/*收*/
	uint8_t cap_capacity;				//电容容量	，0~100	
	int8_t cap_power;						//充电功率
}super_cap_t;





extern super_cap_t super_cap;
extern uint8_t allow_send_bullet;	    //是否允许拨弹
extern uint8_t robot_level;				//机器人等级
extern uint16_t POWER_LIMIT;			//最大限制功率
extern uint16_t chassis_max_speed;					//底盘最大速度
extern uint16_t chassis_top_speed;					//小陀螺速度
extern uint16_t speed_limit;			//射速上限
extern uint8_t vis_speed;				//发给视觉弹速标志
extern float now_bullet_speed;			//当前枪口射速
extern float last_bullet_speed;			//上次枪口射速
extern float chassis_power_buffer;									//底盘实时缓冲能量
extern uint8_t robot_ID;											//机器人ID
extern uint16_t POWER_LIMIT;    									 //最大限制功率
extern super_cap_t super_cap;         

void parameter_number(void);
void chassis_climb_mode(uint8_t flag);      /*爬坡模式*/
void get_chassis_power_buffer_t(void);      /*读取实时功率，实时缓冲能量*/
void get_chassis_id_maxpower_level(void);   /*读取机器人ID，最大限制功率，等级*/
void super_cap_task(void);					/*超级电容任务*/
void heat_limit_task(void);
void chassis_power_limit(void);/*底盘功率限制*/
void send_referee_updata(void);

void die_alive(void);/*判断机器人状态*/

void HeatControl(void);//热量控制
void HeatUpdate(void);//热量更新
#endif

