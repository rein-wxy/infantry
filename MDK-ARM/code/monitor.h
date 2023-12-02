#ifndef __MONITOR_H
#define __MONITOR_H	 

#include "all_task.h"
#include <stdbool.h>

#define constrain(x, min, max)	((x>max)?max:(x<min?min:x))  //限幅
#define MAX_Temperature 90
#define MAX_Time 5

typedef struct{
	uint32_t buzzer_time;
	uint32_t buzzer_tim;
	uint16_t buzzer_cnt;
	
}sys_buzzer_t;

/*检测任务标志*/
typedef struct
{
	//蜂鸣器
	sys_buzzer_t sys_buzzer;
	//rc
	uint32_t rc_cnt;
	uint32_t rc_cnt_last;
	uint8_t rc;
	bool rc_monitor;
	//NUC
	uint32_t nuc_cnt;
	uint32_t nuc_cnt_last;
	uint8_t nuc;
	bool nuc_monitor;
	//referee
	uint32_t referee_cnt;
	uint32_t referee_cnt_last;
	uint8_t referee;
	bool referee_monitor;
	//chassis;
	uint32_t chassis_cnt[4];
	uint32_t chassis_cnt_last[4];
	uint8_t chassis[4],chassis_t[4],chassis_kgf[4];//温度保护次数
	bool chassis_monitor[4];
	
	//yaw
	uint32_t yaw_cnt;
	uint32_t yaw_cnt_last;
	uint8_t yaw,yaw_t,yaw_kgf;
	bool yaw_monitor;
	
	//power
	uint32_t power_cnt;
	uint32_t power_cnt_last;
	uint8_t power;
	bool power_monitor;
	//pitch
	uint32_t pitch_cnt;
	uint32_t pitch_cnt_last;
	uint8_t pitch,pitch_t,pitch_kgf;
	bool pitch_monitor;
	//send
	uint32_t send_cnt;
	uint32_t send_cnt_last;
	uint8_t send,send_t,send_kgf;
	bool send_monitor;
	//摩擦轮
	uint32_t shoot_cnt[2];
	uint32_t shoot_cnt_last[2];
	uint8_t shoot[2],shoot_t[2],shoot_kgf[2];
	bool shoot_monitor[2];
	
	uint8_t type_err;
	
	
	
}monitor_t;


extern monitor_t monitor;
void Buzzer_TASK(void);
void temperature_protect(void);//电机保护




#endif
