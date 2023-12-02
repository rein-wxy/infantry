#ifndef __KEY_TASK_H
#define __KEY_TASK_H	 

#include "chassis.h"
#include <stdbool.h>
#include "referee_limit.h"


typedef struct
{
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	int16_t ch5;
	
}RC16_chn;



typedef struct
{
	uint8_t mode;                    //模式
	uint8_t sys_reset;                   //复位(0,1开)
	
	
 	uint8_t small_gyro;   	         //小陀螺（0关，1开） 				left-1
	uint8_t chassis_climb;      	 //爬坡 （ ，1开）					w+ctrl
	uint8_t zig;        	         //急转  （0，1）					x
	
	uint8_t vision_armor_ok;			//是否识别到
	uint8_t load;                   //拨弹（0关，1开）/-->鼠标左键
	uint8_t load_mo;					//鼠标控制发弹
	uint8_t frictiongear;            //摩擦轮（0，1） 					g---ctrl+g
	uint16_t frictiongear_ramp;		//摩擦轮开启等待发射斜坡
	uint8_t pill_depot;   	         //弹舱（0关，1开）					r-- ctrl+r
	
	uint8_t auto_mode;							//自动模式  0无辅助 1自瞄 2打符
	uint8_t hit_mode;							//击打模式	 		1打符 
	uint8_t automatic_aiming;        //自瞄
	uint8_t aim_predict;             //预测
	uint8_t vision_send;			//视觉是否允许发弹标志位
	
	uint8_t cap_mode;  		         //超级电容模式
	uint32_t cap_cnt;                 //电容延迟开
	
	uint8_t die_or;           //判断反映机器人是否死亡 1——正常 0--死
	
	
}task_flag_t;

					//检测
extern RC16_chn RC16_CHN;
extern task_flag_t task_flag;


void pill_open(void);
void key_task(void);/*遥控控制任务*/
void normal_1(void);/*正常模式*/
void KEY_SYS_RESET(void);/*软件复位*/
void mode_task(void);
#endif




