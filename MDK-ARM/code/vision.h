#ifndef __VISION_H
#define __VISION_H	 

#include "chassis.h"
#include <stdbool.h>

#define Vision_SIZE 18 //接收视觉数据长度

typedef struct
{
	//数据缓冲区
	uint8_t Dislocatoon[18];     //处理串位前数据
	uint8_t Vision_rx[18];				//接收数据缓冲
	uint8_t Vision_tx[16];				//发送数据缓冲
	uint32_t Vision_tx_cnt;				//发送计次
	
//	QueueObj speed_queue_yaw;
//	QueueObj speed_queue_pit;
	
	//处理数据掉帧
	float deal_data[3];
	
	//给云台输出的数据
	float vision_yaw_out;					//最终传输至电机数据
	float vision_pitch_out;				//最终传输至电机数据
	
	float vision_yaw_up;
	float vision_pitch_up;
	float vision_distance_up;

	float vision_yaw_kf;	
	float vision_pitch_kf;
	float vision_distance_kf;
	
	float vision_yaw_err;
	float vision_pitch_err;
	float vision_distance;
	float dt_m ;   //补偿抬枪高度
	
	uint8_t send_or;  //检测到陀螺视觉是否允许发弹
	uint8_t reverse_top;  //视觉是否检测到陀螺
	//uint16_t 
	
	bool aim_flag;   //视觉开启自瞄标志位
	
}varible_vision_t;

/*视觉接收*/
typedef struct
{
	uint32_t time_now;
	uint32_t time_last;
	uint16_t time_fps;
	uint8_t update_fps;
	uint16_t rx_err_cnt;
	bool date_update;
	
}vision_rx_t;

extern vision_rx_t vision_rx;
extern varible_vision_t varible_vision;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void vision_Init(void);/*视觉任务初始化*/

void Vision_on_off(void);
void vision_task(void);/*视觉任务*/
/*发送数据到nuc*/
void send_data_to_nuc(void);
/*视觉信息处理*/
void vision_data_manage(void);
/*距离补偿*/
void vision_compensate(float distance);

#endif
