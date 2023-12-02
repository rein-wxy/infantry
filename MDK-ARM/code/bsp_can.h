#ifndef __BSP_CAN_H
#define __BSP_CAN_H	 



#include "main.h"
#include "can.h"
#include "stm32f4xx_hal.h"
#include "referee_limit.h"

typedef struct
{
	uint16_t angle;            //转子角度 (0~8191)
	int16_t  speed;            //转子速度
	int16_t  last_speed;			 //上次转子速度
	int16_t  real_current;     //电机电流
	int16_t  temperature;      //电机温度 
	uint16_t last_angle;			 //上次转子角度
	uint16_t offset_angle;     //补偿角度
	int32_t  round_cnt;        //转子转动圈数
	int32_t  total_angle;      //转子转动总角度
	int32_t  last_total_angle;
	int32_t  angle_err;
	int16_t  torque;					 //转矩
}motor_data;
/*CAN反馈值结构体功率板*/
typedef struct
{
	uint16_t power;
}power_data;


extern motor_data chassis_motor_RX[4];
extern motor_data yaw_RX;
extern motor_data pitch_RX;
extern motor_data send_RX;		//拨弹盘电机反馈值
extern motor_data shoot_RX[2];
extern power_data power_RX;  //功率板反馈数据

/*函数声明*/
void CAN1_FILTER_CONFIG(CAN_HandleTypeDef* hcan);																																	
void CAN2_FILTER_CONFIG(CAN_HandleTypeDef* hcan);																																
void get_moto_measure(motor_data *ptr, CAN_HandleTypeDef* hcan, uint8_t RXdata[8]);													
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*hcan);
void get_power_measure(power_data *power, CAN_HandleTypeDef* hcan, uint8_t RXdata[8]);//功率板数据处理

void SET_MOTOR_CURRENT_CAN1_1234(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void SET_MOTOR_CURRENT_CAN1_5678(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void SET_MOTOR_CURRENT_CAN2_1234(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void SET_MOTOR_CURRENT_CAN2_5678(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void SET_POWER_CURRENT_CAN(CAN_HandleTypeDef* hcan);
#endif

