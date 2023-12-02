#ifndef __BSP_CAN_H
#define __BSP_CAN_H	 



#include "main.h"
#include "can.h"
#include "stm32f4xx_hal.h"
#include "referee_limit.h"

typedef struct
{
	uint16_t angle;            //ת�ӽǶ� (0~8191)
	int16_t  speed;            //ת���ٶ�
	int16_t  last_speed;			 //�ϴ�ת���ٶ�
	int16_t  real_current;     //�������
	int16_t  temperature;      //����¶� 
	uint16_t last_angle;			 //�ϴ�ת�ӽǶ�
	uint16_t offset_angle;     //�����Ƕ�
	int32_t  round_cnt;        //ת��ת��Ȧ��
	int32_t  total_angle;      //ת��ת���ܽǶ�
	int32_t  last_total_angle;
	int32_t  angle_err;
	int16_t  torque;					 //ת��
}motor_data;
/*CAN����ֵ�ṹ�幦�ʰ�*/
typedef struct
{
	uint16_t power;
}power_data;


extern motor_data chassis_motor_RX[4];
extern motor_data yaw_RX;
extern motor_data pitch_RX;
extern motor_data send_RX;		//�����̵������ֵ
extern motor_data shoot_RX[2];
extern power_data power_RX;  //���ʰ巴������

/*��������*/
void CAN1_FILTER_CONFIG(CAN_HandleTypeDef* hcan);																																	
void CAN2_FILTER_CONFIG(CAN_HandleTypeDef* hcan);																																
void get_moto_measure(motor_data *ptr, CAN_HandleTypeDef* hcan, uint8_t RXdata[8]);													
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*hcan);
void get_power_measure(power_data *power, CAN_HandleTypeDef* hcan, uint8_t RXdata[8]);//���ʰ����ݴ���

void SET_MOTOR_CURRENT_CAN1_1234(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void SET_MOTOR_CURRENT_CAN1_5678(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void SET_MOTOR_CURRENT_CAN2_1234(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void SET_MOTOR_CURRENT_CAN2_5678(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void SET_POWER_CURRENT_CAN(CAN_HandleTypeDef* hcan);
#endif

