#ifndef __VISION_H
#define __VISION_H	 

#include "chassis.h"
#include <stdbool.h>

#define Vision_SIZE 18 //�����Ӿ����ݳ���

typedef struct
{
	//���ݻ�����
	uint8_t Dislocatoon[18];     //����λǰ����
	uint8_t Vision_rx[18];				//�������ݻ���
	uint8_t Vision_tx[16];				//�������ݻ���
	uint32_t Vision_tx_cnt;				//���ͼƴ�
	
//	QueueObj speed_queue_yaw;
//	QueueObj speed_queue_pit;
	
	//�������ݵ�֡
	float deal_data[3];
	
	//����̨���������
	float vision_yaw_out;					//���մ������������
	float vision_pitch_out;				//���մ������������
	
	float vision_yaw_up;
	float vision_pitch_up;
	float vision_distance_up;

	float vision_yaw_kf;	
	float vision_pitch_kf;
	float vision_distance_kf;
	
	float vision_yaw_err;
	float vision_pitch_err;
	float vision_distance;
	float dt_m ;   //����̧ǹ�߶�
	
	uint8_t send_or;  //��⵽�����Ӿ��Ƿ�������
	uint8_t reverse_top;  //�Ӿ��Ƿ��⵽����
	//uint16_t 
	
	bool aim_flag;   //�Ӿ����������־λ
	
}varible_vision_t;

/*�Ӿ�����*/
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
void vision_Init(void);/*�Ӿ������ʼ��*/

void Vision_on_off(void);
void vision_task(void);/*�Ӿ�����*/
/*�������ݵ�nuc*/
void send_data_to_nuc(void);
/*�Ӿ���Ϣ����*/
void vision_data_manage(void);
/*���벹��*/
void vision_compensate(float distance);

#endif
