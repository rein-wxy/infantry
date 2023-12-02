#ifndef __SEND_H
#define __SEND_H	 

#include "chassis.h"
#include "referee_limit.h"
#include "tim.h"
#include "Remote_Control.h"

#define speed_new (1) //0������old 1--new
#define continuous_stir_time_trigger 1000		//��������ʱ��
typedef struct
{
	int32_t set_send_speed;					//������Ŀ���ٶ�
	int32_t set_send_angle;					//������Ŀ��λ��
	uint8_t send_flag;                      //��ֹ��������ת---->������
	uint8_t switch_send_flag;				//���俪�ر�־
	
	int16_t set_shoot_speed[2]; 		//Ħ�����ٶ�
	uint8_t speed_flag[3];				//Ħ�����ٶ����ñ�־---->��
	float speed_speed;				//Ħ�����ٶ�����
	uint8_t fric_send_flag;             //Ħ�������ת�����ˣ�0��1��
	

	uint8_t send_data_update;           //���ٸ���
	uint8_t send_data_limit;            //�°���������
	uint8_t send_mode_flag;				//����ģʽ��0���㣬1���������2����
	uint32_t send_mode_time;				//����ʱ��
	uint32_t send_time_err;   //
	
	
	uint16_t speed_limit;                	//��������
	uint8_t send_freq;						//������Ƶ ����ÿ����Ƶ����
	uint8_t lock_pill;						//������0��1�£�
	uint8_t lock_pid_clear_flag[2];			//��תpid����
	uint16_t lock_flag;						//��ת����ʱ���־λ
	
	
	float low_speed;				//������
	float hight_speed;
	//
	uint8_t heart_data_update;
}varible_send_t;

extern varible_send_t varible_send;


void send_Init(void);			  /*�����ʼ��*/    
void shoot_task(void);			  /*Ħ��������*/
void pill_depot_task(void);       /*���տ���*/
void locked_rotor_tesk(void);     /*��ת���*/
void locked_task(void);			  /*��ת����*/
void stir_task(void);			  /*����������*/
void send_task(void);  	    /*��������*/
void send_mo(void);       //���
void send_rc(void);       //ң��
float speed_offest(float speed,float limit);//���ٲ���
		
#endif
