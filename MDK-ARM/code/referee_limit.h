#ifndef __REFEREE_LIMIT_H
#define __REFEREE_LIMIT_H	 

#include "chassis.h"
#include "send.h"
#include "referee.h"

typedef struct{                          /*�д��о�*/
	/*��*/
	uint8_t cap_capacity;				//��������	��0~100	
	int8_t cap_power;						//��繦��
}super_cap_t;





extern super_cap_t super_cap;
extern uint8_t allow_send_bullet;	    //�Ƿ�������
extern uint8_t robot_level;				//�����˵ȼ�
extern uint16_t POWER_LIMIT;			//������ƹ���
extern uint16_t chassis_max_speed;					//��������ٶ�
extern uint16_t chassis_top_speed;					//С�����ٶ�
extern uint16_t speed_limit;			//��������
extern uint8_t vis_speed;				//�����Ӿ����ٱ�־
extern float now_bullet_speed;			//��ǰǹ������
extern float last_bullet_speed;			//�ϴ�ǹ������
extern float chassis_power_buffer;									//����ʵʱ��������
extern uint8_t robot_ID;											//������ID
extern uint16_t POWER_LIMIT;    									 //������ƹ���
extern super_cap_t super_cap;         

void parameter_number(void);
void chassis_climb_mode(uint8_t flag);      /*����ģʽ*/
void get_chassis_power_buffer_t(void);      /*��ȡʵʱ���ʣ�ʵʱ��������*/
void get_chassis_id_maxpower_level(void);   /*��ȡ������ID��������ƹ��ʣ��ȼ�*/
void super_cap_task(void);					/*������������*/
void heat_limit_task(void);
void chassis_power_limit(void);/*���̹�������*/
void send_referee_updata(void);

void die_alive(void);/*�жϻ�����״̬*/

void HeatControl(void);//��������
void HeatUpdate(void);//��������
#endif

