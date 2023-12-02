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
	uint8_t mode;                    //ģʽ
	uint8_t sys_reset;                   //��λ(0,1��)
	
	
 	uint8_t small_gyro;   	         //С���ݣ�0�أ�1���� 				left-1
	uint8_t chassis_climb;      	 //���� �� ��1����					w+ctrl
	uint8_t zig;        	         //��ת  ��0��1��					x
	
	uint8_t vision_armor_ok;			//�Ƿ�ʶ��
	uint8_t load;                   //������0�أ�1����/-->������
	uint8_t load_mo;					//�����Ʒ���
	uint8_t frictiongear;            //Ħ���֣�0��1�� 					g---ctrl+g
	uint16_t frictiongear_ramp;		//Ħ���ֿ����ȴ�����б��
	uint8_t pill_depot;   	         //���գ�0�أ�1����					r-- ctrl+r
	
	uint8_t auto_mode;							//�Զ�ģʽ  0�޸��� 1���� 2���
	uint8_t hit_mode;							//����ģʽ	 		1��� 
	uint8_t automatic_aiming;        //����
	uint8_t aim_predict;             //Ԥ��
	uint8_t vision_send;			//�Ӿ��Ƿ���������־λ
	
	uint8_t cap_mode;  		         //��������ģʽ
	uint32_t cap_cnt;                 //�����ӳٿ�
	
	uint8_t die_or;           //�жϷ�ӳ�������Ƿ����� 1�������� 0--��
	
	
}task_flag_t;

					//���
extern RC16_chn RC16_CHN;
extern task_flag_t task_flag;


void pill_open(void);
void key_task(void);/*ң�ؿ�������*/
void normal_1(void);/*����ģʽ*/
void KEY_SYS_RESET(void);/*�����λ*/
void mode_task(void);
#endif




