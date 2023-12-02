#ifndef __CHASSIS_T_H
#define __CHASSIS_T_H	 

#include "stm32f4xx_hal.h"
#include "pid.h"
#include "chassis.h"
#include "referee_limit.h"
#include "kalman.h"
#include "imu.h"
#include "math.h"

#define chassis_stop_if  (0)  //����ͣת  -> û̫����  ������
#define expansion_degree  0.1f    //������
#define chassis_middle     4037
#define PI 3.1415926f


/*�ṹ����̼����ƶ�*/
typedef  struct 
{
	float Va;
	float Vd;
	float Vw;
	float Vs;

}chassis_key_t;

typedef struct
{
	uint8_t pid_choice_flag_degree_follow[3];									//���̸����л������־
	uint8_t chassis_follow_pid_change[3];										//���̸���pid�л�
	float degree_1,get_degree_1;									//����ǰ�Ƕȣ������Ƕ�
	float degree_2,get_degree_2;									//����ǰ�Ƕȣ������Ƕ�
	float gyro_x,gyro_y;											//��̨����ϵ�µ�X,Y
	int16_t set_chassis_speed[4],send_chassis_speed[4];				//����Ŀ���ٶȣ����̷���
	uint16_t wait_follow_flag;															//�ȴ���ͷ��־
	int8_t top_direction;																		//С���ݷ���
	uint8_t top_direction_flag;															//С���ݷ����־
	float total_torque;																//��ת��
	float top_s_c;   //��������ϵ��
	float temp1;			//new��ͷ
	float angle1;
}chassis_variable_t;


extern chassis_variable_t varible_chassis;
extern chassis_key_t chassis_key_mode;

void chassis_power_torque(void);   //ת��*�ٶ�*ϵ��*0��954=�������
void CHASSIS_follow_pid_change(void);
void chassis_assist_task(void);

void find_degree(void);//new ����Ƕ�
void chassis_follow_degree(void);
void chassis_caculate(int16_t x,int16_t y,int16_t z);//���ֵ������
void Top_direction(void);                            //С���ݷ���
void CHASSIS_KEY_MODE(void);
float degree_upstep(float degree);                    //�Ƕ��������
void Relative_degree(void);
void gyroscope_remote_resolve(float channel1,float channel2,float re_angle);//�˶�����
void Top_change(void);						//��������


#endif
