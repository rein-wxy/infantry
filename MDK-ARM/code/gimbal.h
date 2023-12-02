#ifndef __GIMBAL_H
#define __GIMBAL_H	 

#include "chassis.h"
#include "kalman.h"
#include "vision.h"
#include "imu.h"
//��λ

#if mode_car == 1
/*���Ϲ�*/
#define pitch_up_angle 4300
#define pitch_down_angle 2800
#endif

#if mode_car == 2
/*�ڱ�����̨*/
#define pitch_up_angle 3210
#define pitch_down_angle 2245
#endif

#define pitch_up_imu 4670
#define pitch_down_imu 3570


/*��̨�ṹ�����*/
typedef struct
{
	uint8_t yaw_imu_angle_flag;          //yaw���¼�����ǽǶȱ�־λ
	double set_yaw_angle;                //yaw��Ŀ���
	double set_yaw_speed;                //yaw��Ŀ���ٶ�
	
	uint8_t pitch_imu_angle_flag;			//pitch���¼�����ǽǱ�־λ
	double set_pitch_angle;						//pitch��Ŀ��Ƕ�
	double set_pitch_speed;						//pitch��Ŀ���ٶ�
	double last_set_pitch_angle;
	uint32_t Wpower;					//�˷���̨��������
	
	
	uint8_t pid_choice_flag[2];          //��̨pid�л���־
	uint8_t yaw_turn_round_flag;         //һ����ͷ��־  0/1 ->��/��
	uint32_t yaw_turn_round_time;        //yaw��ͷʱ��
	float yaw_turn_round_step;           //yaw��ͷ����ֵ->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	uint32_t shoot_stop_time;			//����Ħ����ͣת
	
	//��ͨ
	float mouse_x_db;									//���X��������
	float RC_x_db;										//ң����X��������
	float mouse_y_db;									//���X��������
	float RC_y_db;										//ң����y��������
	//����
	float vis_mouse_x_db;									//���X��������
	float vis_RC_x_db;										//ң����X��������
	float vis_mouse_y_db;									//���X��������
	float vis_RC_y_db;										//ң����X��������
	
	float vis_x_db;
	float vis_y_db;
}gimbal_variable_t;






extern gimbal_variable_t varible_gimbal;

void W_power(void);
void gimbal_key_control(void); //����ģʽ xy �˲�����
void PID_CIMBAL_CHOICE(void);   //����pid�л�

void yaw_turn_round(uint8_t flag);        /*yaw��һ����ͷ*/
void YAW_MOTOR(uint8_t flag);
void PITCH_MOTOR(uint8_t flag);
void disability_gimbal_motor(uint8_t mode);
void gimbal_Init(void);
void gimbal_task(void);

#endif
