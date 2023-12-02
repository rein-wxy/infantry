#ifndef _PID_H
#define _PID_H

#include "main.h"
#include "math.h"

typedef enum time
{
	Last,Now
}time;

typedef struct _pid_typedef        //PID�ṹ��
{
	
	float kp_angle;               //����
	float ki_angle;               //����
	float kd_angle;               //΢��
	float target_angle;           //Ŀ��ֵ
	float kp_speed;               //����
	float ki_speed;               //����
	float kd_speed;               //΢��
	float target_speed;           //Ŀ��ֵ
	
	float set_angle[3];           //���õ�ֵ
	float get_angle[3];           //�õ���ֵ
	float err_angle[3];           //�õ������
	float set_speed[3];           //���õ�ֵ
	float get_speed[3];           //�õ���ֵ
	float err_speed[3];           //�õ������
	
	float pout;             //�������
	float iout_angle;             //�������
	float iout_speed;             //�������
	float dout;             //΢�����
	float pid_out_angle;          //pid�����
	float pid_out_speed;          //pid�����

	
	float I_limit_angle;          //�����޷�
	uint32_t Max_output_angle;    //����������
	uint32_t Max_err_angle;       //������
	float deadband_angle;				//����
	float I_limit_speed;          //�����޷�
	uint32_t Max_output_speed;    //����������
	uint32_t Max_err_speed;       //������
	float deadband_speed;				//����
	
	
	
	
  void (*f_param_init)(struct _pid_typedef *pid,
		float target_angle,float I_limit_angle,uint32_t Max_output_angle,uint32_t Max_err_angle,float deadband_angle,float pid_out_angle,float kp_angle,float ki_angle,float kd_angle,
		float target_speed,float I_limit_speed,uint32_t Max_output_speed,uint32_t Max_err_speed,float deadband_speed,float pid_out_speed,float kp_speed,float ki_speed,float kd_speed);       //Ŀ��ֵ,�����޷�,������,������,����,pid_out,p,i,d
	
	float (*f_calculate)(struct _pid_typedef *pid, uint8_t model, float get_speed, float set_speed);           //PID����
//	float (*f_calculate_position)(struct _pid_typedef *pid, float get_speed, float set_speed);
	
	struct motor           //����ֵ
	{
		
		uint16_t angle;        //ת�ӽǶ�
		int16_t speed_actual;  //�ٶ�
		int16_t speed_set;     //Ŀ���ٶ�
		int16_t real_current;  //ʵ�ʵ���
		int8_t temperature;    //�¶�
		
		int round_cnt,angle_total,angle_last;
		
	}motor;
	
	
}pid_typedef;





/*extern*/
extern pid_typedef pid_Mchassis[4];			//���̵���ٶȻ�
extern pid_typedef pid_chassis_follow;	//���̸���
extern pid_typedef pid_yaw;							//yaw����
extern pid_typedef pid_pitch;						//pitch����
extern pid_typedef pid_send;				  //�����̵��
extern pid_typedef pid_shoot[2];					//Ħ���ֵ��
extern pid_typedef pid_imu_temp;




/*��������*****************************/
float pid_calculate(pid_typedef *pid, uint8_t model, float get_speed, float set_speed);
void pid_init(pid_typedef *pid,
float target_angle,float I_limit_angle,uint32_t Max_output_angle,uint32_t Max_err_angle,float deadband_angle,float pid_out_angle,float kp_angle,float ki_angle,float kd_angle,
float target_speed,float I_limit_speed,uint32_t Max_output_speed,uint32_t Max_err_speed,float deadband_speed,float pid_out_speed,float kp_speed,float ki_speed,float kd_speed);
void pid_change(pid_typedef *pid,float KP_angle,float KI_angle,float KD_angle,
																	float KP_speed,float KI_speed,float KD_speed);
void pid_clear(pid_typedef *pid);

#endif
