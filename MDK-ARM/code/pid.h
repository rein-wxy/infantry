#ifndef _PID_H
#define _PID_H

#include "main.h"
#include "math.h"

typedef enum time
{
	Last,Now
}time;

typedef struct _pid_typedef        //PID结构体
{
	
	float kp_angle;               //比例
	float ki_angle;               //积分
	float kd_angle;               //微分
	float target_angle;           //目标值
	float kp_speed;               //比例
	float ki_speed;               //积分
	float kd_speed;               //微分
	float target_speed;           //目标值
	
	float set_angle[3];           //设置的值
	float get_angle[3];           //得到的值
	float err_angle[3];           //得到的误差
	float set_speed[3];           //设置的值
	float get_speed[3];           //得到的值
	float err_speed[3];           //得到的误差
	
	float pout;             //比例输出
	float iout_angle;             //积分输出
	float iout_speed;             //积分输出
	float dout;             //微分输出
	float pid_out_angle;          //pid总输出
	float pid_out_speed;          //pid总输出

	
	float I_limit_angle;          //积分限幅
	uint32_t Max_output_angle;    //最大输出限制
	uint32_t Max_err_angle;       //最大误差
	float deadband_angle;				//死区
	float I_limit_speed;          //积分限幅
	uint32_t Max_output_speed;    //最大输出限制
	uint32_t Max_err_speed;       //最大误差
	float deadband_speed;				//死区
	
	
	
	
  void (*f_param_init)(struct _pid_typedef *pid,
		float target_angle,float I_limit_angle,uint32_t Max_output_angle,uint32_t Max_err_angle,float deadband_angle,float pid_out_angle,float kp_angle,float ki_angle,float kd_angle,
		float target_speed,float I_limit_speed,uint32_t Max_output_speed,uint32_t Max_err_speed,float deadband_speed,float pid_out_speed,float kp_speed,float ki_speed,float kd_speed);       //目标值,积分限幅,最大输出,最大误差,死区,pid_out,p,i,d
	
	float (*f_calculate)(struct _pid_typedef *pid, uint8_t model, float get_speed, float set_speed);           //PID计算
//	float (*f_calculate_position)(struct _pid_typedef *pid, float get_speed, float set_speed);
	
	struct motor           //反馈值
	{
		
		uint16_t angle;        //转子角度
		int16_t speed_actual;  //速度
		int16_t speed_set;     //目标速度
		int16_t real_current;  //实际电流
		int8_t temperature;    //温度
		
		int round_cnt,angle_total,angle_last;
		
	}motor;
	
	
}pid_typedef;





/*extern*/
extern pid_typedef pid_Mchassis[4];			//底盘电机速度环
extern pid_typedef pid_chassis_follow;	//底盘跟随
extern pid_typedef pid_yaw;							//yaw轴电机
extern pid_typedef pid_pitch;						//pitch轴电机
extern pid_typedef pid_send;				  //拨弹盘电机
extern pid_typedef pid_shoot[2];					//摩擦轮电机
extern pid_typedef pid_imu_temp;




/*函数声明*****************************/
float pid_calculate(pid_typedef *pid, uint8_t model, float get_speed, float set_speed);
void pid_init(pid_typedef *pid,
float target_angle,float I_limit_angle,uint32_t Max_output_angle,uint32_t Max_err_angle,float deadband_angle,float pid_out_angle,float kp_angle,float ki_angle,float kd_angle,
float target_speed,float I_limit_speed,uint32_t Max_output_speed,uint32_t Max_err_speed,float deadband_speed,float pid_out_speed,float kp_speed,float ki_speed,float kd_speed);
void pid_change(pid_typedef *pid,float KP_angle,float KI_angle,float KD_angle,
																	float KP_speed,float KI_speed,float KD_speed);
void pid_clear(pid_typedef *pid);

#endif
