#include "pid.h"


/*电机PID*/
pid_typedef pid_Mchassis[4];				//底盘电机速度环
pid_typedef pid_chassis_follow;			    //底盘跟随
pid_typedef pid_yaw;								//yaw轴电机
pid_typedef pid_pitch;							//pitch轴电机
pid_typedef pid_send;							//拨弹盘电机
pid_typedef pid_shoot[2];						//摩擦轮电机
pid_typedef pid_imu_temp;                       




void limit(float *a, float max_value)    //限幅函数
{
	if(*a > max_value)
		*a = max_value;
	if(*a < -max_value)
		*a = -max_value;
}

/*pid计算*/
float pid_calculate(pid_typedef *pid,uint8_t model, float get_speed, float set_speed)     //PID计算->输出函数
{
	float index;
	
	if(model==0)
	{
			pid->get_angle[Now] = get_speed;                //实际值
			pid->set_angle[Now] = set_speed;                //目标值
			pid->err_angle[Now] = set_speed - get_speed;    //误差
			
			if(fabs(pid->err_angle[Now]) > pid->Max_err_angle && pid->Max_err_angle != 0)
				pid->pid_out_angle=0;
			
			
			if(fabs(pid->err_angle[Now]) >= pid->deadband_angle)             //死区判断
			{
				pid->pout = pid->kp_angle * pid->err_angle[Now];     //P
				if(fabs(pid->err_angle[Now])>1000)               //积分分离标准
				{
					index = 0;
				}else{
					index = 1;
				}
				pid->iout_angle += (pid->ki_angle * pid->err_angle[Now]*index);       //I
				limit(&(pid->iout_angle), pid->I_limit_angle);                  //I限幅		
				
				pid->dout = pid->kd_angle * (pid->err_angle[Now] - pid->err_angle[Last]);        //D			
				pid->pid_out_angle = pid->pout + pid->iout_angle + pid->dout;              //PID输出

				limit(&(pid->pid_out_angle), pid->Max_output_angle);                       //输出限幅
				pid->err_angle[Last] = pid->err_angle[Now];
				return pid->pid_out_angle;
			}
			else
			{
				pid->err_angle[Last] = pid->err_angle[Now];
				pid->pid_out_angle=0;
			}
	}
	else if(model==1)
	{
		  pid->get_speed[Now] = get_speed;                //实际值
			pid->set_speed[Now] = set_speed;                //目标值
			pid->err_speed[Now] = set_speed - get_speed;    //误差
			
			if(fabs(pid->err_speed[Now]) > pid->Max_err_speed && pid->Max_err_speed != 0)
				pid->pid_out_speed=0;
			
			
			if(fabs(pid->err_speed[Now]) >=pid->deadband_speed)             //死区判断
			{
				pid->pout = pid->kp_speed * pid->err_speed[Now];     //P
				if(fabs(pid->err_speed[Now])>1000)               //积分分离标准
				{
					index = 0;
				}else{
					index = 1;
				}
				pid->iout_speed += (pid->ki_speed * pid->err_speed[Now]*index);       //I
				limit(&(pid->iout_speed), pid->I_limit_speed);                  //I限幅		
				
				pid->dout = pid->kd_speed * (pid->err_speed[Now] - pid->err_speed[Last]);        //D			
				pid->pid_out_speed = pid->pout + pid->iout_speed + pid->dout;              //PID输出

				limit(&(pid->pid_out_speed), pid->Max_output_speed);                       //输出限幅
				pid->err_speed[Last] = pid->err_speed[Now];
				return pid->pid_out_speed;
			}
			else
			{
				pid->err_speed[Last] = pid->err_speed[Now];
				pid->pid_out_speed= 0;
			}
	}
	//在非 void 类型函数的最后一行添加 return 语句。
	
}
/*pid初始化*/
void pid_param_init(pid_typedef *pid, 
										float target_angle,float I_limit_angle,uint32_t Max_output_angle,uint32_t Max_err_angle,float deadband_angle,float pid_out_angle,float kp_angle,float ki_angle,float kd_angle,
										float target_speed,float I_limit_speed,uint32_t Max_output_speed,uint32_t Max_err_speed,float deadband_speed,float pid_out_speed,float kp_speed,float ki_speed,float kd_speed)
{
	pid->target_angle = target_angle ;						//目标值
	pid->I_limit_angle  = I_limit_angle ;					//积分限幅
	pid->Max_output_angle  = Max_output_angle ;		//最大输出
	pid->Max_err_angle  = Max_err_angle ;					//最大误差
	pid->deadband_angle  = deadband_angle ;				//死区
	pid->pid_out_angle  = pid_out_angle ;					//pid_out 
	pid->kp_angle  = kp_angle ;										//P
	pid->ki_angle  = ki_angle ;										//I
	pid->kd_angle  = kd_angle ;										//D
	
	pid->target_speed = target_speed;						//目标值
	pid->I_limit_speed = I_limit_speed;					//积分限幅
	pid->Max_output_speed = Max_output_speed;		//最大输出
	pid->Max_err_speed = Max_err_speed;					//最大误差
	pid->deadband_speed = deadband_speed;				//死区
	pid->pid_out_speed = pid_out_speed;					//pid_out 
	pid->kp_speed = kp_speed;										//P
	pid->ki_speed = ki_speed;										//I
	pid->kd_speed = kd_speed;										//D
					
}
/*pid清理*/
void pid_clear(pid_typedef *pid)
{
	pid->pout = 0;
	pid->iout_angle = 0;
	pid->iout_speed = 0;
	pid->dout = 0;
	pid->pid_out_angle = 0;
	pid->pid_out_speed = 0;
	for(int i=0;i<3;i++)
	{
		pid->err_angle[i] = 0;
		pid->get_speed[i] = 0;
	}
}
/*pid变化*/
void pid_change(pid_typedef *pid,float KP_angle,float KI_angle,float KD_angle,
																	float KP_speed,float KI_speed,float KD_speed)
{
	pid->kp_angle = KP_angle;
	pid->ki_angle = KI_angle;
	pid->kd_angle = KD_angle;
	pid->kp_speed = KP_speed;
	pid->ki_speed = KI_speed;
	pid->kd_speed = KD_speed;
}

void pid_init(pid_typedef *pid,
							float target_angle,float I_limit_angle,uint32_t Max_output_angle,uint32_t Max_err_angle,float deadband_angle,float pid_out_angle,float kp_angle,float ki_angle,float kd_angle,
							float target_speed,float I_limit_speed,uint32_t Max_output_speed,uint32_t Max_err_speed,float deadband_speed,float pid_out_speed,float kp_speed,float ki_speed,float kd_speed)                            
		                 // 预期目标                     积分限幅              最大输出              最大误差              死区                 pid_out             p               i               d
							//结构体初始化（将函数放入结构体）
{
	pid->f_param_init = pid_param_init;
	pid->f_calculate = pid_calculate;
	
	//初始化内外环的kp，ki，kd
	pid->f_param_init(pid,
		    target_angle,  I_limit_angle,  Max_output_angle,  Max_err_angle,  deadband_angle,  pid_out_angle,  kp_angle,  ki_angle,  kd_angle,
        target_speed,  I_limit_speed,  Max_output_speed,  Max_err_speed,  deadband_speed,  pid_out_speed,  kp_speed,  ki_speed,  kd_speed);
}
