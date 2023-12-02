 #include "chassis_t.h"

#define light_hand (2)	//2灯条前

chassis_variable_t varible_chassis;//底盘变量储存
chassis_key_t chassis_key_mode;    //键鼠模式平移

/*底盘辅助任务*/
void chassis_assist_task(void)
{
	CHASSIS_follow_pid_change();
	CHASSIS_KEY_MODE();
	Relative_degree();
	//find_degree();
	#if light_hand == 1
	gyroscope_remote_resolve(RC16_CHN.ch3 + chassis_x_ramp.remp_num_now , RC16_CHN.ch4 + chassis_y_ramp.remp_num_now ,varible_chassis.degree_1);
	#endif
	#if light_hand == 2
	gyroscope_remote_resolve(-RC16_CHN.ch3 - chassis_x_ramp.remp_num_now , -RC16_CHN.ch4 - chassis_y_ramp.remp_num_now ,varible_chassis.degree_1);
	#endif
	chassis_power_torque();
}

/*底盘麦轮电机解算
	0--1
	2--3
*/

void chassis_caculate(int16_t x,int16_t y,int16_t z)
{
	chassis_max_speed_ramp.remp_num_target = chassis_max_speed;  //裁判系统/超级电容器限制
	Slow(&chassis_max_speed_ramp.remp_num_now,&chassis_max_speed_ramp.remp_num_target,10,0.8,0.2);  //步进值 目标值在里速率 目标值在外速率
	
	varible_chassis.set_chassis_speed[0] =(+x+y) * chassis_max_speed_ramp.remp_num_now /660.f + z ;
	varible_chassis.set_chassis_speed[2] =(-x+y) * chassis_max_speed_ramp.remp_num_now /660.f + z ;
	varible_chassis.set_chassis_speed[3] =(-x-y) * chassis_max_speed_ramp.remp_num_now /660.f + z ;
	varible_chassis.set_chassis_speed[1] =(+x-y) * chassis_max_speed_ramp.remp_num_now /660.f + z ;
	
}



/*运动解算，云台头为前*/
void gyroscope_remote_resolve(float channel1,float channel2,float re_angle)
{
	float sin1,cos1,Pythagorean;
	float return_angle,tan;
  if(channel2 != 0)
  {
	  tan = channel1/channel2;
	  return_angle = atan(tan)*180/PI;
  }
  else if(channel2 == 0&& channel1>0)
    return_angle = 90;
  else if(channel2 == 0&& channel1<0)
    return_angle = -90;
  if(channel1>0&&channel2<0)
     return_angle = return_angle+180;
  if(channel1<0&&channel2<0)
     return_angle = return_angle-180;
  if(channel1 == 0&& channel2<0)
     return_angle = 180;
  if(channel1 == 0&& channel2>0)
     return_angle = 0;
  if(channel2 == 0&&channel1 == 0)
  {
	sin1 = 0;
	cos1 = 0;
  }
  else
  {
    sin1 = sin((re_angle+return_angle)*PI/180);
    cos1 = cos((re_angle+return_angle)*PI/180);
  }
    Pythagorean = sqrt(channel1*channel1+channel2*channel2);
    varible_chassis.gyro_y = cos1*Pythagorean;   
	varible_chassis.gyro_x = sin1*Pythagorean;
}




//底盘跟随 pid切换 功率改变
void CHASSIS_follow_pid_change(void)
{
	if(POWER_LIMIT>=45 &&POWER_LIMIT <=55  )
	{
		if(varible_chassis.chassis_follow_pid_change[0] == 0)
		{
			pid_clear(&pid_chassis_follow);
			varible_chassis.chassis_follow_pid_change[0] = 1;
			varible_chassis.chassis_follow_pid_change[1] = 0;
			varible_chassis.chassis_follow_pid_change[2] = 0;
		}
		pid_change(&pid_chassis_follow,0,0,0,5.5,0,0);		
	}
	else if(POWER_LIMIT>=60 )
	{
		if(varible_chassis.chassis_follow_pid_change[1] == 0)
		{
			pid_clear(&pid_chassis_follow);
			varible_chassis.chassis_follow_pid_change[0] = 0;
			varible_chassis.chassis_follow_pid_change[1] = 1;
			varible_chassis.chassis_follow_pid_change[2] = 0;
		}
		pid_change(&pid_chassis_follow,0,0,0,6,0,0);
	}
	else if(task_flag.cap_mode != 0 )
	{
		if(varible_chassis.chassis_follow_pid_change[2] == 0)
		{
			pid_clear(&pid_chassis_follow);
			varible_chassis.chassis_follow_pid_change[0] = 0;
			varible_chassis.chassis_follow_pid_change[1] = 0;
			varible_chassis.chassis_follow_pid_change[2] = 1;
		}
		pid_change(&pid_chassis_follow,0,0,0,6.5,0,0);
	}
	
}



/*计算云台/底盘相对角度*/
//设2000位零度  180--->0--->-180


//void Relative_degree(void)
//{
//	
//}
void Relative_degree(void)
{
	if(task_flag.auto_mode != 2)  //打符模式不跟随
	{
		#if light_hand == 1
		if(yaw_RX.angle >= 4044 && yaw_RX.angle < 8140)
			varible_chassis.degree_1 = ((yaw_RX.angle-4044 )/22.75f) * -1.0f;
		else if(yaw_RX.angle >= 8140 &&yaw_RX.angle < 8192)
			varible_chassis.degree_1 = (( 8192-yaw_RX.angle+4044 )/22.75f) * 1.0f;
		else if(yaw_RX.angle >= 0 && yaw_RX.angle < 4043)
			varible_chassis.degree_1 = ((4044- yaw_RX.angle )/22.75f) * 1.0f;
		#endif
		#if light_hand == 2
		if(yaw_RX.angle >= 4044 && yaw_RX.angle < 8140)  //灯条在前
			varible_chassis.degree_1 = ((8140-yaw_RX.angle )/22.75f) * 1.0f;
		else if(yaw_RX.angle >= 8140 &&yaw_RX.angle < 8192)
			varible_chassis.degree_1 = (( yaw_RX.angle -8144 )/22.75f) * -1.0f;
		else if(yaw_RX.angle >= 0 && yaw_RX.angle < 4043)
			varible_chassis.degree_1 = ((52 + yaw_RX.angle )/22.75f) * -1.0f;
		#endif		
	}
	else  varible_chassis.degree_1 = 0;
	
	/*degree_2 --> ui*/
	if(varible_chassis.degree_1 >= 0)
		varible_chassis.degree_2 = (-1.0f) * varible_chassis.degree_1;
	else
		varible_chassis.degree_2 = 180.0f -(-1.0f * varible_chassis.degree_1);
	
	varible_chassis.get_degree_1 = degree_upstep(varible_chassis.degree_1);
	varible_chassis.get_degree_2 = degree_upstep(varible_chassis.degree_2);
	
}

void find_degree(void)
{	
	varible_chassis.temp1 = yaw_RX.angle - 8140;
	
	if(varible_chassis.temp1 > 4096)
		varible_chassis.temp1 -= 8192;
    else if(varible_chassis.temp1 < -4096)
		varible_chassis.temp1 += 8192;
	
	varible_chassis.angle1 = varible_chassis.temp1/22.75f;
	varible_chassis.get_degree_1 = degree_upstep(-varible_chassis.angle1);
}


/*角度扩大计算*/
float degree_upstep(float degree)
{
	float get_degree;
	if(degree <= -expansion_degree && degree >= -180.0f) 
		get_degree =  degree + expansion_degree;
	if(degree >= expansion_degree && degree <= 180.0f) 
		get_degree =  degree - expansion_degree;
	if(degree <= -(180.0f-expansion_degree)) 
		get_degree =  -180.0f;
	if(degree >= (180.0f-expansion_degree)) 
		get_degree =  180.0f;
	if(degree <= expansion_degree && degree >= -expansion_degree) 
		get_degree =  0.0f;
	return get_degree;
}


/*底盘键鼠模式*/
void CHASSIS_KEY_MODE(void)
{
	
	/*没有按下*/
	if((RC_DR16.keyBoard.key_code & KEY_W) == 0 && (RC_DR16.keyBoard.key_code & KEY_S) == 0)
		chassis_y_ramp.remp_num_target = 0;
	if((RC_DR16.keyBoard.key_code & KEY_A) == 0 && (RC_DR16.keyBoard.key_code & KEY_D) == 0)
		chassis_x_ramp.remp_num_target = 0;
	/*按下W*/
	if(RC_DR16.keyBoard.key_code & KEY_W)
		chassis_key_mode.Vw = 660;
	else
		chassis_key_mode.Vw = 0;
	/*按下S*/
	if(RC_DR16.keyBoard.key_code & KEY_S)
		chassis_key_mode.Vs = -660;
	else
		chassis_key_mode.Vs = 0; 
	/*按下D*/
	if(RC_DR16.keyBoard.key_code & KEY_D)
		chassis_key_mode.Vd = 660;
	else
		chassis_key_mode.Vd = 0; 
	/*按下A*/
	if(RC_DR16.keyBoard.key_code & KEY_A)
		chassis_key_mode.Va = -660;
	else
		chassis_key_mode.Va = 0;
	
	chassis_x_ramp.remp_num_target = chassis_key_mode.Va + chassis_key_mode.Vd ;
	chassis_y_ramp.remp_num_target = chassis_key_mode.Vw + chassis_key_mode.Vs ;
	
//	chassis_x_ramp.remp_num_target = chassis_key_mode.Va + chassis_key_mode.Vd + RC16_CHN.ch4;
//	chassis_y_ramp.remp_num_target = chassis_key_mode.Vw + chassis_key_mode.Vs + RC16_CHN.ch3;
	
	/*斜坡处理X,Y值*/  
	/*停止*/
	/*正常移动*/
	Slow(&chassis_x_ramp.remp_num_now,&chassis_x_ramp.remp_num_target,6,0.35,0.35);
	Slow(&chassis_y_ramp.remp_num_now,&chassis_y_ramp.remp_num_target,6,0.35,0.35);
	
	if(chassis_y_ramp.remp_num_target == 0 && chassis_x_ramp.remp_num_target == 0)
	{
		Slow(&chassis_x_ramp.remp_num_now,&chassis_x_ramp.remp_num_target,0,0,0);
		Slow(&chassis_y_ramp.remp_num_now,&chassis_y_ramp.remp_num_target,0,0,0);
	}
	
	
}
/*小陀螺方向*/
void Top_direction(void)
{
     //yaw_RX.angle%2 ? varible_chassis.top_direction = 1.0f : varible_chassis.top_direction = -1.0f;
	if(yaw_RX.angle % 2 == 0)
		varible_chassis.top_direction = 1.0f;
	else
		varible_chassis.top_direction = -1.0f;
}

void Top_change(void)
{
	static	uint16_t i = 0;
	i++;
	varible_chassis.top_s_c = (sin(i* 0.001571) * 0.25 + 0.75);
	if(i >=4000)
		i = 0;
	
}

void chassis_follow_degree(void)
{
	if(varible_chassis.get_degree_1 == 0)
	{
		pid_clear(&pid_chassis_follow);
	}
	/*#if预编译调试用*/
	#if chassis_stop_if == 0                           //目标0.0度                        *1度步进值
	pid_chassis_follow.f_calculate(&pid_chassis_follow,1,0.0,varible_chassis.get_degree_1 * 22.755555f); //********************
	#elif chassis_stop_if == 1
	pid_chassis_follow.f_calculate(&pid_chassis_follow,1,0.0,0.0);
	#endif
	
}



void chassis_power_torque(void)
{
	varible_chassis.total_torque  = (chassis_motor_RX[0].speed *chassis_motor_RX[0].torque * 0.001)+(chassis_motor_RX[1].speed *chassis_motor_RX[1].torque * 0.001)+
												(chassis_motor_RX[2].speed *chassis_motor_RX[2].torque * 0.001)+(chassis_motor_RX[3].speed *chassis_motor_RX[3].torque * 0.001);
	
}























