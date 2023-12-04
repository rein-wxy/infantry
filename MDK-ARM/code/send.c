#include "send.h"

varible_send_t varible_send;    //发射变量存储


void send_Init(void)
{
	/*拨弹盘2006电机*/   //0.31
	pid_init(&pid_send,	 0, 5000, 16384, 368640,  0, 0, 0.4f, 0.0, 0
						,0,	5000, 16384, 0xFFFFFF,0, 0, 20, 0.003f, 0.01);
//	/*拨弹盘3508电机*/
//	pid_init(&pid_send,	 0, 5000, 16384, 194560,  0, 0, 0.38f, 0,7.6f
//						,0,	5000, 16384, 0xFFFFFF,0, 0, 15.0f, 0,0);
	/*摩擦轮电机*/
	for(int i = 0;i < 2;i++)
	{
		pid_init(&pid_shoot[i],0,0,0,0,0,0,0,0,0
							,0,2000,16384,0xffffff, 0, 0, 10.0f, 0.0f, 0);
	}
	
	/*摩擦轮电机*/
	//HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	/*弹舱舵机PWM使能*/
//	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);//占空比0--2000
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
}

void send_task(void)
{
	heat_limit_task();	        //热量射频限制
	pill_depot_task();          //弹舱
	shoot_task();               //摩擦轮
	locked_rotor_tesk();
	if(varible_send.lock_pill == 0&& allow_send_bullet == 1 && varible_send.fric_send_flag == 1 )   //允许发弹   卡弹--热量--摩擦轮
		stir_task();	
	else
		pid_send.pid_out_speed = 0;
	if(varible_send.lock_pill == 1)/*堵转处理*/
		locked_task();  		
}

/*摩擦轮任务*/
void shoot_task(void)
{
	switch(speed_limit)
	{
		case 15:
			if(varible_send.speed_flag[0] == 0)
			{
				varible_send.speed_speed = 4150;//------->
				varible_send.speed_flag[0] = 1;
				varible_send.speed_flag[1] = 0;
				varible_send.speed_flag[2] = 0;
			}
			if(varible_send.send_data_update == 1)//射速更新
			{
				varible_send.send_data_update = 0;
#if	speed_new  == 1					
				//varible_send.speed_speed -=speed_offest(now_bullet_speed,14.0f);
				varible_send.speed_speed -= (now_bullet_speed - 13.6f) * 1.0;
#endif				
#if	speed_new  == 0		
							
				if(now_bullet_speed > 14.8f)
					varible_send.speed_speed -= 50;
				else if(now_bullet_speed > 14.7f)
					varible_send.speed_speed -= 32;
				else if(now_bullet_speed > 14.6f)
					varible_send.speed_speed -= 20;
				else if(now_bullet_speed > 14.5f)
					varible_send.speed_speed -= 16;
				else if(now_bullet_speed > 14.4f)
					varible_send.speed_speed -= 8;
				else if(now_bullet_speed > 14.3f)
					varible_send.speed_speed -= 4;
				else if(now_bullet_speed > 14.2f)
					varible_send.speed_speed -= 2;
				else if(now_bullet_speed < 14.0f)
					varible_send.speed_speed += 0;
				else if(now_bullet_speed < 13.8f)
					varible_send.speed_speed += 8;
				else if(now_bullet_speed < 13.6f)
					varible_send.speed_speed += 16;
				else if(now_bullet_speed < 13.5f)
					varible_send.speed_speed += 20;
				else varible_send.speed_speed = varible_send.speed_speed;
			
#endif				
			}
					
			break;
		case 18:
			
			if(varible_send.speed_flag[1] == 0)
			{
				varible_send.speed_speed = 4450;//------->
				varible_send.speed_flag[0] = 0;
				varible_send.speed_flag[1] = 1;
				varible_send.speed_flag[2] = 0;
			}
			if(varible_send.send_data_update == 1)//射速更新
			{
				varible_send.send_data_update = 0;
#if	speed_new  == 1					
				varible_send.speed_speed -= (now_bullet_speed - 16.6f) *1;
#endif				
#if	speed_new  == 0					
				
				if(now_bullet_speed > 17.9f)
					varible_send.speed_speed -= 50;
				else if(now_bullet_speed > 17.8f)
					varible_send.speed_speed -= 32;
				else if(now_bullet_speed > 17.6f)
					varible_send.speed_speed -= 20;
				else if(now_bullet_speed > 17.4f)
					varible_send.speed_speed -= 10;
				else if(now_bullet_speed > 17.2f)
					varible_send.speed_speed -= 4;
				else if(now_bullet_speed < 17.0f)
					varible_send.speed_speed += 0;
				else if(now_bullet_speed < 16.8f)
					varible_send.speed_speed += 3;
				else if(now_bullet_speed < 16.6f)
					varible_send.speed_speed += 10;
				else if(now_bullet_speed < 16.4f)
					varible_send.speed_speed += 20;
				else varible_send.speed_speed = varible_send.speed_speed;
				
#endif				
			}
		
			break;
		case 30:	
			if(varible_send.speed_flag[2] == 0)
			{
				varible_send.speed_speed = 6850;//------->
				varible_send.speed_flag[0] = 0;
				varible_send.speed_flag[1] = 0;
				varible_send.speed_flag[2] = 1;
			}
			if(varible_send.send_data_update == 1)//射速更新
			{
				varible_send.send_data_update = 0;
#if	speed_new  == 1					
				varible_send.speed_speed -= (now_bullet_speed - 27.00f) * 0.43;
#endif				
#if	speed_new  == 0					
				if(now_bullet_speed > 28.1f)
					varible_send.speed_speed -= 50;
				else if(now_bullet_speed > 27.9f)
					varible_send.speed_speed -= 32;
				
				else if(now_bullet_speed > 27.6f)
					varible_send.speed_speed -= 16;
				else if(now_bullet_speed > 27.4f)
					varible_send.speed_speed -= 4;
				else if(now_bullet_speed > 27.2f)
					varible_send.speed_speed -= 1;
				else if(now_bullet_speed > 27.1f)
					varible_send.speed_speed += 0;
				else if(now_bullet_speed < 26.9f)
					varible_send.speed_speed += 2;
				else if(now_bullet_speed < 26.7f)
					varible_send.speed_speed += 8;
				else if(now_bullet_speed < 26.5f)
					varible_send.speed_speed += 20;
				else if(now_bullet_speed < 26.3f)
					varible_send.speed_speed += 32;
				else varible_send.speed_speed = varible_send.speed_speed;
#endif								
			}	
			break;
		default:
			varible_send.speed_flag[0] = 0;
			varible_send.speed_flag[1] = 0;
			varible_send.speed_flag[2]= 0;	
			varible_send.speed_speed = 4000;
	}
	if(task_flag.frictiongear == 1)        //开启摩擦轮
	{
		varible_send.set_shoot_speed[0] = varible_send.speed_speed;//上供
		varible_send.set_shoot_speed[1] = -varible_send.speed_speed;
		
//		varible_send.set_shoot_speed[0] = 3500;
//		varible_send.set_shoot_speed[1] = -3500;
	
		/*确保摩擦轮转起来了----->为什么是||或*/
		if(shoot_RX[0].speed > 1000 ||shoot_RX[1].speed < -1000)
		{
			varible_send.fric_send_flag = 1;			//摩擦轮已转动
		}
		else 
			varible_send.fric_send_flag = 0;
	}
	else if(task_flag.frictiongear == 0)    //关闭摩擦轮
	{
		varible_send.set_shoot_speed[0] = 0;
		varible_send.set_shoot_speed[1] = 0;
		varible_send.fric_send_flag = 0;
		/*拨弹恢复*/
		varible_send.lock_pill= 0;			//卡弹标志位重置
		pid_send.err_angle[0] = 0;			//清除误差
		pid_send.err_angle[1] = 0;			//清除误差
		varible_send.set_send_angle = send_RX.total_angle;    //记录当前值为目标值
	}
	for(int i = 0;i < 2;i++)	
	{
		pid_shoot[i].f_calculate(&pid_shoot[i],1,shoot_RX[i].speed,varible_send.set_shoot_speed[i]);
	}
}

/*发射模式  单发/连发*/
/*拨弹盘任务*/


void stir_task(void)
{
	if(varible_send.send_flag == 0)   //防止拨弹盘乱转
	{
		varible_send.set_send_angle = send_RX.total_angle;
		varible_send.send_flag = 1;		
	}
	if(task_flag.load == 0 && task_flag.load_mo == 0)
	{
		varible_send.send_mode_flag = 0;
		varible_send.send_mode_time = 0;
	}
	send_mo();
	send_rc();
	
	varible_send.set_send_speed = pid_send.f_calculate(&pid_send,0,send_RX.total_angle,varible_send.set_send_angle);
	pid_send.f_calculate(&pid_send,1,send_RX.speed,varible_send.set_send_speed);
}

void send_mo(void)
{
	 if(task_flag.load_mo == 1)
	{	
		if(varible_send.send_mode_flag == 0)
		{	
			varible_send.set_send_angle -= 36864.0f;   // \拨弹盘齿数
			varible_send.send_mode_flag = 1;
			task_flag.pill_depot = 0;
			varible_send.send_mode_time = HAL_GetTick();
		}
		else if(varible_send.send_mode_flag == 1)
		{

			if(HAL_GetTick()-varible_send.send_mode_time > 250)
			{
				
				varible_send.set_send_angle -=36864.0f;//36864.0f;// ////拨弹盘齿数
				varible_send.send_mode_flag = 2;
				varible_send.send_mode_time = HAL_GetTick();
			} 
		}
		else if(varible_send.send_mode_flag == 2)
		{
			if(HAL_GetTick()-varible_send.send_mode_time >varible_send.send_freq)
			{
			
				varible_send.set_send_angle -= 36864.0f;//;//36864.0f;   //转一格（拨弹盘齿数）
				varible_send.send_mode_time = HAL_GetTick();
			}
		}	
	}
	
}

void send_rc(void)
{
	if(task_flag.load == 1)
	{	
		if(varible_send.send_mode_flag == 0)
		{	
			varible_send.set_send_angle -= 36864.0f;   // \拨弹盘齿数
			varible_send.send_mode_flag = 1;
			task_flag.pill_depot = 0;
			varible_send.send_mode_time = HAL_GetTick();
		}
		else if(varible_send.send_mode_flag == 1)
		{
			if(HAL_GetTick()-varible_send.send_mode_time > 250)
			{			
				varible_send.set_send_angle -=36864.0f;//36864.0f;// ////拨弹盘齿数
				varible_send.send_mode_flag = 2;
				varible_send.send_mode_time = HAL_GetTick();
			} 
		}
		else if(varible_send.send_mode_flag == 2)
		{
			if(HAL_GetTick()-varible_send.send_mode_time >varible_send.send_freq)
			{		
				varible_send.set_send_angle -= 36864.0f;//;//36864.0f;   //转一格（拨弹盘齿数）
				varible_send.send_mode_time = HAL_GetTick();
			}
		}	
	}
}

/*堵转检测*/
void locked_rotor_tesk(void)
{
	//if(pid_send.err_angle[0] > 184320 || pid_send.err_angle[0] < -184320)    //----->
	if(pid_send.err_angle[0] > 100000 || pid_send.err_angle[0] < -100000)    	
	varible_send.lock_pill = 1;     //堵
	else
		varible_send.lock_pill = 0;
}

/*堵转处理*/
void locked_task(void)
{
	if(varible_send.lock_pid_clear_flag[0] == 0)
	{
		pid_clear(&pid_send);
		varible_send.lock_pid_clear_flag[0] = 1;
		varible_send.lock_pid_clear_flag[1] = 0;
	}
	if(varible_send.lock_flag <=150)//反转时间
	{
		pid_send.f_calculate(&pid_send,1,send_RX.speed,1500);  //反转速度------>
		varible_send.lock_flag ++;
	}
	else
	{
		if(varible_send.lock_pid_clear_flag[1] == 0)
		{
			pid_clear(&pid_send);
			varible_send.lock_pid_clear_flag[0] = 0;
			varible_send.lock_pid_clear_flag[1] = 1;
		}
		varible_send.set_send_angle = send_RX.total_angle;
		varible_send.lock_flag = 0;  //反转处理时间清零
		varible_send.lock_pill = 0;  //堵转处理完成	
		varible_send.send_flag = 0;  //防止乱转标志恢复
	}
}

/*
	speed:预期目标值
	limit:限制速度15 18 30
	将将系数映射到tan正负二分之Π之间
*/

float speed_offest(float speed,float limit)
{
	float a = (speed - limit),b;

	if(a<=0.5 && a >=-0.5) 	 
		b =tan( a*1.571);	//pai/2
	else if	(a>=0.5 )		 
		b = tan(0.785);			//限制x防止弹速变化大
	else if	(a<=-0.5 ) 		 
		b = tan(-0.785) ;	
	return b* 10;
}



/*弹舱*/

void pill_depot_task(void)
{
	if(task_flag.pill_depot == 1)
	{
		TIM1->CCR1= 110;
	}
	if(task_flag.pill_depot == 0)
	{
		TIM1->CCR1 = 202;
	}
}
