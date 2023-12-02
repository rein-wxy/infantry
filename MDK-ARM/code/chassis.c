#include "chassis.h"

/*底盘初始化*/
void chassis_Init(void)
{
	for(int i=0;i<4;i++)
	{
		pid_init(&pid_Mchassis[i],
								0,  0, 0, 0, 0, 0, 0, 0, 0, //外环位置环kp,ki,kd
								0,	1000,	16384,	0xFFFFFF, 0, 0, 7.6f, 	0, 	0.01);//内环速度环kp,ki,kd
	
	}
	/*底盘跟随初始化*/
	pid_init(&pid_chassis_follow,
								0,	0,	0, 0,	0,0, 0, 0,0, //外环位置环kp,ki,kd
								0,	150,	4000,		0xFFFFFF,   25,        0, 4.0f, 0, 0);//内环速度环kp,ki,kd
}	
//死区  pid_out

void chassis_task(void)
{
	chassis_assist_task();//辅助任务
	super_cap_task();/*超级电容*/
	if(task_flag.mode == 1)
	{
		if(task_flag.small_gyro == 0 )	
			chassis_follow();       //底盘跟随
		if(task_flag.small_gyro == 1)
			chassis_top();          //小陀螺
		for(int i = 0;i < 4;i++)
		{
			pid_Mchassis[i].f_calculate(&pid_Mchassis[i],1,chassis_motor_RX[i].speed,varible_chassis.set_chassis_speed[i]);
		}
		chassis_power_limit();      //功率限制
	}
	chassis_motor_diversion(task_flag.mode);   //底盘电机无力
	Motor_current_send_chassis();
}



/*底盘跟随*/
void chassis_follow(void)
{	
	/*清理小陀螺,标志位恢复*/
	Slow_clear(&top_ramp);
	varible_chassis.top_direction_flag = 0;
	if(varible_chassis.wait_follow_flag <=150)
	{
		/****************等待大约150ms后找头*************************************/
		varible_chassis.get_degree_1 = 0;
		varible_chassis.get_degree_2 = 0;
		varible_chassis.wait_follow_flag ++;
	}
	chassis_follow_degree();	//跟随角度
	chassis_caculate(varible_chassis.gyro_x,varible_chassis.gyro_y,pid_chassis_follow.pid_out_speed);//pid计算电机找头->pid_chassis_follow.pid_out_speed
}



/*小陀螺*/
void chassis_top(void)
{
	if(varible_chassis.top_direction_flag == 0)
	{
		Top_direction();
		varible_chassis.top_direction_flag = 1;
	}
	//Top_change();
	/*小陀螺时可控制移动->控制功率*/
	if(RC_DR16.ch3 != 0 || RC_DR16.ch4 != 0 ||chassis_x_ramp.remp_num_now != 0 ||chassis_x_ramp.remp_num_now != 0)
		chassis_top_speed = chassis_top_speed * 0.78f;
	else
		chassis_top_speed = chassis_top_speed;
	top_ramp.remp_num_target = chassis_top_speed;
	Slow(&top_ramp.remp_num_now ,&top_ramp.remp_num_target ,10,0.4,0.5);

#if chassis_stop_if == 0
	chassis_caculate(varible_chassis.gyro_x,varible_chassis.gyro_y,top_ramp.remp_num_now * varible_chassis.top_direction /**varible_chassis.top_s_c*/);
#elif chassis_stop_if == 1
	chassis_caculate(varible_chassis.gyro_x,varible_chassis.gyro_y,0);

#endif
	varible_chassis.wait_follow_flag  = 1;   //打开等待找头标志位
}



/*
	电机卸力
	电机不转啦
*/
void chassis_motor_diversion(uint8_t mode)
{
	if(mode == 0)
	{
		pid_clear(&pid_chassis_follow);
		for(int i = 0;i < 4;i++)
		{
			pid_clear(&pid_Mchassis[i]);
			varible_chassis.send_chassis_speed[i] = 0;
		}
		Slow_clear(&top_ramp);
		varible_chassis.top_direction_flag = 0;            
	}

}












