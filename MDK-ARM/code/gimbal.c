#include "gimbal.h"

#define gimbal_debug (0)             //debug 真没搞懂

gimbal_variable_t varible_gimbal;   //云台变量存储
uint32_t angle_sss;
void gimbal_Init(void)
{
	send_Init();//发射初始化

	#if mode_car ==1	
	/*yaw轴电机初始化*/
	pid_init(&pid_yaw,
						0,6000,	16384, 0xFFFFFF,	0,0, 22.0f, 0.015,0, //外环位置环kp,ki,kd
						0,6000,	16384, 0xFFFFFF,   15, 0, 24.0f, 0.1f, 1.0f);//内环速度环kp,ki,kd
	/*pitch轴电机初始化*/
	pid_init(&pid_pitch,
						0,6000,16384,0xFFFFFF, 0, 0,   19.0f,  0.0f, 1, //外环位置环kp,ki,kd
						0,6000,16384,0xFFFFFF, 0, 0, 16.0f, 0.02f, 0);//内环速度环kp,ki,kd
#endif
	
/////////////////////--------------------------------------------------------------------------/////////////////////////////////////

#if mode_car ==2
/*哨兵上云台*/	
/*yaw轴电机初始化*/
	pid_init(&pid_yaw,
						0,6000,	16384, 0xFFFFFF,	0,0, 5.0f, 0,0, //外环位置环kp,ki,kd
						0,6000,	16384, 0xFFFFFF,   0, 0, 16.0f, 0.f, 0.0f);//内环速度环kp,ki,kd
	/*pitch轴电机初始化*/
	pid_init(&pid_pitch,
						0,6000,16384,0xFFFFFF, 0, 0,   22.5f,  0, 0, //外环位置环kp,ki,kd
						0,6000,16384,0xFFFFFF, 0, 0, 15.0f, 0.015f, 1.0f);//内环速度环kp,ki,kd	
#endif	
	
/////////////////////----------------------------------------------------------------------------------///////////////////////////////
	
	
	
	/*鼠标x轴滤波*/
	first_order_filter_init(&mouse_x_first,0.01,0.7f);//时间间隔    参数
	average_init(&mouse_x_moving,10);
	KalmanCreate(&mous_x_kalman,1,60);
	/*鼠标y轴滤波*/
	first_order_filter_init(&mouse_y_first,0.01,0.7f);//时间间隔    参数
	average_init(&mouse_y_moving,20);
	KalmanCreate(&mous_y_kalman,1,60);
	
	/*灵敏度模式*/
	//普通
	varible_gimbal.mouse_x_db = 0.04;
	varible_gimbal.mouse_y_db = 0.04;
	varible_gimbal.RC_x_db = 0.012;
	varible_gimbal.RC_y_db = 0.009;
	//视觉自瞄
	varible_gimbal.vis_mouse_x_db = 0.03;
	varible_gimbal.vis_mouse_y_db = 0.03;
	varible_gimbal.vis_RC_x_db = 0.012;
	varible_gimbal.vis_RC_y_db = 0.009;
	
	//视觉处理系数	
	varible_gimbal.vis_x_db =	0.012;
	varible_gimbal.vis_y_db =	0.009;
}

void gimbal_task(void)
{
	gimbal_key_control();		//鼠标数据滤波
	vision_task();/*视觉任务*/
	if(task_flag.mode == 1)     //判断模式
	{
		#if gimbal_debug == 0
				PID_CIMBAL_CHOICE();     //pid参数切换
		#endif
		if(task_flag.automatic_aiming == 1)   //开
		{
			YAW_MOTOR(1);
			PITCH_MOTOR(1);
		}			
		else if(task_flag.automatic_aiming == 0)
		{
			YAW_MOTOR(0);
			PITCH_MOTOR(0);
		} 
	
		send_task();  //发射任务
	}	

	pill_depot_task();	
	disability_gimbal_motor(task_flag.mode);   //云台无力
	motor_current_send_gimbal();
	
}

/*云台键鼠模式
	滤波处理鼠标数据
*/
void gimbal_key_control(void)
{
	/*鼠标滤波*/
	average_add(&mouse_x_moving,RC_DR16.mouse.x);
	first_order_filter_cali(&mouse_x_first,mouse_x_moving.aver_num);
	KalmanFilter(&mous_x_kalman,mouse_x_first.out);
	RC_DR16.mouse_x = mous_x_kalman.X_now;
	/*Y*/
	average_add(&mouse_y_moving,RC_DR16.mouse.y);
	first_order_filter_cali(&mouse_y_first,mouse_y_moving.aver_num);
	KalmanFilter(&mous_y_kalman,mouse_y_first.out);
	RC_DR16.mouse_y= mous_y_kalman.X_now;
}


/*云台pid切换*/
void PID_CIMBAL_CHOICE(void)
{	
	if(task_flag.auto_mode !=0) //自瞄开启
	{
		//切换或者首次进入清除pid
		if(varible_gimbal.pid_choice_flag[0] == 0)
		{
			pid_clear(&pid_yaw);
			pid_clear(&pid_pitch);
			/******************************************************/
			
			varible_gimbal.pid_choice_flag[0] = 1;
			varible_gimbal.pid_choice_flag[1] = 0;
		}
		pid_change(&pid_yaw,22,0.015,0                 //角度
									,24,0.1,1);	  //速度
		pid_change(&pid_pitch,19,0,1               //角度
									,16,0.02,0);	  //速度1
	}
	else if(task_flag.auto_mode ==0)
	{
		//切换或者首次进入清除pid
		if(varible_gimbal.pid_choice_flag[1] == 0)
		{
			pid_clear(&pid_yaw);
			pid_clear(&pid_pitch);
			/******************************************************/
			
			varible_gimbal.pid_choice_flag[0] = 0;
			varible_gimbal.pid_choice_flag[1] = 1;
		}
		pid_change(&pid_yaw,22,0.015,0                 //角度
									,24,0.1,1);	  //速度
		pid_change(&pid_pitch,19,0,1               //角度
									,16,0.02,0);	  //速度1
	}
	
}


/*yaw轴电机任务*/
void YAW_MOTOR(uint8_t flag)
{
	yaw_turn_round(task_flag.zig);
	if(varible_gimbal.yaw_imu_angle_flag == 0)
	{
		varible_gimbal.set_yaw_angle = imu.yaw_total;   //陀螺仪的yaw轴总角 yaw_total;
		varible_gimbal.yaw_imu_angle_flag = 1;
	}
	if(flag == 0)                                                                 //这些 - 负号 原理   
		varible_gimbal.set_yaw_angle -= RC16_CHN.ch1 * varible_gimbal.RC_x_db + RC_DR16.mouse_x *  varible_gimbal.mouse_x_db + varible_gimbal.yaw_turn_round_step;
	else if(flag == 1)
		varible_gimbal.set_yaw_angle -= RC16_CHN.ch1 * varible_gimbal.vis_RC_x_db + RC_DR16.mouse_x * varible_gimbal.vis_mouse_x_db  +varible_vision.vision_yaw_out *varible_gimbal.vis_x_db;
	
	/*PID计算*/
	varible_gimbal.set_yaw_speed = pid_yaw.f_calculate(&pid_yaw,0,imu.yaw_total,varible_gimbal.set_yaw_angle); //陀螺仪的yaw轴总角 yaw_total;
	pid_yaw.f_calculate(&pid_yaw,1,imu.yaw_speed,varible_gimbal.set_yaw_speed);             //             ->>>

}

//pitch轴电机任务
void PITCH_MOTOR(uint8_t flag)
{
	
	if(varible_gimbal.pitch_imu_angle_flag == 0)
	{
		varible_gimbal.set_pitch_angle = imu.pitch;
		varible_gimbal.pitch_imu_angle_flag = 1;
	}
	
	if(flag == 0)
	{
		varible_gimbal.set_pitch_angle += RC16_CHN.ch2 * varible_gimbal.RC_y_db - RC_DR16.mouse_y * varible_gimbal.mouse_y_db;
	}
	else if(flag == 1)
	{
		varible_gimbal.set_pitch_angle += RC16_CHN.ch2 * varible_gimbal.vis_RC_y_db - RC_DR16.mouse_y * varible_gimbal.vis_mouse_y_db + varible_vision.vision_pitch_out *varible_gimbal.vis_y_db ;
	}
	if((pitch_RX.angle >= pitch_up_angle &&(RC16_CHN.ch2 * varible_gimbal.RC_y_db - RC_DR16.mouse_y * varible_gimbal.mouse_y_db) > 0) ||
		(pitch_RX.angle <= pitch_down_angle &&(RC16_CHN.ch2 * varible_gimbal.RC_y_db - RC_DR16.mouse_y * varible_gimbal.mouse_y_db ) < 0))
		varible_gimbal.set_pitch_angle = imu.pitch;
	/*pid计算->串级*/
	varible_gimbal.set_pitch_speed = pid_pitch.f_calculate(&pid_pitch,0,imu.pitch,varible_gimbal.set_pitch_angle);
		pid_pitch.f_calculate(&pid_pitch,1,imu.pitch_speed,varible_gimbal.set_pitch_speed);

}



/*yaw轴一键掉头*/
void yaw_turn_round(uint8_t flag)
{
	if(flag == 1)
	{	
//		static uint16_t i = 0;
//		for(;i < 500;i++)
//		{
//			if(varible_chassis.top_direction == -1)
//				varible_gimbal.set_yaw_angle += 8.2f;
//			else 
//				varible_gimbal.set_yaw_angle += -8.2f;
//		}
//		if(i >= 498)
//		{	
//			task_flag.zig = 0;
//			i = 0;
//		}
			
//		task_flag.zig = 0;
//		varible_gimbal.set_yaw_angle -=4096;
		//记录触发时间
		if(varible_gimbal.yaw_turn_round_flag == 0)
		{
			varible_gimbal.yaw_turn_round_time = HAL_GetTick();
			varible_gimbal.yaw_turn_round_flag = 1;
		}
		//在触发后一定时间内掉头步进值赋值  500ms就可以转一圈？？？？
		if(HAL_GetTick () - varible_gimbal.yaw_turn_round_time <= 530)
		{
//			if(varible_chassis.top_direction == -1)
				varible_gimbal.yaw_turn_round_step = -8.7f;
//			else varible_gimbal.yaw_turn_round_step = -8.7f;
		}
		//触发一定时间后步进值为0，恢复标志位
		else
		{
			varible_gimbal.yaw_turn_round_step = 0;
			varible_gimbal.yaw_turn_round_time = 0;
			varible_gimbal.yaw_turn_round_flag = 0;
			task_flag.zig = 0;
		
		}
	}	
}


/*云台无力*/
void disability_gimbal_motor(uint8_t mode)
{
	if(mode == 0)
	{
		pid_clear(&pid_pitch);
		pid_clear(&pid_yaw);
		pid_clear(&pid_send);/*拨弹盘电机*/
		if(varible_gimbal.shoot_stop_time >=10)
		{
			varible_send.set_shoot_speed[0] = 0;
			varible_send.set_shoot_speed[1] = 0;
			varible_gimbal.shoot_stop_time --;
			for(int i = 0;i < 2;i++)	
			{
				pid_shoot[i].f_calculate(&pid_shoot[i],1,shoot_RX[i].speed,varible_send.set_shoot_speed[i]);
			}
		}
		else
		{
			task_flag.frictiongear = 0;
			pid_clear(&pid_shoot[0]);
			pid_clear(&pid_shoot[1]);	
		}	
		/*标志位恢复*/
		varible_gimbal.pitch_imu_angle_flag = 0;
		varible_gimbal.yaw_imu_angle_flag = 0;
		varible_send.send_flag = 0;				//拨弹盘乱转
		
	}
	else
		varible_gimbal.shoot_stop_time = 200;
}









