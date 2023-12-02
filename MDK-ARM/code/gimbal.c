#include "gimbal.h"

#define gimbal_debug (0)             //debug ��û�㶮

gimbal_variable_t varible_gimbal;   //��̨�����洢
uint32_t angle_sss;
void gimbal_Init(void)
{
	send_Init();//�����ʼ��

	#if mode_car ==1	
	/*yaw������ʼ��*/
	pid_init(&pid_yaw,
						0,6000,	16384, 0xFFFFFF,	0,0, 22.0f, 0.015,0, //�⻷λ�û�kp,ki,kd
						0,6000,	16384, 0xFFFFFF,   15, 0, 24.0f, 0.1f, 1.0f);//�ڻ��ٶȻ�kp,ki,kd
	/*pitch������ʼ��*/
	pid_init(&pid_pitch,
						0,6000,16384,0xFFFFFF, 0, 0,   19.0f,  0.0f, 1, //�⻷λ�û�kp,ki,kd
						0,6000,16384,0xFFFFFF, 0, 0, 16.0f, 0.02f, 0);//�ڻ��ٶȻ�kp,ki,kd
#endif
	
/////////////////////--------------------------------------------------------------------------/////////////////////////////////////

#if mode_car ==2
/*�ڱ�����̨*/	
/*yaw������ʼ��*/
	pid_init(&pid_yaw,
						0,6000,	16384, 0xFFFFFF,	0,0, 5.0f, 0,0, //�⻷λ�û�kp,ki,kd
						0,6000,	16384, 0xFFFFFF,   0, 0, 16.0f, 0.f, 0.0f);//�ڻ��ٶȻ�kp,ki,kd
	/*pitch������ʼ��*/
	pid_init(&pid_pitch,
						0,6000,16384,0xFFFFFF, 0, 0,   22.5f,  0, 0, //�⻷λ�û�kp,ki,kd
						0,6000,16384,0xFFFFFF, 0, 0, 15.0f, 0.015f, 1.0f);//�ڻ��ٶȻ�kp,ki,kd	
#endif	
	
/////////////////////----------------------------------------------------------------------------------///////////////////////////////
	
	
	
	/*���x���˲�*/
	first_order_filter_init(&mouse_x_first,0.01,0.7f);//ʱ����    ����
	average_init(&mouse_x_moving,10);
	KalmanCreate(&mous_x_kalman,1,60);
	/*���y���˲�*/
	first_order_filter_init(&mouse_y_first,0.01,0.7f);//ʱ����    ����
	average_init(&mouse_y_moving,20);
	KalmanCreate(&mous_y_kalman,1,60);
	
	/*������ģʽ*/
	//��ͨ
	varible_gimbal.mouse_x_db = 0.04;
	varible_gimbal.mouse_y_db = 0.04;
	varible_gimbal.RC_x_db = 0.012;
	varible_gimbal.RC_y_db = 0.009;
	//�Ӿ�����
	varible_gimbal.vis_mouse_x_db = 0.03;
	varible_gimbal.vis_mouse_y_db = 0.03;
	varible_gimbal.vis_RC_x_db = 0.012;
	varible_gimbal.vis_RC_y_db = 0.009;
	
	//�Ӿ�����ϵ��	
	varible_gimbal.vis_x_db =	0.012;
	varible_gimbal.vis_y_db =	0.009;
}

void gimbal_task(void)
{
	gimbal_key_control();		//��������˲�
	vision_task();/*�Ӿ�����*/
	if(task_flag.mode == 1)     //�ж�ģʽ
	{
		#if gimbal_debug == 0
				PID_CIMBAL_CHOICE();     //pid�����л�
		#endif
		if(task_flag.automatic_aiming == 1)   //��
		{
			YAW_MOTOR(1);
			PITCH_MOTOR(1);
		}			
		else if(task_flag.automatic_aiming == 0)
		{
			YAW_MOTOR(0);
			PITCH_MOTOR(0);
		} 
	
		send_task();  //��������
	}	

	pill_depot_task();	
	disability_gimbal_motor(task_flag.mode);   //��̨����
	motor_current_send_gimbal();
	
}

/*��̨����ģʽ
	�˲������������
*/
void gimbal_key_control(void)
{
	/*����˲�*/
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


/*��̨pid�л�*/
void PID_CIMBAL_CHOICE(void)
{	
	if(task_flag.auto_mode !=0) //���鿪��
	{
		//�л������״ν������pid
		if(varible_gimbal.pid_choice_flag[0] == 0)
		{
			pid_clear(&pid_yaw);
			pid_clear(&pid_pitch);
			/******************************************************/
			
			varible_gimbal.pid_choice_flag[0] = 1;
			varible_gimbal.pid_choice_flag[1] = 0;
		}
		pid_change(&pid_yaw,22,0.015,0                 //�Ƕ�
									,24,0.1,1);	  //�ٶ�
		pid_change(&pid_pitch,19,0,1               //�Ƕ�
									,16,0.02,0);	  //�ٶ�1
	}
	else if(task_flag.auto_mode ==0)
	{
		//�л������״ν������pid
		if(varible_gimbal.pid_choice_flag[1] == 0)
		{
			pid_clear(&pid_yaw);
			pid_clear(&pid_pitch);
			/******************************************************/
			
			varible_gimbal.pid_choice_flag[0] = 0;
			varible_gimbal.pid_choice_flag[1] = 1;
		}
		pid_change(&pid_yaw,22,0.015,0                 //�Ƕ�
									,24,0.1,1);	  //�ٶ�
		pid_change(&pid_pitch,19,0,1               //�Ƕ�
									,16,0.02,0);	  //�ٶ�1
	}
	
}


/*yaw��������*/
void YAW_MOTOR(uint8_t flag)
{
	yaw_turn_round(task_flag.zig);
	if(varible_gimbal.yaw_imu_angle_flag == 0)
	{
		varible_gimbal.set_yaw_angle = imu.yaw_total;   //�����ǵ�yaw���ܽ� yaw_total;
		varible_gimbal.yaw_imu_angle_flag = 1;
	}
	if(flag == 0)                                                                 //��Щ - ���� ԭ��   
		varible_gimbal.set_yaw_angle -= RC16_CHN.ch1 * varible_gimbal.RC_x_db + RC_DR16.mouse_x *  varible_gimbal.mouse_x_db + varible_gimbal.yaw_turn_round_step;
	else if(flag == 1)
		varible_gimbal.set_yaw_angle -= RC16_CHN.ch1 * varible_gimbal.vis_RC_x_db + RC_DR16.mouse_x * varible_gimbal.vis_mouse_x_db  +varible_vision.vision_yaw_out *varible_gimbal.vis_x_db;
	
	/*PID����*/
	varible_gimbal.set_yaw_speed = pid_yaw.f_calculate(&pid_yaw,0,imu.yaw_total,varible_gimbal.set_yaw_angle); //�����ǵ�yaw���ܽ� yaw_total;
	pid_yaw.f_calculate(&pid_yaw,1,imu.yaw_speed,varible_gimbal.set_yaw_speed);             //             ->>>

}

//pitch��������
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
	/*pid����->����*/
	varible_gimbal.set_pitch_speed = pid_pitch.f_calculate(&pid_pitch,0,imu.pitch,varible_gimbal.set_pitch_angle);
		pid_pitch.f_calculate(&pid_pitch,1,imu.pitch_speed,varible_gimbal.set_pitch_speed);

}



/*yaw��һ����ͷ*/
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
		//��¼����ʱ��
		if(varible_gimbal.yaw_turn_round_flag == 0)
		{
			varible_gimbal.yaw_turn_round_time = HAL_GetTick();
			varible_gimbal.yaw_turn_round_flag = 1;
		}
		//�ڴ�����һ��ʱ���ڵ�ͷ����ֵ��ֵ  500ms�Ϳ���תһȦ��������
		if(HAL_GetTick () - varible_gimbal.yaw_turn_round_time <= 530)
		{
//			if(varible_chassis.top_direction == -1)
				varible_gimbal.yaw_turn_round_step = -8.7f;
//			else varible_gimbal.yaw_turn_round_step = -8.7f;
		}
		//����һ��ʱ��󲽽�ֵΪ0���ָ���־λ
		else
		{
			varible_gimbal.yaw_turn_round_step = 0;
			varible_gimbal.yaw_turn_round_time = 0;
			varible_gimbal.yaw_turn_round_flag = 0;
			task_flag.zig = 0;
		
		}
	}	
}


/*��̨����*/
void disability_gimbal_motor(uint8_t mode)
{
	if(mode == 0)
	{
		pid_clear(&pid_pitch);
		pid_clear(&pid_yaw);
		pid_clear(&pid_send);/*�����̵��*/
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
		/*��־λ�ָ�*/
		varible_gimbal.pitch_imu_angle_flag = 0;
		varible_gimbal.yaw_imu_angle_flag = 0;
		varible_send.send_flag = 0;				//��������ת
		
	}
	else
		varible_gimbal.shoot_stop_time = 200;
}









