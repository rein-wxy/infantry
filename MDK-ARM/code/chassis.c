#include "chassis.h"

/*���̳�ʼ��*/
void chassis_Init(void)
{
	for(int i=0;i<4;i++)
	{
		pid_init(&pid_Mchassis[i],
								0,  0, 0, 0, 0, 0, 0, 0, 0, //�⻷λ�û�kp,ki,kd
								0,	1000,	16384,	0xFFFFFF, 0, 0, 7.6f, 	0, 	0.01);//�ڻ��ٶȻ�kp,ki,kd
	
	}
	/*���̸����ʼ��*/
	pid_init(&pid_chassis_follow,
								0,	0,	0, 0,	0,0, 0, 0,0, //�⻷λ�û�kp,ki,kd
								0,	150,	4000,		0xFFFFFF,   25,        0, 4.0f, 0, 0);//�ڻ��ٶȻ�kp,ki,kd
}	
//����  pid_out

void chassis_task(void)
{
	chassis_assist_task();//��������
	super_cap_task();/*��������*/
	if(task_flag.mode == 1)
	{
		if(task_flag.small_gyro == 0 )	
			chassis_follow();       //���̸���
		if(task_flag.small_gyro == 1)
			chassis_top();          //С����
		for(int i = 0;i < 4;i++)
		{
			pid_Mchassis[i].f_calculate(&pid_Mchassis[i],1,chassis_motor_RX[i].speed,varible_chassis.set_chassis_speed[i]);
		}
		chassis_power_limit();      //��������
	}
	chassis_motor_diversion(task_flag.mode);   //���̵������
	Motor_current_send_chassis();
}



/*���̸���*/
void chassis_follow(void)
{	
	/*����С����,��־λ�ָ�*/
	Slow_clear(&top_ramp);
	varible_chassis.top_direction_flag = 0;
	if(varible_chassis.wait_follow_flag <=150)
	{
		/****************�ȴ���Լ150ms����ͷ*************************************/
		varible_chassis.get_degree_1 = 0;
		varible_chassis.get_degree_2 = 0;
		varible_chassis.wait_follow_flag ++;
	}
	chassis_follow_degree();	//����Ƕ�
	chassis_caculate(varible_chassis.gyro_x,varible_chassis.gyro_y,pid_chassis_follow.pid_out_speed);//pid��������ͷ->pid_chassis_follow.pid_out_speed
}



/*С����*/
void chassis_top(void)
{
	if(varible_chassis.top_direction_flag == 0)
	{
		Top_direction();
		varible_chassis.top_direction_flag = 1;
	}
	//Top_change();
	/*С����ʱ�ɿ����ƶ�->���ƹ���*/
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
	varible_chassis.wait_follow_flag  = 1;   //�򿪵ȴ���ͷ��־λ
}



/*
	���ж��
	�����ת��
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












