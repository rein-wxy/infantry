#include "send.h"

varible_send_t varible_send;    //��������洢


void send_Init(void)
{
	/*������2006���*/   //0.31
	pid_init(&pid_send,	 0, 5000, 16384, 368640,  0, 0, 0.4f, 0.0, 0
						,0,	5000, 16384, 0xFFFFFF,0, 0, 20, 0.003f, 0.01);
//	/*������3508���*/
//	pid_init(&pid_send,	 0, 5000, 16384, 194560,  0, 0, 0.38f, 0,7.6f
//						,0,	5000, 16384, 0xFFFFFF,0, 0, 15.0f, 0,0);
	/*Ħ���ֵ��*/
	for(int i = 0;i < 2;i++)
	{
		pid_init(&pid_shoot[i],0,0,0,0,0,0,0,0,0
							,0,2000,16384,0xffffff, 0, 0, 10.0f, 0.0f, 0);
	}
	
	/*Ħ���ֵ��*/
	//HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	/*���ն��PWMʹ��*/
//	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);//ռ�ձ�0--2000
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
}

void send_task(void)
{
	heat_limit_task();	        //������Ƶ����
	pill_depot_task();          //����
	shoot_task();               //Ħ����
	locked_rotor_tesk();
	if(varible_send.lock_pill == 0&& allow_send_bullet == 1 && varible_send.fric_send_flag == 1 )   //������   ����--����--Ħ����
		stir_task();	
	else
		pid_send.pid_out_speed = 0;
	if(varible_send.lock_pill == 1)/*��ת����*/
		locked_task();  		
}

/*Ħ��������*/
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
			if(varible_send.send_data_update == 1)//���ٸ���
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
			if(varible_send.send_data_update == 1)//���ٸ���
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
			if(varible_send.send_data_update == 1)//���ٸ���
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
	if(task_flag.frictiongear == 1)        //����Ħ����
	{
		varible_send.set_shoot_speed[0] = varible_send.speed_speed;//�Ϲ�
		varible_send.set_shoot_speed[1] = -varible_send.speed_speed;
		
//		varible_send.set_shoot_speed[0] = 3500;
//		varible_send.set_shoot_speed[1] = -3500;
	
		/*ȷ��Ħ����ת������----->Ϊʲô��||��*/
		if(shoot_RX[0].speed > 1000 ||shoot_RX[1].speed < -1000)
		{
			varible_send.fric_send_flag = 1;			//Ħ������ת��
		}
		else 
			varible_send.fric_send_flag = 0;
	}
	else if(task_flag.frictiongear == 0)    //�ر�Ħ����
	{
		varible_send.set_shoot_speed[0] = 0;
		varible_send.set_shoot_speed[1] = 0;
		varible_send.fric_send_flag = 0;
		/*�����ָ�*/
		varible_send.lock_pill= 0;			//������־λ����
		pid_send.err_angle[0] = 0;			//������
		pid_send.err_angle[1] = 0;			//������
		varible_send.set_send_angle = send_RX.total_angle;    //��¼��ǰֵΪĿ��ֵ
	}
	for(int i = 0;i < 2;i++)	
	{
		pid_shoot[i].f_calculate(&pid_shoot[i],1,shoot_RX[i].speed,varible_send.set_shoot_speed[i]);
	}
}

/*����ģʽ  ����/����*/
/*����������*/


void stir_task(void)
{
	if(varible_send.send_flag == 0)   //��ֹ��������ת
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
			varible_send.set_send_angle -= 36864.0f;   // \�����̳���
			varible_send.send_mode_flag = 1;
			task_flag.pill_depot = 0;
			varible_send.send_mode_time = HAL_GetTick();
		}
		else if(varible_send.send_mode_flag == 1)
		{

			if(HAL_GetTick()-varible_send.send_mode_time > 250)
			{
				
				varible_send.set_send_angle -=36864.0f;//36864.0f;// ////�����̳���
				varible_send.send_mode_flag = 2;
				varible_send.send_mode_time = HAL_GetTick();
			} 
		}
		else if(varible_send.send_mode_flag == 2)
		{
			if(HAL_GetTick()-varible_send.send_mode_time >varible_send.send_freq)
			{
			
				varible_send.set_send_angle -= 36864.0f;//;//36864.0f;   //תһ�񣨲����̳�����
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
			varible_send.set_send_angle -= 36864.0f;   // \�����̳���
			varible_send.send_mode_flag = 1;
			task_flag.pill_depot = 0;
			varible_send.send_mode_time = HAL_GetTick();
		}
		else if(varible_send.send_mode_flag == 1)
		{
			if(HAL_GetTick()-varible_send.send_mode_time > 250)
			{			
				varible_send.set_send_angle -=36864.0f;//36864.0f;// ////�����̳���
				varible_send.send_mode_flag = 2;
				varible_send.send_mode_time = HAL_GetTick();
			} 
		}
		else if(varible_send.send_mode_flag == 2)
		{
			if(HAL_GetTick()-varible_send.send_mode_time >varible_send.send_freq)
			{		
				varible_send.set_send_angle -= 36864.0f;//;//36864.0f;   //תһ�񣨲����̳�����
				varible_send.send_mode_time = HAL_GetTick();
			}
		}	
	}
}

/*��ת���*/
void locked_rotor_tesk(void)
{
	//if(pid_send.err_angle[0] > 184320 || pid_send.err_angle[0] < -184320)    //----->
	if(pid_send.err_angle[0] > 100000 || pid_send.err_angle[0] < -100000)    	
	varible_send.lock_pill = 1;     //��
	else
		varible_send.lock_pill = 0;
}

/*��ת����*/
void locked_task(void)
{
	if(varible_send.lock_pid_clear_flag[0] == 0)
	{
		pid_clear(&pid_send);
		varible_send.lock_pid_clear_flag[0] = 1;
		varible_send.lock_pid_clear_flag[1] = 0;
	}
	if(varible_send.lock_flag <=150)//��תʱ��
	{
		pid_send.f_calculate(&pid_send,1,send_RX.speed,1500);  //��ת�ٶ�------>
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
		varible_send.lock_flag = 0;  //��ת����ʱ������
		varible_send.lock_pill = 0;  //��ת�������	
		varible_send.send_flag = 0;  //��ֹ��ת��־�ָ�
	}
}

/*
	speed:Ԥ��Ŀ��ֵ
	limit:�����ٶ�15 18 30
	����ϵ��ӳ�䵽tan��������֮��֮��
*/

float speed_offest(float speed,float limit)
{
	float a = (speed - limit),b;

	if(a<=0.5 && a >=-0.5) 	 
		b =tan( a*1.571);	//pai/2
	else if	(a>=0.5 )		 
		b = tan(0.785);			//����x��ֹ���ٱ仯��
	else if	(a<=-0.5 ) 		 
		b = tan(-0.785) ;	
	return b* 10;
}



/*����*/

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
