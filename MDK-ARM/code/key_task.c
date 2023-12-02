#include "key_task.h"

//����rgb ���̺� PH10  PH11  PH12

#define mode1  10 
#define mode2 01
#define mode3 00


RC16_chn RC16_CHN;        //ң��ͨ����ֵ
task_flag_t task_flag;   //�����־λ


uint32_t rc_close_send_time;			//ң�����رշ���ʱ��
uint8_t rc_send_init_flag;				//ң���������־


/*ҡ��ͨ���жϸ�ֵ*/
void access_assiqnment(void)
{
	if(RC_DR16.ch1 >= -660 && RC_DR16.ch1 <= 660&&
			RC_DR16.ch2 >= -660 && RC_DR16.ch2 <= 660&&
			RC_DR16.ch3 >= -660 && RC_DR16.ch3 <= 660&&
			RC_DR16.ch4 >= -660 && RC_DR16.ch4 <= 660&&
			RC_DR16.ch5 >= -660 && RC_DR16.ch5 <= 660)
	{
		RC16_CHN.ch1 = RC_DR16.ch1;
		RC16_CHN.ch2 = RC_DR16.ch2;
		RC16_CHN.ch3 = RC_DR16.ch3;
		RC16_CHN.ch4 = RC_DR16.ch4;
		RC16_CHN.ch5 = RC_DR16.ch5;
	}
	else  {}//ң��ͨ�����ݴ���
}


/*ң�ؿ�������*/
void key_task(void)
{
	Buzzer_TASK();				//���߼��
	temperature_protect();		//�������
	access_assiqnment();       //����ͨ����ֵ
	if (monitor.rc_monitor == true)
		mode_task();
	else
	{
		task_flag.mode = 0;      //����
		mode3;   /*������ʾ��ȫ��*/
		RC16_CHN.ch1 = 0;
		RC16_CHN.ch2 = 0;
		RC16_CHN.ch3 = 0;
		RC16_CHN.ch4 = 0;
		RC16_CHN.ch5 = 0;
	}

}

void mode_task(void)
{
	if(RC_DR16.switch_left == 2 ||task_flag.die_or == 0)
	{
		task_flag.mode = 0;  //����
		pill_open();
		rc_send_init_flag = 0;
		KEY_SYS_RESET();    //�����λ
		mode1;      //���ص�
	}
	else if(RC_DR16.switch_left != 2 && RC_DR16.switch_left != 0)
	{
		normal_1();
		task_flag.mode = 1;
		mode2;
	}
}
void pill_open(void)
{
	if( task_flag.mode == 0 && RC_DR16.switch_right == 1 ) //����)
	{
		task_flag.pill_depot = 1;
	}
	if(task_flag.mode == 0 && RC_DR16.switch_right == 2)
	{
		task_flag.pill_depot = 0;
	}

}

int asdw = 0;
/*����ģʽ*/
void normal_1(void)
{
	/*����<--->����Ҽ�*/
	if(RC_DR16.mouse.press_right == 1 ||RC_DR16.ch5 < -200 ||POWER_LIMIT == 70)            
		task_flag.automatic_aiming = 1;
	else
		task_flag.automatic_aiming = 0;
	
	//���״̬
	if(RC_DR16.keyBoard.key_code == KEY_F  )
		task_flag.hit_mode = 0;		//���
	else if(RC_DR16.keyBoard.key_code == KEY_CTRL_F)
		task_flag.hit_mode = 1;	
	
	if(task_flag.automatic_aiming == 1 && task_flag.hit_mode == 0)
		task_flag.auto_mode = 1;
	else if(task_flag.automatic_aiming == 1 && task_flag.hit_mode == 1)
		task_flag.auto_mode = 2;
	else 
		task_flag.auto_mode = 0;
		
	
//	if(RC_DR16.keyBoard.key_code == KEY_Z)
//		task_flag.hit_mode = 0;						//����
	
	/*����Ħ����<----->KEY_G*/
	if(RC_DR16.keyBoard.key_code & KEY_G)
	{
		task_flag.frictiongear = 1;				//����
	}
	if(RC_DR16.keyBoard.key_code == KEY_CTRL_G)
	{
		task_flag.frictiongear = 0;				//�ر�
		task_flag.frictiongear_ramp = 200;  
	}
	//Ħ����δ������
	if(RC_DR16.mouse.press_left == 1 && task_flag.frictiongear == 0)
	{
		task_flag.frictiongear = 1;				//����		
		task_flag.frictiongear_ramp = 200;    //Ħ���ֿ����ȴ�б��  �ӳ�
	}
	
	/*����Ԥ��<-------->KEY_B*/
	if(key_repetition_2(&key_repetition_B,RC_DR16.keyBoard.key_code,KEY_B) == 1)       /*�����������δ���-->�е㶫��*/
		task_flag.aim_predict = 1;					//ģʽ2
	else if(key_repetition_2(&key_repetition_B,RC_DR16.keyBoard.key_code,KEY_B) == 0)
		task_flag.aim_predict = 0;					//ģʽ2
	
	if(vision_rx.date_update == true && task_flag.auto_mode == 1 && varible_vision.reverse_top == 1 && varible_vision.send_or == 1)
		task_flag.vision_send = 1; //�Ӿ�ʶ�𵽲�������
	else 
		task_flag.vision_send = 0; 
	
	
	
	//����
	if(task_flag.frictiongear == 1)
	{
		if(task_flag.frictiongear_ramp > 0) 
			task_flag.frictiongear_ramp--;  //�����ʲô˵��
		else
		{
			/*��������<------->������*/
			if(RC_DR16.mouse.press_left == 1 && task_flag.vision_send == 0)
				task_flag.load_mo = 1;		
			else
				task_flag.load_mo = 0;					
		}			
	}
	//ң�ط��䣬��-��-�� 1-3-2
	if(RC_DR16.switch_right ==1)
	{
		if(HAL_GetTick() - rc_close_send_time < 1000)
			task_flag.frictiongear = 0;				//�ر�
		rc_send_init_flag = 1;
	}
	else if(RC_DR16.switch_right == 3 && rc_send_init_flag == 1)
	{
		rc_close_send_time = HAL_GetTick();
		task_flag.frictiongear = 1;				//����
		task_flag.load = 0;				//�ر�
	}
	else if(RC_DR16.switch_right == 2 && rc_send_init_flag == 1)
	{
		task_flag.load = 1;				//����
	}
	

	/*С����<------>KEY_SHIFT*/
	if(RC_DR16.keyBoard.key_code & KEY_SHIFT||RC_DR16.switch_left == 1)
		task_flag.small_gyro = 1;				//����
	else
		task_flag.small_gyro = 0;				//�ر�
	
	/*���տ���<----->KEY_CTRL_R*/
	if(RC_DR16.keyBoard.key_code == KEY_CTRL_R)
		task_flag.pill_depot = 1;				//����
	else if(RC_DR16.keyBoard.key_code & KEY_R)
		task_flag.pill_depot = 0;				//�ر�
	
	/*��������*///1ȫ�Զ�2ֻ��粻�ŵ�3����Ҳ����
	if((RC_DR16.keyBoard.key_code & KEY_V || RC_DR16.ch5 > 200) && super_cap.cap_capacity > 3 && task_flag.chassis_climb == 0)//-----------------------------------ģʽ-------------
	{
		if(task_flag.cap_cnt == 0)		task_flag.cap_mode = 1;
		else							task_flag.cap_cnt--;
	}
	else if((RC_DR16.keyBoard.key_code & KEY_C ) && super_cap.cap_capacity > 3 && task_flag.chassis_climb == 0)
	{
		if(task_flag.cap_cnt == 0)		task_flag.cap_mode = 2;
		else 							task_flag.cap_cnt--;
	}
	else if(task_flag.chassis_climb == 0)
	{
//		if( task_flag.small_gyro == 0 && varible_chassis.gyro_y == 0&&varible_chassis.gyro_x == 0)
//	
//		{
//			task_flag.cap_mode = 0;
//		}
		 	task_flag.cap_mode = 0;
		if(super_cap.cap_capacity >= 5) 	task_flag.cap_cnt = 0;
		else 								task_flag.cap_cnt = 200;
	}
	
	
	
	//����ģʽ                                     //û�м�λ
	if(RC_DR16.keyBoard.key_code & KEY_W && RC_DR16.keyBoard.key_code & KEY_Q && super_cap.cap_capacity > 8)
		task_flag.chassis_climb = 1;
	else 
		task_flag.chassis_climb = 0;
	
	
	/*ԭ�ص�ͷ<----->X      ������ʹ��*/
	if(RC_DR16.keyBoard.key_code & KEY_X && task_flag.zig == 0)			//180
		task_flag.zig = 1;				//����
	
}
 

uint32_t sys_reset_time;
uint8_t sys_reset_time_flag;

/*�����λ*/
void KEY_SYS_RESET(void)
{
	if(RC_DR16.keyBoard.key_code == KEY_CTRL_B || RC16_CHN.ch5 >656)  //ѡ�񰴼�------>
		task_flag.sys_reset = 1;

	if(task_flag.sys_reset == 1 && sys_reset_time_flag == 0)
	{
		sys_reset_time = HAL_GetTick ();
		sys_reset_time_flag = 1;                        //��λ֮���ʼ����0<--->ɵ������  m
	}
	if(HAL_GetTick () - sys_reset_time > 0 && HAL_GetTick () - sys_reset_time <= 1000 && task_flag.sys_reset == 1)
		task_flag.mode = 0;						//����
	else if(HAL_GetTick () - sys_reset_time > 1000 && HAL_GetTick () - sys_reset_time <= 1200 && task_flag.sys_reset == 1)
	{
		task_flag.sys_reset = 0;
		HAL_NVIC_SystemReset();  //�����λ
	}
	
}



























