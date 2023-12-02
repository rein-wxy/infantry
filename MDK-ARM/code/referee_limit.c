#include "referee_limit.h"

super_cap_t super_cap;


/*����*/
uint8_t robot_ID;                   //������ID
uint8_t robot_level;				//�����˵ȼ�
float chassis_power;                //����ʵʱ����
float chassis_power_buffer;         //����ʵʱ��������
float WARNING_POWER_BUFF;
float power_scale;                  //����
uint16_t POWER_LIMIT  = 40;          //������ƹ���

uint16_t chassis_max_speed;
uint16_t chassis_top_speed;

/*���� */
uint8_t allow_send_bullet;		    //�Ƿ�������
uint16_t now_bullet_heat;			//��ǰǹ������
float now_bullet_speed;				//��ǰǹ������
float last_bullet_speed;			//�ϴ�ǹ������
uint16_t heat_limit;					//��������
uint16_t speed_limit;					//��������
uint16_t now_bullet_cool;  //ǹ����ȴֵ
super_cap_t super_cap;
uint8_t vis_speed;
int heat_limit_time;                 //��������ʱ��
/*������������*/
void heat_limit_task(void)
{
	send_referee_updata();	
	
	//��������
	if(now_bullet_heat <= heat_limit * 0.80f)
	{
		if(HAL_GetTick() - heat_limit_time > 300)     //��������ϵͳ���  ��ֹ����������
			allow_send_bullet = 1;             //������
	}
	else
	{	
		allow_send_bullet = 0;
		heat_limit_time = HAL_GetTick();
	}
	/*�°���������*/
	//HeatUpdate();
	
	
}

void send_referee_updata(void)
{
	last_bullet_speed = now_bullet_speed;     //���ٸ�ֵ
	heat_limit = Referee_system_data.extgame_robot_status_t.shooter_id1_17mm_cooling_limit;			//����ϵͳ��������
	//speed_limit = Referee_system_data.extgame_robot_status_t.shooter_id1_17mm_speed_limit;			//����ϵͳ��ȡ��������
	now_bullet_heat = Referee_system_data.extpower_heat_data_t.shooter_id1_17mm_cooling_heat;		//����ϵͳ��ȡ��ǰ����
	now_bullet_speed = Referee_system_data.extshoot_data_t.bullet_speed;							//����ϵͳ��ȡ��ǰ����
	now_bullet_cool = Referee_system_data.extgame_robot_status_t.shooter_id1_17mm_cooling_rate;   //add-��ȴֵ
	
	switch ( Referee_system_data.extgame_robot_status_t.shooter_id1_17mm_speed_limit)
    {
    	case 18:	speed_limit = 18;vis_speed = 2;
    		break;
    	case 30:	speed_limit = 30;vis_speed = 1;
    		break;
    	default:	speed_limit = 15;vis_speed = 3;
    		break;
    }
	
	
	//��������-�����մ���
	if(now_bullet_speed - last_bullet_speed != 0)
	{
		varible_send.send_data_update = 1;         //Ħ������
		varible_send.send_data_limit = 1;			//��������
	}
	

	//��ͬ��ȴ��Ƶ  --------->ÿ����Ƶ
	if(Referee_system_data.extgame_robot_status_t.shooter_id1_17mm_cooling_rate >=60)
		varible_send.send_freq = 67;
	else if(Referee_system_data.extgame_robot_status_t.shooter_id1_17mm_cooling_rate >= 30)
		varible_send.send_freq = 77;
	else if(Referee_system_data.extgame_robot_status_t.shooter_id1_17mm_cooling_rate >= 15)
		varible_send.send_freq = 100;
	else 
		varible_send.send_freq = 100;
}



/*���̹�������*/
void chassis_power_limit(void)
{
	parameter_number();
	/*����pid���-------->û��������*/
	//total_current = (float)( fabs(pid_Mchassis[0].pid_out_speed) + fabs(pid_Mchassis[1].pid_out_speed) + fabs(pid_Mchassis[2].pid_out_speed) + fabs(pid_Mchassis[3].pid_out_speed) );

	if(robot_ID == 0) 
		power_scale = 1; //ID������	
	else
	{
		if(chassis_power_buffer < 0) 
			power_scale = 0;		
		else if(chassis_power_buffer < WARNING_POWER_BUFF)		//����ʵʱ�������� �� Σ�ջ��幦�ʣ����ʳ���������ƹ���
		{
			power_scale = chassis_power_buffer/WARNING_POWER_BUFF;
			if(task_flag.cap_mode == 0)				
				power_scale = power_scale*power_scale;
			else								
				power_scale = power_scale*power_scale*power_scale*power_scale;
		}
		else	
			power_scale = 1;	//����δ���������
	}
	/*������������ܵ���*/
	for(int i=0;i<4;i++)
	{
		varible_chassis.send_chassis_speed[i] = power_scale * pid_Mchassis[i].pid_out_speed; 
	}
	
}






/*�����ٶȸ�ֵ*/
void parameter_number(void)
{
/*��ʼ״̬ 40
		�������� 60---80---100
		Ѫ������ 45---50---55
		ƽ�ⲽ�� 60---80---100 */
	switch (POWER_LIMIT)
	{
		case 40:
			if(task_flag.cap_mode == 1)
			{
				chassis_top_speed = 5700;														//С�����ٶ�
				chassis_max_speed = 5600;														//�����ٶ�
				WARNING_POWER_BUFF = 40.0f;													//Σ�ջ��幦��
			}
			else if(task_flag.cap_mode == 2)
			{
				chassis_top_speed = 7300;														//С�����ٶ�
				chassis_max_speed = 8500;														//�����ٶ�
				WARNING_POWER_BUFF = 40.0f;													//Σ�ջ��幦��
			}
			else
			{
				chassis_top_speed = 3000;														//С�����ٶ�
				chassis_max_speed = 2650;														//�����ٶ�
				WARNING_POWER_BUFF = 45.0f;													//Σ�ջ��幦��
			}
			break;
		
		case 45:
			if(task_flag.cap_mode == 1)
			{
				chassis_top_speed = 5800;														//С�����ٶ�
				chassis_max_speed = 5500;														//�����ٶ�
				WARNING_POWER_BUFF = 40.0f;													//Σ�ջ��幦��
			}
			else if(task_flag.cap_mode == 2)
			{
				chassis_top_speed = 7300;														//С�����ٶ�
				chassis_max_speed = 8500;														//�����ٶ�
				WARNING_POWER_BUFF = 40.0f;													//Σ�ջ��幦��
			}
			else
			{
				chassis_top_speed = 3000;														//С�����ٶ�
				chassis_max_speed = 2850;														//�����ٶ�
				WARNING_POWER_BUFF = 45.0f;													//Σ�ջ��幦��
			}
			break;
		case 50:
			if(task_flag.cap_mode == 1)
			{
				chassis_top_speed = 6000;														//С�����ٶ�
				chassis_max_speed = 5600;														//�����ٶ�
				WARNING_POWER_BUFF = 40.0f;													//Σ�ջ��幦��
			}
			else if(task_flag.cap_mode == 2)
			{
				chassis_top_speed = 7300;														//С�����ٶ�
				chassis_max_speed = 8500;														//�����ٶ�
				WARNING_POWER_BUFF = 40.0f;													//Σ�ջ��幦��
			}
			else
			{
				chassis_top_speed = 3200;														//С�����ٶ�
				chassis_max_speed = 3050;														//�����ٶ�
				WARNING_POWER_BUFF = 45.0f;													//Σ�ջ��幦��
			}
			break;
		case 55:
			if(task_flag.cap_mode == 1)
			{
				chassis_top_speed = 6100;														//С�����ٶ�
				chassis_max_speed = 5700;														//�����ٶ�
				WARNING_POWER_BUFF = 40.0f;													//Σ�ջ��幦��.
			}
			else if(task_flag.cap_mode == 2)
			{
				chassis_top_speed = 7300;														//С�����ٶ�
				chassis_max_speed = 8600;														//�����ٶ�
				WARNING_POWER_BUFF = 40.0f;													//Σ�ջ��幦��
			}
			else
			{
				chassis_top_speed = 3400;														//С�����ٶ�
				chassis_max_speed = 3250;														//�����ٶ�
				WARNING_POWER_BUFF = 45.0f;													//Σ�ջ��幦��
			}
			break;
		case 60:
			if(task_flag.cap_mode == 1)
			{
				chassis_top_speed = 6200;														//С�����ٶ�
				chassis_max_speed = 6000;														//�����ٶ�
				WARNING_POWER_BUFF = 40.0f;													//Σ�ջ��幦��
			}
			else if(task_flag.cap_mode == 2)
			{
				chassis_top_speed = 8000;														//С�����ٶ�
				chassis_max_speed = 8500;														//�����ٶ�
				WARNING_POWER_BUFF = 40.0f;													//Σ�ջ��幦��
			}
			else
			{
				chassis_top_speed = 3700;														//С�����ٶ�
				chassis_max_speed = 3500;														//�����ٶ�
				WARNING_POWER_BUFF = 45.0f;													//Σ�ջ��幦��
			}
			break;
		case 80:
			if(task_flag.cap_mode == 1)
			{
				chassis_top_speed = 6300;														//С�����ٶ�
				chassis_max_speed = 6100;														//�����ٶ�
				WARNING_POWER_BUFF = 40.0f;													//Σ�ջ��幦��
			}
			else if(task_flag.cap_mode == 2)
			{
				chassis_top_speed = 7900;														//С�����ٶ�
				chassis_max_speed = 8500;														//�����ٶ�
				WARNING_POWER_BUFF = 40.0f;													//Σ�ջ��幦��
			}
			else
			{
				chassis_top_speed = 5200;														//С�����ٶ�
				chassis_max_speed = 4500;														//�����ٶ�
				WARNING_POWER_BUFF = 45.0f;													//Σ�ջ��幦��
			}
			break;
		case 100:
			if(task_flag.cap_mode == 1)
			{
				chassis_top_speed = 7500;														//С�����ٶ�
				chassis_max_speed = 6500;														//�����ٶ�
				WARNING_POWER_BUFF = 40.0f;													//Σ�ջ��幦��
			}
			else if(task_flag.cap_mode == 2)
			{
				chassis_top_speed = 9000;														//С�����ٶ�
				chassis_max_speed = 9000;														//�����ٶ�
				WARNING_POWER_BUFF = 40.0f;													//Σ�ջ��幦��
			}
			else
			{
				chassis_top_speed = 6700;														//С�����ٶ�
				chassis_max_speed = 5900;														//�����ٶ�
				WARNING_POWER_BUFF = 45.0f;													//Σ�ջ��幦��
			}
			break;
		default:
			if(task_flag.cap_mode == 1)
			{
				chassis_top_speed = 6000;														//С�����ٶ�
				chassis_max_speed = 5500;														//�����ٶ�
				WARNING_POWER_BUFF = 40.0f;													//Σ�ջ��幦��
			}
			else if(task_flag.cap_mode == 2)
			{
				chassis_top_speed = 7900;														//С�����ٶ�
				chassis_max_speed = 8100;														//�����ٶ�
				WARNING_POWER_BUFF = 40.0f;													//Σ�ջ��幦��
			}
			else
			{
				chassis_top_speed = 4800;														//С�����ٶ�
				chassis_max_speed = 4500;														//�����ٶ�
				WARNING_POWER_BUFF = 45.0f;													//Σ�ջ��幦��
			}
			break;
			
	}
	chassis_climb_mode(task_flag.chassis_climb);
}



/*����ģʽ*/
void chassis_climb_mode(uint8_t flag)
{
	if(flag == 1)
	{
		task_flag.cap_mode = 1;  //������������
		chassis_max_speed = 3600;   //%%%%%
	}
}

/*��ȡʵʱ���ʣ�ʵʱ��������*/
void get_chassis_power_buffer_t(void)
{
	chassis_power = Referee_system_data.extpower_heat_data_t.chassis_power;
	chassis_power_buffer = Referee_system_data.extpower_heat_data_t.chassis_power_buffer;		
}

/*��ȡ������ID��������ƹ��ʣ��ȼ�*/
void get_chassis_id_maxpower_level(void)
{
	robot_ID = Referee_system_data.extgame_robot_status_t.robot_id;
	POWER_LIMIT = Referee_system_data.extgame_robot_status_t.chassis_power_limit;       //�����˵��̹�����������
	robot_level = Referee_system_data.extgame_robot_status_t.robot_level;	
}


/*������������*/
void super_cap_task(void)
{
	/*��ȡʵʱ���ʣ�ʵʱ��������*/
	get_chassis_power_buffer_t();
	/*��ȡ������ID��������ƹ��ʣ��ȼ�*/
	get_chassis_id_maxpower_level();
	/*���ݷ���*/
	SET_POWER_CURRENT_CAN(&hcan1);
	
	die_alive();
}

/*�жϻ�����״̬*/
void die_alive(void)
{
	if(Referee_system_data.extgame_robot_status_t.remain_HP <= 0)	
		task_flag.die_or =0;
	else task_flag.die_or = 1;
}


/**********************************************************************************************************
*�� �� ��: HeatControl
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/


uint16_t HeatMax17 ,HeatCool17; //��������-10,��ȴֵ/10
const short BulletHeat17 = 10;
short CurHeat17, LastHeat17, AvailableHeat17; //��ǰ������ ��һ������, ���м�������

uint16_t Shooted17Cnt;	//һ�������Ѵ���ӵ���
uint16_t AvailableBullet17;	//��һ�����������

void HeatControl(void)
{
	if(varible_send.heart_data_update == 1)
	{
		Shooted17Cnt = 0;
		AvailableHeat17 =  constrain(HeatMax17 - now_bullet_heat + now_bullet_cool,heat_limit,0);    //�����Ҫ�����о�
		if(varible_send.send_data_limit == 1)  //��⵽����--���ٸ���
		{
			AvailableHeat17 =  constrain(AvailableHeat17 - BulletHeat17,heat_limit,0);
			varible_send.send_data_limit = 0;
		}
		AvailableBullet17 = AvailableHeat17 / BulletHeat17;
		allow_send_bullet = (AvailableBullet17 < 1)?0:1;		//��������������
	}
	else if((varible_send.send_data_limit == 1) && (varible_send.heart_data_update == 0))//��ⷢ��������δ����
	{
		Shooted17Cnt++;
		allow_send_bullet = (Shooted17Cnt >= AvailableBullet17)?0:1;		
		varible_send.send_data_limit = 0;
	}
}


/**********************************************************************************************************
*�� �� ��: HeatUpdate
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/

const float HeatControlThreshold = 0.85f;   	//�����������Ƶ���ֵ
void HeatUpdate(void)
{
	HeatMax17 =  heat_limit - 20;
	HeatCool17 = now_bullet_cool/10;
	CurHeat17 = now_bullet_heat;  //��ǰǹ������
	
	if(CurHeat17 != LastHeat17 )
	{
		varible_send.heart_data_update = 1;
		varible_send.send_data_limit  = 0; //���������򽫷����־λ����(û�д�����Ĵ�)  ֻ����һ����������
	}
	if(CurHeat17 < HeatControlThreshold*HeatMax17)  //�������Ʋ�����
	{
		allow_send_bullet = 1;
		varible_send.send_data_limit  = 0;
	}
	else
	{
		if((varible_send.send_data_limit  == 1)||(varible_send.heart_data_update == 1))
		 HeatControl();
	}
	varible_send.heart_data_update = 0;
	LastHeat17 = CurHeat17;
}














