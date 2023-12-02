#include "monitor.h"

monitor_t monitor;					//检测
sys_buzzer_t sys_buzzer;
uint16_t my_time;
/*系统监测*/
void Buzzer_TASK(void)
{
	//遥控器
	if(monitor.rc_cnt == monitor.rc_cnt_last)      				monitor.rc++;
	else  														monitor.rc = 0;
	monitor.rc = constrain(monitor.rc,0.0,250);   //限幅
	monitor.rc_cnt = monitor.rc_cnt_last;
	if(monitor.rc > 200) 										monitor.rc_monitor = false;
	else 														monitor.rc_monitor = true;
	
    //nuc
	if(monitor.nuc_cnt == monitor.rc_cnt_last)      			monitor.nuc++;
	else  														monitor.nuc = 0;
	monitor.nuc = constrain(monitor.nuc,0.0,250);   //限幅
	monitor.nuc_cnt = monitor.nuc_cnt_last;
	if(monitor.nuc > 200) 										monitor.nuc_monitor = false;
	else 														monitor.nuc_monitor = true;

	//REFEREE
	if(monitor.referee_cnt == monitor.referee_cnt_last)			monitor.referee ++;
	else 														monitor.referee = 0;
	monitor.referee = constrain(monitor.referee,0.0,250);
	monitor.referee_cnt_last = monitor.referee_cnt;
	if(monitor.referee > 200)									monitor.referee_monitor = false;
	else														monitor.referee_monitor = true;

	//chassis
	for(int i=0;i<4;i++)
	{
		if(monitor.chassis_cnt[i] == monitor.chassis_cnt_last[i])		monitor.chassis[i] ++;
		else 															monitor.chassis[i] = 0;
		monitor.chassis[i] = constrain(monitor.chassis[i],0.0,250);
		monitor.chassis_cnt_last[i] = monitor.chassis_cnt[i];
		if(monitor.chassis[i] > 200)									monitor.chassis_monitor[i] = false;
		else														    monitor.chassis_monitor[i] = true;
	}
	
	//yaw
	if(monitor.yaw_cnt == monitor.yaw_cnt_last)						monitor.yaw ++;
	else 															monitor.yaw = 0;
	monitor.yaw = constrain(monitor.yaw,0.0,250);
	monitor.yaw_cnt_last = monitor.yaw_cnt;
	if(monitor.yaw > 200)											monitor.yaw_monitor = false;
	else															monitor.yaw_monitor = true;


	//pitch
	if(monitor.pitch_cnt == monitor.pitch_cnt_last)					monitor.pitch ++;
	else 															monitor.pitch = 0;
	monitor.pitch = constrain(monitor.pitch,0.0,250);
	monitor.pitch_cnt_last = monitor.pitch_cnt;
	if(monitor.pitch > 200)											monitor.pitch_monitor = false;
	else															monitor.pitch_monitor = true;

	//send拨弹盘
	if(monitor.send_cnt == monitor.send_cnt_last)					monitor.send ++;
	else 															monitor.send = 0;
	monitor.send = constrain(monitor.send,0.0,250);
	monitor.send_cnt_last = monitor.send_cnt;
	if(monitor.send > 200)											monitor.send_monitor = false;
	else															monitor.send_monitor = true;
	
	//摩擦轮
	for(int i = 0;i < 2;i++)
	{
		if(monitor.shoot_cnt[i] == monitor.shoot_cnt_last[i])			monitor.shoot[i] ++;
		else 															monitor.shoot[i] = 0;
		monitor.shoot[i] = constrain(monitor.shoot[i],0.0,250);
		monitor.shoot_cnt_last[i] = monitor.shoot_cnt[i];
		if(monitor.shoot[i] > 200)										monitor.shoot_monitor[i] = false;
		else															monitor.shoot_monitor[i] = true;
	}	
	
	//power电容板
	if(monitor.power_cnt == monitor.power_cnt_last)					monitor.power ++;
	else 															monitor.power = 0;
		monitor.power = constrain(monitor.power,0.0,250);
		monitor.power_cnt_last = monitor.power_cnt;
	if(monitor.power > 240)											monitor.power_monitor = false;
	else															monitor.power_monitor = true;
	
	//1--3--4--5
	//赋值
	if(!monitor.rc_monitor)             //否定布尔变量  离线进入
		monitor.type_err = 2;
	else if(!monitor.chassis_monitor[0] || !monitor.chassis_monitor[1] || !monitor.chassis_monitor[2] || !monitor.chassis_monitor[3] || !monitor.yaw_monitor  
				|| !monitor.pitch_monitor || !monitor.send_monitor || !monitor.shoot_monitor[0]|| !monitor.shoot_monitor[1])
		monitor.type_err =4 ;
	else if(!monitor.referee_monitor)
		monitor.type_err = 5;
	else if(!monitor.power_monitor)
		monitor.type_err = 3;
	else if(!monitor.nuc_monitor)		
		monitor.type_err = 1;
	else
		monitor.type_err = 0;

	

//技术有限还没想到什么好方法结构
if(monitor.type_err != 0)
{
	if(HAL_GetTick() - monitor.sys_buzzer.buzzer_tim > 100)
	{
		monitor.sys_buzzer.buzzer_cnt ++;
		monitor.sys_buzzer.buzzer_tim = HAL_GetTick();
	}
	
	switch (monitor.type_err)
    {
    	case 1: 
			if(monitor.sys_buzzer.buzzer_cnt>=14)
			{
				TIM4->CCR3=80;
				if(monitor.sys_buzzer.buzzer_cnt >=15)
				{
					monitor.sys_buzzer.buzzer_cnt=0;
					TIM4->CCR3=0;
				}
			}
    		break;
    	case 2: 
			if(monitor.sys_buzzer.buzzer_cnt>=10)
			{
				TIM4->CCR3=80;
				if(monitor.sys_buzzer.buzzer_cnt ==11)
				{
					TIM4->CCR3=0;
				}
				if(monitor.sys_buzzer.buzzer_cnt>=13)
				{
					TIM4->CCR3=0;
					monitor.sys_buzzer.buzzer_cnt=0;
				}
			}
    		break;
		case 3:
			if(monitor.sys_buzzer.buzzer_cnt>=10)
			{
				TIM4->CCR3=80;
				if(monitor.sys_buzzer.buzzer_cnt ==11)
				{
					TIM4->CCR3=0;
				}
				if(monitor.sys_buzzer.buzzer_cnt==13)
				{
					TIM4->CCR3=0;
				}
				if(monitor.sys_buzzer.buzzer_cnt>=15)
				{
					TIM4->CCR3=0;
					monitor.sys_buzzer.buzzer_cnt=0;
				}
			}
    		break;
    	case 4:
			if(monitor.sys_buzzer.buzzer_cnt>=10)
			{
				TIM4->CCR3=80;
				if(monitor.sys_buzzer.buzzer_cnt ==11)
				{
					TIM4->CCR3=0;
				}
				if(monitor.sys_buzzer.buzzer_cnt==13)
				{
					TIM4->CCR3=0;
				}
				if(monitor.sys_buzzer.buzzer_cnt==15)
				{
					TIM4->CCR3=0;
				}
				if(monitor.sys_buzzer.buzzer_cnt>=17)
				{
					TIM4->CCR3=0;
					monitor.sys_buzzer.buzzer_cnt=0;
				}
			}
			break;
		case 5: 
			if(monitor.sys_buzzer.buzzer_cnt>=10)
			{
				TIM4->CCR3=80;
				if(monitor.sys_buzzer.buzzer_cnt ==11)
				{
					TIM4->CCR3=0;
				}
				if(monitor.sys_buzzer.buzzer_cnt==13)
				{
					TIM4->CCR3=0;
				}
				if(monitor.sys_buzzer.buzzer_cnt==15)
				{
					TIM4->CCR3=0;
				}
				if(monitor.sys_buzzer.buzzer_cnt==17)
				{
					TIM4->CCR3=0;
				}
				if(monitor.sys_buzzer.buzzer_cnt>=19)
				{
					TIM4->CCR3=0;
					monitor.sys_buzzer.buzzer_cnt=0;
				}
			}
    		break;
    	default: TIM4->CCR3=0;
    		break;
    }
}
else if(monitor.type_err == 0)
	TIM4->CCR3=0;
		
	

	
}
void temperature_protect(void)
{
	if(yaw_RX.temperature > MAX_Temperature)
		monitor.yaw_t ++;
	if(monitor.yaw_t >= MAX_Time)
		monitor.yaw_kgf = 0;
	else
	{
		monitor.yaw_t = 0;
		monitor.yaw_kgf = 1;
	}
	
	
	if(pitch_RX.temperature > MAX_Temperature)
		monitor.pitch_t ++;
	if(monitor.pitch_t >= MAX_Time)
		monitor.pitch_kgf = 0;
	else
	{
		monitor.pitch_t = 0;
		monitor.pitch_kgf = 1;
	}

	
	if(send_RX.temperature > MAX_Temperature)
		monitor.send_t ++;
	if(monitor.send_t >= MAX_Time)
		monitor.send_kgf = 0;
	else
	{
		monitor.send_t = 0;
		monitor.send_kgf = 1;
	}
		
	
	
	for(uint8_t i = 0;i < 4;i++)
	{	
		if(chassis_motor_RX[i].temperature > MAX_Temperature)
		{
			monitor.chassis_t[i] ++;
			if(monitor.chassis_t[i] >= MAX_Time)
				monitor.chassis_kgf[i] = 0;			
		}	
		else
		{
			monitor.chassis_t[i] = 0;
			monitor.chassis_kgf[i] = 1;
		}				
	}
	
	for(uint8_t i = 0;i < 2;i++)
	{	
		if(shoot_RX[i].temperature > MAX_Temperature)
		{
			monitor.shoot_t[i] ++;
			if(monitor.shoot_t[i] >= MAX_Time)
				monitor.shoot_kgf[i] = 0;			
		}	
		else
		{
			monitor.shoot_t[i] = 0;
			monitor.shoot_kgf[i] = 1;
		}				
	}

	
}

