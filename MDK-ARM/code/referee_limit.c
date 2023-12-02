#include "referee_limit.h"

super_cap_t super_cap;


/*功控*/
uint8_t robot_ID;                   //机器人ID
uint8_t robot_level;				//机器人等级
float chassis_power;                //底盘实时功率
float chassis_power_buffer;         //底盘实时缓冲能量
float WARNING_POWER_BUFF;
float power_scale;                  //比率
uint16_t POWER_LIMIT  = 40;          //最大限制功率

uint16_t chassis_max_speed;
uint16_t chassis_top_speed;

/*发射 */
uint8_t allow_send_bullet;		    //是否允许拨弹
uint16_t now_bullet_heat;			//当前枪口热量
float now_bullet_speed;				//当前枪口射速
float last_bullet_speed;			//上次枪口射速
uint16_t heat_limit;					//热量限制
uint16_t speed_limit;					//射速限制
uint16_t now_bullet_cool;  //枪口冷却值
super_cap_t super_cap;
uint8_t vis_speed;
int heat_limit_time;                 //热量限制时间
/*热量限制任务*/
void heat_limit_task(void)
{
	send_referee_updata();	
	
	//热量限制
	if(now_bullet_heat <= heat_limit * 0.80f)
	{
		if(HAL_GetTick() - heat_limit_time > 300)     //跳过裁判系统检测  防止继续超热量
			allow_send_bullet = 1;             //允许发射
	}
	else
	{	
		allow_send_bullet = 0;
		heat_limit_time = HAL_GetTick();
	}
	/*新版热量控制*/
	//HeatUpdate();
	
	
}

void send_referee_updata(void)
{
	last_bullet_speed = now_bullet_speed;     //弹速赋值
	heat_limit = Referee_system_data.extgame_robot_status_t.shooter_id1_17mm_cooling_limit;			//裁判系统热量上限
	//speed_limit = Referee_system_data.extgame_robot_status_t.shooter_id1_17mm_speed_limit;			//裁判系统读取射速上限
	now_bullet_heat = Referee_system_data.extpower_heat_data_t.shooter_id1_17mm_cooling_heat;		//裁判系统读取当前热量
	now_bullet_speed = Referee_system_data.extshoot_data_t.bullet_speed;							//裁判系统读取当前射速
	now_bullet_cool = Referee_system_data.extgame_robot_status_t.shooter_id1_17mm_cooling_rate;   //add-冷却值
	
	switch ( Referee_system_data.extgame_robot_status_t.shooter_id1_17mm_speed_limit)
    {
    	case 18:	speed_limit = 18;vis_speed = 2;
    		break;
    	case 30:	speed_limit = 30;vis_speed = 1;
    		break;
    	default:	speed_limit = 15;vis_speed = 3;
    		break;
    }
	
	
	//发弹更新-》接收处理
	if(now_bullet_speed - last_bullet_speed != 0)
	{
		varible_send.send_data_update = 1;         //摩擦轮用
		varible_send.send_data_limit = 1;			//热量控制
	}
	

	//不同冷却射频  --------->每秒射频
	if(Referee_system_data.extgame_robot_status_t.shooter_id1_17mm_cooling_rate >=60)
		varible_send.send_freq = 67;
	else if(Referee_system_data.extgame_robot_status_t.shooter_id1_17mm_cooling_rate >= 30)
		varible_send.send_freq = 77;
	else if(Referee_system_data.extgame_robot_status_t.shooter_id1_17mm_cooling_rate >= 15)
		varible_send.send_freq = 100;
	else 
		varible_send.send_freq = 100;
}



/*底盘功率限制*/
void chassis_power_limit(void)
{
	parameter_number();
	/*计算pid输出-------->没看懂作用*/
	//total_current = (float)( fabs(pid_Mchassis[0].pid_out_speed) + fabs(pid_Mchassis[1].pid_out_speed) + fabs(pid_Mchassis[2].pid_out_speed) + fabs(pid_Mchassis[3].pid_out_speed) );

	if(robot_ID == 0) 
		power_scale = 1; //ID不存在	
	else
	{
		if(chassis_power_buffer < 0) 
			power_scale = 0;		
		else if(chassis_power_buffer < WARNING_POWER_BUFF)		//底盘实时缓冲能量 《 危险缓冲功率，功率超过最大限制功率
		{
			power_scale = chassis_power_buffer/WARNING_POWER_BUFF;
			if(task_flag.cap_mode == 0)				
				power_scale = power_scale*power_scale;
			else								
				power_scale = power_scale*power_scale*power_scale*power_scale;
		}
		else	
			power_scale = 1;	//功率未到最大限制
	}
	/*输出大于限制总电流*/
	for(int i=0;i<4;i++)
	{
		varible_chassis.send_chassis_speed[i] = power_scale * pid_Mchassis[i].pid_out_speed; 
	}
	
}






/*参数速度赋值*/
void parameter_number(void)
{
/*初始状态 40
		功率优先 60---80---100
		血量优先 45---50---55
		平衡步兵 60---80---100 */
	switch (POWER_LIMIT)
	{
		case 40:
			if(task_flag.cap_mode == 1)
			{
				chassis_top_speed = 5700;														//小陀螺速度
				chassis_max_speed = 5600;														//底盘速度
				WARNING_POWER_BUFF = 40.0f;													//危险缓冲功率
			}
			else if(task_flag.cap_mode == 2)
			{
				chassis_top_speed = 7300;														//小陀螺速度
				chassis_max_speed = 8500;														//底盘速度
				WARNING_POWER_BUFF = 40.0f;													//危险缓冲功率
			}
			else
			{
				chassis_top_speed = 3000;														//小陀螺速度
				chassis_max_speed = 2650;														//底盘速度
				WARNING_POWER_BUFF = 45.0f;													//危险缓冲功率
			}
			break;
		
		case 45:
			if(task_flag.cap_mode == 1)
			{
				chassis_top_speed = 5800;														//小陀螺速度
				chassis_max_speed = 5500;														//底盘速度
				WARNING_POWER_BUFF = 40.0f;													//危险缓冲功率
			}
			else if(task_flag.cap_mode == 2)
			{
				chassis_top_speed = 7300;														//小陀螺速度
				chassis_max_speed = 8500;														//底盘速度
				WARNING_POWER_BUFF = 40.0f;													//危险缓冲功率
			}
			else
			{
				chassis_top_speed = 3000;														//小陀螺速度
				chassis_max_speed = 2850;														//底盘速度
				WARNING_POWER_BUFF = 45.0f;													//危险缓冲功率
			}
			break;
		case 50:
			if(task_flag.cap_mode == 1)
			{
				chassis_top_speed = 6000;														//小陀螺速度
				chassis_max_speed = 5600;														//底盘速度
				WARNING_POWER_BUFF = 40.0f;													//危险缓冲功率
			}
			else if(task_flag.cap_mode == 2)
			{
				chassis_top_speed = 7300;														//小陀螺速度
				chassis_max_speed = 8500;														//底盘速度
				WARNING_POWER_BUFF = 40.0f;													//危险缓冲功率
			}
			else
			{
				chassis_top_speed = 3200;														//小陀螺速度
				chassis_max_speed = 3050;														//底盘速度
				WARNING_POWER_BUFF = 45.0f;													//危险缓冲功率
			}
			break;
		case 55:
			if(task_flag.cap_mode == 1)
			{
				chassis_top_speed = 6100;														//小陀螺速度
				chassis_max_speed = 5700;														//底盘速度
				WARNING_POWER_BUFF = 40.0f;													//危险缓冲功率.
			}
			else if(task_flag.cap_mode == 2)
			{
				chassis_top_speed = 7300;														//小陀螺速度
				chassis_max_speed = 8600;														//底盘速度
				WARNING_POWER_BUFF = 40.0f;													//危险缓冲功率
			}
			else
			{
				chassis_top_speed = 3400;														//小陀螺速度
				chassis_max_speed = 3250;														//底盘速度
				WARNING_POWER_BUFF = 45.0f;													//危险缓冲功率
			}
			break;
		case 60:
			if(task_flag.cap_mode == 1)
			{
				chassis_top_speed = 6200;														//小陀螺速度
				chassis_max_speed = 6000;														//底盘速度
				WARNING_POWER_BUFF = 40.0f;													//危险缓冲功率
			}
			else if(task_flag.cap_mode == 2)
			{
				chassis_top_speed = 8000;														//小陀螺速度
				chassis_max_speed = 8500;														//底盘速度
				WARNING_POWER_BUFF = 40.0f;													//危险缓冲功率
			}
			else
			{
				chassis_top_speed = 3700;														//小陀螺速度
				chassis_max_speed = 3500;														//底盘速度
				WARNING_POWER_BUFF = 45.0f;													//危险缓冲功率
			}
			break;
		case 80:
			if(task_flag.cap_mode == 1)
			{
				chassis_top_speed = 6300;														//小陀螺速度
				chassis_max_speed = 6100;														//底盘速度
				WARNING_POWER_BUFF = 40.0f;													//危险缓冲功率
			}
			else if(task_flag.cap_mode == 2)
			{
				chassis_top_speed = 7900;														//小陀螺速度
				chassis_max_speed = 8500;														//底盘速度
				WARNING_POWER_BUFF = 40.0f;													//危险缓冲功率
			}
			else
			{
				chassis_top_speed = 5200;														//小陀螺速度
				chassis_max_speed = 4500;														//底盘速度
				WARNING_POWER_BUFF = 45.0f;													//危险缓冲功率
			}
			break;
		case 100:
			if(task_flag.cap_mode == 1)
			{
				chassis_top_speed = 7500;														//小陀螺速度
				chassis_max_speed = 6500;														//底盘速度
				WARNING_POWER_BUFF = 40.0f;													//危险缓冲功率
			}
			else if(task_flag.cap_mode == 2)
			{
				chassis_top_speed = 9000;														//小陀螺速度
				chassis_max_speed = 9000;														//底盘速度
				WARNING_POWER_BUFF = 40.0f;													//危险缓冲功率
			}
			else
			{
				chassis_top_speed = 6700;														//小陀螺速度
				chassis_max_speed = 5900;														//底盘速度
				WARNING_POWER_BUFF = 45.0f;													//危险缓冲功率
			}
			break;
		default:
			if(task_flag.cap_mode == 1)
			{
				chassis_top_speed = 6000;														//小陀螺速度
				chassis_max_speed = 5500;														//底盘速度
				WARNING_POWER_BUFF = 40.0f;													//危险缓冲功率
			}
			else if(task_flag.cap_mode == 2)
			{
				chassis_top_speed = 7900;														//小陀螺速度
				chassis_max_speed = 8100;														//底盘速度
				WARNING_POWER_BUFF = 40.0f;													//危险缓冲功率
			}
			else
			{
				chassis_top_speed = 4800;														//小陀螺速度
				chassis_max_speed = 4500;														//底盘速度
				WARNING_POWER_BUFF = 45.0f;													//危险缓冲功率
			}
			break;
			
	}
	chassis_climb_mode(task_flag.chassis_climb);
}



/*爬坡模式*/
void chassis_climb_mode(uint8_t flag)
{
	if(flag == 1)
	{
		task_flag.cap_mode = 1;  //开启超级电容
		chassis_max_speed = 3600;   //%%%%%
	}
}

/*读取实时功率，实时缓冲能量*/
void get_chassis_power_buffer_t(void)
{
	chassis_power = Referee_system_data.extpower_heat_data_t.chassis_power;
	chassis_power_buffer = Referee_system_data.extpower_heat_data_t.chassis_power_buffer;		
}

/*读取机器人ID，最大限制功率，等级*/
void get_chassis_id_maxpower_level(void)
{
	robot_ID = Referee_system_data.extgame_robot_status_t.robot_id;
	POWER_LIMIT = Referee_system_data.extgame_robot_status_t.chassis_power_limit;       //机器人底盘功率限制上限
	robot_level = Referee_system_data.extgame_robot_status_t.robot_level;	
}


/*超级电容任务*/
void super_cap_task(void)
{
	/*读取实时功率，实时缓冲能量*/
	get_chassis_power_buffer_t();
	/*读取机器人ID，最大限制功率，等级*/
	get_chassis_id_maxpower_level();
	/*电容发送*/
	SET_POWER_CURRENT_CAN(&hcan1);
	
	die_alive();
}

/*判断机器人状态*/
void die_alive(void)
{
	if(Referee_system_data.extgame_robot_status_t.remain_HP <= 0)	
		task_flag.die_or =0;
	else task_flag.die_or = 1;
}


/**********************************************************************************************************
*函 数 名: HeatControl
*功能说明: 热量控制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/


uint16_t HeatMax17 ,HeatCool17; //热量上限-10,冷却值/10
const short BulletHeat17 = 10;
short CurHeat17, LastHeat17, AvailableHeat17; //当前热量， 上一次热量, 自行计算热量

uint16_t Shooted17Cnt;	//一周期内已打出子弹数
uint16_t AvailableBullet17;	//下一周期允许打弹数

void HeatControl(void)
{
	if(varible_send.heart_data_update == 1)
	{
		Shooted17Cnt = 0;
		AvailableHeat17 =  constrain(HeatMax17 - now_bullet_heat + now_bullet_cool,heat_limit,0);    //这个需要继续研究
		if(varible_send.send_data_limit == 1)  //检测到发弹--射速更新
		{
			AvailableHeat17 =  constrain(AvailableHeat17 - BulletHeat17,heat_limit,0);
			varible_send.send_data_limit = 0;
		}
		AvailableBullet17 = AvailableHeat17 / BulletHeat17;
		allow_send_bullet = (AvailableBullet17 < 1)?0:1;		//不进行四舍五入
	}
	else if((varible_send.send_data_limit == 1) && (varible_send.heart_data_update == 0))//检测发弹但热量未更新
	{
		Shooted17Cnt++;
		allow_send_bullet = (Shooted17Cnt >= AvailableBullet17)?0:1;		
		varible_send.send_data_limit = 0;
	}
}


/**********************************************************************************************************
*函 数 名: HeatUpdate
*功能说明: 热量更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/

const float HeatControlThreshold = 0.85f;   	//开启热量控制的阈值
void HeatUpdate(void)
{
	HeatMax17 =  heat_limit - 20;
	HeatCool17 = now_bullet_cool/10;
	CurHeat17 = now_bullet_heat;  //当前枪口热量
	
	if(CurHeat17 != LastHeat17 )
	{
		varible_send.heart_data_update = 1;
		varible_send.send_data_limit  = 0; //热量更新则将发射标志位清零(没有代处理的打弹)  只计算一个热量周期
	}
	if(CurHeat17 < HeatControlThreshold*HeatMax17)  //热量控制不介入
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














