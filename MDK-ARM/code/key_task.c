#include "key_task.h"

//板载rgb 蓝绿红 PH10  PH11  PH12

#define mode1  10 
#define mode2 01
#define mode3 00


RC16_chn RC16_CHN;        //遥控通道赋值
task_flag_t task_flag;   //任务标志位


uint32_t rc_close_send_time;			//遥控器关闭发射时间
uint8_t rc_send_init_flag;				//遥控器发射标志


/*摇杆通道判断赋值*/
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
	else  {}//遥杆通道数据错误
}


/*遥控控制任务*/
void key_task(void)
{
	Buzzer_TASK();				//掉线检测
	temperature_protect();		//电机保护
	access_assiqnment();       //拨杆通道赋值
	if (monitor.rc_monitor == true)
		mode_task();
	else
	{
		task_flag.mode = 0;      //无力
		mode3;   /*板载显示灯全灭*/
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
		task_flag.mode = 0;  //无力
		pill_open();
		rc_send_init_flag = 0;
		KEY_SYS_RESET();    //软件复位
		mode1;      //板载灯
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
	if( task_flag.mode == 0 && RC_DR16.switch_right == 1 ) //无力)
	{
		task_flag.pill_depot = 1;
	}
	if(task_flag.mode == 0 && RC_DR16.switch_right == 2)
	{
		task_flag.pill_depot = 0;
	}

}

int asdw = 0;
/*正常模式*/
void normal_1(void)
{
	/*自瞄<--->鼠标右键*/
	if(RC_DR16.mouse.press_right == 1 ||RC_DR16.ch5 < -200 ||POWER_LIMIT == 70)            
		task_flag.automatic_aiming = 1;
	else
		task_flag.automatic_aiming = 0;
	
	//打击状态
	if(RC_DR16.keyBoard.key_code == KEY_F  )
		task_flag.hit_mode = 0;		//打符
	else if(RC_DR16.keyBoard.key_code == KEY_CTRL_F)
		task_flag.hit_mode = 1;	
	
	if(task_flag.automatic_aiming == 1 && task_flag.hit_mode == 0)
		task_flag.auto_mode = 1;
	else if(task_flag.automatic_aiming == 1 && task_flag.hit_mode == 1)
		task_flag.auto_mode = 2;
	else 
		task_flag.auto_mode = 0;
		
	
//	if(RC_DR16.keyBoard.key_code == KEY_Z)
//		task_flag.hit_mode = 0;						//自瞄
	
	/*开启摩擦轮<----->KEY_G*/
	if(RC_DR16.keyBoard.key_code & KEY_G)
	{
		task_flag.frictiongear = 1;				//开启
	}
	if(RC_DR16.keyBoard.key_code == KEY_CTRL_G)
	{
		task_flag.frictiongear = 0;				//关闭
		task_flag.frictiongear_ramp = 200;  
	}
	//摩擦轮未开发射
	if(RC_DR16.mouse.press_left == 1 && task_flag.frictiongear == 0)
	{
		task_flag.frictiongear = 1;				//开启		
		task_flag.frictiongear_ramp = 200;    //摩擦轮开启等待斜坡  延迟
	}
	
	/*开启预测<-------->KEY_B*/
	if(key_repetition_2(&key_repetition_B,RC_DR16.keyBoard.key_code,KEY_B) == 1)       /*按键调用两次触发-->有点东西*/
		task_flag.aim_predict = 1;					//模式2
	else if(key_repetition_2(&key_repetition_B,RC_DR16.keyBoard.key_code,KEY_B) == 0)
		task_flag.aim_predict = 0;					//模式2
	
	if(vision_rx.date_update == true && task_flag.auto_mode == 1 && varible_vision.reverse_top == 1 && varible_vision.send_or == 1)
		task_flag.vision_send = 1; //视觉识别到不允许发弹
	else 
		task_flag.vision_send = 0; 
	
	
	
	//拨弹
	if(task_flag.frictiongear == 1)
	{
		if(task_flag.frictiongear_ramp > 0) 
			task_flag.frictiongear_ramp--;  //这个有什么说法
		else
		{
			/*拨弹发射<------->鼠标左键*/
			if(RC_DR16.mouse.press_left == 1 && task_flag.vision_send == 0)
				task_flag.load_mo = 1;		
			else
				task_flag.load_mo = 0;					
		}			
	}
	//遥控发射，上-中-下 1-3-2
	if(RC_DR16.switch_right ==1)
	{
		if(HAL_GetTick() - rc_close_send_time < 1000)
			task_flag.frictiongear = 0;				//关闭
		rc_send_init_flag = 1;
	}
	else if(RC_DR16.switch_right == 3 && rc_send_init_flag == 1)
	{
		rc_close_send_time = HAL_GetTick();
		task_flag.frictiongear = 1;				//开启
		task_flag.load = 0;				//关闭
	}
	else if(RC_DR16.switch_right == 2 && rc_send_init_flag == 1)
	{
		task_flag.load = 1;				//开启
	}
	

	/*小陀螺<------>KEY_SHIFT*/
	if(RC_DR16.keyBoard.key_code & KEY_SHIFT||RC_DR16.switch_left == 1)
		task_flag.small_gyro = 1;				//开启
	else
		task_flag.small_gyro = 0;				//关闭
	
	/*弹舱开启<----->KEY_CTRL_R*/
	if(RC_DR16.keyBoard.key_code == KEY_CTRL_R)
		task_flag.pill_depot = 1;				//开启
	else if(RC_DR16.keyBoard.key_code & KEY_R)
		task_flag.pill_depot = 0;				//关闭
	
	/*超级电容*///1全自动2只充电不放电3不冲也不放
	if((RC_DR16.keyBoard.key_code & KEY_V || RC_DR16.ch5 > 200) && super_cap.cap_capacity > 3 && task_flag.chassis_climb == 0)//-----------------------------------模式-------------
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
	
	
	
	//爬坡模式                                     //没有键位
	if(RC_DR16.keyBoard.key_code & KEY_W && RC_DR16.keyBoard.key_code & KEY_Q && super_cap.cap_capacity > 8)
		task_flag.chassis_climb = 1;
	else 
		task_flag.chassis_climb = 0;
	
	
	/*原地掉头<----->X      非自瞄使用*/
	if(RC_DR16.keyBoard.key_code & KEY_X && task_flag.zig == 0)			//180
		task_flag.zig = 1;				//开启
	
}
 

uint32_t sys_reset_time;
uint8_t sys_reset_time_flag;

/*软件复位*/
void KEY_SYS_RESET(void)
{
	if(RC_DR16.keyBoard.key_code == KEY_CTRL_B || RC16_CHN.ch5 >656)  //选择按键------>
		task_flag.sys_reset = 1;

	if(task_flag.sys_reset == 1 && sys_reset_time_flag == 0)
	{
		sys_reset_time = HAL_GetTick ();
		sys_reset_time_flag = 1;                        //复位之后初始化变0<--->傻逼了我  m
	}
	if(HAL_GetTick () - sys_reset_time > 0 && HAL_GetTick () - sys_reset_time <= 1000 && task_flag.sys_reset == 1)
		task_flag.mode = 0;						//无力
	else if(HAL_GetTick () - sys_reset_time > 1000 && HAL_GetTick () - sys_reset_time <= 1200 && task_flag.sys_reset == 1)
	{
		task_flag.sys_reset = 0;
		HAL_NVIC_SystemReset();  //软件复位
	}
	
}



























