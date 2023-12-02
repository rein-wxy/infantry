#include "vision.h"

varible_vision_t varible_vision;     //视觉变量存储
vision_rx_t vision_rx;               //视觉接收缓存

/*视觉任务初始化*/
float kalm_yaw;
float kalm_pitch;
void vision_Init(void)
{
	//卡尔曼滤波
	KalmanCreate(&kalman_visionYaw,1,1);
	KalmanCreate(&kalman_targetYaw,1,0);
	KalmanCreate(&kalman_visionPitch,1,1);
	KalmanCreate(&kalman_targetPitch,1,0);
	KalmanCreate(&kalman_visionDistance,1,5);
	KalmanCreate(&kalman_targetDistance,1,0);
	KalmanCreate(&kalman_accelYaw,1,0);
	KalmanCreate(&kalman_speedYaw,1,0);
	KalmanCreate(&kalman_accelPit,1,0);
	KalmanCreate(&kalman_speedPit,1,0);
	
	average_init(&vision_distance,40);
	//平滑滤波
//	varible_vision.speed_queue_yaw.queueLength = 60;
//	varible_vision.speed_queue_pit.queueLength = 60;
	varible_vision.dt_m = 0;
}



/*视觉任务*/
void vision_task(void)
{
	send_data_to_nuc();
	vision_data_manage();
	Vision_on_off();
	
}

int isp = 0;
/*
	串口接收数据
	进入数据处理函数->Kerman滤波->转换
*/

/*360度*/
/*视觉信息处理*/
void vision_data_manage(void)
{
	static uint16_t active_cnt=0,lost_cnt=0;
	if(vision_rx.date_update == true)
	{
		memcpy(&varible_vision.vision_yaw_up,&varible_vision.Vision_rx[3],4);
		memcpy(&varible_vision.vision_pitch_up,&varible_vision.Vision_rx[7],4);
		memcpy(&varible_vision.vision_distance_up,&varible_vision.Vision_rx[11],4);
		memcpy(&varible_vision.reverse_top,&varible_vision.Vision_rx[15],1);
		memcpy(&varible_vision.send_or,&varible_vision.Vision_rx[16],1);
	
		if((varible_vision.vision_distance_up != 0 ||varible_vision.vision_pitch_up != 0 || varible_vision.vision_yaw_up != 0) )			
		{
			active_cnt++;
			if(active_cnt >= 2)
			{
				varible_vision.aim_flag = true;
				active_cnt = 0;   //防溢出
				lost_cnt = 0;
			}			
		}
		else
		{
			lost_cnt++;
			if(lost_cnt >= 10)
			{
				varible_vision.aim_flag = false;
				active_cnt = 0;
				lost_cnt = 0;
				varible_vision.vision_distance_up = 0;
				varible_vision.vision_pitch_up = 0;
				varible_vision.vision_yaw_up = 0;
				//清除平滑滤波队列
//				Clear_Queue(&varible_vision.speed_queue_pit);
//				Clear_Queue(&varible_vision.speed_queue_yaw);	
			}
			
		}
		
		varible_vision.vision_pitch_err = varible_vision.vision_pitch_up ;
		varible_vision.vision_yaw_err = varible_vision.vision_yaw_up ;
		varible_vision.vision_distance = varible_vision.vision_distance_up*0.01f;

		
		//卡尔曼
		varible_vision.vision_yaw_kf = KalmanFilter(&kalman_visionYaw,varible_vision.vision_yaw_err);
		varible_vision.vision_pitch_kf = KalmanFilter(&kalman_visionPitch,varible_vision.vision_pitch_err);	
		varible_vision.vision_distance_kf = KalmanFilter(&kalman_visionDistance,varible_vision.vision_distance);	
		
		//vision_compensate(varible_vision.vision_distance_kf);	
			
		if(varible_vision.aim_flag == true && monitor.nuc_monitor  )  //要测试
		{
			varible_vision.vision_pitch_out = varible_vision.vision_pitch_up * 22.755; //+ varible_vision.dt_m;
			varible_vision.vision_yaw_out = varible_vision.vision_yaw_up * 22.755f;
//			varible_vision.vision_pitch_out = varible_vision.vision_pitch_kf * 22.755f;
//			varible_vision.vision_yaw_out = varible_vision.vision_yaw_kf * 22.755f;
		}
		else 
		{
			varible_vision.vision_pitch_out = 0;
			varible_vision.vision_yaw_out = 0;
		}		
	}
	else 
	{
		varible_vision.vision_distance_up = 0;
		varible_vision.vision_pitch_up = 0;
		varible_vision.vision_yaw_up = 0;
	}
	if(task_flag.automatic_aiming == 0)
	{
		varible_vision.vision_distance_up = 0;
		varible_vision.vision_pitch_up = 0;
		varible_vision.vision_yaw_up = 0;
		
	}
}




/*开火补偿*/
float yaw_speed_r,yaw_accel_r,pit_speed_r,pit_accel_r;
void vision_compensate(float distance)
{
	
	average_add(&vision_distance,distance);//视觉解算距离跳动；
	if(vision_distance.aver_num >= 8)//视觉发送距离限幅
	{
		vision_distance.aver_num = 8.00;
	}
	switch (vis_speed)
    {
    	case 1: 			//30
    		varible_vision.dt_m = 2.05 * distance *distance -14.5 * distance + 39.5;
			break;
    	case 2:				//18
			varible_vision.dt_m = 3.3 * distance *distance +6.69 * distance - 0;
			break;
    	default:			//15
    		varible_vision.dt_m = 3.3 * distance *distance +6.69 * distance + 20;
			break;
    }
	
	//varible_vision.dt_m
}

/*预测*/
void Vision_predict(void)
{

}



/*视觉前馈处理*/
void Vision_handle(void)
{
	//yaw
	//pitch

}
/*重接收处理*/
uint32_t cnt_reset;
void Vision_on_off(void)
{
	if(monitor.nuc_monitor == 0 ||vision_rx.date_update == false)	
	{
		vision_rx.rx_err_cnt++;
		vision_rx.date_update = false;					
	}
	if(vision_rx.rx_err_cnt != 0 && cnt_reset < 10)
	{
		cnt_reset++;
		HAL_UART_DMAPause(&huart1);//关
		if(cnt_reset >= 9)
		{ 
			HAL_UART_DMAResume(&huart1);//开
			HAL_UART_Receive_DMA(&huart1, varible_vision.Dislocatoon, 18);			//与视觉接收
		}
	}
	else cnt_reset = 0;
}


/*视觉信息DMA接收*/
uint8_t l = 0;    //串位强制拉回位置

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)	//暂时<-------->
	{	
		for(uint8_t i = 0;i < Vision_SIZE;i++)
		{
			uint8_t j = i + 17;
			if(j >= Vision_SIZE )
				j = j - Vision_SIZE ;
			if(varible_vision.Dislocatoon[i] == 0xA5 && varible_vision.Dislocatoon[j] == 0x0D) //校验
			{
				l = i;
				//处理数据
				for(uint8_t k = 0;k < Vision_SIZE;k++)
				{
					if(l >= Vision_SIZE)
						l = l - Vision_SIZE ;
					varible_vision.Vision_rx[k] = varible_vision.Dislocatoon[l];
					l++;
				}
			}	
		}	
		monitor.nuc_cnt++;	
		vision_rx.rx_err_cnt = 0;	
		vision_rx.date_update = true;	
	}
//		vision_rx.date_update = false;			//只有一个huart1
	
/*	
		//校验解析
		if(varible_vision.Vision_rx[0] == 0xA5 && varible_vision.Vision_rx[17] == 0x00)
		{	
//			if(Verify_CRC8_Check_Sum(varible_vision.Vision_rx,3))
//			{
//				if(Verify_CRC16_Check_Sum(varible_vision.Vision_rx,18))
//				{
					memcpy(&varible_vision.vision_yaw_up,&varible_vision.Vision_rx[3],4);
					memcpy(&varible_vision.vision_pitch_up,&varible_vision.Vision_rx[7],4);
					memcpy(&varible_vision.vision_distance_up,&varible_vision.Vision_rx[11],4);
					memcpy(&varible_vision.reverse_top,&varible_vision.Vision_rx[15],1);
					memcpy(&varible_vision.send_or,&varible_vision.Vision_rx[16],1);				

			

			//帧数测试
//					if(HAL_GetTick()-vision_rx.time_now >= 999)
//					{
//						vision_rx.update_fps = vision_rx.time_fps;
//						vision_rx.time_now = HAL_GetTick();
//						vision_rx.time_fps = 0;
//					}
//					vision_rx.time_fps++;	
					vision_rx.rx_err_cnt = 0;	
					vision_rx.date_update = true;			
								

//				}
//			}
		}
		else //if(varible_vision.Vision_rx[0] != 0xA5 && varible_vision.Vision_rx[18] != 0x00)
		{
			vision_rx.rx_err_cnt++;
			vision_rx.date_update = false;					
		}
		
	}
*/	
	
	HAL_UART_Receive_DMA(&huart1,varible_vision.Dislocatoon,18);
}




/*
	[0] 头帧0xA5
	[1] 命令码
	[2] CRC8效验码
	[3]	敌我红蓝判断  红-1 蓝-0
	[4]	模式	0 手动 1自瞄反陀螺  2打符 
	[5]-[6]    yaw轴 高-低  (-18000)--0--(+18000)
	[7]-[8]    yitch轴		(-18000)--0--(+18000)
	[9]	最大射速标志	1--2--3	
	[10] 0
	[11] 0
	[12] 0
	[15]CRC16效验码


*/

uint32_t cnt_reset;
/*发送数据到nuc*/
void send_data_to_nuc(void)
{
	
	varible_vision.Vision_tx[0] = 0xA5;          //头帧
	varible_vision.Vision_tx[1] = 0x9;             //命令   步兵 英雄 哨兵	
	
	Append_CRC8_Check_Sum(varible_vision.Vision_tx,3);			//校验头   添加CRC8校验到 varible_vision.Vision_tx[2]
	varible_vision.Vision_tx[3] = Red_or_Blue_judge();       //敌我红蓝判断
	varible_vision.Vision_tx[4] = task_flag.auto_mode;		//模式						//8bit 0--65535
	varible_vision.Vision_tx[5] = ((int16_t)((imu.yaw/22.755f)*182) >> 8)&0xFF;	
	varible_vision.Vision_tx[6] = ((int16_t)((imu.yaw/22.755f)*182))&0xFF;			
	varible_vision.Vision_tx[7] = ((int16_t)((imu.pitch/22.755f)*182) >> 8)&0xFF;				
	varible_vision.Vision_tx[8] = ((int16_t)((imu.pitch/22.755f)*182))&0xFF;			
	varible_vision.Vision_tx[9] = vis_speed; //弹速
	varible_vision.Vision_tx[10]= 0;										
	varible_vision.Vision_tx[11]= 0;
	varible_vision.Vision_tx[12]= 0;
	varible_vision.Vision_tx[13]= 0;										
	varible_vision.Vision_tx[14]= 0;

	Append_CRC16_Check_Sum(varible_vision.Vision_tx,16);	//整包CRC16校验			varible_vision.Vision_tx[14][15]


	//发送
	HAL_UART_Transmit_DMA(&huart1 ,varible_vision.Vision_tx ,16);	
	
}







