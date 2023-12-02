#include "all_task.h"



/*初始化*/
void base_Init(void)
{
	CAN1_FILTER_CONFIG(&hcan1);								//过滤器配置CAN1
	CAN2_FILTER_CONFIG(&hcan2);								//过滤器配置CAN2
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);   //can1中断
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);   //can2中断
	
	//串口配置  闲时中断  占位
	
	HAL_UART_Receive_IT_IDLE(&huart3, remote_control_lever_buff, 100);	//遥控器	
	HAL_UART_Receive_IT_IDLE(&huart6, uart6_rx_buff, 100);												//裁判系统串口初始化	
	HAL_UART_Receive_DMA(&huart1,varible_vision.Dislocatoon,18);
	//HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);			//蜂鸣器定时器使能
	
}

/*任务初始化*/
void task_Init(void)
{
	imu_sensor_init();//陀螺仪
	chassis_Init();//底盘
	gimbal_Init();//云台
	vision_Init();//视觉
	for(int i=80;i < 82;i++) //初始化成功标志蜂鸣器响
	{
		Buzzer_task(i+50,i*2,i*1.5);
	}

}

/*各种任务*/

/*底盘任务*/
void chassis_all_task(void)
{
	chassis_task();
}
/*云台任务*/
void gimbal_all_task(void)
{
	gimbal_task();
}
/*按键任务*/
void key_all_task(void)
{
	key_task();    //rc遥控器串口空闲中断
}
/*陀螺仪任务*/
void imu_all_task(void)
{
	imu_task(&imu);	
}
/*裁判系统任务*/     //与电管——串口DMA
void referee_all_task(void)
{
	UI_task();					//自定义UI任务
}













