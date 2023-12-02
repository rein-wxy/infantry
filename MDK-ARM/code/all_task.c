#include "all_task.h"



/*��ʼ��*/
void base_Init(void)
{
	CAN1_FILTER_CONFIG(&hcan1);								//����������CAN1
	CAN2_FILTER_CONFIG(&hcan2);								//����������CAN2
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);   //can1�ж�
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);   //can2�ж�
	
	//��������  ��ʱ�ж�  ռλ
	
	HAL_UART_Receive_IT_IDLE(&huart3, remote_control_lever_buff, 100);	//ң����	
	HAL_UART_Receive_IT_IDLE(&huart6, uart6_rx_buff, 100);												//����ϵͳ���ڳ�ʼ��	
	HAL_UART_Receive_DMA(&huart1,varible_vision.Dislocatoon,18);
	//HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);			//��������ʱ��ʹ��
	
}

/*�����ʼ��*/
void task_Init(void)
{
	imu_sensor_init();//������
	chassis_Init();//����
	gimbal_Init();//��̨
	vision_Init();//�Ӿ�
	for(int i=80;i < 82;i++) //��ʼ���ɹ���־��������
	{
		Buzzer_task(i+50,i*2,i*1.5);
	}

}

/*��������*/

/*��������*/
void chassis_all_task(void)
{
	chassis_task();
}
/*��̨����*/
void gimbal_all_task(void)
{
	gimbal_task();
}
/*��������*/
void key_all_task(void)
{
	key_task();    //rcң�������ڿ����ж�
}
/*����������*/
void imu_all_task(void)
{
	imu_task(&imu);	
}
/*����ϵͳ����*/     //���ܡ�������DMA
void referee_all_task(void)
{
	UI_task();					//�Զ���UI����
}













