#include "bsp_can.h"



CAN_RxHeaderTypeDef RxMessage;											//CANͨѶ�õ�����Ϣ
CAN_TxHeaderTypeDef TxMessage;											//CANͨѶ��������Ϣ
uint8_t RXData[8];																	//CANͨѶ�õ�����Ϣ����
uint8_t TxData[8]; 																	//CANͨѶ��������Ϣ����

motor_data chassis_motor_RX[4];
motor_data yaw_RX;
motor_data pitch_RX;
motor_data send_RX;													//�����̵������ֵ
motor_data shoot_RX[2];												//Ħ���ֵ������ֵ
power_data power_RX;                                                //���ʰ巴������


void CAN1_FILTER_CONFIG(CAN_HandleTypeDef* hcan)
{
		CAN_FilterTypeDef		CAN_FilterConfigStructure;
		CAN_FilterConfigStructure.FilterBank = 0;													//ʹ�ù�����0
		CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;			//����ģʽ
		CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;		//����Ϊ32λ
		CAN_FilterConfigStructure.FilterIdHigh = 0x7FFE;
		CAN_FilterConfigStructure.FilterIdLow = 0x0000;
		CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
		CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
		CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;	//����FIFO0��
		CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
		CAN_FilterConfigStructure.FilterActivation = ENABLE;
			
	if(HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfigStructure) != HAL_OK)
	{
		Error_Handler();
    }
    if(HAL_CAN_Start(&hcan1) != HAL_OK)
    {
		Error_Handler();
    }
    if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
		Error_Handler();
	}
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	
}

void CAN2_FILTER_CONFIG(CAN_HandleTypeDef* hcan)
{
		CAN_FilterTypeDef		CAN_FilterConfigStructure;
		CAN_FilterConfigStructure.FilterBank = 14;													//ʹ�ù�����0
		CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;			//����ģʽ
		CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;		//����Ϊ32λ
		CAN_FilterConfigStructure.FilterIdHigh = 0xFFFF;
		CAN_FilterConfigStructure.FilterIdLow = 0x7FFF;
		CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
		CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
		CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;	//����FIFO0��
		CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
		CAN_FilterConfigStructure.FilterActivation = ENABLE;
			
	if(HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfigStructure) != HAL_OK)
	{
		Error_Handler();
    }
    if(HAL_CAN_Start(&hcan2) != HAL_OK)
    {
		Error_Handler();
    }
    if(HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
		Error_Handler();
	}
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
	
}

void get_moto_measure(motor_data *ptr, CAN_HandleTypeDef* hcan, uint8_t RXdata[8])
{
	ptr->last_angle = ptr->angle;
	ptr->last_speed = ptr->speed;
	ptr->last_total_angle = ptr->total_angle;
	
	ptr->angle = (RXdata[0] << 8)|RXdata[1];
	ptr->speed = (RXdata[2] << 8)|RXdata[3];
	ptr->torque = (RXdata[4] << 8)|RXdata[5];
	ptr->temperature = RXdata[6];
	
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle; 				//��������ʲô�ã�����cnt������ ����>��ȷ����
	ptr->angle_err = ptr->last_total_angle - ptr->total_angle;
}

/***********power**************/

void get_power_measure(power_data *power, CAN_HandleTypeDef* hcan, uint8_t RXdata[8])
{
	super_cap.cap_capacity = RXdata[0];
	super_cap.cap_power = RXdata[1]-128;			//��������
}
int zi;
/*CAN�ص�����*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*hcan)
{
	if(	HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RxMessage,RXData) == HAL_OK)
	{
		if(hcan->Instance == CAN1)
		{
			switch (RxMessage.StdId)
			{
				case 0x201:
					get_moto_measure(&chassis_motor_RX[0], &hcan1, RXData);monitor.chassis_cnt[0]++; break;
				case 0x202:
					get_moto_measure(&chassis_motor_RX[1], &hcan1, RXData);monitor.chassis_cnt[1]++;break;
				case 0x203:
					get_moto_measure(&chassis_motor_RX[2], &hcan1, RXData);monitor.chassis_cnt[2]++;break;
				case 0x204:
					get_moto_measure(&chassis_motor_RX[3], &hcan1, RXData);monitor.chassis_cnt[3]++;break;
					
				case 0x205: /*yaw*/
					get_moto_measure(&yaw_RX, &hcan1, RXData);monitor.yaw_cnt++;break;		
				case 0x301: 
					get_power_measure(&power_RX,&hcan1,RXData);monitor.power_cnt++;break;
			}
		}
	}
	if( HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&RxMessage,RXData) == HAL_OK)
	{
		/*CAN2��̨*/
		if(hcan->Instance == CAN2)
		{
			switch (RxMessage.StdId)
			{
				/*Ħ����*/
				case 0x206:
					get_moto_measure(&shoot_RX[0], &hcan2, RXData); monitor.shoot_cnt[0]++; break;
				case 0x207:
					get_moto_measure(&shoot_RX[1], &hcan2, RXData); monitor.shoot_cnt[1]++; break;
				/*������*/
				case 0x208:
					get_moto_measure(&send_RX, &hcan2, RXData); monitor.send_cnt++; break;			
				case 0x205:
					get_moto_measure(&pitch_RX, &hcan2, RXData); monitor.pitch_cnt++; break;
		
			}
		}
	}
	zi++;
}


//��ʶ��  ��׼֡  ����֡  ���ݳ���

void SET_MOTOR_CURRENT_CAN1_1234(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{	           	 
    TxMessage.StdId = 0x200;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;    
    TxMessage.DLC = 8;
    TxMessage.TransmitGlobalTime = DISABLE;
    
    TxData[0] = (iq1 >> 8)&0xff;
	TxData[1] = (iq1)&0xff;
	TxData[2] = (iq2 >> 8)&0xff;
	TxData[3] = (iq2)&0xff;
	TxData[4] = (iq3 >> 8)&0xff;
	TxData[5] = (iq3)&0xff;
    TxData[6] = (iq4 >> 8)&0xff;
	TxData[7] = (iq4)&0xff;
	
	HAL_CAN_AddTxMessage(&hcan1,&TxMessage,TxData,CAN_FILTER_FIFO0);
}
void SET_MOTOR_CURRENT_CAN1_5678(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{	           	 
    TxMessage.StdId = 0x1FF;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;    
    TxMessage.DLC = 8;
    TxMessage.TransmitGlobalTime = DISABLE;
    
    TxData[0] = (iq1 >> 8)&0xff;
	  TxData[1] = (iq1)&0xff;
	  TxData[2] = (iq2 >> 8)&0xff;
	  TxData[3] = (iq2)&0xff;
	  TxData[4] = (iq3 >> 8)&0xff;
	  TxData[5] = (iq3)&0xff;
    TxData[6] = (iq4 >> 8)&0xff;
	  TxData[7] = (iq4)&0xff;
	
		HAL_CAN_AddTxMessage(&hcan1,&TxMessage,TxData,CAN_FILTER_FIFO0);
}
/** @brief �������**//**CAN2**//**0x200**/

void SET_MOTOR_CURRENT_CAN2_1234(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
	TxMessage.StdId = 0x200;
  TxMessage.IDE = CAN_ID_STD;
  TxMessage.RTR = CAN_RTR_DATA;    
  TxMessage.DLC = 8;
  TxMessage.TransmitGlobalTime = DISABLE;
    
  TxData[0] = (iq1 >> 8)&0xff;
	TxData[1] = (iq1)&0xff;
	TxData[2] = (iq2 >> 8)&0xff;
  TxData[3] = (iq2)&0xff;
  TxData[4] = (iq3 >> 8)&0xff;
	TxData[5] = (iq3)&0xff;
	TxData[6] = (iq4 >> 8)&0xff;
	TxData[7] = (iq4)&0xff;
	
	HAL_CAN_AddTxMessage(&hcan2,&TxMessage,TxData,CAN_FILTER_FIFO0);
}

/** @brief �������**//**CAN2**//**0x1FF**/

void SET_MOTOR_CURRENT_CAN2_5678(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
	TxMessage.StdId = 0x1FF;
  TxMessage.IDE = CAN_ID_STD;
  TxMessage.RTR = CAN_RTR_DATA;    
  TxMessage.DLC = 8;
  TxMessage.TransmitGlobalTime = DISABLE;
    
  TxData[0] = (iq1 >> 8)&0xff;
	TxData[1] = (iq1)&0xff;
	TxData[2] = (iq2 >> 8)&0xff;
  TxData[3] = (iq2)&0xff;
  TxData[4] = (iq3 >> 8)&0xff;
	TxData[5] = (iq3)&0xff;
	TxData[6] = (iq4 >> 8)&0xff;
	TxData[7] = (iq4)&0xff;
	
	HAL_CAN_AddTxMessage(&hcan2,&TxMessage,TxData,CAN_FILTER_FIFO0);
}

/** @brief �������**//**CAN1**//**power**/

void SET_POWER_CURRENT_CAN(CAN_HandleTypeDef* hcan)
{
	TxMessage.StdId = 0x100;
  TxMessage.IDE = CAN_ID_STD;
  TxMessage.RTR = CAN_RTR_DATA;    
  TxMessage.DLC = 6;
  TxMessage.TransmitGlobalTime = DISABLE;
  
	if(POWER_LIMIT > 110)
		TxData[1] = (uint8_t)110;
	else 
		TxData[1] = (uint8_t)POWER_LIMIT;
	
	TxData[2] = (uint8_t)chassis_power_buffer*1.0f;

	//1ȫ�Զ�2ֻ��粻�ŵ�3����Ҳ����
	if(task_flag.cap_mode == 0)		TxData[4] = 2;
	else 							TxData[4] = 1;
	
	HAL_CAN_AddTxMessage(&hcan1,&TxMessage,TxData,CAN_FILTER_FIFO0);
}





