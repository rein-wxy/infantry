#include "function_lib.h"

/*б��*/
ramp_t top_ramp;         //С����б��
ramp_t chassis_x_ramp;   //����б��
ramp_t chassis_y_ramp;
ramp_t chassis_max_speed_ramp;   //��������ٶ�б��
/*һ���˲�*/
first_order_filter_type_t mouse_x_first;
first_order_filter_type_t mouse_y_first;
/*����ƽ���˲�->���û������*/
moving_Average_Filter mouse_x_moving;
moving_Average_Filter mouse_y_moving;
key_repetition_t key_repetition_B;

moving_Average_Filter vision_distance;

/*б��3.0*/
/*            ��ǰֵ��        Ŀ��ֵ��       ����ֵ��     Ŀ��ֵ�������ʣ�Ŀ��ֵ��������*/
void Slow(double *rec , double *target , float slow_Inc , float in_rate , float out_rate)
{
  if(fabs(*rec) - fabs(*target) < 0)
		slow_Inc = out_rate*slow_Inc;
  if(fabs(*rec) - fabs(*target) > 0)
		slow_Inc = in_rate*slow_Inc;
  if(fabs(*rec - *target) < slow_Inc) 
		*rec = *target;
  else 
	{
    if((*rec) > *target) (*rec) -= slow_Inc;
    if((*rec) < *target) (*rec) += slow_Inc;
  }
}
/*б��3.0����*/
void Slow_clear(ramp_t *ramp)
{
	ramp->remp_num_now = 0;
	ramp->remp_num_target = 0;
}

/*************************
***һ�׵�ͨ�˲���ʼ��
***�˲��ṹ��
***ʱ����
***���� 	//Խ��Խ�ȶ���Ӧ�ٶ�Խ��
*************************/
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num)
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num = num;
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/*************************
***һ�׵�ͨ�˲�
***�˲��ṹ��
***��������
*************************/
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input)
{
	first_order_filter_type->last_out = first_order_filter_type->out;
    first_order_filter_type->input = input;
    first_order_filter_type->out =
    first_order_filter_type->num / (first_order_filter_type->num + first_order_filter_type->frame_period) * first_order_filter_type->out + 
	first_order_filter_type->frame_period / (first_order_filter_type->num + first_order_filter_type->frame_period) * first_order_filter_type->input;
}



/**
  * @brief    average_init
  * @note    �����˲�����ʼ�������ó���
  * @param  None
  * @retval None
  * @author  RobotPilots
  */
void average_init(moving_Average_Filter *Aver, uint8_t lenth)
{
	uint16_t i;
	
	for(i = 0; i<100; i++)
		Aver->num[i] = 0;
	
	if(lenth >100)
	{
		lenth = 100;
	}
	
	Aver->lenth = lenth;
	Aver->pot = 0;
	Aver->aver_num = 0;
	Aver->total = 0;
	
}
/**
  * @brief    average_add
  * @note    ����ƽ���˲���������У��Ƚ��ȳ�
  * @param  None
  * @retval None
  * @author  RobotPilots
  */
void average_add(moving_Average_Filter *Aver, float add_data)
{
	
	Aver->total -=  Aver->num[Aver->pot];
	Aver->total += add_data;
	
	Aver->num[Aver->pot] = add_data;
	
	Aver->aver_num = (Aver->total)/(Aver->lenth);
	Aver->pot++;
	
	if(Aver->pot == Aver->lenth)
	{
		Aver->pot = 0;
	}

}

float Get_Diff(uint8_t queue_len, QueueObj *Data,float add_data)
{
    if(queue_len>=Data->queueLength)
        queue_len=Data->queueLength;
    //��ֹ���
    Data->queueTotal-=Data->queue[Data->nowLength];
    Data->queueTotal+=add_data;

    Data->queue[Data->nowLength]=add_data;
	
    Data->nowLength++;

    if(Data->full_flag==0)//��ʼ����δ��
    {
        Data->aver_num=Data->queueTotal/Data->nowLength;
    }else if(Data->full_flag == 1)
	{
	    Data->aver_num=(Data->queueTotal)/queue_len;			
	}
    if(Data->nowLength>=queue_len)
    {
        Data->nowLength=0;
        Data->full_flag=1;
    }

    Data->Diff=add_data - Data->aver_num;
    return Data->Diff;
}
void Clear_Queue(QueueObj* queue)
{
    for(uint16_t i=0; i<queue->queueLength; i++)
    {
        queue->queue[i]=0;
    }
    queue->nowLength = 0;
    queue->queueTotal = 0;
    queue->aver_num=0;
    queue->Diff=0;
    queue->full_flag=0;
}



/*************************
***�����ظ�����KEY����
***�����ṹ��
***����
***����ֵ
*************************/
int key_repetition_2(key_repetition_t* K,uint16_t key, uint16_t key_target)
{
	if(key & key_target && K->i == 1)
	{
		K->cnt++;
		K->i = 0;
	}
	else if((key & key_target) == 0)
	{
		K->i = 1;
	}
	
	if(K->cnt % 2 == 1)
		return 1;
	else
		return 0;
}


/*�������ٶ���Ư����*/
float imu_speed_deal(int i,int j,float speed)
{
	if((speed <= i ) && (speed >= j))
		return 0;
	else 
		return speed;
}

/*���߼��*/
void on_online(online* ol,int time,int SUM)
{
	ol->TIME2 =HAL_GetTick();
	if(ol->TIME2 - ol->TIME1 > time)
	{
		ol->flag = 0;
		ol->TIME1 = ol->TIME2;
	}
	if(ol->flag == 0)
	{
		ol->sum ++;
		if(ol->sum >= SUM) ol->FLAG = 0;     //��ֹ��ʶ��
		else ol->FLAG = 1;
	}
}

/*����������*/
void Buzzer_task(uint16_t buzzer,uint16_t time1,uint16_t time2)
{
	TIM4 -> CCR3 = buzzer;
	HAL_Delay(time1);
	TIM4 -> CCR3 = 0;
	HAL_Delay(time2);
}

//uint8_t my_delay(uint16_t a,uint8_t b,uint8_t c)
//{
//	static uint16_t time = 0;
//	
//	time++;
//	for(int i = 1;i<=c;i++)
//	{
//		
//	}
////	if(time > (a/b)*i)
////	{
////		time = 0;
////		TIM4->CCR3 = 80;
////		
////	}
//	//else return 0;		
//}

void Buzzer_task_h(uint16_t a,uint16_t b,uint16_t c)
{
	TIM4->CCR3 = a;
	vTaskDelay(b);
	TIM4->CCR3 = 0;
	vTaskDelay(c);
}



/*�����������*/
void motor_gfk_c(void)
{
	varible_chassis.send_chassis_speed[0] *=monitor.chassis_kgf[0];
	varible_chassis.send_chassis_speed[1] *=monitor.chassis_kgf[1];
	varible_chassis.send_chassis_speed[2] *=monitor.chassis_kgf[2];
	varible_chassis.send_chassis_speed[3] *=monitor.chassis_kgf[3];	
}
void motor_gfk_g(void)
{
	pid_yaw.pid_out_speed *= monitor.yaw_kgf;
	pid_shoot[0].pid_out_speed *= monitor.shoot_kgf[0];
	pid_shoot[1].pid_out_speed *= monitor.shoot_kgf[1];
	pid_send.pid_out_speed *= monitor.send_kgf;
	pid_pitch.pid_out_speed *= monitor.pitch_kgf;
	
}


/*
	����3508���
	�������͵����*/
void Motor_current_send_chassis(void)
{
	//motor_gfk_c();
	SET_MOTOR_CURRENT_CAN1_1234(&hcan1,varible_chassis.send_chassis_speed[0],
											varible_chassis.send_chassis_speed[1],
											varible_chassis.send_chassis_speed[2],
											varible_chassis.send_chassis_speed[3]);
}



void  motor_current_send_gimbal(void)
{
	//motor_gfk_g();
	SET_MOTOR_CURRENT_CAN1_5678(&hcan1,	pid_yaw.pid_out_speed,0,0,0 );	
	SET_MOTOR_CURRENT_CAN2_5678(&hcan2,pid_pitch.pid_out_speed,pid_shoot[0].pid_out_speed ,pid_shoot[1].pid_out_speed ,pid_send.pid_out_speed);
	SET_MOTOR_CURRENT_CAN2_1234(&hcan2,0,0,0,0);
}


























