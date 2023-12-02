#ifndef __FUNCTION_LIB_H
#define __FUNCTION_LIB_H	 
/*��*/
#include "chassis.h"
#include "math.h"
#include "cmsis_os.h"

#define abs(x) 			((x)>0? (x):(-(x)))    //��������ֵ fabs��������������ֵ
#define constrain(x, min, max)	((x>max)?max:(x<min?min:x))  //�޷�




/*б�º����ṹ��*/
typedef __packed struct             //__packed ȡ���ֽڶ��� ->��С�ֽڡ�����ϵͳ��ȡ
{
	float value;
  float input;						//��������
  float out;							//�������
  float min_value;				//�޷���Сֵ
  float max_value;				//�޷����ֵ
  float frame_period; 		//ʱ����
} ramp_function_source_t;

/*б��3.0*/
typedef struct
{
	double remp_num_now;						//б�µ�ǰ��ֵ
	double remp_num_target;				//б��Ŀ����ֵ
}ramp_t;




/*�����ظ����ýṹ��*/
typedef struct {
	uint8_t i;			//��־λ
	uint16_t cnt;		//�ƴ�
}key_repetition_t;


/*һ�׵�ͨ�˲������ṹ��*/
typedef __packed struct
{
    float input;        	//��������
    float out;						//�˲����������
    float num;						//�˲�����
    float frame_period; 	//�˲���ʱ���� ��λ s
	float last_out;
} first_order_filter_type_t;



/*�����˲�*/
typedef struct 
{
	float num[100];
	uint8_t lenth;
	uint8_t pot;//��ǰλ��
	float total;
	float aver_num;			//���
}moving_Average_Filter;	//�������MAF_MaxSize��
typedef struct
{
    uint16_t nowLength;
    uint16_t queueLength;
    float queueTotal;
    //����
    float queue[100];
    //ָ��
    float aver_num;//ƽ��ֵ

    float Diff;//���ֵ

    uint8_t full_flag;
}QueueObj;


/*���߼��ṹ��*/
typedef struct {
	uint8_t flag;			//�Ƿ�����жϱ�־λ
	uint8_t FLAG;			//�Ƿ����
	int sum;					//����ʱ���жϣ���ֹ���У�
	uint64_t TIME1;		//ʱ����1
	uint64_t TIME2;		//ʱ����2
}online;





extern ramp_t top_ramp;         //С����б��
extern ramp_t chassis_x_ramp;   //����б��
extern ramp_t chassis_y_ramp;
extern ramp_t chassis_max_speed_ramp;   //��������ٶ�б��
extern first_order_filter_type_t mouse_x_first;
extern first_order_filter_type_t mouse_y_first;
extern moving_Average_Filter mouse_x_moving;
extern moving_Average_Filter mouse_y_moving;
extern key_repetition_t key_repetition_B;
extern moving_Average_Filter vision_distance;

/*�������ٶ���Ư����*/
float imu_speed_deal(int i,int j,float speed);
void Buzzer_task(uint16_t buzzer,uint16_t time1,uint16_t time2);/*����������*/
void Buzzer_task_h(uint16_t a,uint16_t b,uint16_t c);      //������
uint8_t my_delay(uint16_t a,uint8_t b);    //��ʱ

void Slow_clear(ramp_t *ramp);   //б������
void Slow(double *rec , double *target , float slow_Inc , float in_rate , float out_rate);//б�º���
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num);
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
void average_init(moving_Average_Filter *Aver, uint8_t lenth);
void average_add(moving_Average_Filter *Aver, float add_data);
float Get_Diff(uint8_t queue_len, QueueObj *Data,float add_data);
void Clear_Queue(QueueObj* queue);
int key_repetition_2(key_repetition_t* K,uint16_t key, uint16_t key_target);  //�������δ���

void on_online(online* ol,int time,int SUM);/*���߼��*/




void Motor_current_send_chassis(void);     /*�������͵����*/
void motor_current_send_gimbal(void);
void motor_gfk_c(void);
void motor_gfk_g(void);



#endif



























