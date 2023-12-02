#ifndef __FUNCTION_LIB_H
#define __FUNCTION_LIB_H	 
/*抄*/
#include "chassis.h"
#include "math.h"
#include "cmsis_os.h"

#define abs(x) 			((x)>0? (x):(-(x)))    //整数绝对值 fabs（）浮点数绝对值
#define constrain(x, min, max)	((x>max)?max:(x<min?min:x))  //限幅




/*斜坡函数结构体*/
typedef __packed struct             //__packed 取消字节对齐 ->减小字节、方便系统读取
{
	float value;
  float input;						//输入数据
  float out;							//输出数据
  float min_value;				//限幅最小值
  float max_value;				//限幅最大值
  float frame_period; 		//时间间隔
} ramp_function_source_t;

/*斜坡3.0*/
typedef struct
{
	double remp_num_now;						//斜坡当前数值
	double remp_num_target;				//斜坡目标数值
}ramp_t;




/*按键重复调用结构体*/
typedef struct {
	uint8_t i;			//标志位
	uint16_t cnt;		//计次
}key_repetition_t;


/*一阶低通滤波函数结构体*/
typedef __packed struct
{
    float input;        	//输入数据
    float out;						//滤波输出的数据
    float num;						//滤波参数
    float frame_period; 	//滤波的时间间隔 单位 s
	float last_out;
} first_order_filter_type_t;



/*滑动滤波*/
typedef struct 
{
	float num[100];
	uint8_t lenth;
	uint8_t pot;//当前位置
	float total;
	float aver_num;			//输出
}moving_Average_Filter;	//最大设置MAF_MaxSize个
typedef struct
{
    uint16_t nowLength;
    uint16_t queueLength;
    float queueTotal;
    //长度
    float queue[100];
    //指针
    float aver_num;//平均值

    float Diff;//差分值

    uint8_t full_flag;
}QueueObj;


/*掉线检测结构体*/
typedef struct {
	uint8_t flag;			//是否进入中断标志位
	uint8_t FLAG;			//是否掉线
	int sum;					//掉线时间判断（防止误判）
	uint64_t TIME1;		//时间间隔1
	uint64_t TIME2;		//时间间隔2
}online;





extern ramp_t top_ramp;         //小陀螺斜坡
extern ramp_t chassis_x_ramp;   //键盘斜坡
extern ramp_t chassis_y_ramp;
extern ramp_t chassis_max_speed_ramp;   //底盘最大速度斜坡
extern first_order_filter_type_t mouse_x_first;
extern first_order_filter_type_t mouse_y_first;
extern moving_Average_Filter mouse_x_moving;
extern moving_Average_Filter mouse_y_moving;
extern key_repetition_t key_repetition_B;
extern moving_Average_Filter vision_distance;

/*陀螺仪速度零漂处理*/
float imu_speed_deal(int i,int j,float speed);
void Buzzer_task(uint16_t buzzer,uint16_t time1,uint16_t time2);/*蜂鸣器任务*/
void Buzzer_task_h(uint16_t a,uint16_t b,uint16_t c);      //蜂鸣器
uint8_t my_delay(uint16_t a,uint8_t b);    //延时

void Slow_clear(ramp_t *ramp);   //斜坡清理
void Slow(double *rec , double *target , float slow_Inc , float in_rate , float out_rate);//斜坡函数
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num);
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
void average_init(moving_Average_Filter *Aver, uint8_t lenth);
void average_add(moving_Average_Filter *Aver, float add_data);
float Get_Diff(uint8_t queue_len, QueueObj *Data,float add_data);
void Clear_Queue(QueueObj* queue);
int key_repetition_2(key_repetition_t* K,uint16_t key, uint16_t key_target);  //按键两次触发

void on_online(online* ol,int time,int SUM);/*掉线检测*/




void Motor_current_send_chassis(void);     /*电流发送到电调*/
void motor_current_send_gimbal(void);
void motor_gfk_c(void);
void motor_gfk_g(void);



#endif



























