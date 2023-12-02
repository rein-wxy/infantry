#ifndef __REFEREE_H
#define __REFEREE_H	 

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "string.h"
#include "key_task.h"
#include "all_task.h"

/*define*/
#define U8	unsigned char
#define U16	unsigned short int
#define U32	unsigned int
#define BLUE  0
#define RED   1

/*	比赛状态数据	*/
/*	0x0001	发送频率：1Hz*/
typedef __packed struct
{
	uint8_t game_type : 4;					//比赛类型
	uint8_t game_progress : 4;			//当前比赛阶段
	uint16_t stage_remain_time;			//当前剩余时间
	uint64_t SyncTimeStamp;					//机器人接收到该指令的精确Unix时间
}ext_game_status_t;

/*比赛结果数据*/
/*	0x0002	发送频率：比赛结束后发送*/
typedef __packed struct
{
	uint8_t winner;									//0平局	1红方胜利	2蓝方胜利
}ext_game_result_t;

/*机器热血量数据*/
/*	0x0003	发送频率：1Hz*/
typedef __packed struct
{
	uint16_t red_1_robot_HP;				//红1
	uint16_t red_2_robot_HP; 				//红2
	uint16_t red_3_robot_HP;  			//红3
	uint16_t red_4_robot_HP;   			//红4
	uint16_t red_5_robot_HP;   			//红5
	uint16_t red_7_robot_HP;   			//红7
	uint16_t red_outpost_HP;  			//红前哨站
	uint16_t red_base_HP; 					//红基地
	uint16_t blue_1_robot_HP; 			//蓝1
	uint16_t blue_2_robot_HP;  			//蓝2
	uint16_t blue_3_robot_HP;  			//蓝3
	uint16_t blue_4_robot_HP;  			//蓝4
	uint16_t blue_5_robot_HP;  			//蓝5
	uint16_t blue_7_robot_HP;  			//蓝7
	uint16_t blue_outpost_HP; 			//蓝前哨战
	uint16_t blue_base_HP;		 			//蓝基地
}ext_game_robot_HP_t;

/*飞镖发射状态*/
/*	0x0004	发送频率：飞镖发射后发送至所有机器人	*/
typedef __packed struct
{
	uint8_t dart_belong; 						//1红方飞镖	2蓝方飞镖
	uint16_t stage_remaining_time; 	//发射时的剩余比赛时间/s
}ext_dart_status_t;

/*场地事件数据*/
/*	0x0101	发送频率：事件改变后发送*/
typedef __packed struct
{
	uint32_t event_type;
}ext_event_data_t;

/*补给站动作标识*/
/*	0x0102	发送频率：动作改变后发送至己方机器人*/
typedef __packed struct
{
	uint8_t supply_projectile_id; 		//补给站ID
	uint8_t supply_robot_id; 					//补弹机器人ID
	uint8_t supply_projectile_step; 	//出弹口开闭状态 0关闭	1准备中	2子弹下落
	uint8_t supply_projectile_num;		//补弹数量	50~100~150~200
}ext_supply_projectile_action_t;

/*裁判警告信息*/
/*	0x0104	发送频率：警告后发送*/
typedef __packed struct
{
	uint8_t level;										//警告等级	1黄牌	2红牌	3判负
	uint8_t foul_robot_id; 						//犯规机器人ID	
}ext_referee_warning_t;

/*飞镖发射口倒计时*/
/*	0x0105	发射频率：1Hz	发送范围：己方机器人*/
typedef __packed struct
{
	uint8_t dart_remaining_time;			//15s倒计时
}ext_dart_remaining_time_t;

/*比赛机器人状态*/
/*	0x0201	发送频率：10Hz*/
typedef __packed struct
{
	uint8_t robot_id;													//本机器人ID
	uint8_t robot_level;											//机器人等级
	uint16_t remain_HP;												//机器人剩余血量
	uint16_t max_HP;													//机器人血量上限
	uint16_t shooter_id1_17mm_cooling_rate;		//1号机器人17mm枪口每秒冷却值
	uint16_t shooter_id1_17mm_cooling_limit;	//1号机器人17mm枪口热量上限
	uint16_t shooter_id1_17mm_speed_limit;		//1号机器人17mm枪口速度上限 m/s
	uint16_t shooter_id2_17mm_cooling_rate;		//2号机器人17mm枪口每秒冷却值
	uint16_t shooter_id2_17mm_cooling_limit;	//2号机器人17mm枪口热量上限
	uint16_t shooter_id2_17mm_speed_limit;		//2号机器人17mm枪口速度上限 m/s
	uint16_t shooter_id1_42mm_cooling_rate;		//机器人42mm枪口每秒冷却值
	uint16_t shooter_id1_42mm_cooling_limit;	//机器人42mm枪口热量上限
	uint16_t shooter_id1_42mm_speed_limit;		//机器人42mm枪口速度上限 m/s
	uint16_t chassis_power_limit;							//机器人底盘功率限制上限
	uint8_t mains_power_gimbal_output : 1;		//gimbal口输出	1为有24V输出	0为无24V输出
	uint8_t mains_power_chassis_output : 1;		//chassis口输出	1为有24V输出	0为无24V输出
	uint8_t mains_power_shooter_output : 1;		//shooter口输出	1为有24V输出	0为无24V输出
}ext_game_robot_status_t;

/*实时功率热量数据*/
/*	0x0202	发送频率：50Hz*/
typedef __packed struct
{
	uint16_t chassis_volt; 										//底盘输出电压	mV
	uint16_t chassis_current; 								//底盘输出电流	mA
	float chassis_power; 											//底盘输出功率	W
	uint16_t chassis_power_buffer; 						//底盘功率缓冲	J
	uint16_t shooter_id1_17mm_cooling_heat;		//1号17mm枪口热量
	uint16_t shooter_id2_17mm_cooling_heat;		//2号17mm枪口热量
	uint16_t shooter_id1_42mm_cooling_heat;		//42mm枪口热量
}ext_power_heat_data_t;

/*机器人位置*/
/* 0x0203	发送频率：10Hz*/
typedef __packed struct
{
	float x;																	//位置X坐标	m
	float y;																	//位置Y坐标	m
	float z;																	//位置Z坐标	m
	float yaw;																//位置枪口	度
}ext_game_robot_pos_t;

/*机器人增益*/
/* 0x0204	发送频率：1Hz*/
typedef __packed struct
{
	uint8_t power_rune_buff;
}ext_buff_t;

/*空中机器人能量状态*/
/*	0x0205 发送频率：10Hz*/
typedef __packed struct
{
	uint8_t attack_time;											//可攻击时间	s 30s~0s
}aerial_robot_energy_t;

/*伤害状态*/
/* 0x0206	发送频率：伤害发生后发生*/
typedef __packed struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
}ext_robot_hurt_t;

/*实时射击信息*/
/*	0x0207	发生频率：射击后发送*/
typedef __packed struct
{
	uint8_t bullet_type;						//子弹类型 1：17mm 2:	42mm
	uint8_t shooter_id;							//发射机构ID	
	uint8_t bullet_freq;						//子弹射频
	float bullet_speed;							//子弹射速
}ext_shoot_data_t;

/*子弹剩余发射数*/
/* 0x0208	发送频率：10Hz	所有机器人发送*/
typedef __packed struct
{
	uint16_t bullet_remaining_num_17mm;			//17mm子弹剩余发射数目
	uint16_t bullet_remaining_num_42mm;			//42mm子弹剩余发射数目
	uint16_t coin_remaining_num;						//剩余金币数量
}ext_bullet_remaining_t;

/*机器人RFID状态*/
/* 0x0209	发送频率：1Hz 发送范围：单一机器人*/
typedef __packed struct
{
	uint32_t rfid_status;
}ext_rfid_status_t;

/*飞镖机器人客户端指令数据*/
/*	0x020A 发送频率：10Hz	发送范围：单一机器人*/
typedef __packed struct
{
	uint8_t dart_launch_opening_status;				//当前飞镖发射状态
	uint8_t dart_attack_target;								//飞镖打击目标
	uint16_t target_change_time;
	uint8_t first_dart_speed;
	uint8_t second_dart_speed;
	uint8_t third_dart_speed;
	uint8_t fourth_dart_speed;
	uint16_t last_dart_launch_time;
	uint16_t operate_launch_cmd_time;
}ext_dart_client_cmd_t;

/*交互数据接收信息*/
/*	0x0301	*/
typedef __packed struct
{
	uint16_t data_cmd_id;										//数据段内容ID
	uint16_t sender_ID;											//发送者的ID
	uint16_t receiver_ID;										//接收者的ID
}ext_student_interactive_header_data_t;		//内容数据段

/*交互数据	机器人间通讯*/
/*0x0301*/
typedef __packed struct
{
	uint8_t data[112];
}robot_interactive_data_t;

/*客户端删除图形	机器人间通讯*/
/* 0x0301 */
typedef __packed struct
{
	uint8_t operate_tpye; 
	uint8_t layer; 
}ext_client_custom_graphic_delete_t;

/*图形数据*/
typedef __packed struct
{ 
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; 
	uint32_t layer:4; 
	uint32_t color:4; 
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10; 
	uint32_t start_x:11; 
	uint32_t start_y:11; 
	uint32_t radius:10; 
	uint32_t end_x:11; 
	uint32_t end_y:11; 
}graphic_data_struct_t;

/*客户端绘制一个图形*/
/*	0x0301	*/
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
}ext_client_custom_graphic_single_t;

/*客户端绘制二个图形*/
/*	0x0301	*/
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[2];
}ext_client_custom_graphic_double_t;

/*客户端绘制五个图形*/
/*	0x0301	*/
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[5];
}ext_client_custom_graphic_five_t;

/*客户端绘制字符*/
/*	0x0301	*/
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
}ext_client_custom_character_t;

/*客户端绘制七个图形*/
/*	0x0301	*/
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[7];
}ext_client_custom_graphic_seven_t;

/*交互数据接收信息*/
/*	0x0302	发送频率：上限30Hz*/
typedef __packed struct
{
	uint8_t data[30];
}ext_robot_interactive_data_t;

/*客户端下发信息*/
/*	0x0303	*/
typedef __packed struct
{
	float target_position_x;
	float target_position_y;
	float target_position_z;
	uint8_t commd_keyboard;
	uint16_t target_robot_ID;
}ext_robot_command_t;

/*客户端下发信息*/
typedef __packed struct
{
int16_t mouse_x;
int16_t mouse_y;
int16_t mouse_z;
int8_t left_button_down;
int8_t right_button_down;
uint16_t keyboard_value;
uint16_t reserved;
}ext_robot_command_t_1;

/*帧头部分*/
typedef __packed struct
{
	U8 SOF;
	U16 DataLength;
	U8 Seq;
	U8 CRC8;
}frame_header_t;

/*ID*/
typedef __packed struct   
{
  U8 Low;
  U8 High;
}rxCmdId_rx_t;

typedef union
{
    uint8_t U[4];
    float F;
    int I;
}FormatTrans;
/*当前机器人相关ID信息*/
typedef struct {
	uint8_t robot_colour;   //0blue,1red
	uint8_t robot_ID;
	uint16_t robot_client_ID;
}The_robot_ID_data;
/*划线5_1*/
typedef __packed struct
{
	frame_header_t	Frameheader;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t Client_Custom_ID;  
	graphic_data_struct_t grapic_data_struct[5];
	uint16_t crc16;
}ext_draw_line_5_1;
/*划线5_2*/
typedef __packed struct
{
	frame_header_t	Frameheader;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t Client_Custom_ID;  
	graphic_data_struct_t grapic_data_struct[5];
	uint16_t crc16;
}ext_draw_line_5_2;
/*划线7*/
typedef __packed struct
{
	frame_header_t	Frameheader;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t Client_Custom_ID;  
	graphic_data_struct_t grapic_data_struct[7];
	uint16_t crc16;
}ext_draw_line_7;
/*字符*/
typedef __packed struct
{
	frame_header_t	Frameheader;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t Client_Custom_ID;  
	graphic_data_struct_t grapic_data_struct;
	char data[30];
	uint16_t crc16;
}ext_draw_char;
/*清理*/
typedef __packed struct
{
	frame_header_t	Frameheader;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t Client_Custom_ID;  
	graphic_data_struct_t grapic_data_struct;
	uint16_t crc16;
}ext_ui_clear;
/*机器人数据*/
typedef __packed struct
{
	frame_header_t	Frameheader;															//帧头
	uint16_t cmd_id;																					//cmd_id 命令字符
	ext_student_interactive_header_data_t Client_Custom_ID;  	//交互数据信息
	robot_interactive_data_t robot_interactive_data;					//交互数据	机器人间通讯				
	uint16_t crc16;
}ext_robot_tx;



/*裁判系统结构体集合*/
typedef __packed struct
{
	ext_game_status_t extgame_status_t;
	ext_game_result_t extgame_result_t;
	ext_game_robot_HP_t extgame_robot_HP_t;
	ext_dart_status_t extdart_status_t;
	ext_event_data_t extevent_data_t;
	ext_supply_projectile_action_t extsupply_projectile_action_t;
	ext_referee_warning_t extreferee_warning_t;
	ext_dart_remaining_time_t extdart_remaining_time_t;
	ext_game_robot_status_t extgame_robot_status_t;
	ext_power_heat_data_t extpower_heat_data_t;
	ext_game_robot_pos_t extgame_robot_pos_t;
	ext_buff_t extbuff_t;
	
	aerial_robot_energy_t aerialrobot_energy_t;
	ext_robot_hurt_t extrobot_hurt_t;
	ext_shoot_data_t extshoot_data_t;
	ext_bullet_remaining_t extbullet_remaining_t;
	ext_rfid_status_t extrfid_status_t;
	ext_dart_client_cmd_t extdart_client_cmd_t;
	ext_student_interactive_header_data_t extstudent_interactive_header_data_t;
	robot_interactive_data_t robotinteractive_data_t;
	ext_client_custom_graphic_delete_t extclient_custom_graphic_delete_t;
	graphic_data_struct_t graphicdata_struct_t;
	ext_client_custom_graphic_single_t extclient_custom_graphic_single_t;
	ext_client_custom_graphic_double_t extclient_custom_graphic_double_t;
	ext_client_custom_graphic_five_t extclient_custom_graphic_five_t;
	ext_client_custom_character_t extclient_custom_character_t;
	ext_client_custom_graphic_seven_t extclient_custom_graphic_seven_t;
	ext_robot_interactive_data_t extrobot_interactive_data_t;
	ext_robot_command_t ext_robot_command_t;
	ext_robot_command_t_1 extrobot_command_t_1;
	frame_header_t frameheader_t;
	rxCmdId_rx_t rxCmdId_rx;
}Referee_system_data_t;








extern uint8_t uart6_rx_buff[200];
extern uint8_t uart6_tx_buff[200];
extern Referee_system_data_t Referee_system_data;
extern The_robot_ID_data Robot_ID_Data;

/*掉线检测*/



void cmd_rc_callback_handler(void);		//接收数据
void judgeCalculate(uint8_t *data);		//解析数据
int Red_or_Blue_judge(void);/*红蓝阵营判断*/
void Robot_client_ID(void);						//客户端ID
void get_chassis_power_and_buffer(float *power, float *buffer);
void get_chassis_id_maxpower_maxbuffer(uint8_t *ID,float *max_power,uint8_t *robot_level);



unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint32_t Verify_CRC16_Check_Sum(uint8_t    *pchMessage, uint32_t    dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);


void Robot_client_ID(void);/*客户端ID*/


#endif
