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

/*	����״̬����	*/
/*	0x0001	����Ƶ�ʣ�1Hz*/
typedef __packed struct
{
	uint8_t game_type : 4;					//��������
	uint8_t game_progress : 4;			//��ǰ�����׶�
	uint16_t stage_remain_time;			//��ǰʣ��ʱ��
	uint64_t SyncTimeStamp;					//�����˽��յ���ָ��ľ�ȷUnixʱ��
}ext_game_status_t;

/*�����������*/
/*	0x0002	����Ƶ�ʣ�������������*/
typedef __packed struct
{
	uint8_t winner;									//0ƽ��	1�췽ʤ��	2����ʤ��
}ext_game_result_t;

/*������Ѫ������*/
/*	0x0003	����Ƶ�ʣ�1Hz*/
typedef __packed struct
{
	uint16_t red_1_robot_HP;				//��1
	uint16_t red_2_robot_HP; 				//��2
	uint16_t red_3_robot_HP;  			//��3
	uint16_t red_4_robot_HP;   			//��4
	uint16_t red_5_robot_HP;   			//��5
	uint16_t red_7_robot_HP;   			//��7
	uint16_t red_outpost_HP;  			//��ǰ��վ
	uint16_t red_base_HP; 					//�����
	uint16_t blue_1_robot_HP; 			//��1
	uint16_t blue_2_robot_HP;  			//��2
	uint16_t blue_3_robot_HP;  			//��3
	uint16_t blue_4_robot_HP;  			//��4
	uint16_t blue_5_robot_HP;  			//��5
	uint16_t blue_7_robot_HP;  			//��7
	uint16_t blue_outpost_HP; 			//��ǰ��ս
	uint16_t blue_base_HP;		 			//������
}ext_game_robot_HP_t;

/*���ڷ���״̬*/
/*	0x0004	����Ƶ�ʣ����ڷ�����������л�����	*/
typedef __packed struct
{
	uint8_t dart_belong; 						//1�췽����	2��������
	uint16_t stage_remaining_time; 	//����ʱ��ʣ�����ʱ��/s
}ext_dart_status_t;

/*�����¼�����*/
/*	0x0101	����Ƶ�ʣ��¼��ı����*/
typedef __packed struct
{
	uint32_t event_type;
}ext_event_data_t;

/*����վ������ʶ*/
/*	0x0102	����Ƶ�ʣ������ı����������������*/
typedef __packed struct
{
	uint8_t supply_projectile_id; 		//����վID
	uint8_t supply_robot_id; 					//����������ID
	uint8_t supply_projectile_step; 	//�����ڿ���״̬ 0�ر�	1׼����	2�ӵ�����
	uint8_t supply_projectile_num;		//��������	50~100~150~200
}ext_supply_projectile_action_t;

/*���о�����Ϣ*/
/*	0x0104	����Ƶ�ʣ��������*/
typedef __packed struct
{
	uint8_t level;										//����ȼ�	1����	2����	3�и�
	uint8_t foul_robot_id; 						//���������ID	
}ext_referee_warning_t;

/*���ڷ���ڵ���ʱ*/
/*	0x0105	����Ƶ�ʣ�1Hz	���ͷ�Χ������������*/
typedef __packed struct
{
	uint8_t dart_remaining_time;			//15s����ʱ
}ext_dart_remaining_time_t;

/*����������״̬*/
/*	0x0201	����Ƶ�ʣ�10Hz*/
typedef __packed struct
{
	uint8_t robot_id;													//��������ID
	uint8_t robot_level;											//�����˵ȼ�
	uint16_t remain_HP;												//������ʣ��Ѫ��
	uint16_t max_HP;													//������Ѫ������
	uint16_t shooter_id1_17mm_cooling_rate;		//1�Ż�����17mmǹ��ÿ����ȴֵ
	uint16_t shooter_id1_17mm_cooling_limit;	//1�Ż�����17mmǹ����������
	uint16_t shooter_id1_17mm_speed_limit;		//1�Ż�����17mmǹ���ٶ����� m/s
	uint16_t shooter_id2_17mm_cooling_rate;		//2�Ż�����17mmǹ��ÿ����ȴֵ
	uint16_t shooter_id2_17mm_cooling_limit;	//2�Ż�����17mmǹ����������
	uint16_t shooter_id2_17mm_speed_limit;		//2�Ż�����17mmǹ���ٶ����� m/s
	uint16_t shooter_id1_42mm_cooling_rate;		//������42mmǹ��ÿ����ȴֵ
	uint16_t shooter_id1_42mm_cooling_limit;	//������42mmǹ����������
	uint16_t shooter_id1_42mm_speed_limit;		//������42mmǹ���ٶ����� m/s
	uint16_t chassis_power_limit;							//�����˵��̹�����������
	uint8_t mains_power_gimbal_output : 1;		//gimbal�����	1Ϊ��24V���	0Ϊ��24V���
	uint8_t mains_power_chassis_output : 1;		//chassis�����	1Ϊ��24V���	0Ϊ��24V���
	uint8_t mains_power_shooter_output : 1;		//shooter�����	1Ϊ��24V���	0Ϊ��24V���
}ext_game_robot_status_t;

/*ʵʱ������������*/
/*	0x0202	����Ƶ�ʣ�50Hz*/
typedef __packed struct
{
	uint16_t chassis_volt; 										//���������ѹ	mV
	uint16_t chassis_current; 								//�����������	mA
	float chassis_power; 											//�����������	W
	uint16_t chassis_power_buffer; 						//���̹��ʻ���	J
	uint16_t shooter_id1_17mm_cooling_heat;		//1��17mmǹ������
	uint16_t shooter_id2_17mm_cooling_heat;		//2��17mmǹ������
	uint16_t shooter_id1_42mm_cooling_heat;		//42mmǹ������
}ext_power_heat_data_t;

/*������λ��*/
/* 0x0203	����Ƶ�ʣ�10Hz*/
typedef __packed struct
{
	float x;																	//λ��X����	m
	float y;																	//λ��Y����	m
	float z;																	//λ��Z����	m
	float yaw;																//λ��ǹ��	��
}ext_game_robot_pos_t;

/*����������*/
/* 0x0204	����Ƶ�ʣ�1Hz*/
typedef __packed struct
{
	uint8_t power_rune_buff;
}ext_buff_t;

/*���л���������״̬*/
/*	0x0205 ����Ƶ�ʣ�10Hz*/
typedef __packed struct
{
	uint8_t attack_time;											//�ɹ���ʱ��	s 30s~0s
}aerial_robot_energy_t;

/*�˺�״̬*/
/* 0x0206	����Ƶ�ʣ��˺���������*/
typedef __packed struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
}ext_robot_hurt_t;

/*ʵʱ�����Ϣ*/
/*	0x0207	����Ƶ�ʣ��������*/
typedef __packed struct
{
	uint8_t bullet_type;						//�ӵ����� 1��17mm 2:	42mm
	uint8_t shooter_id;							//�������ID	
	uint8_t bullet_freq;						//�ӵ���Ƶ
	float bullet_speed;							//�ӵ�����
}ext_shoot_data_t;

/*�ӵ�ʣ�෢����*/
/* 0x0208	����Ƶ�ʣ�10Hz	���л����˷���*/
typedef __packed struct
{
	uint16_t bullet_remaining_num_17mm;			//17mm�ӵ�ʣ�෢����Ŀ
	uint16_t bullet_remaining_num_42mm;			//42mm�ӵ�ʣ�෢����Ŀ
	uint16_t coin_remaining_num;						//ʣ��������
}ext_bullet_remaining_t;

/*������RFID״̬*/
/* 0x0209	����Ƶ�ʣ�1Hz ���ͷ�Χ����һ������*/
typedef __packed struct
{
	uint32_t rfid_status;
}ext_rfid_status_t;

/*���ڻ����˿ͻ���ָ������*/
/*	0x020A ����Ƶ�ʣ�10Hz	���ͷ�Χ����һ������*/
typedef __packed struct
{
	uint8_t dart_launch_opening_status;				//��ǰ���ڷ���״̬
	uint8_t dart_attack_target;								//���ڴ��Ŀ��
	uint16_t target_change_time;
	uint8_t first_dart_speed;
	uint8_t second_dart_speed;
	uint8_t third_dart_speed;
	uint8_t fourth_dart_speed;
	uint16_t last_dart_launch_time;
	uint16_t operate_launch_cmd_time;
}ext_dart_client_cmd_t;

/*�������ݽ�����Ϣ*/
/*	0x0301	*/
typedef __packed struct
{
	uint16_t data_cmd_id;										//���ݶ�����ID
	uint16_t sender_ID;											//�����ߵ�ID
	uint16_t receiver_ID;										//�����ߵ�ID
}ext_student_interactive_header_data_t;		//�������ݶ�

/*��������	�����˼�ͨѶ*/
/*0x0301*/
typedef __packed struct
{
	uint8_t data[112];
}robot_interactive_data_t;

/*�ͻ���ɾ��ͼ��	�����˼�ͨѶ*/
/* 0x0301 */
typedef __packed struct
{
	uint8_t operate_tpye; 
	uint8_t layer; 
}ext_client_custom_graphic_delete_t;

/*ͼ������*/
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

/*�ͻ��˻���һ��ͼ��*/
/*	0x0301	*/
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
}ext_client_custom_graphic_single_t;

/*�ͻ��˻��ƶ���ͼ��*/
/*	0x0301	*/
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[2];
}ext_client_custom_graphic_double_t;

/*�ͻ��˻������ͼ��*/
/*	0x0301	*/
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[5];
}ext_client_custom_graphic_five_t;

/*�ͻ��˻����ַ�*/
/*	0x0301	*/
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
}ext_client_custom_character_t;

/*�ͻ��˻����߸�ͼ��*/
/*	0x0301	*/
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[7];
}ext_client_custom_graphic_seven_t;

/*�������ݽ�����Ϣ*/
/*	0x0302	����Ƶ�ʣ�����30Hz*/
typedef __packed struct
{
	uint8_t data[30];
}ext_robot_interactive_data_t;

/*�ͻ����·���Ϣ*/
/*	0x0303	*/
typedef __packed struct
{
	float target_position_x;
	float target_position_y;
	float target_position_z;
	uint8_t commd_keyboard;
	uint16_t target_robot_ID;
}ext_robot_command_t;

/*�ͻ����·���Ϣ*/
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

/*֡ͷ����*/
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
/*��ǰ���������ID��Ϣ*/
typedef struct {
	uint8_t robot_colour;   //0blue,1red
	uint8_t robot_ID;
	uint16_t robot_client_ID;
}The_robot_ID_data;
/*����5_1*/
typedef __packed struct
{
	frame_header_t	Frameheader;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t Client_Custom_ID;  
	graphic_data_struct_t grapic_data_struct[5];
	uint16_t crc16;
}ext_draw_line_5_1;
/*����5_2*/
typedef __packed struct
{
	frame_header_t	Frameheader;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t Client_Custom_ID;  
	graphic_data_struct_t grapic_data_struct[5];
	uint16_t crc16;
}ext_draw_line_5_2;
/*����7*/
typedef __packed struct
{
	frame_header_t	Frameheader;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t Client_Custom_ID;  
	graphic_data_struct_t grapic_data_struct[7];
	uint16_t crc16;
}ext_draw_line_7;
/*�ַ�*/
typedef __packed struct
{
	frame_header_t	Frameheader;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t Client_Custom_ID;  
	graphic_data_struct_t grapic_data_struct;
	char data[30];
	uint16_t crc16;
}ext_draw_char;
/*����*/
typedef __packed struct
{
	frame_header_t	Frameheader;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t Client_Custom_ID;  
	graphic_data_struct_t grapic_data_struct;
	uint16_t crc16;
}ext_ui_clear;
/*����������*/
typedef __packed struct
{
	frame_header_t	Frameheader;															//֡ͷ
	uint16_t cmd_id;																					//cmd_id �����ַ�
	ext_student_interactive_header_data_t Client_Custom_ID;  	//����������Ϣ
	robot_interactive_data_t robot_interactive_data;					//��������	�����˼�ͨѶ				
	uint16_t crc16;
}ext_robot_tx;



/*����ϵͳ�ṹ�弯��*/
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

/*���߼��*/



void cmd_rc_callback_handler(void);		//��������
void judgeCalculate(uint8_t *data);		//��������
int Red_or_Blue_judge(void);/*������Ӫ�ж�*/
void Robot_client_ID(void);						//�ͻ���ID
void get_chassis_power_and_buffer(float *power, float *buffer);
void get_chassis_id_maxpower_maxbuffer(uint8_t *ID,float *max_power,uint8_t *robot_level);



unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint32_t Verify_CRC16_Check_Sum(uint8_t    *pchMessage, uint32_t    dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);


void Robot_client_ID(void);/*�ͻ���ID*/


#endif
