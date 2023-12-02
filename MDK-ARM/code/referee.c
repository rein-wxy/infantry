#include "referee.h"

uint8_t uart6_rx_buff[200];								//接收数据缓冲
uint8_t uart6_tx_buff[200];								//发送数据缓冲
uint16_t u6_cmd_data;											//命令码ID
/*总结构体(裁判系统接收数据)*/
Referee_system_data_t Referee_system_data;
/*当前机器人相关ID*/
The_robot_ID_data Robot_ID_Data;



/*掉线检测*/





/*
		波特率 115200
		数据为 8
		停止位 1
		无校验
*/
/*  	通讯格式  	*/
/* 		 frame_header (5-byte) + cmd_id (2-byte) +  data (n-byte) + frame_tail (2-byte,CRC16,整包校验) 			*/
		
/*		SOF			data_length			seq			CRC8
			1-byte	2-byte					1-byte	1-byte
			0xA5		data长度				包序号	CRC8校验			*/


/*接收数据*/
void cmd_rc_callback_handler(void)
{
	monitor.referee_cnt ++;                   // 监测任务标志
	/*数据接收*/
	static uint16_t start_pos=0,next_start_pos=0;//start_pos 起始位
	while(1)
	{
		memcpy(&Referee_system_data.frameheader_t, &uart6_rx_buff[start_pos],5);
		/*校验正确*/
		if((Referee_system_data.frameheader_t.SOF == (uint16_t)0xA5)													//帧头第一字节校验
			&&(Verify_CRC8_Check_Sum(&uart6_rx_buff[start_pos],5) == 1)													//CRC8头校验
			&&(Verify_CRC16_Check_Sum(&uart6_rx_buff[start_pos], Referee_system_data.frameheader_t.DataLength+9)) == 1)		//CRC16整包校验
		{
			memcpy(&Referee_system_data.rxCmdId_rx, (&uart6_rx_buff[start_pos]+5), sizeof(Referee_system_data.rxCmdId_rx));		//将命令码ID复制到相应结构体
			u6_cmd_data = (uint16_t)Referee_system_data.rxCmdId_rx.High << 8 | Referee_system_data.rxCmdId_rx.Low;								//命令码ID
			uart6_rx_buff[start_pos]++;																					//防止重复处理数据
			next_start_pos=start_pos+9+Referee_system_data.frameheader_t.DataLength;									//下一帧数据起始位 = 当前数据起始位+当前数据长度+9-byte(帧头5+命令码2+校验2)
			judgeCalculate(&uart6_rx_buff[start_pos]+7);																//将n-byte数据进行解析
			start_pos=next_start_pos;																					//下一帧数据作为其实新起始位
		}
		/*校验错误*/
		else
		{
			start_pos=0;
			
			break;
		}
		if(start_pos>200)	//头指针越界
		{
			start_pos=0;
			break;
		}
	}
}



/*解析数据*/
void judgeCalculate(uint8_t *data)
{
	FormatTrans FT;
	switch(u6_cmd_data)
	{
		case 0x0001:
		{
			Referee_system_data.extgame_status_t.game_type = data[0] & 0x0f;
			Referee_system_data.extgame_status_t.game_progress = data[0]>>4 & 0x0f;
			Referee_system_data.extgame_status_t.stage_remain_time = (uint16_t) data[2]<<8 | data[1];
		}break;
		case 0x0002:
		{
			Referee_system_data.extgame_result_t.winner = data[0];
		}break;
		case 0x0003:
		{
			Referee_system_data.extgame_robot_HP_t.red_1_robot_HP = (uint16_t)data[1]<<8 | data[0];
			Referee_system_data.extgame_robot_HP_t.red_2_robot_HP = (uint16_t)data[3]<<8 | data[2];
			Referee_system_data.extgame_robot_HP_t.red_3_robot_HP = (uint16_t)data[5]<<8 | data[4];
			Referee_system_data.extgame_robot_HP_t.red_4_robot_HP = (uint16_t)data[7]<<8 | data[6];
			Referee_system_data.extgame_robot_HP_t.red_5_robot_HP = (uint16_t)data[9]<<8 | data[8];
			Referee_system_data.extgame_robot_HP_t.red_7_robot_HP = (uint16_t)data[11]<<8 | data[10];
			Referee_system_data.extgame_robot_HP_t.red_outpost_HP = (uint16_t)data[13]<<8 | data[12];
			Referee_system_data.extgame_robot_HP_t.red_base_HP = (uint16_t)data[15]<<8 | data[14];
			Referee_system_data.extgame_robot_HP_t.blue_1_robot_HP = (uint16_t)data[17]<<8 | data[16];
			Referee_system_data.extgame_robot_HP_t.blue_2_robot_HP = (uint16_t)data[19]<<8 | data[18];
			Referee_system_data.extgame_robot_HP_t.blue_3_robot_HP = (uint16_t)data[21]<<8 | data[20];
			Referee_system_data.extgame_robot_HP_t.blue_4_robot_HP = (uint16_t)data[23]<<8 | data[22];
			Referee_system_data.extgame_robot_HP_t.blue_5_robot_HP = (uint16_t)data[25]<<8 | data[24];
			Referee_system_data.extgame_robot_HP_t.blue_7_robot_HP = (uint16_t)data[27]<<8 | data[26];
			Referee_system_data.extgame_robot_HP_t.blue_outpost_HP = (uint16_t)data[29]<<8 | data[28];
			Referee_system_data.extgame_robot_HP_t.blue_base_HP = (uint16_t)data[31]<<8 | data[30];
		}break;
		case 0x0004:
		{
			Referee_system_data.extdart_status_t.dart_belong = data[0];
			Referee_system_data.extdart_status_t.stage_remaining_time = (uint16_t)data[2]<<8 | data[1];
		}break;
		case 0x0101:
		{
			Referee_system_data.extevent_data_t.event_type = (uint32_t)data[3]<<24 | (uint32_t)data[2]<<16 | (uint32_t)data[1]<<8 | data[0];
		}break;
		case 0x0102:
		{
			Referee_system_data.extsupply_projectile_action_t.supply_projectile_id = data[0];
			Referee_system_data.extsupply_projectile_action_t.supply_robot_id    = data[1];      //补弹机器人信息
			Referee_system_data.extsupply_projectile_action_t.supply_projectile_step = data[2];  //出弹口状态
			Referee_system_data.extsupply_projectile_action_t.supply_projectile_num = data[3];   //补弹数量
		}break;
		case 0x0104:
		{
			Referee_system_data.extreferee_warning_t.level = data[0];
			Referee_system_data.extreferee_warning_t.foul_robot_id = data[0];
		}break;
		case 0x0105:
		{
			Referee_system_data.extdart_remaining_time_t.dart_remaining_time = data[0];
		}break;
		case 0x0201:
		{
			Referee_system_data.extgame_robot_status_t.robot_id = data[0];      //robot id
			Referee_system_data.extgame_robot_status_t.robot_level = data[1];   //robot level
			Referee_system_data.extgame_robot_status_t.remain_HP = (uint16_t)data[3]<<8 | data[2];//机器人剩余血量
			Referee_system_data.extgame_robot_status_t.max_HP = (uint16_t)data[5]<<8 | data[4];
			Referee_system_data.extgame_robot_status_t.shooter_id1_17mm_cooling_rate= (uint16_t)data[7]<<8 | data[6];
			Referee_system_data.extgame_robot_status_t.shooter_id1_17mm_cooling_limit= (uint16_t)data[9]<<8 | data[8];
			Referee_system_data.extgame_robot_status_t.shooter_id1_17mm_speed_limit= (uint16_t)data[11]<<8 | data[10];
			Referee_system_data.extgame_robot_status_t.shooter_id2_17mm_cooling_rate= (uint16_t)data[13]<<8 | data[12];
			Referee_system_data.extgame_robot_status_t.shooter_id2_17mm_cooling_limit= (uint16_t)data[15]<<8 | data[14];
			Referee_system_data.extgame_robot_status_t.shooter_id2_17mm_speed_limit= (uint16_t)data[17]<<8 | data[16];
			Referee_system_data.extgame_robot_status_t.shooter_id1_42mm_cooling_rate= (uint16_t)data[19]<<8 | data[18];//冷却速度
			Referee_system_data.extgame_robot_status_t.shooter_id1_42mm_cooling_limit= (uint16_t)data[21]<<8 | data[20];//热量上限
			Referee_system_data.extgame_robot_status_t.shooter_id1_42mm_speed_limit= (uint16_t)data[23]<<8 | data[22];//速度上限				 
			Referee_system_data.extgame_robot_status_t.chassis_power_limit = (uint16_t)data[25]<<8 | data[24];//底盘功率上限
			Referee_system_data.extgame_robot_status_t.mains_power_gimbal_output = (data[26] & 0x01);        //电源输出情况         
			Referee_system_data.extgame_robot_status_t.mains_power_chassis_output= (data[27] & 0x02)>>1;
			Referee_system_data.extgame_robot_status_t.mains_power_shooter_output= (data[28] & 0x04)>>2;
		}break;
		case 0x0202:
		{
			Referee_system_data.extpower_heat_data_t.chassis_volt = (uint16_t)data[1]<<8 | data[0];   //mV 底盘输出电压
			Referee_system_data.extpower_heat_data_t.chassis_current=(uint16_t)data[3]<<8 | data[2];  //mA	底盘输出电流
			FT.U[3] = data[7];       //保留小数
			FT.U[2] = data[6];		//
			FT.U[1] = data[5];		//
			FT.U[0] = data[4];      //
			Referee_system_data.extpower_heat_data_t.chassis_power = FT.F;//w(采用联合体转换数据类型) 功率
			Referee_system_data.extpower_heat_data_t.chassis_power_buffer = (uint16_t)data[9]<<8 | data[8];   //j
			Referee_system_data.extpower_heat_data_t.shooter_id1_17mm_cooling_heat= (uint16_t)data[11]<<8 | data[10];
			Referee_system_data.extpower_heat_data_t.shooter_id2_17mm_cooling_heat = (uint16_t)data[13]<<8 | data[12];
			Referee_system_data.extpower_heat_data_t.shooter_id1_42mm_cooling_heat = (uint16_t)data[15]<<8 | data[14];//实时热量
		}break;
		case 0x0203:
		{
			Referee_system_data.extgame_robot_pos_t.x = (float)((uint32_t)data[3]<<24 | (uint32_t)data[2]<<16 | (uint32_t)data[1]<<8 | data[0]);
			Referee_system_data.extgame_robot_pos_t.y = (float)((uint32_t)data[7]<<24 | (uint32_t)data[6]<<16 | (uint32_t)data[5]<<8 | data[4]);
			Referee_system_data.extgame_robot_pos_t.z = (float)((uint32_t)data[11]<<24 | (uint32_t)data[10]<<16 | (uint32_t)data[9]<<8 | data[8]);
			Referee_system_data.extgame_robot_pos_t.yaw = (float)((uint32_t)data[15]<<24 | (uint32_t)data[14]<<16 | (uint32_t)data[13]<<8 | data[12]);
		}break;
		case 0x0204:
		{
			Referee_system_data.extbuff_t.power_rune_buff = data[0];//机器人增益，未拆解
		}break;
		case 0x0205:
		{
			Referee_system_data.aerialrobot_energy_t.attack_time = data[0]; 
		}break;
		case 0x0206:
		{
			Referee_system_data.extrobot_hurt_t.armor_id = data[0] & 0x0f;      //扣血装甲id
			Referee_system_data.extrobot_hurt_t.hurt_type = data[0]>>4 & 0x0f;  //扣血类型
		}break;
		case 0x0207:
		{
			Referee_system_data.extshoot_data_t.bullet_type = data[0];   //弹丸类型
			Referee_system_data.extshoot_data_t.shooter_id = data[1];    //发射机构ID
			Referee_system_data.extshoot_data_t.bullet_freq = data[2];   //射频Hz
			FT.U[3] = data[6];
			FT.U[2] = data[5];
			FT.U[1] = data[4];
			FT.U[0] = data[3];
			Referee_system_data.extshoot_data_t.bullet_speed = FT.F;//弹速
	//		varible_send.send_data_update = 1;            //射击更新标志位 --------------新版
		}break;
		case 0x0208:
		{
			Referee_system_data.extbullet_remaining_t.bullet_remaining_num_17mm = (uint16_t)data[1]<<8 | data[0];  //17mm子弹剩余发射数目
			Referee_system_data.extbullet_remaining_t.bullet_remaining_num_42mm = (uint16_t)data[3]<<8 | data[2];  //42mm子弹剩余发射数目
			Referee_system_data.extbullet_remaining_t.coin_remaining_num = (uint16_t)data[5]<<8 | data[4];         //剩余金币数量
		}break;
		case 0x0209:
		{
			Referee_system_data.extrfid_status_t.rfid_status = data[0];
		}break;
		case 0x020A:
		{
			Referee_system_data.extdart_client_cmd_t.dart_launch_opening_status = data[0];
			Referee_system_data.extdart_client_cmd_t.dart_attack_target = data[1];
			Referee_system_data.extdart_client_cmd_t.target_change_time = (uint16_t)data[3]<<8 | data[2];
			Referee_system_data.extdart_client_cmd_t.first_dart_speed = data[4];
			Referee_system_data.extdart_client_cmd_t.second_dart_speed = data[5];
			Referee_system_data.extdart_client_cmd_t.third_dart_speed = data[6];
			Referee_system_data.extdart_client_cmd_t.fourth_dart_speed = data[7];
			Referee_system_data.extdart_client_cmd_t.last_dart_launch_time = (uint16_t)data[9]<<8 | data[8];
			Referee_system_data.extdart_client_cmd_t.operate_launch_cmd_time = (uint16_t)data[11]<<8 | data[10];
		}break;
		case 0x0301:
		{
			Referee_system_data.extstudent_interactive_header_data_t.data_cmd_id = data[6];
			Referee_system_data.extstudent_interactive_header_data_t.sender_ID = data[7];
			Referee_system_data.extstudent_interactive_header_data_t.receiver_ID = data[8];
		}break;
		case 0x0302:
		{
			
		}break;
		case 0x0303:
		{
			
		}break;
		case 0x0304:
		{
			Referee_system_data.extrobot_command_t_1.mouse_x = (uint16_t)data[1]<<8 | data[0]; 
			Referee_system_data.extrobot_command_t_1.mouse_y = (uint16_t)data[3]<<8 | data[2]; 
			Referee_system_data.extrobot_command_t_1.mouse_z = (uint16_t)data[5]<<8 | data[4]; 
			Referee_system_data.extrobot_command_t_1.left_button_down = data[6];
			Referee_system_data.extrobot_command_t_1.right_button_down = data[7];
			Referee_system_data.extrobot_command_t_1.keyboard_value = (uint16_t)data[9]<<8 | data[8];
			Referee_system_data.extrobot_command_t_1.reserved = (uint16_t)data[11]<<8 | data[10];
		}break;
	}
}



/*红蓝阵营判断*/
int Red_or_Blue_judge(void)
{
	if(Referee_system_data.extgame_robot_status_t.robot_id > 100)  //1---101
		return BLUE;
	else
		return RED;
}


/*客户端ID*/
void Robot_client_ID(void)
{
	Robot_ID_Data.robot_ID = Referee_system_data.extgame_robot_status_t.robot_id;
	Robot_ID_Data.robot_colour = Red_or_Blue_judge();
	if(Robot_ID_Data.robot_colour == BLUE)
		Robot_ID_Data.robot_client_ID = 0x0164 + (Robot_ID_Data.robot_ID-100);
	else	if(Robot_ID_Data.robot_colour == RED)
		Robot_ID_Data.robot_client_ID = 0x0100 + Robot_ID_Data.robot_ID;
}


/*读取实时功率、实时缓冲能量*/
void get_chassis_power_and_buffer(float *power, float *buffer)
{
   *power = Referee_system_data.extpower_heat_data_t.chassis_power;
   *buffer = Referee_system_data.extpower_heat_data_t.chassis_power_buffer;
}

/*读取机器人ID、最大限制功率、等级*/
void get_chassis_id_maxpower_maxbuffer(uint8_t *ID,float *max_power,uint8_t *robot_level)
{
	*ID = Referee_system_data.extgame_robot_status_t.robot_id;
	*max_power = Referee_system_data.extgame_robot_status_t.chassis_power_limit;
	*robot_level = Referee_system_data.extgame_robot_status_t.robot_level;
}

///*ui绘制任务*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//uint8_t ui_init_flag;
//uint16_t ui_task_flag;
//void UI_task(void)
//{
//	
//	
//	/*************
//	开始时清除UI
//	尽量用DMA
//	*************/
//	if(ui_init_flag < 2)
//	{
//		//清理图层
//		if(ui_init_flag % 2 == 0)
//		{
//			UI_all_clear(0);
//			HAL_UART_Transmit_DMA(&huart6,uart6_tx_buff,200);
//		}
//		else if(ui_init_flag % 2 == 1)
//		{
//			UI_all_clear(1);
//			HAL_UART_Transmit_DMA(&huart6,uart6_tx_buff,200);
//		}
//		ui_init_flag ++;
//		ui_task_flag = 0;
//	}
//	
//	
//	/*****************
//	增添，修改，删除
//	******************/
//	//增加，修改,删除
//	else if(ui_init_flag >= 2)
//	{
//		if(ui_task_flag < 10)
//		{
//			if(ui_task_flag % 2 == 0)
//			{
//				draw_UI_line_5(0x0301,		0x0103,     1);
//									/*命令字符， 数据内容ID，图形操作*/
//				HAL_UART_Transmit_DMA(&huart6,uart6_tx_buff,200);
//			}
//			else if(ui_task_flag % 2 == 1)
//			{
//				draw_UI_line_7(0x0301,		0x0104,     1);
//									/*命令字符， 数据内容ID，图形操作*/
//				HAL_UART_Transmit_DMA(&huart6,uart6_tx_buff,200);
//			}
//		}
//		else if(ui_task_flag >= 10 && ui_task_flag < 90)
//		{
//			if(ui_task_flag % 2 == 0)
//			{
//				draw_UI_line_5(0x0301,		0x0103,     2);
//									/*命令字符， 数据内容ID，图形操作*/
//				HAL_UART_Transmit_DMA(&huart6,uart6_tx_buff,200);
//			}
//			else if(ui_task_flag % 2 == 1)
//			{
//				draw_UI_line_7(0x0301,		0x0104,     2);
//									/*命令字符， 数据内容ID，图形操作*/
//			HAL_UART_Transmit_DMA(&huart6,uart6_tx_buff,200);
//			}
//		}
//		else if(ui_task_flag >= 90)
//			ui_init_flag = 0;
//		ui_task_flag ++;
//	}
//	
//	
//}



/*			CRC校验			*/
/*			CRC校验			*/
/*			CRC校验			*/
#if 1
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
{
0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};
uint16_t CRC_INIT = 0xffff;
const uint16_t wCRC_Table[256] =
{
0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
/**
  * @brief          计算CRC8
  * @param[in]      pch_message: 数据
  * @param[in]      dw_length: 数据和校验的长度
  * @param[in]      ucCRC8:初始CRC8
  * @retval         计算完的CRC8
  */
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8)
{
	unsigned char ucIndex;
	while (dwLength--)
	{
		ucIndex = ucCRC8^(*pchMessage++);
		ucCRC8 = CRC8_TAB[ucIndex];
	}
	return(ucCRC8);
}
/**
  * @brief          CRC8校验函数
  * @param[in]      pch_message: 数据
  * @param[in]      dw_length: 数据和校验的长度
  * @retval         真或者假
  */
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
	unsigned char ucExpected = 0;
	if ((pchMessage == 0) || (dwLength <= 2)) return 0;
	ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
	return ( ucExpected == pchMessage[dwLength-1] );
}
/**
  * @brief          添加CRC8到数据的结尾
  * @param[in]      pch_message: 数据
  * @param[in]      dw_length: 数据和校验的长度
  * @retval         none
  */
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
	unsigned char ucCRC = 0;
	if ((pchMessage == 0) || (dwLength <= 2)) return;
	ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
	pchMessage[dwLength-1] = ucCRC;
}
/**
  * @brief          计算CRC16
  * @param[in]      pch_message: 数据
  * @param[in]      dw_length: 数据和校验的长度
  * @param[in]      wCRC:初始CRC16
  * @retval         计算完的CRC16
  */
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
	uint8_t chData;
	if (pchMessage == NULL)
	{
		return 0xFFFF;
	}
	while(dwLength--)
	{
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) &
		0x00ff];
	}
	return wCRC;
}
/**
  * @brief          CRC16校验函数
  * @param[in]      pch_message: 数据
  * @param[in]      dw_length: 数据和校验的长度
  * @retval         真或者假
  */
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
	uint16_t wExpected = 0;
	if ((pchMessage == NULL) || (dwLength <= 2))
	{
		return 0;
	}
	wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
	return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) ==
	pchMessage[dwLength - 1]);
}
/**
  * @brief          添加CRC16到数据的结尾
  * @param[in]      pch_message: 数据
  * @param[in]      dw_length: 数据和校验的长度
  * @retval         none
  */
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength)
{
	uint16_t wCRC = 0;
	if ((pchMessage == NULL) || (dwLength <= 2))
	{
		return;
	}
	wCRC = Get_CRC16_Check_Sum ( (U8 *)pchMessage, dwLength-2, CRC_INIT );
	pchMessage[dwLength-2] = (U8)(wCRC & 0x00ff);
	pchMessage[dwLength-1] = (U8)((wCRC >> 8)& 0x00ff);
}	

#endif


