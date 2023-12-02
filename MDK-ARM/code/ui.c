#include "UI.h"


/*ui绘制任务
	
	电容
	弹道下坠线
//	车位线
	标志：弹舱，自瞄，电容
	



*/





///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int qwer;
uint8_t ui_init_flag;
uint16_t ui_task_flag;
uint8_t ui_flag;
void UI_task(void)
{

	/*************
	开始时清除UI
	尽量用DMA
	*************/
	if(ui_init_flag < 2)
	{
		//清理图层
		if(ui_init_flag % 2 == 0)
		{
			UI_all_clear(0);
			HAL_UART_Transmit(&huart6,uart6_tx_buff,130,50);
		}
		else if(ui_init_flag % 2 == 1)
		{
			UI_all_clear(1);
			HAL_UART_Transmit(&huart6,uart6_tx_buff,130,50);
		}
		ui_init_flag ++;
		ui_task_flag = 0;
	}
	
	
	/*****************
	增添，修改，删除
	******************/
	//增加，修改,删除
	else if(ui_init_flag >= 2)
	{
		if(ui_task_flag < 10)
		{
			if(ui_task_flag % 3 == 0)
			{
				draw_UI_line_1(0x0301,		0x0104,     1);
									/*命令字符， 数据内容ID，图形操作*/
				HAL_UART_Transmit(&huart6,uart6_tx_buff,130,50);
			}
			else if(ui_task_flag % 3 == 1)
			{
				draw_UI_line_2(0x0301,		0x0103,     1);
									/*命令字符， 数据内容ID，图形操作*/
				HAL_UART_Transmit(&huart6,uart6_tx_buff,130,50);
			}
			else if(ui_task_flag % 3 == 2)
			{
				draw_UI_line_3(0x0301,		0x0103,     1);
									/*命令字符， 数据内容ID，图形操作*/
				HAL_UART_Transmit(&huart6,uart6_tx_buff,130,50);
			}
		}
		else if(ui_task_flag >= 10 && ui_task_flag < 90)
		{
			if(ui_task_flag % 3 == 0)
			{
				draw_UI_line_1(0x0301,		0x0104,     2);
									/*命令字符， 数据内容ID，图形操作*/
				HAL_UART_Transmit(&huart6,uart6_tx_buff,130,50);
			}
			else if(ui_task_flag % 3 == 1)
			{
				draw_UI_line_2(0x0301,		0x0103,     2);
									/*命令字符， 数据内容ID，图形操作*/
			HAL_UART_Transmit(&huart6,uart6_tx_buff,130,50);
			}
			else if(ui_task_flag % 3 == 2)
			{
				draw_UI_line_3(0x0301,		0x0103,     2  );
									/*命令字符， 数据内容ID，图形操作*/
				HAL_UART_Transmit(&huart6,uart6_tx_buff,130,50);
			}
		}
		
		else if(ui_task_flag >= 90)
			ui_init_flag = 0;
		ui_task_flag ++;
	}
	
	
}





#define line_0 975




/*机器人间通信*/
void robot_robot_data_tx(uint16_t data_id	)					//数据内容ID											
{
	Robot_client_ID();//读取机器人ID
	/*定义*/
	static ext_robot_tx draw_p;
	static uint8_t seq=0;
	/*ID设置*/
	draw_p.cmd_id = 0x0301;  	
	draw_p.Client_Custom_ID.data_cmd_id= data_id; 													//数据内容ID
	draw_p.Client_Custom_ID.sender_ID = Robot_ID_Data.robot_ID;							//发送者的ID
	if(Red_or_Blue_judge() == BLUE)				//己方蓝色
		draw_p.Client_Custom_ID.receiver_ID = 107;													//接受者的ID
	else if(Red_or_Blue_judge() == RED)				//己方红色
		draw_p.Client_Custom_ID.receiver_ID = 7;													//接受者的ID
	
	/*发送的数据*/
	draw_p.robot_interactive_data.data[0] = 0xAA;
	/*头配置*/
	draw_p.Frameheader.SOF = 0xA5; 					//0xA5
  draw_p.Frameheader.DataLength=6+1;			//头结构长度+数据段长度
	draw_p.Frameheader.Seq=seq;							//包序号
	/*复制头数据至发送缓冲*/
	memcpy(uart6_tx_buff, &draw_p.Frameheader, sizeof(frame_header_t));
	/*CRC8校验头*/
	Append_CRC8_Check_Sum(uart6_tx_buff,sizeof(frame_header_t)); 

	/*复制数据段至发送缓冲*/
    memcpy(uart6_tx_buff + 5,
				 (uint8_t*)&draw_p.cmd_id, 
				 (sizeof(draw_p.cmd_id) + sizeof(draw_p.Client_Custom_ID)+ 1));
	/*CRC16校验整包数据*/
	Append_CRC16_Check_Sum(uart6_tx_buff, draw_p.Frameheader.DataLength+9);					//数据段+5+2+2

}


//车位线-辅助瞄准
void draw_UI_line_1	 
	(uint16_t cmd_id,	uint16_t data_id,uint32_t operate_tpye)
 /*命令字符，				数据内容ID，		 图形操作*/
{
	/*读取机器人颜色和ID*/
	Red_or_Blue_judge();
	Robot_client_ID();
	/*定义*/
	static ext_draw_line_7 draw_1;
	static uint8_t seq=1;
	/*ID设置*/
	 draw_1.cmd_id = cmd_id;  
	 draw_1.Client_Custom_ID.data_cmd_id= data_id; 
	 draw_1.Client_Custom_ID.sender_ID = Robot_ID_Data.robot_ID;
	 draw_1.Client_Custom_ID.receiver_ID = Robot_ID_Data.robot_client_ID;	
	/*数据*/
	for(int i = 0 ;i < 7 ;i++)
	{
		 draw_1.grapic_data_struct[i].graphic_name[0]=i;					//图形名字
		 draw_1.grapic_data_struct[i].operate_tpye=operate_tpye;		//图形操作
		 draw_1.grapic_data_struct[i].layer=0;											//图层数
//		 draw_1.grapic_data_struct[i].start_angle=0;								//空
//		 draw_1.grapic_data_struct[i].end_angle=0;									//空
		// draw_1.grapic_data_struct[i].radius=0;										//空
	}
	uint8_t car_color;
	if(	task_flag.cap_mode==0) 	     car_color = 8;									//图形颜色0红蓝主色1黄色2绿色3橙色4紫红色5粉色6青色7黑色8白色
	else if(task_flag.cap_mode==1)	 car_color = 1;
	else if(task_flag.cap_mode==2)	 car_color = 4;
	
	
	//电容
	draw_1.grapic_data_struct[0].graphic_tpye=0;							//图形类型
	draw_1.grapic_data_struct[0].width=16;											//线条宽度
	draw_1.grapic_data_struct[0].color=car_color;
	draw_1.grapic_data_struct[0].start_x=660;							//起点坐标X
	draw_1.grapic_data_struct[0].start_y=84;								//起点坐标Y
	draw_1.grapic_data_struct[0].end_x=660+6*super_cap.cap_capacity;								//终点坐标X
	draw_1.grapic_data_struct[0].end_y=84;								//终点坐标Y
	draw_1.grapic_data_struct[0].start_angle=0;								//空
	draw_1.grapic_data_struct[0].end_angle=0;									//空
	draw_1.grapic_data_struct[0].radius=0;
	
	//电容框
	draw_1.grapic_data_struct[1].graphic_tpye=1;							//图形类型
	draw_1.grapic_data_struct[1].width=2;											//线条宽度
	draw_1.grapic_data_struct[1].color=car_color;
	draw_1.grapic_data_struct[1].start_x=660;							//起点坐标X
	draw_1.grapic_data_struct[1].start_y=76;								//起点坐标Y
	draw_1.grapic_data_struct[1].end_x=1260;								//终点坐标X
	draw_1.grapic_data_struct[1].end_y=92;								//终点坐标Y
	draw_1.grapic_data_struct[1].start_angle=0;								//空
	draw_1.grapic_data_struct[1].end_angle=0;									//空
	draw_1.grapic_data_struct[1].radius=0;




//自瞄
	
	draw_1.grapic_data_struct[2].graphic_tpye=1;						//graphic_tpye
	if(varible_vision.aim_flag  == 1) 
		draw_1.grapic_data_struct[2].color= 5 ;									//color
	if(task_flag.hit_mode == 1 && varible_vision.aim_flag == 1)			
		draw_1.grapic_data_struct[2].color= 1 ;
	if(varible_vision.aim_flag  == 0)
		draw_1.grapic_data_struct[2].color= 0 ;
	draw_1.grapic_data_struct[2].start_angle=0;								//空
	draw_1.grapic_data_struct[2].end_angle=0;									//空
//	draw_1.grapic_data_struct[2].width=4;									//width	
	draw_1.grapic_data_struct[2].start_x=line_0 - 280;							//start_x
	draw_1.grapic_data_struct[2].start_y=540 + 220;
//	if(vision_rx.date_update == true)	//start_y
//		draw_1.grapic_data_struct[2].radius=220;									//radius
//	else
//		draw_1.grapic_data_struct[2].radius=10;	

//
	if(vision_rx.date_update == true)	//start_y
		draw_1.grapic_data_struct[2].width=4;								//radius
	else if(vision_rx.date_update == true && task_flag.hit_mode == 1)
		draw_1.grapic_data_struct[2].width=8;
	else
		draw_1.grapic_data_struct[2].width=0;
	draw_1.grapic_data_struct[2].radius=0;	
	draw_1.grapic_data_struct[2].end_x=line_0 + 280;								//end_x
	draw_1.grapic_data_struct[2].end_y=540 - 220;		
	
//	draw_1.grapic_data_struct[2].graphic_tpye=2;						//graphic_tpye
//	if(varible_vision.aim_flag  == 1) 
//		draw_1.grapic_data_struct[2].color= 5 ;									//color
//	if(task_flag.hit_mode == 1 && varible_vision.aim_flag == 1)			
//		draw_1.grapic_data_struct[2].color= 1 ;
//	if(varible_vision.aim_flag  == 0)
//		draw_1.grapic_data_struct[2].color= 0 ;
//	draw_1.grapic_data_struct[2].start_angle=0;								//空
//	draw_1.grapic_data_struct[2].end_angle=0;									//空
//	draw_1.grapic_data_struct[2].width=4;									//width	
//	draw_1.grapic_data_struct[2].start_x=line_0;							//start_x
//	draw_1.grapic_data_struct[2].start_y=540;
//	if(vision_rx.date_update == true)	//start_y
//		draw_1.grapic_data_struct[2].radius=220;									//radius
//	else
//		draw_1.grapic_data_struct[2].radius=10;	
//	draw_1.grapic_data_struct[2].end_x=0;								//end_x
//	draw_1.grapic_data_struct[2].end_y=0;								//end_y						//end_yp
	
	//摩擦轮
	draw_1.grapic_data_struct[5].graphic_tpye=2;						//graphic_tpye
	if(task_flag.frictiongear == 1)	     draw_1.grapic_data_struct[5].color=5;									//color
	else								 draw_1.grapic_data_struct[5].color=8;									//color
	draw_1.grapic_data_struct[5].start_angle=0;								//空
	draw_1.grapic_data_struct[5].end_angle=0;									//空
	draw_1.grapic_data_struct[5].width=5;									//width
	draw_1.grapic_data_struct[5].start_x=1660;							//start_x
	draw_1.grapic_data_struct[5].start_y=740;							//start_y
	draw_1.grapic_data_struct[5].radius=35;									//radius
	draw_1.grapic_data_struct[5].end_x=0;								//end_x
	draw_1.grapic_data_struct[5].end_y=0;								//end_y

	//小陀螺	
	draw_1.grapic_data_struct[3].graphic_tpye=2;						//graphic_tpye
	if(task_flag.small_gyro == 1) 		draw_1.grapic_data_struct[3].color=5;		//color
	else    							draw_1.grapic_data_struct[3].color=8;	
	draw_1.grapic_data_struct[3].start_angle=0;								//空
	draw_1.grapic_data_struct[3].end_angle=360;									//空
	draw_1.grapic_data_struct[3].width=5;									//width
	draw_1.grapic_data_struct[3].start_x=1790;							//start_x
	draw_1.grapic_data_struct[3].start_y=740;							//start_y
	draw_1.grapic_data_struct[3].radius=35;									//radius
	draw_1.grapic_data_struct[3].end_x=0;								//end_x
	draw_1.grapic_data_struct[3].end_y=0;								//end_y
	
	//弹舱
	draw_1.grapic_data_struct[4].graphic_tpye=0;						//graphic_tpye
	if(varible_send.lock_pill == 1)		draw_1.grapic_data_struct[4].color=2;									//color		 //堵转
	else   								draw_1.grapic_data_struct[4].color=4;		   
	draw_1.grapic_data_struct[4].start_angle=0;								//空
	draw_1.grapic_data_struct[4].end_angle=0;									//空
	draw_1.grapic_data_struct[4].width=8;									//width
	draw_1.grapic_data_struct[4].start_x=1640;							//start_x
	draw_1.grapic_data_struct[4].start_y=650;							//start_y
	draw_1.grapic_data_struct[4].radius=0;									//radius
	draw_1.grapic_data_struct[4].end_x=1730;								//end_x
	if(task_flag.pill_depot == 1)	  draw_1.grapic_data_struct[4].end_y=700;									//color
	else 						 draw_1.grapic_data_struct[4].end_y=650;								//end_y

//	
//	//底盘位置框图
//	draw_1.grapic_data_struct[2].graphic_tpye=0;							//图形类型
//	draw_1.grapic_data_struct[2].width=3;											//线条宽度
//	draw_1.grapic_data_struct[2].color=6;
//	draw_1.grapic_data_struct[2].start_x=640-100;							//起点坐标X
//	draw_1.grapic_data_struct[2].start_y=0;								//起点坐标Y
//	draw_1.grapic_data_struct[2].end_x=740-50;								//终点坐标X
//	draw_1.grapic_data_struct[2].end_y=272;								//终点坐标Y
//	//
//	draw_1.grapic_data_struct[3].graphic_tpye=0;							//图形类型
//	draw_1.grapic_data_struct[3].width=3;											//线条宽度
//	draw_1.grapic_data_struct[3].color=6;
//	draw_1.grapic_data_struct[3].start_x=1280+100;							//起点坐标X
//	draw_1.grapic_data_struct[3].start_y=0;							//起点坐标Y
//	draw_1.grapic_data_struct[3].end_x=1180+50;								//终点坐标X
//	draw_1.grapic_data_struct[3].end_y=272;								//终点坐标Y
//	//
//	draw_1.grapic_data_struct[4].graphic_tpye=0;							//图形类型
//	draw_1.grapic_data_struct[4].width=3;											//线条宽度
//	draw_1.grapic_data_struct[4].color=6;
//	draw_1.grapic_data_struct[4].start_x=1180+50;							//起点坐标X
//	draw_1.grapic_data_struct[4].start_y=272;							//起点坐标Y
//	draw_1.grapic_data_struct[4].end_x=1080;								//终点坐标X
//	draw_1.grapic_data_struct[4].end_y=272;								//终点坐标Y

//	draw_1.grapic_data_struct[5].graphic_tpye=0;							//图形类型
//	draw_1.grapic_data_struct[5].width=3;											//线条宽度
//	draw_1.grapic_data_struct[5].color=6;
//	draw_1.grapic_data_struct[5].start_x=740-50;							//起点坐标X
//	draw_1.grapic_data_struct[5].start_y=272;							//起点坐标Y
//	draw_1.grapic_data_struct[5].end_x=840;								//终点坐标X
//	draw_1.grapic_data_struct[5].end_y=272;								
	
	
	/*头配置*/
	draw_1.Frameheader.SOF = 0xA5; 					//0xA5
 draw_1.Frameheader.DataLength=6+105;		//头结构长度+数据段长度
	draw_1.Frameheader.Seq=seq;							//包序号
	/*复制头数据至发送缓冲*/
	memcpy(uart6_tx_buff, &draw_1.Frameheader, sizeof(frame_header_t));
	/*CRC8校验头*/
	Append_CRC8_Check_Sum(uart6_tx_buff,sizeof(frame_header_t)); 
	/*复制数据段至发送缓冲*/
  memcpy(uart6_tx_buff + 5, 
				 (uint8_t*)&draw_1.cmd_id, 
				 (sizeof(draw_1.cmd_id)+ sizeof(draw_1.Client_Custom_ID)+ sizeof(draw_1.grapic_data_struct)));
	/*CRC16校验整包数据*/
	Append_CRC16_Check_Sum(uart6_tx_buff, draw_1.Frameheader.DataLength+9);
}


/*辅助瞄准线*/
void draw_UI_line_2	 
	(uint16_t cmd_id,	uint16_t data_id,uint32_t operate_tpye)
 /*命令字符，				数据内容ID，		 图形操作*/
{
	/*读取机器人颜色和ID*/
	Red_or_Blue_judge();
	Robot_client_ID();
	/*定义*/
	static ext_draw_line_5_1 draw_static;  //划线
	static uint8_t seq=0;
	/*ID设置*/
	draw_static.cmd_id = cmd_id;  
	draw_static.Client_Custom_ID.data_cmd_id= data_id; 
	draw_static.Client_Custom_ID.sender_ID = Robot_ID_Data.robot_ID;
	draw_static.Client_Custom_ID.receiver_ID = Robot_ID_Data.robot_client_ID;	

	/*数据*/
	for(int i = 0 ;i < 5 ;i++)
	{
		draw_static.grapic_data_struct[i].graphic_name[0]=i+7;					//图形名字
		draw_static.grapic_data_struct[i].operate_tpye=operate_tpye;		//图形操作
		draw_static.grapic_data_struct[i].graphic_tpye=0;							//图形类型
		draw_static.grapic_data_struct[i].layer=1;											//图层数
		draw_static.grapic_data_struct[i].start_angle=0;								//空
		draw_static.grapic_data_struct[i].end_angle=0;									//空
		draw_static.grapic_data_struct[i].radius=0;										//空
		draw_static.grapic_data_struct[i].width=2;											//线条宽度
	}
	/*0红蓝主色1黄色2绿色3橙色4紫红色5粉色6青色7黑色8白色*/
	
	/*竖线*/
	
	draw_static.grapic_data_struct[0].color=0;
	draw_static.grapic_data_struct[0].start_x=line_0 ;							//起点坐标X
	draw_static.grapic_data_struct[0].start_y=250;								//起点坐标Y
	draw_static.grapic_data_struct[0].end_x=line_0 ;								//终点坐标X
	draw_static.grapic_data_struct[0].end_y=540;								//终点坐标Y
	
	/*横1*/
	#define Line_1 484
	draw_static.grapic_data_struct[1].color = 1;
	draw_static.grapic_data_struct[1].start_x=line_0 - 30;							//起点坐标X
	draw_static.grapic_data_struct[1].start_y=Line_1;							//起点坐标Y
	draw_static.grapic_data_struct[1].end_x=line_0 + 30;							//终点坐标X
	draw_static.grapic_data_struct[1].end_y=Line_1;								//终点坐标Y
	/*横2*/
	#define Line_2 400
	draw_static.grapic_data_struct[2].color = 2;
	draw_static.grapic_data_struct[2].start_x=line_0 - 60;							//起点坐标X
	draw_static.grapic_data_struct[2].start_y=Line_2;							//起点坐标Y
	draw_static.grapic_data_struct[2].end_x=line_0 + 60;							//终点坐标X
	draw_static.grapic_data_struct[2].end_y=Line_2;								//终点坐标Y
	/*横3*/
	#define Line_3 350
	draw_static.grapic_data_struct[3].color = 3;
	draw_static.grapic_data_struct[3].start_x=line_0 + 90;							//起点坐标X
	draw_static.grapic_data_struct[3].start_y=Line_3;							//起点坐标Y
	draw_static.grapic_data_struct[3].end_x=line_0 - 90;							//终点坐标X
	draw_static.grapic_data_struct[3].end_y=Line_3;								//终点坐标Y
	/*横4*/
	#define Line_4 360
	draw_static.grapic_data_struct[4].color = 4;
	draw_static.grapic_data_struct[4].start_x=line_0 + 110;							//起点坐标X
	draw_static.grapic_data_struct[4].start_y=Line_4;							//起点坐标Y
	draw_static.grapic_data_struct[4].end_x=line_0 + 110;							//终点坐标X
	draw_static.grapic_data_struct[4].end_y=Line_4;								//终点坐标Y
	
	/*头配置*/
	draw_static.Frameheader.SOF = 0xA5; 					//0xA5
  draw_static.Frameheader.DataLength=6+75;		//头结构长度+数据段长度
	draw_static.Frameheader.Seq=seq;							//包序号
	/*复制头数据至发送缓冲*/
	memcpy(uart6_tx_buff, &draw_static.Frameheader, sizeof(frame_header_t));
	/*CRC8校验头*/
	Append_CRC8_Check_Sum(uart6_tx_buff,sizeof(frame_header_t)); 
	/*复制数据段至发送缓冲*/
  memcpy(uart6_tx_buff + 5, 
				 (uint8_t*)&draw_static.cmd_id, 
				 (sizeof(draw_static.cmd_id)+ sizeof(draw_static.Client_Custom_ID)+ sizeof(draw_static.grapic_data_struct)));
	/*CRC16校验整包数据*/
	Append_CRC16_Check_Sum(uart6_tx_buff, draw_static.Frameheader.DataLength+9);
}


/*7个*/
void draw_UI_line_3	 
	(uint16_t cmd_id,	uint16_t data_id,uint32_t operate_tpye)
 /*命令字符，				数据内容ID，		 图形操作*/
{
	/*读取机器人颜色和ID*/
	Red_or_Blue_judge();
	Robot_client_ID();
	/*定义*/
	static ext_draw_line_5_2 draw_trends;
	static uint8_t seq=1;
	/*ID设置*/
	draw_trends.cmd_id = cmd_id;  
	draw_trends.Client_Custom_ID.data_cmd_id= data_id; 
	draw_trends.Client_Custom_ID.sender_ID = Robot_ID_Data.robot_ID;
	draw_trends.Client_Custom_ID.receiver_ID = Robot_ID_Data.robot_client_ID;	
	/*数据*/
	for(int i = 0 ;i < 5 ;i++)
	{
		draw_trends.grapic_data_struct[i].graphic_name[0]=i+12;					//图形名字
		draw_trends.grapic_data_struct[i].operate_tpye=operate_tpye;		//图形操作
		//draw_trends.grapic_data_struct[i].graphic_tpye=0;							//图形类型
		draw_trends.grapic_data_struct[i].layer=3;											//图层数
		draw_trends.grapic_data_struct[i].start_angle=0;								//空
		 draw_trends.grapic_data_struct[i].end_angle=0;									//空
		draw_trends.grapic_data_struct[i].radius=0;	
	}
	//底盘位置框图
	draw_trends.grapic_data_struct[2].graphic_tpye=0;							//图形类型
	draw_trends.grapic_data_struct[2].width=3;											//线条宽度
	draw_trends.grapic_data_struct[2].color=6;
	draw_trends.grapic_data_struct[2].start_x=640-100;							//起点坐标X
	draw_trends.grapic_data_struct[2].start_y=0;								//起点坐标Y
	draw_trends.grapic_data_struct[2].end_x=740-50;								//终点坐标X
	draw_trends.grapic_data_struct[2].end_y=272;								//终点坐标Y
	//
	draw_trends.grapic_data_struct[3].graphic_tpye=0;							//图形类型
	draw_trends.grapic_data_struct[3].width=3;											//线条宽度
	draw_trends.grapic_data_struct[3].color=6;
	draw_trends.grapic_data_struct[3].start_x=1280+100;							//起点坐标X
	draw_trends.grapic_data_struct[3].start_y=0;							//起点坐标Y
	draw_trends.grapic_data_struct[3].end_x=1180+50;								//终点坐标X
	draw_trends.grapic_data_struct[3].end_y=272;								//终点坐标Y
	//
	draw_trends.grapic_data_struct[4].graphic_tpye=0;							//图形类型
	draw_trends.grapic_data_struct[4].width=3;											//线条宽度
	draw_trends.grapic_data_struct[4].color=6;
	draw_trends.grapic_data_struct[4].start_x=1180+50;							//起点坐标X
	draw_trends.grapic_data_struct[4].start_y=272;							//起点坐标Y
	draw_trends.grapic_data_struct[4].end_x=1080;								//终点坐标X
	draw_trends.grapic_data_struct[4].end_y=272;								//终点坐标Y
	
	draw_trends.grapic_data_struct[1].graphic_tpye=0;							//图形类型
	draw_trends.grapic_data_struct[1].width=3;											//线条宽度
	draw_trends.grapic_data_struct[1].color=6;
	draw_trends.grapic_data_struct[1].start_x=740-50;							//起点坐标X
	draw_trends.grapic_data_struct[1].start_y=272;							//起点坐标Y
	draw_trends.grapic_data_struct[1].end_x=840;								//终点坐标X
	draw_trends.grapic_data_struct[1].end_y=272;								
	
	
	
	
	
	
	
//	//自瞄
//	
//	draw_trends.grapic_data_struct[0].graphic_tpye=0;						//graphic_tpye
//	if(task_flag.automatic_aiming == 1) draw_trends.grapic_data_struct[0].color= 5 ;									//color
//	else								 draw_trends.grapic_data_struct[0].color= 8 ;			
//	draw_trends.grapic_data_struct[0].start_angle=0;								//空
//	draw_trends.grapic_data_struct[0].end_angle=0;									//空
//	draw_trends.grapic_data_struct[0].width=20;									//width
//	draw_trends.grapic_data_struct[0].start_x=1650;							//start_x
//	draw_trends.grapic_data_struct[0].start_y=780;							//start_y
//	draw_trends.grapic_data_struct[0].radius=0;									//radius
//	draw_trends.grapic_data_struct[0].end_x=1700;								//end_x
//	draw_trends.grapic_data_struct[0].end_y=780;								//end_y
//	
//	//摩擦轮
//	draw_trends.grapic_data_struct[1].graphic_tpye=2;						//graphic_tpye
//	if(task_flag.frictiongear == 1)	     draw_trends.grapic_data_struct[1].color=5;									//color
//	else								 draw_trends.grapic_data_struct[1].color=8;									//color
//	draw_trends.grapic_data_struct[1].start_angle=0;								//空
//	draw_trends.grapic_data_struct[1].end_angle=360;									//空
//	draw_trends.grapic_data_struct[1].width=5;									//width
//	draw_trends.grapic_data_struct[1].start_x=1660;							//start_x
//	draw_trends.grapic_data_struct[1].start_y=740;							//start_y
//	draw_trends.grapic_data_struct[1].radius=35;									//radius
//	draw_trends.grapic_data_struct[1].end_x=0;								//end_x
//	draw_trends.grapic_data_struct[1].end_y=0;								//end_y

//	//小陀螺	
//	draw_trends.grapic_data_struct[2].graphic_tpye=2;						//graphic_tpye
//	if(task_flag.small_gyro == 1) 		draw_trends.grapic_data_struct[2].color=5;		//color
//	else    							draw_trends.grapic_data_struct[2].color=8;	
//	draw_trends.grapic_data_struct[2].start_angle=0;								//空
//	draw_trends.grapic_data_struct[2].end_angle=360;									//空
//	draw_trends.grapic_data_struct[2].width=5;									//width
//	draw_trends.grapic_data_struct[2].start_x=1790;							//start_x
//	draw_trends.grapic_data_struct[2].start_y=740;							//start_y
//	draw_trends.grapic_data_struct[2].radius=35;									//radius
//	draw_trends.grapic_data_struct[2].end_x=0;								//end_x
//	draw_trends.grapic_data_struct[2].end_y=0;								//end_y
//	
//	//弹舱
//	draw_trends.grapic_data_struct[3].graphic_tpye=0;						//graphic_tpye
//	if(varible_send.lock_pill == 1)		draw_trends.grapic_data_struct[3].color=2;									//color		 //堵转
//	else   								draw_trends.grapic_data_struct[3].color=4;		   
//	draw_trends.grapic_data_struct[3].start_angle=0;								//空
//	draw_trends.grapic_data_struct[3].end_angle=0;									//空
//	draw_trends.grapic_data_struct[3].width=8;									//width
//	draw_trends.grapic_data_struct[3].start_x=1650;							//start_x
//	draw_trends.grapic_data_struct[3].start_y=650;							//start_y
//	draw_trends.grapic_data_struct[3].radius=0;									//radius
//	draw_trends.grapic_data_struct[3].end_x=1720;								//end_x
//	if(task_flag.pill_depot == 1)	  draw_trends.grapic_data_struct[3].end_y=700;									//color
//	else 						 draw_trends.grapic_data_struct[3].end_y=650;								//end_y
	  
	qwer++;
		
	
	/*头配置*/
	draw_trends.Frameheader.SOF = 0xA5; 					//0xA5
  draw_trends.Frameheader.DataLength=6+75;		//头结构长度+数据段长度
	draw_trends.Frameheader.Seq=seq;							//包序号
	/*复制头数据至发送缓冲*/
	memcpy(uart6_tx_buff, &draw_trends.Frameheader, sizeof(frame_header_t));
	/*CRC8校验头*/
	Append_CRC8_Check_Sum(uart6_tx_buff,sizeof(frame_header_t)); 
	/*复制数据段至发送缓冲*/
  memcpy(uart6_tx_buff + 5, 
				 (uint8_t*)&draw_trends.cmd_id, 
				 (sizeof(draw_trends.cmd_id)+ sizeof(draw_trends.Client_Custom_ID)+ sizeof(draw_trends.grapic_data_struct)));
	/*CRC16校验整包数据*/
	Append_CRC16_Check_Sum(uart6_tx_buff, draw_trends.Frameheader.DataLength+9);			
}


/*清理UI*/
void UI_all_clear(uint8_t coverage)
{
	/*读取机器人颜色和ID*/
	Red_or_Blue_judge();
	Robot_client_ID();
	/*定义*/
	static ext_ui_clear draw_clear;
	static uint8_t seq=0;
	/*ID设置*/
	draw_clear.cmd_id = 0x0301;  
	draw_clear.Client_Custom_ID.data_cmd_id= 0x0100; 
	draw_clear.Client_Custom_ID.sender_ID = Robot_ID_Data.robot_ID;
	draw_clear.Client_Custom_ID.receiver_ID = Robot_ID_Data.robot_client_ID;	
	
	draw_clear.grapic_data_struct.operate_tpye=2;								//图形操作
	draw_clear.grapic_data_struct.layer=coverage;								//图层数
	
	/*头配置*/
	draw_clear.Frameheader.SOF = 0xA5; 					//0xA5
  draw_clear.Frameheader.DataLength=6+2;		//头结构长度+数据段长度
	draw_clear.Frameheader.Seq=seq;							//包序号
	/*复制头数据至发送缓冲*/
	memcpy(uart6_tx_buff, &draw_clear.Frameheader, sizeof(frame_header_t));
	/*CRC8校验头*/
	Append_CRC8_Check_Sum(uart6_tx_buff,sizeof(frame_header_t)); 
	/*复制数据段至发送缓冲*/
  memcpy(uart6_tx_buff + 5, 
				 (uint8_t*)&draw_clear.cmd_id, 
				 (sizeof(draw_clear.cmd_id)+ sizeof(draw_clear.Client_Custom_ID)+ sizeof(draw_clear.grapic_data_struct)));
	/*CRC16校验整包数据*/
	Append_CRC16_Check_Sum(uart6_tx_buff, draw_clear.Frameheader.DataLength+9);	
	
	
	
}







