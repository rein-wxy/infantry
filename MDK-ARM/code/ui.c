#include "UI.h"


/*ui��������
	
	����
	������׹��
//	��λ��
	��־�����գ����飬����
	



*/





///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int qwer;
uint8_t ui_init_flag;
uint16_t ui_task_flag;
uint8_t ui_flag;
void UI_task(void)
{

	/*************
	��ʼʱ���UI
	������DMA
	*************/
	if(ui_init_flag < 2)
	{
		//����ͼ��
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
	�����޸ģ�ɾ��
	******************/
	//���ӣ��޸�,ɾ��
	else if(ui_init_flag >= 2)
	{
		if(ui_task_flag < 10)
		{
			if(ui_task_flag % 3 == 0)
			{
				draw_UI_line_1(0x0301,		0x0104,     1);
									/*�����ַ��� ��������ID��ͼ�β���*/
				HAL_UART_Transmit(&huart6,uart6_tx_buff,130,50);
			}
			else if(ui_task_flag % 3 == 1)
			{
				draw_UI_line_2(0x0301,		0x0103,     1);
									/*�����ַ��� ��������ID��ͼ�β���*/
				HAL_UART_Transmit(&huart6,uart6_tx_buff,130,50);
			}
			else if(ui_task_flag % 3 == 2)
			{
				draw_UI_line_3(0x0301,		0x0103,     1);
									/*�����ַ��� ��������ID��ͼ�β���*/
				HAL_UART_Transmit(&huart6,uart6_tx_buff,130,50);
			}
		}
		else if(ui_task_flag >= 10 && ui_task_flag < 90)
		{
			if(ui_task_flag % 3 == 0)
			{
				draw_UI_line_1(0x0301,		0x0104,     2);
									/*�����ַ��� ��������ID��ͼ�β���*/
				HAL_UART_Transmit(&huart6,uart6_tx_buff,130,50);
			}
			else if(ui_task_flag % 3 == 1)
			{
				draw_UI_line_2(0x0301,		0x0103,     2);
									/*�����ַ��� ��������ID��ͼ�β���*/
			HAL_UART_Transmit(&huart6,uart6_tx_buff,130,50);
			}
			else if(ui_task_flag % 3 == 2)
			{
				draw_UI_line_3(0x0301,		0x0103,     2  );
									/*�����ַ��� ��������ID��ͼ�β���*/
				HAL_UART_Transmit(&huart6,uart6_tx_buff,130,50);
			}
		}
		
		else if(ui_task_flag >= 90)
			ui_init_flag = 0;
		ui_task_flag ++;
	}
	
	
}





#define line_0 975




/*�����˼�ͨ��*/
void robot_robot_data_tx(uint16_t data_id	)					//��������ID											
{
	Robot_client_ID();//��ȡ������ID
	/*����*/
	static ext_robot_tx draw_p;
	static uint8_t seq=0;
	/*ID����*/
	draw_p.cmd_id = 0x0301;  	
	draw_p.Client_Custom_ID.data_cmd_id= data_id; 													//��������ID
	draw_p.Client_Custom_ID.sender_ID = Robot_ID_Data.robot_ID;							//�����ߵ�ID
	if(Red_or_Blue_judge() == BLUE)				//������ɫ
		draw_p.Client_Custom_ID.receiver_ID = 107;													//�����ߵ�ID
	else if(Red_or_Blue_judge() == RED)				//������ɫ
		draw_p.Client_Custom_ID.receiver_ID = 7;													//�����ߵ�ID
	
	/*���͵�����*/
	draw_p.robot_interactive_data.data[0] = 0xAA;
	/*ͷ����*/
	draw_p.Frameheader.SOF = 0xA5; 					//0xA5
  draw_p.Frameheader.DataLength=6+1;			//ͷ�ṹ����+���ݶγ���
	draw_p.Frameheader.Seq=seq;							//�����
	/*����ͷ���������ͻ���*/
	memcpy(uart6_tx_buff, &draw_p.Frameheader, sizeof(frame_header_t));
	/*CRC8У��ͷ*/
	Append_CRC8_Check_Sum(uart6_tx_buff,sizeof(frame_header_t)); 

	/*�������ݶ������ͻ���*/
    memcpy(uart6_tx_buff + 5,
				 (uint8_t*)&draw_p.cmd_id, 
				 (sizeof(draw_p.cmd_id) + sizeof(draw_p.Client_Custom_ID)+ 1));
	/*CRC16У����������*/
	Append_CRC16_Check_Sum(uart6_tx_buff, draw_p.Frameheader.DataLength+9);					//���ݶ�+5+2+2

}


//��λ��-������׼
void draw_UI_line_1	 
	(uint16_t cmd_id,	uint16_t data_id,uint32_t operate_tpye)
 /*�����ַ���				��������ID��		 ͼ�β���*/
{
	/*��ȡ��������ɫ��ID*/
	Red_or_Blue_judge();
	Robot_client_ID();
	/*����*/
	static ext_draw_line_7 draw_1;
	static uint8_t seq=1;
	/*ID����*/
	 draw_1.cmd_id = cmd_id;  
	 draw_1.Client_Custom_ID.data_cmd_id= data_id; 
	 draw_1.Client_Custom_ID.sender_ID = Robot_ID_Data.robot_ID;
	 draw_1.Client_Custom_ID.receiver_ID = Robot_ID_Data.robot_client_ID;	
	/*����*/
	for(int i = 0 ;i < 7 ;i++)
	{
		 draw_1.grapic_data_struct[i].graphic_name[0]=i;					//ͼ������
		 draw_1.grapic_data_struct[i].operate_tpye=operate_tpye;		//ͼ�β���
		 draw_1.grapic_data_struct[i].layer=0;											//ͼ����
//		 draw_1.grapic_data_struct[i].start_angle=0;								//��
//		 draw_1.grapic_data_struct[i].end_angle=0;									//��
		// draw_1.grapic_data_struct[i].radius=0;										//��
	}
	uint8_t car_color;
	if(	task_flag.cap_mode==0) 	     car_color = 8;									//ͼ����ɫ0������ɫ1��ɫ2��ɫ3��ɫ4�Ϻ�ɫ5��ɫ6��ɫ7��ɫ8��ɫ
	else if(task_flag.cap_mode==1)	 car_color = 1;
	else if(task_flag.cap_mode==2)	 car_color = 4;
	
	
	//����
	draw_1.grapic_data_struct[0].graphic_tpye=0;							//ͼ������
	draw_1.grapic_data_struct[0].width=16;											//�������
	draw_1.grapic_data_struct[0].color=car_color;
	draw_1.grapic_data_struct[0].start_x=660;							//�������X
	draw_1.grapic_data_struct[0].start_y=84;								//�������Y
	draw_1.grapic_data_struct[0].end_x=660+6*super_cap.cap_capacity;								//�յ�����X
	draw_1.grapic_data_struct[0].end_y=84;								//�յ�����Y
	draw_1.grapic_data_struct[0].start_angle=0;								//��
	draw_1.grapic_data_struct[0].end_angle=0;									//��
	draw_1.grapic_data_struct[0].radius=0;
	
	//���ݿ�
	draw_1.grapic_data_struct[1].graphic_tpye=1;							//ͼ������
	draw_1.grapic_data_struct[1].width=2;											//�������
	draw_1.grapic_data_struct[1].color=car_color;
	draw_1.grapic_data_struct[1].start_x=660;							//�������X
	draw_1.grapic_data_struct[1].start_y=76;								//�������Y
	draw_1.grapic_data_struct[1].end_x=1260;								//�յ�����X
	draw_1.grapic_data_struct[1].end_y=92;								//�յ�����Y
	draw_1.grapic_data_struct[1].start_angle=0;								//��
	draw_1.grapic_data_struct[1].end_angle=0;									//��
	draw_1.grapic_data_struct[1].radius=0;




//����
	
	draw_1.grapic_data_struct[2].graphic_tpye=1;						//graphic_tpye
	if(varible_vision.aim_flag  == 1) 
		draw_1.grapic_data_struct[2].color= 5 ;									//color
	if(task_flag.hit_mode == 1 && varible_vision.aim_flag == 1)			
		draw_1.grapic_data_struct[2].color= 1 ;
	if(varible_vision.aim_flag  == 0)
		draw_1.grapic_data_struct[2].color= 0 ;
	draw_1.grapic_data_struct[2].start_angle=0;								//��
	draw_1.grapic_data_struct[2].end_angle=0;									//��
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
//	draw_1.grapic_data_struct[2].start_angle=0;								//��
//	draw_1.grapic_data_struct[2].end_angle=0;									//��
//	draw_1.grapic_data_struct[2].width=4;									//width	
//	draw_1.grapic_data_struct[2].start_x=line_0;							//start_x
//	draw_1.grapic_data_struct[2].start_y=540;
//	if(vision_rx.date_update == true)	//start_y
//		draw_1.grapic_data_struct[2].radius=220;									//radius
//	else
//		draw_1.grapic_data_struct[2].radius=10;	
//	draw_1.grapic_data_struct[2].end_x=0;								//end_x
//	draw_1.grapic_data_struct[2].end_y=0;								//end_y						//end_yp
	
	//Ħ����
	draw_1.grapic_data_struct[5].graphic_tpye=2;						//graphic_tpye
	if(task_flag.frictiongear == 1)	     draw_1.grapic_data_struct[5].color=5;									//color
	else								 draw_1.grapic_data_struct[5].color=8;									//color
	draw_1.grapic_data_struct[5].start_angle=0;								//��
	draw_1.grapic_data_struct[5].end_angle=0;									//��
	draw_1.grapic_data_struct[5].width=5;									//width
	draw_1.grapic_data_struct[5].start_x=1660;							//start_x
	draw_1.grapic_data_struct[5].start_y=740;							//start_y
	draw_1.grapic_data_struct[5].radius=35;									//radius
	draw_1.grapic_data_struct[5].end_x=0;								//end_x
	draw_1.grapic_data_struct[5].end_y=0;								//end_y

	//С����	
	draw_1.grapic_data_struct[3].graphic_tpye=2;						//graphic_tpye
	if(task_flag.small_gyro == 1) 		draw_1.grapic_data_struct[3].color=5;		//color
	else    							draw_1.grapic_data_struct[3].color=8;	
	draw_1.grapic_data_struct[3].start_angle=0;								//��
	draw_1.grapic_data_struct[3].end_angle=360;									//��
	draw_1.grapic_data_struct[3].width=5;									//width
	draw_1.grapic_data_struct[3].start_x=1790;							//start_x
	draw_1.grapic_data_struct[3].start_y=740;							//start_y
	draw_1.grapic_data_struct[3].radius=35;									//radius
	draw_1.grapic_data_struct[3].end_x=0;								//end_x
	draw_1.grapic_data_struct[3].end_y=0;								//end_y
	
	//����
	draw_1.grapic_data_struct[4].graphic_tpye=0;						//graphic_tpye
	if(varible_send.lock_pill == 1)		draw_1.grapic_data_struct[4].color=2;									//color		 //��ת
	else   								draw_1.grapic_data_struct[4].color=4;		   
	draw_1.grapic_data_struct[4].start_angle=0;								//��
	draw_1.grapic_data_struct[4].end_angle=0;									//��
	draw_1.grapic_data_struct[4].width=8;									//width
	draw_1.grapic_data_struct[4].start_x=1640;							//start_x
	draw_1.grapic_data_struct[4].start_y=650;							//start_y
	draw_1.grapic_data_struct[4].radius=0;									//radius
	draw_1.grapic_data_struct[4].end_x=1730;								//end_x
	if(task_flag.pill_depot == 1)	  draw_1.grapic_data_struct[4].end_y=700;									//color
	else 						 draw_1.grapic_data_struct[4].end_y=650;								//end_y

//	
//	//����λ�ÿ�ͼ
//	draw_1.grapic_data_struct[2].graphic_tpye=0;							//ͼ������
//	draw_1.grapic_data_struct[2].width=3;											//�������
//	draw_1.grapic_data_struct[2].color=6;
//	draw_1.grapic_data_struct[2].start_x=640-100;							//�������X
//	draw_1.grapic_data_struct[2].start_y=0;								//�������Y
//	draw_1.grapic_data_struct[2].end_x=740-50;								//�յ�����X
//	draw_1.grapic_data_struct[2].end_y=272;								//�յ�����Y
//	//
//	draw_1.grapic_data_struct[3].graphic_tpye=0;							//ͼ������
//	draw_1.grapic_data_struct[3].width=3;											//�������
//	draw_1.grapic_data_struct[3].color=6;
//	draw_1.grapic_data_struct[3].start_x=1280+100;							//�������X
//	draw_1.grapic_data_struct[3].start_y=0;							//�������Y
//	draw_1.grapic_data_struct[3].end_x=1180+50;								//�յ�����X
//	draw_1.grapic_data_struct[3].end_y=272;								//�յ�����Y
//	//
//	draw_1.grapic_data_struct[4].graphic_tpye=0;							//ͼ������
//	draw_1.grapic_data_struct[4].width=3;											//�������
//	draw_1.grapic_data_struct[4].color=6;
//	draw_1.grapic_data_struct[4].start_x=1180+50;							//�������X
//	draw_1.grapic_data_struct[4].start_y=272;							//�������Y
//	draw_1.grapic_data_struct[4].end_x=1080;								//�յ�����X
//	draw_1.grapic_data_struct[4].end_y=272;								//�յ�����Y

//	draw_1.grapic_data_struct[5].graphic_tpye=0;							//ͼ������
//	draw_1.grapic_data_struct[5].width=3;											//�������
//	draw_1.grapic_data_struct[5].color=6;
//	draw_1.grapic_data_struct[5].start_x=740-50;							//�������X
//	draw_1.grapic_data_struct[5].start_y=272;							//�������Y
//	draw_1.grapic_data_struct[5].end_x=840;								//�յ�����X
//	draw_1.grapic_data_struct[5].end_y=272;								
	
	
	/*ͷ����*/
	draw_1.Frameheader.SOF = 0xA5; 					//0xA5
 draw_1.Frameheader.DataLength=6+105;		//ͷ�ṹ����+���ݶγ���
	draw_1.Frameheader.Seq=seq;							//�����
	/*����ͷ���������ͻ���*/
	memcpy(uart6_tx_buff, &draw_1.Frameheader, sizeof(frame_header_t));
	/*CRC8У��ͷ*/
	Append_CRC8_Check_Sum(uart6_tx_buff,sizeof(frame_header_t)); 
	/*�������ݶ������ͻ���*/
  memcpy(uart6_tx_buff + 5, 
				 (uint8_t*)&draw_1.cmd_id, 
				 (sizeof(draw_1.cmd_id)+ sizeof(draw_1.Client_Custom_ID)+ sizeof(draw_1.grapic_data_struct)));
	/*CRC16У����������*/
	Append_CRC16_Check_Sum(uart6_tx_buff, draw_1.Frameheader.DataLength+9);
}


/*������׼��*/
void draw_UI_line_2	 
	(uint16_t cmd_id,	uint16_t data_id,uint32_t operate_tpye)
 /*�����ַ���				��������ID��		 ͼ�β���*/
{
	/*��ȡ��������ɫ��ID*/
	Red_or_Blue_judge();
	Robot_client_ID();
	/*����*/
	static ext_draw_line_5_1 draw_static;  //����
	static uint8_t seq=0;
	/*ID����*/
	draw_static.cmd_id = cmd_id;  
	draw_static.Client_Custom_ID.data_cmd_id= data_id; 
	draw_static.Client_Custom_ID.sender_ID = Robot_ID_Data.robot_ID;
	draw_static.Client_Custom_ID.receiver_ID = Robot_ID_Data.robot_client_ID;	

	/*����*/
	for(int i = 0 ;i < 5 ;i++)
	{
		draw_static.grapic_data_struct[i].graphic_name[0]=i+7;					//ͼ������
		draw_static.grapic_data_struct[i].operate_tpye=operate_tpye;		//ͼ�β���
		draw_static.grapic_data_struct[i].graphic_tpye=0;							//ͼ������
		draw_static.grapic_data_struct[i].layer=1;											//ͼ����
		draw_static.grapic_data_struct[i].start_angle=0;								//��
		draw_static.grapic_data_struct[i].end_angle=0;									//��
		draw_static.grapic_data_struct[i].radius=0;										//��
		draw_static.grapic_data_struct[i].width=2;											//�������
	}
	/*0������ɫ1��ɫ2��ɫ3��ɫ4�Ϻ�ɫ5��ɫ6��ɫ7��ɫ8��ɫ*/
	
	/*����*/
	
	draw_static.grapic_data_struct[0].color=0;
	draw_static.grapic_data_struct[0].start_x=line_0 ;							//�������X
	draw_static.grapic_data_struct[0].start_y=250;								//�������Y
	draw_static.grapic_data_struct[0].end_x=line_0 ;								//�յ�����X
	draw_static.grapic_data_struct[0].end_y=540;								//�յ�����Y
	
	/*��1*/
	#define Line_1 484
	draw_static.grapic_data_struct[1].color = 1;
	draw_static.grapic_data_struct[1].start_x=line_0 - 30;							//�������X
	draw_static.grapic_data_struct[1].start_y=Line_1;							//�������Y
	draw_static.grapic_data_struct[1].end_x=line_0 + 30;							//�յ�����X
	draw_static.grapic_data_struct[1].end_y=Line_1;								//�յ�����Y
	/*��2*/
	#define Line_2 400
	draw_static.grapic_data_struct[2].color = 2;
	draw_static.grapic_data_struct[2].start_x=line_0 - 60;							//�������X
	draw_static.grapic_data_struct[2].start_y=Line_2;							//�������Y
	draw_static.grapic_data_struct[2].end_x=line_0 + 60;							//�յ�����X
	draw_static.grapic_data_struct[2].end_y=Line_2;								//�յ�����Y
	/*��3*/
	#define Line_3 350
	draw_static.grapic_data_struct[3].color = 3;
	draw_static.grapic_data_struct[3].start_x=line_0 + 90;							//�������X
	draw_static.grapic_data_struct[3].start_y=Line_3;							//�������Y
	draw_static.grapic_data_struct[3].end_x=line_0 - 90;							//�յ�����X
	draw_static.grapic_data_struct[3].end_y=Line_3;								//�յ�����Y
	/*��4*/
	#define Line_4 360
	draw_static.grapic_data_struct[4].color = 4;
	draw_static.grapic_data_struct[4].start_x=line_0 + 110;							//�������X
	draw_static.grapic_data_struct[4].start_y=Line_4;							//�������Y
	draw_static.grapic_data_struct[4].end_x=line_0 + 110;							//�յ�����X
	draw_static.grapic_data_struct[4].end_y=Line_4;								//�յ�����Y
	
	/*ͷ����*/
	draw_static.Frameheader.SOF = 0xA5; 					//0xA5
  draw_static.Frameheader.DataLength=6+75;		//ͷ�ṹ����+���ݶγ���
	draw_static.Frameheader.Seq=seq;							//�����
	/*����ͷ���������ͻ���*/
	memcpy(uart6_tx_buff, &draw_static.Frameheader, sizeof(frame_header_t));
	/*CRC8У��ͷ*/
	Append_CRC8_Check_Sum(uart6_tx_buff,sizeof(frame_header_t)); 
	/*�������ݶ������ͻ���*/
  memcpy(uart6_tx_buff + 5, 
				 (uint8_t*)&draw_static.cmd_id, 
				 (sizeof(draw_static.cmd_id)+ sizeof(draw_static.Client_Custom_ID)+ sizeof(draw_static.grapic_data_struct)));
	/*CRC16У����������*/
	Append_CRC16_Check_Sum(uart6_tx_buff, draw_static.Frameheader.DataLength+9);
}


/*7��*/
void draw_UI_line_3	 
	(uint16_t cmd_id,	uint16_t data_id,uint32_t operate_tpye)
 /*�����ַ���				��������ID��		 ͼ�β���*/
{
	/*��ȡ��������ɫ��ID*/
	Red_or_Blue_judge();
	Robot_client_ID();
	/*����*/
	static ext_draw_line_5_2 draw_trends;
	static uint8_t seq=1;
	/*ID����*/
	draw_trends.cmd_id = cmd_id;  
	draw_trends.Client_Custom_ID.data_cmd_id= data_id; 
	draw_trends.Client_Custom_ID.sender_ID = Robot_ID_Data.robot_ID;
	draw_trends.Client_Custom_ID.receiver_ID = Robot_ID_Data.robot_client_ID;	
	/*����*/
	for(int i = 0 ;i < 5 ;i++)
	{
		draw_trends.grapic_data_struct[i].graphic_name[0]=i+12;					//ͼ������
		draw_trends.grapic_data_struct[i].operate_tpye=operate_tpye;		//ͼ�β���
		//draw_trends.grapic_data_struct[i].graphic_tpye=0;							//ͼ������
		draw_trends.grapic_data_struct[i].layer=3;											//ͼ����
		draw_trends.grapic_data_struct[i].start_angle=0;								//��
		 draw_trends.grapic_data_struct[i].end_angle=0;									//��
		draw_trends.grapic_data_struct[i].radius=0;	
	}
	//����λ�ÿ�ͼ
	draw_trends.grapic_data_struct[2].graphic_tpye=0;							//ͼ������
	draw_trends.grapic_data_struct[2].width=3;											//�������
	draw_trends.grapic_data_struct[2].color=6;
	draw_trends.grapic_data_struct[2].start_x=640-100;							//�������X
	draw_trends.grapic_data_struct[2].start_y=0;								//�������Y
	draw_trends.grapic_data_struct[2].end_x=740-50;								//�յ�����X
	draw_trends.grapic_data_struct[2].end_y=272;								//�յ�����Y
	//
	draw_trends.grapic_data_struct[3].graphic_tpye=0;							//ͼ������
	draw_trends.grapic_data_struct[3].width=3;											//�������
	draw_trends.grapic_data_struct[3].color=6;
	draw_trends.grapic_data_struct[3].start_x=1280+100;							//�������X
	draw_trends.grapic_data_struct[3].start_y=0;							//�������Y
	draw_trends.grapic_data_struct[3].end_x=1180+50;								//�յ�����X
	draw_trends.grapic_data_struct[3].end_y=272;								//�յ�����Y
	//
	draw_trends.grapic_data_struct[4].graphic_tpye=0;							//ͼ������
	draw_trends.grapic_data_struct[4].width=3;											//�������
	draw_trends.grapic_data_struct[4].color=6;
	draw_trends.grapic_data_struct[4].start_x=1180+50;							//�������X
	draw_trends.grapic_data_struct[4].start_y=272;							//�������Y
	draw_trends.grapic_data_struct[4].end_x=1080;								//�յ�����X
	draw_trends.grapic_data_struct[4].end_y=272;								//�յ�����Y
	
	draw_trends.grapic_data_struct[1].graphic_tpye=0;							//ͼ������
	draw_trends.grapic_data_struct[1].width=3;											//�������
	draw_trends.grapic_data_struct[1].color=6;
	draw_trends.grapic_data_struct[1].start_x=740-50;							//�������X
	draw_trends.grapic_data_struct[1].start_y=272;							//�������Y
	draw_trends.grapic_data_struct[1].end_x=840;								//�յ�����X
	draw_trends.grapic_data_struct[1].end_y=272;								
	
	
	
	
	
	
	
//	//����
//	
//	draw_trends.grapic_data_struct[0].graphic_tpye=0;						//graphic_tpye
//	if(task_flag.automatic_aiming == 1) draw_trends.grapic_data_struct[0].color= 5 ;									//color
//	else								 draw_trends.grapic_data_struct[0].color= 8 ;			
//	draw_trends.grapic_data_struct[0].start_angle=0;								//��
//	draw_trends.grapic_data_struct[0].end_angle=0;									//��
//	draw_trends.grapic_data_struct[0].width=20;									//width
//	draw_trends.grapic_data_struct[0].start_x=1650;							//start_x
//	draw_trends.grapic_data_struct[0].start_y=780;							//start_y
//	draw_trends.grapic_data_struct[0].radius=0;									//radius
//	draw_trends.grapic_data_struct[0].end_x=1700;								//end_x
//	draw_trends.grapic_data_struct[0].end_y=780;								//end_y
//	
//	//Ħ����
//	draw_trends.grapic_data_struct[1].graphic_tpye=2;						//graphic_tpye
//	if(task_flag.frictiongear == 1)	     draw_trends.grapic_data_struct[1].color=5;									//color
//	else								 draw_trends.grapic_data_struct[1].color=8;									//color
//	draw_trends.grapic_data_struct[1].start_angle=0;								//��
//	draw_trends.grapic_data_struct[1].end_angle=360;									//��
//	draw_trends.grapic_data_struct[1].width=5;									//width
//	draw_trends.grapic_data_struct[1].start_x=1660;							//start_x
//	draw_trends.grapic_data_struct[1].start_y=740;							//start_y
//	draw_trends.grapic_data_struct[1].radius=35;									//radius
//	draw_trends.grapic_data_struct[1].end_x=0;								//end_x
//	draw_trends.grapic_data_struct[1].end_y=0;								//end_y

//	//С����	
//	draw_trends.grapic_data_struct[2].graphic_tpye=2;						//graphic_tpye
//	if(task_flag.small_gyro == 1) 		draw_trends.grapic_data_struct[2].color=5;		//color
//	else    							draw_trends.grapic_data_struct[2].color=8;	
//	draw_trends.grapic_data_struct[2].start_angle=0;								//��
//	draw_trends.grapic_data_struct[2].end_angle=360;									//��
//	draw_trends.grapic_data_struct[2].width=5;									//width
//	draw_trends.grapic_data_struct[2].start_x=1790;							//start_x
//	draw_trends.grapic_data_struct[2].start_y=740;							//start_y
//	draw_trends.grapic_data_struct[2].radius=35;									//radius
//	draw_trends.grapic_data_struct[2].end_x=0;								//end_x
//	draw_trends.grapic_data_struct[2].end_y=0;								//end_y
//	
//	//����
//	draw_trends.grapic_data_struct[3].graphic_tpye=0;						//graphic_tpye
//	if(varible_send.lock_pill == 1)		draw_trends.grapic_data_struct[3].color=2;									//color		 //��ת
//	else   								draw_trends.grapic_data_struct[3].color=4;		   
//	draw_trends.grapic_data_struct[3].start_angle=0;								//��
//	draw_trends.grapic_data_struct[3].end_angle=0;									//��
//	draw_trends.grapic_data_struct[3].width=8;									//width
//	draw_trends.grapic_data_struct[3].start_x=1650;							//start_x
//	draw_trends.grapic_data_struct[3].start_y=650;							//start_y
//	draw_trends.grapic_data_struct[3].radius=0;									//radius
//	draw_trends.grapic_data_struct[3].end_x=1720;								//end_x
//	if(task_flag.pill_depot == 1)	  draw_trends.grapic_data_struct[3].end_y=700;									//color
//	else 						 draw_trends.grapic_data_struct[3].end_y=650;								//end_y
	  
	qwer++;
		
	
	/*ͷ����*/
	draw_trends.Frameheader.SOF = 0xA5; 					//0xA5
  draw_trends.Frameheader.DataLength=6+75;		//ͷ�ṹ����+���ݶγ���
	draw_trends.Frameheader.Seq=seq;							//�����
	/*����ͷ���������ͻ���*/
	memcpy(uart6_tx_buff, &draw_trends.Frameheader, sizeof(frame_header_t));
	/*CRC8У��ͷ*/
	Append_CRC8_Check_Sum(uart6_tx_buff,sizeof(frame_header_t)); 
	/*�������ݶ������ͻ���*/
  memcpy(uart6_tx_buff + 5, 
				 (uint8_t*)&draw_trends.cmd_id, 
				 (sizeof(draw_trends.cmd_id)+ sizeof(draw_trends.Client_Custom_ID)+ sizeof(draw_trends.grapic_data_struct)));
	/*CRC16У����������*/
	Append_CRC16_Check_Sum(uart6_tx_buff, draw_trends.Frameheader.DataLength+9);			
}


/*����UI*/
void UI_all_clear(uint8_t coverage)
{
	/*��ȡ��������ɫ��ID*/
	Red_or_Blue_judge();
	Robot_client_ID();
	/*����*/
	static ext_ui_clear draw_clear;
	static uint8_t seq=0;
	/*ID����*/
	draw_clear.cmd_id = 0x0301;  
	draw_clear.Client_Custom_ID.data_cmd_id= 0x0100; 
	draw_clear.Client_Custom_ID.sender_ID = Robot_ID_Data.robot_ID;
	draw_clear.Client_Custom_ID.receiver_ID = Robot_ID_Data.robot_client_ID;	
	
	draw_clear.grapic_data_struct.operate_tpye=2;								//ͼ�β���
	draw_clear.grapic_data_struct.layer=coverage;								//ͼ����
	
	/*ͷ����*/
	draw_clear.Frameheader.SOF = 0xA5; 					//0xA5
  draw_clear.Frameheader.DataLength=6+2;		//ͷ�ṹ����+���ݶγ���
	draw_clear.Frameheader.Seq=seq;							//�����
	/*����ͷ���������ͻ���*/
	memcpy(uart6_tx_buff, &draw_clear.Frameheader, sizeof(frame_header_t));
	/*CRC8У��ͷ*/
	Append_CRC8_Check_Sum(uart6_tx_buff,sizeof(frame_header_t)); 
	/*�������ݶ������ͻ���*/
  memcpy(uart6_tx_buff + 5, 
				 (uint8_t*)&draw_clear.cmd_id, 
				 (sizeof(draw_clear.cmd_id)+ sizeof(draw_clear.Client_Custom_ID)+ sizeof(draw_clear.grapic_data_struct)));
	/*CRC16У����������*/
	Append_CRC16_Check_Sum(uart6_tx_buff, draw_clear.Frameheader.DataLength+9);	
	
	
	
}







