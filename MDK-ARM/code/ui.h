#ifndef __UI_H
#define __UI_H	 

#include "key_task.h" 
#include "referee.h"
#include "function_lib.h"



void UI_task(void);
void UI_all_clear(uint8_t coverage);
//��λ��-������׼
void draw_UI_line_1	(uint16_t cmd_id,	uint16_t data_id,uint32_t operate_tpye);
/*������׼��*/
void draw_UI_line_2(uint16_t cmd_id,	uint16_t data_id,uint32_t operate_tpye);
/*״̬*/
void draw_UI_line_3(uint16_t cmd_id,	uint16_t data_id,uint32_t operate_tpye);
#endif
