#ifndef __RC__
#define __RC__
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "function_lib.h"
//#include "key_task.h"

#define KEY_W 1
#define KEY_S 2
#define KEY_A 4
#define KEY_D 8
#define KEY_SHIFT 16
#define KEY_CTRL 32

#define KEY_CTRL_W 33

#define KEY_Q 64
#define KEY_SHIFT_Q 80
#define KEY_CTRL_Q 96
#define KEY_E 128
#define KEY_SHIFT_E 144
#define KEY_CTRL_E 160
#define KEY_R 256
#define KEY_SHIFT_R 272
#define KEY_CTRL_R 288
#define KEY_F 512
#define KEY_SHIFT_F 528
#define KEY_CTRL_F 544
#define KEY_G 1024
#define KEY_SHIFT_G 1040
#define KEY_CTRL_G 1056
#define KEY_Z 2048
#define KEY_SHIFT_Z 2064
#define KEY_CTRL_Z 2080
#define KEY_X 4096
#define KEY_SHIFT_X 4112
#define KEY_CTRL_X 4128
#define KEY_C 8192
#define KEY_SHIFT_C 8208
#define KEY_CTRL_C 8224
#define KEY_V 16384
#define KEY_SHIFT_V 16400
#define KEY_CTRL_V 16416
#define KEY_B 32768
#define KEY_SHIFT_B 32784
#define KEY_CTRL_B 32800

#define KEY_Z_F 2560
#define KEY_Z_G 3072
#define KEY_X_F 4608
#define KEY_X_G 5120
#define KEY_C_F 8704
#define KEY_C_G 9216
#define KEY_F_G 1536
#define KEY_V_F 16896
#define KEY_V_G 17408
#define KEY_C_V 24576
#define KEY_Z_X_C_V 30720

typedef struct {
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	int16_t ch5;
	uint8_t switch_left;	//3 value  1--3--2
	uint8_t switch_right;
	
	/*鼠标*/
	struct {
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_left;
		uint8_t press_right;
	}mouse;
	
	/*键盘*/
	struct {
			uint16_t key_code;
	}keyBoard;
	
	int16_t mouse_x;
	int16_t mouse_y;
}RC_Type;

extern RC_Type RC_DR16;													//外部接收遥控数据

void Callback_RC_Handle(RC_Type* rc, uint8_t* buff);

#endif


