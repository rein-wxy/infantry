#ifndef __IMU_H
#define __IMU_H	 

#include "chassis.h"
#include "stm32f4xx_hal.h"
#include "pid.h"
#include "spi.h"
#include "math.h"
#include "BMI.h"
#include "tim.h"


#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_ACCEL_3G_SEN     0.0008974358974f        //灵敏度10920     3G/32767=0.0008974358974f M/S^2
#define BMI088_ACCEL_6G_SEN     0.00179443359375f       //灵敏度5460
#define BMI088_ACCEL_12G_SEN    0.0035888671875f        //灵敏度2730        0.000091555528427991f
#define BMI088_ACCEL_24G_SEN    0.007177734375f         //灵敏度1365


#define BMI088_GYRO_2000_SEN    0.00106526443603169529841533860381f        // 1/(16.384*57.3)  
/*陀螺仪是16位ADC,2^16=65536，如果量程为-2000——2000，65536/4000=16.384,表示16.384代表1度*/
#define BMI088_GYRO_1000_SEN    0.00053263221801584764920766930190693f     
#define BMI088_GYRO_500_SEN     0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN     0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN     0.000066579027251980956150958662738366f




typedef struct
{
  int16_t gx;
	int16_t gy;
	int16_t gz;
	
	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
}GYRO_t;
typedef struct
{
  int16_t ax;
	int16_t ay;
	int16_t az;
}ACCEL_t;


typedef struct imu_sensor_info_struct
{
	float yaw;
	float pitch;
	float roll;
	
	float yaw_speed;
	float pitch_speed;
	float roll_speed;
	
	float rate_yaw;
	float rate_pitch;
	float rate_roll;
	
	float yaw_total;
	float pitch_total;
    float yaw_last;
	float pitch_last;
	int yaw_cnt,pitch_cnt;
	
	float temperature;
	
	uint8_t offline_cnt;
	uint8_t offline_max_cnt;
	
	uint16_t kp_change_time;  //imu pid 参数相关处理
	uint8_t time_flag;
}imu_sensor_info_t;



extern float Kp; 
extern GYRO_t GYRO;
extern ACCEL_t ACCEL;
extern float YAW,PITCH,ROLL;
extern imu_sensor_info_t imu;

void imu_sensor_update(imu_sensor_info_t *imu_info);
void imu_sensor_init(void);
void BMI_Init(void);
void BMI_GET_DATA(void);
uint8_t BMI_Get_EulerAngle(float *pitch,float *roll,float *yaw);
void BMI_SET_Kp(float KP_set);
void BMI_Temperature_Ctrl(void);
void imu_task(imu_sensor_info_t *imu_info);



#endif
