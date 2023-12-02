#include "imu.h"

#define GYRO_NSS_LOW   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET)
#define GYRO_NSS_HIGH  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET)
#define ACCEL_NSS_LOW  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)
#define ACCEL_NSS_HIGH HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)




static uint8_t   tx, rx;
uint8_t  GYRO_Data[6];
uint8_t  ACCEL_Data[6];
uint8_t  TEMP_Data[2];
GYRO_t GYRO;
ACCEL_t ACCEL;
int16_t TEMP;

imu_sensor_info_t imu;



//void imu_task(void)
//{

//}


 void BMI_Temperature_Ctrl()
{
	pid_imu_temp.f_calculate(&pid_imu_temp, 1,imu.temperature , 39.5f);	
	TIM10->CCR1 = pid_imu_temp.pid_out_speed;
}

void imu_sensor_init()
{
	BMI_Init();
	pid_init(&pid_imu_temp,
						0,0,  0,0xFFFFFF, 0, 0,   0.0f, 0.0, 0.0, //外环位置环kp,ki,kd
						0,4500,4500,0xFFFFFF, 0, 0,  1600.0f, 0.2f,0.0);//内环速度环kp,ki,kd
	KalmanCreate(&rate_roll_kf, 1, 10);
	KalmanCreate(&rate_yaw_kf, 1, 10);
	KalmanCreate(&rate_pitch_kf, 1, 10);

	HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
}

void imu_task(imu_sensor_info_t *imu_info)
{
	//温控
	BMI_Temperature_Ctrl();
	
//	imu_info->rate_pitch = KalmanFilter(&rate_pitch_kf,  GYRO.gx);
//	imu_info->rate_roll  = KalmanFilter(&rate_roll_kf, GYRO.gy);
//	imu_info->rate_yaw   = KalmanFilter(&rate_yaw_kf,   GYRO.gz);
	
	//处理
	if(imu_info->yaw-imu_info->yaw_last>4096)
		imu_info->yaw_cnt--;
	else if(imu_info->yaw-imu_info->yaw_last<-4096)
		imu_info->yaw_cnt++;
	imu_info->yaw_total=imu_info->yaw_cnt*8192+imu_info->yaw;
	imu_info->yaw_last=imu_info->yaw;
	
	
	if(imu_info->pitch-imu_info->pitch_last>4096)
		imu_info->pitch_cnt--;
	else if(imu_info->pitch-imu_info->pitch_last<-4096)
		imu_info->pitch_cnt++;
	imu_info->pitch_total=imu_info->pitch_cnt*8192+imu_info->pitch;
	imu_info->pitch_last=imu_info->pitch;
	
	imu_info->pitch_speed=-GYRO.gy ;
	imu_info->yaw_speed  =GYRO.gz ;
    imu_info->roll_speed =GYRO.gx ;
	
//	imu_info->pitch_speed=imu_info->rate_roll *0.1f;
//	imu_info->yaw_speed  =imu_info->rate_yaw  *0.1f;
//    imu_info->roll_speed =imu_info->rate_pitch * 0.1f;
	/*改变KP参数*/
	
	if(imu_info->time_flag == 0 )
	{
		imu_info->kp_change_time =HAL_GetTick();
		if(imu_info->kp_change_time >= 6000 )
		{
			Kp= 0.1f;
			imu_info->time_flag = 1;
		}
		
	}
	
}
void imu_sensor_update(imu_sensor_info_t *imu_info)
{
	//读取数据
	BMI_GET_DATA();

	//角度解算
	
	
	BMI_Get_EulerAngle(&imu_info->pitch, &imu_info->roll, &imu_info->yaw);

}

uint8_t BMI088_Read_Write_Byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

uint8_t GYRO_Write_Byte(uint8_t const reg, uint8_t const data)
{
    GYRO_NSS_LOW;
	  tx = reg;
	  HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 1000);
	  tx = data;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 1000);
    GYRO_NSS_HIGH;
	  return 0;
}

uint8_t GYRO_Read_Byte(uint8_t const reg)
{
    GYRO_NSS_LOW;
	  tx = reg | 0x80;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 1000);
	  tx = 0x55;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 1000);
    GYRO_NSS_HIGH;
	  return rx;
}

uint8_t GYRO_Read_Bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len)
{
	  GYRO_NSS_LOW;
	  tx      = regAddr | 0x80;
	  HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 1000);
	  while (len != 0)
    {
	     *pData = BMI088_Read_Write_Byte(0x55);
        pData++;
        len--;
		}
	  GYRO_NSS_HIGH;
    return 0;
}

uint8_t ACCEL_Write_Byte(uint8_t const reg, uint8_t const data)
{
    ACCEL_NSS_LOW;
	  tx = reg;
	  HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 1000);
	  tx = data;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 1000);
    ACCEL_NSS_HIGH;
	  return 0; 
}

uint8_t ACCEL_Read_Byte(uint8_t const reg)
{
    ACCEL_NSS_LOW;
    tx = reg | 0x80;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 1000);
	  tx = 0x55;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 1000);
	  HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 1000);
    ACCEL_NSS_HIGH;
	  return rx;
}

uint8_t ACCEL_Read_Bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len)
{
    ACCEL_NSS_LOW;
   	tx      = regAddr | 0x80;
	  HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 1000);
	  HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 1000);
//	  tx_buff[0] = tx;
	  while (len != 0)
    {
	     *pData = BMI088_Read_Write_Byte(0x55);
        pData++;
        len--;
		}
    ACCEL_NSS_HIGH;
	  return 0;
}
/**
	* @brief  set BMI gyroscope measure range
  * @param  fsr: range(0,2000dps;1,1000dps;2,500dps;3,250dps;4,125dps)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t GYRO_SET_Range(uint8_t Range)
{
   return GYRO_Write_Byte( BMI088_GYRO_RANGE, Range << BMI088_GYRO_RANGE_SHFITS  );
}
/**
	* @brief  set imu 6050/6500 accelerate measure range
  * @param  fsr: range(0,3g;1,6g;2,12g;3,24g)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t ACCEL_SET_Range(uint8_t Range)
{
   return ACCEL_Write_Byte( BMI088_ACC_RANGE, Range << BMI088_ACC_RANGE_SHFITS  );
}

uint8_t GYRO_ID;
void GYRO_Init(void)
{
   
	 GYRO_ID = GYRO_Read_Byte(BMI088_GYRO_CHIP_ID);
	 HAL_Delay(1);
	 GYRO_Write_Byte( BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE );
	 HAL_Delay(80);
	 GYRO_ID = GYRO_Read_Byte(BMI088_GYRO_CHIP_ID);
	 HAL_Delay(1);
	 GYRO_ID = GYRO_Read_Byte(BMI088_GYRO_CHIP_ID);
	 HAL_Delay(1);
	 uint8_t i                        = 0;
	 uint8_t BMI088_GYRO_Init_Data[6][2] = {  
												{ BMI088_GYRO_RANGE, BMI088_GYRO_2000 },     //更改量程
												{ BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set },    
												{ BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE },         
												{ BMI088_GYRO_CTRL, BMI088_DRDY_ON },   
												{ BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW },   
												{ BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3 },  
																		 };    
	
	for (i = 0; i < 6; i++)
	{
		GYRO_Write_Byte(BMI088_GYRO_Init_Data[i][0], BMI088_GYRO_Init_Data[i][1]);
		HAL_Delay(1);
	}
//	GYRO_SET_Range(0x0);
}	


uint8_t ACCEL_ID;
void ACCEL_Init(void)
{
   
	 ACCEL_ID = ACCEL_Read_Byte(BMI088_ACC_CHIP_ID);
	 HAL_Delay(1);
	 ACCEL_Write_Byte(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE );
	 HAL_Delay(80);
	 ACCEL_ID = ACCEL_Read_Byte(BMI088_ACC_CHIP_ID);
	 HAL_Delay(1);
	 ACCEL_ID = ACCEL_Read_Byte(BMI088_ACC_CHIP_ID);
	 HAL_Delay(1);
	 uint8_t i                        = 0;
	 uint8_t BMI088_ACCEL_Init_Data[6][2] = {
                                      { BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON }, 
                                    	{ BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE },      
                                      { BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set },
											    						{ BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G }, 		//在这里直接更改量程								 
																			{ BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW },   /* +-8G */ 
																			{ BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT }, 
																		 };    
	
	for (i = 0; i < 6; i++)
	{
		ACCEL_Write_Byte(BMI088_ACCEL_Init_Data[i][0], BMI088_ACCEL_Init_Data[i][1]);
		HAL_Delay(1);
	}

//  ACCEL_SET_Range(0x0);

}

void GYRO_Date_OFFSET(void)
{
	GYRO.gx_offset = 0;
	GYRO.gy_offset = 0;
	GYRO.gz_offset = 0;
	int i=0,cnt=0,sum_x=0,sum_y=0,sum_z=0;
	for (i=0; i<10000;i++)
	{
  GYRO_Read_Bytes(BMI088_GYRO_X_L,GYRO_Data,6);
	GYRO.gx_offset = (int16_t)((GYRO_Data[1]) << 8) | GYRO_Data[0];
	GYRO.gy_offset = (int16_t)((GYRO_Data[3]) << 8) | GYRO_Data[2];
	GYRO.gz_offset = (int16_t)((GYRO_Data[5]) << 8) | GYRO_Data[4];
  if(i>1000)// 前300次数据不用
		{
		sum_x += GYRO.gx_offset ;
		sum_y += GYRO.gy_offset ; 
		sum_z += GYRO.gz_offset ;
		cnt++;
//		HAL_Delay(1);
		}
	}
  GYRO.gx_offset = sum_x / cnt;
	GYRO.gy_offset = sum_y / cnt;
	GYRO.gz_offset = sum_z / cnt;
}

void BMI_Init(void)
{
  ACCEL_Init();
	HAL_Delay(10);
  GYRO_Init();
  HAL_Delay(10);
	
	GYRO_Date_OFFSET();
}

void GYRO_GET_DaTa()
{
    GYRO_Read_Bytes(BMI088_GYRO_X_L,GYRO_Data,6);             //陀螺仪数据长度为16位
    GYRO.gx   = (((int16_t)((GYRO_Data[1]) << 8) | GYRO_Data[0])-GYRO.gx_offset);
    GYRO.gy   = (((int16_t)((GYRO_Data[3]) << 8) | GYRO_Data[2])-GYRO.gy_offset);
    GYRO.gz   = (((int16_t)((GYRO_Data[5]) << 8) | GYRO_Data[4])-GYRO.gz_offset);
}

void ACCEL_GET_DaTa()
{
    ACCEL_Read_Bytes(BMI088_ACCEL_XOUT_L,ACCEL_Data,6);
    ACCEL.ax   =    (int16_t)((ACCEL_Data[1]) << 8) | ACCEL_Data[0];
    ACCEL.ay   =    (int16_t)((ACCEL_Data[3]) << 8) | ACCEL_Data[2];
    ACCEL.az   =    (int16_t)((ACCEL_Data[5]) << 8) | ACCEL_Data[4];
}



void TEMP_GET_Data()
{  
	 
   ACCEL_Read_Bytes(BMI088_TEMP_M,TEMP_Data,2);               //温度数据长度为11位
	 TEMP = (int16_t)((TEMP_Data[0] << 3) | (TEMP_Data[1] >> 5));
	  if (TEMP > 1023)
    {
        TEMP -= 2048;
    }
	  imu.temperature = TEMP * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

void BMI_GET_DATA(void)
{
   GYRO_GET_DaTa();
	 ACCEL_GET_DaTa();
	 TEMP_GET_Data();
}


/**
  * @brief  fast inverse square-root, to calculate 1/Sqrt(x)
  * @param  x: the number need to be calculated
  * @retval 1/Sqrt(x)
  * @usage  call in imu_ahrs_update() function
  */
float inv_sqrt(float x) 
{
	float halfx = 0.5f * x;
	float y     = x;
	long  i     = *(long*)&y;
	
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	
	return y;
}


/**
    @param
    @Kp
        越大表示越信任加速度，但快速晃动时，yaw轴角度可能会变化或者快速漂移。Kp越大，初始化的时候数据稳定越快。
    @Ki
        越小积分误差越小
    @halfT
        解算周期的一半，比如1ms解算1次则halfT为0.0005f
          如果角度转大了就调小，角度小了就调大
*/
float Kp = 6.f;    
float Ki = 0.0f;
float halfT = 0.000540f;
float norm;
float vx, vy, vz;
float ex, ey, ez;
float gx,gy,gz,ax,ay,az;	 
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float exInt,eyInt,ezInt;
float tempq0,tempq1,tempq2,tempq3;
short dead_zone = 5;
uint8_t BMI_Get_EulerAngle(float *pitch,float *roll,float *yaw)
{
  float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;   
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;   

	gx = (int16_t) GYRO.gx;
	gy = (int16_t) GYRO.gy;
	gz = (int16_t) (GYRO.gz/dead_zone)*dead_zone;
	
	if(gx >= -5 && gx<= 5)gx = 0;
	if(gy >= -5 && gy<= 5)gy = 0;
	if(gz >= -5 && gz<= 5)gz = 0;
	
//	if((gx >= -3 && gx<= 3)&&(gy >= -3 && gy<= 3)&&(gz >= -3 && gz<= 3))
//	{
//	    gx = 0;
//      gy = 0;
//      gz = 0;		
//	}
	
//	gz = imu.wz;
	ax = ACCEL.ax;
	ay = ACCEL.ay;
	az = ACCEL.az;

	if(ax * ay *az == 0)
	{
		return 0;
	}		

   gx = gx*BMI088_GYRO_2000_SEN;
	 gy = gy*BMI088_GYRO_2000_SEN;
	 gz = gz*BMI088_GYRO_2000_SEN;
	
//	gx = gx * 0.0174f;//陀螺仪角速度
//	gy = gy * 0.0174f;
//	gz = gz * 0.0174f;
	
	 ax = ax*BMI088_ACCEL_3G_SEN;
	 ay = ay*BMI088_ACCEL_3G_SEN;
	 az = az*BMI088_ACCEL_3G_SEN;
	
//	now_update  = HAL_GetTick(); //ms
//	halfT       = ((float)(now_update - last_update) / 2000.0f);
//	last_update = now_update;
	/* Fast inverse square-root */
	norm = inv_sqrt(ax*ax + ay*ay + az*az);       
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	
	
	/* estimated direction of gravity and flux (v and w) */
	vx = 2.0f*(q1q3 - q0q2);
	vy = 2.0f*(q0q1 + q2q3);
//	vz = q0q0 - q1q1 - q2q2 + q3q3;
  vz = 2.0f*q0q0 - 1.0f + 2.0f*q3q3;
//  vz = 1.0f-2*(q1*q1+q2*q2);
	
	/* 
	 * error is sum of cross product between reference direction 
	 * of fields and direction measured by sensors 
	 */
	ex = (ay*vz - az*vy) ;
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;

	/* PI */
	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
//		exInt = exInt + ex * Ki ;
//		eyInt = eyInt + ey * Ki ;	
//		ezInt = ezInt + ez * Ki ;
//		
//		gx = gx + Kp*ex + exInt;
//		gy = gy + Kp*ey + eyInt;
//		gz = gz + Kp*ez + ezInt;
		gx = gx + Kp*ex ;
		gy = gy + Kp*ey ;
		gz = gz + Kp*ez ;
	}
	
	tempq0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
	tempq1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT;
	tempq2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT;
	tempq3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT;  

	/* normalise quaternion */
	norm = inv_sqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
	q0 = tempq0 * norm;
	q1 = tempq1 * norm;
	q2 = tempq2 * norm;
	q3 = tempq3 * norm;
//	    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
//    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
//    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
	
	
	*yaw = ( atan2f(2.0f*q1*q2 + 2.0f*q0*q3,2.0f*(q0*q0+q1*q1)-1.0f)*57.3f+180)*8192/360  ;
	*pitch =( -asinf( -2.0f*(q1*q3-q0*q2))*57.3f+180)*8192/360;
//	*roll = atan2f(2.0f*(q0*q1+q2*q3),2.0f*(q0*q0+q3*q3)-1.0f)*57.3f;
  *roll =( atan2(2 * q2 * q3 + 2 * q0 * q1,q0*q0 - q1 * q1 -  q2 * q2 + q3 *q3)* 57.3f+180)*8192/360;

//	*yaw =  atan2(2*(q1*q2 + q0*q3),q0*q0 +q1*q1-q2*q2 -q3*q3)*57.3f ;
	return 0;
}

void BMI_SET_Kp(float KP_set)
{
	Kp = KP_set;
}


