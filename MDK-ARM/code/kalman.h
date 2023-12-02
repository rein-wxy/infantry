/**
  * @author  
  * �������˲�������RoboMaster��̳  
  */
  
#ifndef _KALMAN_H
#define _KALMAN_H


typedef struct {
    float X_last; //��һʱ�̵����Ž��  X(k-|k-1)
    float X_mid;  //��ǰʱ�̵�Ԥ����  X(k|k-1)
    float X_now;  //��ǰʱ�̵����Ž��  X(k|k)
    float P_mid;  //��ǰʱ��Ԥ������Э����  P(k|k-1)
    float P_now;  //��ǰʱ�����Ž����Э����  P(k|k)
    float P_last; //��һʱ�����Ž����Э����  P(k-1|k-1)
    float kg;     //kalman����
    float A;      //ϵͳ����
	float B;
    float Q;
    float R;
    float H;
}extKalman_t;

extern extKalman_t mous_x_kalman;
extern extKalman_t mous_y_kalman;

extern extKalman_t kalman_visionYaw	        ,			kalman_targetYaw;
extern extKalman_t kalman_visionPitch	    ,			kalman_targetPitch;
extern extKalman_t kalman_visionDistance	,			kalman_targetDistance;
extern extKalman_t kalman_accelYaw			,			kalman_speedYaw;
extern extKalman_t kalman_accelPit			,			kalman_speedPit;


extern extKalman_t  rate_pitch_kf;
extern extKalman_t  rate_roll_kf;
extern extKalman_t  rate_yaw_kf;




//�����˲���
void KalmanCreate(extKalman_t *p,float T_Q,float T_R);

//���˲���������
float KalmanFilter(extKalman_t* p,float dat);


//��տ������˲���
void KalmanClear(extKalman_t *p);

#endif
