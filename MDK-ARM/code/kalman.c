/**    
  * @author  Liu heng
  * һ�׿������˲�������RoboMaster��̳  
  *   һά�������˲���                     
  *   ʹ��ʱ�ȶ���һ��kalmanָ�룬Ȼ�����kalmanCreate()����һ���˲��� 
  *   ÿ�ζ�ȡ�����������ݺ󼴿ɵ���KalmanFilter()�������ݽ����˲�
  *          ʹ��ʾ��                                             
  *          extKalman p;                  //����һ���������˲����ṹ��                                                 
  *          float SersorData;             //��Ҫ�����˲�������                                          
  *          KalmanCreate(&p,20,200);      //��ʼ�����˲�����Q=20 R=200����                                                  
  *          while(1)                                                                
  *          {                                                                            
  *             SersorData = sersor();                     //��ȡ����                                           
  *             SersorData = KalmanFilter(&p,SersorData);  //�����ݽ����˲�                                                                            
  *          }                                                                            
  */

#include "kalman.h"

extKalman_t mous_x_kalman;
extKalman_t mous_y_kalman;

extKalman_t kalman_visionYaw	    ,			kalman_targetYaw;
extKalman_t kalman_visionPitch	    ,			kalman_targetPitch;
extKalman_t kalman_visionDistance	,			kalman_targetDistance;
extKalman_t kalman_accelYaw			,			kalman_speedYaw;
extKalman_t kalman_accelPit			,			kalman_speedPit;


extKalman_t  rate_pitch_kf;
extKalman_t  rate_roll_kf;
extKalman_t  rate_yaw_kf;
/**
  * @name   kalmanCreate
  * @brief  ����һ���������˲���
  * @param  p:  �˲���
  *         T_Q:ϵͳ����Э����
  *         T_R:��������Э����
  *         
  * @retval none
  * @attention R�̶���QԽ�󣬴���Խ���β���ֵ��Q�������ֻ�ò���ֵ
  *		       	��֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ��
  */       
	//RԽ���ʾԽ����Ԥ��ֵ��QԽ���ʾԽ���η���ֵ
void KalmanCreate(extKalman_t *p,float T_Q,float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
	p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}

/**
  * @name   KalmanClear
  * @brief  ��տ������˲���
  * @param  p:  �˲���
  *         T_Q:ϵͳ����Э����
  *         T_R:��������Э����
  *         
  * @retval none
  * @attention 
  */
void KalmanClear(extKalman_t *p)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->A = 1;
		p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}

/**
  * @name   KalmanFilter
  * @brief  �������˲���
  * @param  p:  �˲���
  *         dat:���˲�����
            T_Q:ϵͳ����Э����
  *         T_R:��������Э����
  * @retval �˲��������
  * @attention Z(k)��ϵͳ����,������ֵ   X(k|k)�ǿ������˲����ֵ,���������
  *            A=1 B=0 H=1 I=1  W(K)  V(k)�Ǹ�˹������,�����ڲ���ֵ����,���Բ��ù�
  *            �����ǿ�������5�����Ĺ�ʽ
  *            һ��H'��Ϊ������,����Ϊת�þ���
  */

float KalmanFilter(extKalman_t* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //�ٶȶ�Ӧ��ʽ(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
	/*ֱ�Ӱ���һʱ�̵�Ԥ��ֵ�������ڵ�ֵ����Ԥ��*/
    p->P_mid = p->A*p->P_last+p->Q;               //�ٶȶ�Ӧ��ʽ(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
	/*P->Qֵ���ֲ��䣬��һ����ȫ��P->last����*/
    p->kg = p->P_mid/(p->P_mid+p->R);             //�ٶȶ�Ӧ��ʽ(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
	/*p->kg = p->P_mid/(p->P_mid+p->R); 
	        = 1-p->R/(p->P_mid+p->R)      */
	/*P->R��ֵ�Ǳ��ֲ���ģ���һ���Ĵ�С��ȫ��P->mid��������ȡ���ڵڶ���*/
    p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //�ٶȶ�Ӧ��ʽ(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
	/*����һ����kgԽ�󣬾�Խ�ӽ�����ʵ����DAT��������kg�ģ��ǵ�����
	       p->X_now = p->kg*dat+(1-P->kg)*p->X_mid;
         	p->X_now = [p->P_mid/(p->P_mid+p->R)]*dat+[p->R/(p->P_mid+p->R)]*p->X_mid;
    �൱��һֱ�ڸı䲻ͬռ��	��*/
	     
    p->P_now = (1-p->kg)*p->P_mid;                //�ٶȶ�Ӧ��ʽ(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
	/*
	        p->P_now = (p->R*p->P_mid)/(p->P_mid+p->R)                                     */
	
    p->P_last = p->P_now;                         //״̬����
    p->X_last = p->X_now;
    return p->X_now;							  //���Ԥ����x(k|k)
}