#ifndef __PID_H
#define __PID_H

#include "stm32f4xx_hal.h"

typedef struct
{
  float kp;                       //����ϵ��Proportional
  float ki;                       //����ϵ��Integral
  float kd;                       //΢��ϵ��Derivative
//	float ti;                       //����ʱ�䳣��
//  float td;                       //΢��ʱ�䳣��
//	float period;										//��������
  float ek;                       //��ǰ���
  float ek1;                      //ǰһ�����e(k-1)
  float ek2;                      //��ǰһ�����e(k-2)
  double location_sum;             //�ۼƻ���λ��
	float out;											//PID���ֵ
}PID_LocTypeDef;
float PID_location(float setvalue, float actualvalue, PID_LocTypeDef *PID);//λ��ʽPID
float PID_increment(float setvalue, float actualvalue, PID_LocTypeDef *PID);//����ʽPID

float PID_location_2(float setvalue, float actualvalue, PID_LocTypeDef *PID,int limit_ek);
float PID_location_3(float setvalue, float actualvalue, PID_LocTypeDef *PID,int limit_ek,int *finish_flag);
#endif