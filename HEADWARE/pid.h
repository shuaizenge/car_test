#ifndef __PID_H
#define __PID_H

#include "stm32f4xx_hal.h"

typedef struct
{
  float kp;                       //比例系数Proportional
  float ki;                       //积分系数Integral
  float kd;                       //微分系数Derivative
//	float ti;                       //积分时间常数
//  float td;                       //微分时间常数
//	float period;										//采样周期
  float ek;                       //当前误差
  float ek1;                      //前一次误差e(k-1)
  float ek2;                      //再前一次误差e(k-2)
  double location_sum;             //累计积分位置
	float out;											//PID输出值
}PID_LocTypeDef;
float PID_location(float setvalue, float actualvalue, PID_LocTypeDef *PID);//位置式PID
float PID_increment(float setvalue, float actualvalue, PID_LocTypeDef *PID);//增量式PID

float PID_location_2(float setvalue, float actualvalue, PID_LocTypeDef *PID,int limit_ek);
float PID_location_3(float setvalue, float actualvalue, PID_LocTypeDef *PID,int limit_ek,int *finish_flag);
#endif