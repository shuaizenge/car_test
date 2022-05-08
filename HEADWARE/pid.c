
#include <math.h>
#include "stm32f4xx_hal.h"
#include "pid.h"
#define PID_LIMIT_MIN -8400		//PID输出最低值-8400
#define PID_LIMIT_MAX 8400	//PID输出最大值8400
//注意：PID结构体必须定义为全局变量或静态变量，然后在函数中给KP,KI,KD赋值
/************************采样周期未知且不变************************************/
//位置式PID
//pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
//setvalue : 设置值（期望值）
//actualvalue: 实际值
//由于全量输出，每次输出均与过去状态有关，计算时要对ek累加，计算量大
float PID_location(float setvalue, float actualvalue, PID_LocTypeDef *PID)
{ 
	PID->ek =setvalue-actualvalue;
	PID->location_sum += PID->ek;                         //计算累计误差值
	if((PID->ki!=0)&&(PID->location_sum>(PID_LIMIT_MAX/PID->ki))) PID->location_sum=PID_LIMIT_MAX/PID->ki;
	if((PID->ki!=0)&&(PID->location_sum<(PID_LIMIT_MIN/PID->ki))) PID->location_sum=PID_LIMIT_MIN/PID->ki;//积分限幅

    PID->out=PID->kp*PID->ek+(PID->ki*PID->location_sum)+PID->kd*(PID->ek-PID->ek1);
    PID->ek1 = PID->ek;
	if(PID->out<PID_LIMIT_MIN)	PID->out=PID_LIMIT_MIN;
	if(PID->out>PID_LIMIT_MAX)	PID->out=PID_LIMIT_MAX;//PID->out限幅
	
	return PID->out;
}
//增量式PID
//pidout+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
//setvalue : 设置值（期望值）
//actualvalue: 实际值
float PID_increment(float setvalue, float actualvalue, PID_LocTypeDef *PID)
{                                
	PID->ek =setvalue-actualvalue;
  PID->out+=PID->kp*(PID->ek-PID->ek1)+PID->ki*PID->ek+PID->kd*(PID->ek-2*PID->ek1+PID->ek2);
//	PID->out+=PID->kp*PID->ek-PID->ki*PID->ek1+PID->kd*PID->ek2;
  PID->ek2 = PID->ek1;
  PID->ek1 = PID->ek;  
		
	if(PID->out<PID_LIMIT_MIN)	PID->out=PID_LIMIT_MIN;
	if(PID->out>PID_LIMIT_MAX)	PID->out=PID_LIMIT_MAX;//限幅
	
	return PID->out;
}

/************************采样周期未知且不变************************************/
//位置式PID
//pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
//setvalue : 设置值（期望值）
//actualvalue: 实际值
//由于全量输出，每次输出均与过去状态有关，计算时要对ek累加，计算量大
float PID_location_2(float setvalue, float actualvalue, PID_LocTypeDef *PID,int limit_ek)
{
    PID->ek =setvalue-actualvalue;
    if (PID->ek<limit_ek&&PID->ek>-limit_ek){
        PID->ek=0;
    }
    PID->location_sum += PID->ek;                         //计算累计误差值
    if((PID->ki!=0)&&(PID->location_sum>(PID_LIMIT_MAX/PID->ki))) PID->location_sum=PID_LIMIT_MAX/PID->ki;
    if((PID->ki!=0)&&(PID->location_sum<(PID_LIMIT_MIN/PID->ki))) PID->location_sum=PID_LIMIT_MIN/PID->ki;//积分限幅

    PID->out=PID->kp*PID->ek+(PID->ki*PID->location_sum)+PID->kd*(PID->ek-PID->ek1);
    PID->ek1 = PID->ek;
    if(PID->out<PID_LIMIT_MIN)	PID->out=PID_LIMIT_MIN;
    if(PID->out>PID_LIMIT_MAX)	PID->out=PID_LIMIT_MAX;//PID->out限幅

    return PID->out;
}

/************************采样周期未知且不变************************************/
//位置式PID
//pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
//setvalue : 设置值（期望值）
//actualvalue: 实际值
//由于全量输出，每次输出均与过去状态有关，计算时要对ek累加，计算量大
float PID_location_3(float setvalue, float actualvalue, PID_LocTypeDef *PID,int limit_ek,int *finish_flag)
{
    PID->ek =setvalue-actualvalue;
    if (PID->ek<limit_ek&&PID->ek>-limit_ek){
        PID->ek=0;
        finish_flag=0;
    } else{
        finish_flag=1;
    }
    PID->location_sum += PID->ek;                         //计算累计误差值
    if((PID->ki!=0)&&(PID->location_sum>(PID_LIMIT_MAX/PID->ki))) PID->location_sum=PID_LIMIT_MAX/PID->ki;
    if((PID->ki!=0)&&(PID->location_sum<(PID_LIMIT_MIN/PID->ki))) PID->location_sum=PID_LIMIT_MIN/PID->ki;//积分限幅

    PID->out=PID->kp*PID->ek+(PID->ki*PID->location_sum)+PID->kd*(PID->ek-PID->ek1);
    PID->ek1 = PID->ek;
    if(PID->out<PID_LIMIT_MIN)	PID->out=PID_LIMIT_MIN;
    if(PID->out>PID_LIMIT_MAX)	PID->out=PID_LIMIT_MAX;//PID->out限幅

    return PID->out;
}