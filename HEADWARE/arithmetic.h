#ifndef _ARITHMETIC_H_
#define _ARITHMETIC_H_
#include "stm32f4xx_hal.h"
#include "pid.h"
#include "motor.h"
#include "usart.h"
#include "stdlib.h"
#include "HX711.h"
#include "tim.h"
//转弯状态机

typedef struct {
    float CNT_prevalue;
    float value;//编码器数字
    float current_speed;//小车速度
    PID_LocTypeDef PID_AutoCar;
    float pid_speed;
    float standard_speed;
}Wheel;

typedef struct {
    uint8_t car_mode[4];
    uint8_t car_date[9];
}Car_station;

extern uint16_t pid_flag;//进入PID状态获取

extern Wheel Car_wheel[2];//小车轮子参数
extern Car_station carstaion;

extern uint8_t Res;//因为串口每次传递一个数据，所以用RES来保存

extern float dirction_distance;

extern uint32_t EncCnt; //解决TIM4的编码器ARR溢出
extern uint32_t EncCnt_2;

extern uint8_t Date[3];

extern int UART_printf(UART_HandleTypeDef *huart, const char *fmt, ...);

extern float average_speed_34;

//PID结构体
static PID_LocTypeDef PID_position_2;//面积环，根据目标离小车距离不同而面积大小不同来调整小车与目标距离
static PID_LocTypeDef PID_angle;//位置角度环，调整小车到目标角度
static PID_LocTypeDef PID_angle_1;//位置角度环，调整小车到目标角度

//目标追踪
void Target_tracking();
void Target_tracking_3();
//转弯
void Turn_1(int around,uint8_t angle);
void Turn_2(int around,uint8_t angle);
void Turn_1_3(int around, uint8_t angle) ;//基于1改的3
void Turn_2_4(int around, uint8_t angle) ;//基于2改的4
//自转180
void Totation();
void Totation_3();
void Totation_4();
void Totation_5();

//发送准备自转信号
void ready_Totation();

#endif