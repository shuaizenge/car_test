#ifndef _ARITHMETIC_H_
#define _ARITHMETIC_H_
#include "stm32f4xx_hal.h"
#include "pid.h"
#include "motor.h"
#include "usart.h"
#include "stdlib.h"
#include "HX711.h"
#include "tim.h"
//ת��״̬��

typedef struct {
    float CNT_prevalue;
    float value;//����������
    float current_speed;//С���ٶ�
    PID_LocTypeDef PID_AutoCar;
    float pid_speed;
    float standard_speed;
}Wheel;

typedef struct {
    uint8_t car_mode[4];
    uint8_t car_date[9];
}Car_station;

extern uint16_t pid_flag;//����PID״̬��ȡ

extern Wheel Car_wheel[2];//С�����Ӳ���
extern Car_station carstaion;

extern uint8_t Res;//��Ϊ����ÿ�δ���һ�����ݣ�������RES������

extern float dirction_distance;

extern uint32_t EncCnt; //���TIM4�ı�����ARR���
extern uint32_t EncCnt_2;

extern uint8_t Date[3];

extern int UART_printf(UART_HandleTypeDef *huart, const char *fmt, ...);

extern float average_speed_34;

//PID�ṹ��
static PID_LocTypeDef PID_position_2;//�����������Ŀ����С�����벻ͬ�������С��ͬ������С����Ŀ�����
static PID_LocTypeDef PID_angle;//λ�ýǶȻ�������С����Ŀ��Ƕ�
static PID_LocTypeDef PID_angle_1;//λ�ýǶȻ�������С����Ŀ��Ƕ�

//Ŀ��׷��
void Target_tracking();
void Target_tracking_3();
//ת��
void Turn_1(int around,uint8_t angle);
void Turn_2(int around,uint8_t angle);
void Turn_1_3(int around, uint8_t angle) ;//����1�ĵ�3
void Turn_2_4(int around, uint8_t angle) ;//����2�ĵ�4
//��ת180
void Totation();
void Totation_3();
void Totation_4();
void Totation_5();

//����׼����ת�ź�
void ready_Totation();

#endif