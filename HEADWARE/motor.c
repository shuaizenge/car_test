#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "motor.h"
#include "tim.h"
#include "math.h"
void motor_init(void)
{
    HAL_GPIO_WritePin(SLEEP_2_GPIO_Port,SLEEP_2_Pin,1);//关闭二号舵机睡眠模式
    HAL_GPIO_WritePin(DIR_2_GPIO_Port,DIR_2_Pin,1);//正转
    TIM3->CCR2=0;//起始速度

    HAL_GPIO_WritePin(SLEEP_1_GPIO_Port,SLEEP_1_Pin,1);//关闭一号舵机睡眠模式
    HAL_GPIO_WritePin(DIR_1_GPIO_Port,DIR_1_Pin,0);//正转
    TIM3->CCR1=0;//起始速度

}

//direction:方向，0，正转，1：反转
//speed:速度，范围：0~8400

void motor_1(uint16_t direction, uint16_t speed) {
    if (direction == 1) {
        TIM3->CCR1 = speed;
        HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, 0);
    }//正传
    else {
        TIM3->CCR1 = speed;
        HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, 1);
    }//反转
}

//direction:方向，1，正转，0：反转
//speed:速度，范围：0~8400
void motor_2(uint16_t direction,uint16_t speed)
{
    if (direction == 1) {
        TIM3->CCR2 = speed;
        HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, 1);
    }//正传
    else {
        TIM3->CCR2 = speed;
        HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, 0);
    }//反转
}

void motor_control(float speed_1,float speed_2){//行走驱动
    if(speed_1>=0) {motor_1(1,(int)speed_1);}
    else	{ motor_1(-1,(int )-1*speed_1);}

    if(speed_2>=0) {motor_2(1,(int)speed_2);}
    else	{motor_2(-1,(int )-1*speed_2);}
}
void motor_corner(float speed){//转弯驱动，正为左转，负为右转
    if(speed>=0) {motor_1(1,(int )speed);motor_2(-1,(int )speed);}
    else	{ motor_1(-1,-(int )speed);motor_2(1,-(int )speed);}
}