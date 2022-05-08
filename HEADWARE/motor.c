#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "motor.h"
#include "tim.h"
#include "math.h"
void motor_init(void)
{
    HAL_GPIO_WritePin(SLEEP_2_GPIO_Port,SLEEP_2_Pin,1);//�رն��Ŷ��˯��ģʽ
    HAL_GPIO_WritePin(DIR_2_GPIO_Port,DIR_2_Pin,1);//��ת
    TIM3->CCR2=0;//��ʼ�ٶ�

    HAL_GPIO_WritePin(SLEEP_1_GPIO_Port,SLEEP_1_Pin,1);//�ر�һ�Ŷ��˯��ģʽ
    HAL_GPIO_WritePin(DIR_1_GPIO_Port,DIR_1_Pin,0);//��ת
    TIM3->CCR1=0;//��ʼ�ٶ�

}

//direction:����0����ת��1����ת
//speed:�ٶȣ���Χ��0~8400

void motor_1(uint16_t direction, uint16_t speed) {
    if (direction == 1) {
        TIM3->CCR1 = speed;
        HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, 0);
    }//����
    else {
        TIM3->CCR1 = speed;
        HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, 1);
    }//��ת
}

//direction:����1����ת��0����ת
//speed:�ٶȣ���Χ��0~8400
void motor_2(uint16_t direction,uint16_t speed)
{
    if (direction == 1) {
        TIM3->CCR2 = speed;
        HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, 1);
    }//����
    else {
        TIM3->CCR2 = speed;
        HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, 0);
    }//��ת
}

void motor_control(float speed_1,float speed_2){//��������
    if(speed_1>=0) {motor_1(1,(int)speed_1);}
    else	{ motor_1(-1,(int )-1*speed_1);}

    if(speed_2>=0) {motor_2(1,(int)speed_2);}
    else	{motor_2(-1,(int )-1*speed_2);}
}
void motor_corner(float speed){//ת����������Ϊ��ת����Ϊ��ת
    if(speed>=0) {motor_1(1,(int )speed);motor_2(-1,(int )speed);}
    else	{ motor_1(-1,-(int )speed);motor_2(1,-(int )speed);}
}