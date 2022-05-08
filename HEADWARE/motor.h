#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"
#include "main.h"

void motor_init(void);
void motor_1(uint16_t direction,uint16_t speed);
void motor_2(uint16_t direction,uint16_t speed);
void motor_control(float speed_1,float speed_2);//行走驱动
void motor_corner(float speed);//转弯驱动，正为左转，负为右转

#endif