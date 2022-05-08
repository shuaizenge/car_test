#include "arithmetic.h"

static float Differential_speed=0;//转弯速度
static float average_speed=0;//平均速度

static int cnn=0;//转弯次数计数
float average_speed_34 = 19;
int weight_falg=0;

void Target_tracking(){
    //两个轮子的PID参数150 4 10
    Car_wheel[0].PID_AutoCar.kp = 150;
    Car_wheel[0].PID_AutoCar.ki = 4;
    Car_wheel[0].PID_AutoCar.kd = 10;
    Car_wheel[1].PID_AutoCar.kp = 150;
    Car_wheel[1].PID_AutoCar.ki = 4;
    Car_wheel[1].PID_AutoCar.kd = 10;

    //(左右)位置环PID参数
    PID_position_2.kp =0.75;
    PID_position_2.kd = 5;//2
    if (pid_flag == 0) {//如果获得了当前速度（TIM2中断控制），进行一次PID控制
        pid_flag = 1;//消除标志位

        average_speed=19;
        Differential_speed=-PID_location_2(80.0,dirction_distance,&PID_position_2,3);//左右位置追踪

        Car_wheel[0].standard_speed = average_speed + Differential_speed * 0.5;//右轮子目标速度设定
        Car_wheel[1].standard_speed = average_speed - Differential_speed * 0.5;//左轮子目标速度设定

        Car_wheel[0].pid_speed = PID_location(Car_wheel[0].standard_speed, Car_wheel[0].current_speed,
                                              &Car_wheel[0].PID_AutoCar);//PID控制来达到右轮子目标速度设定
        Car_wheel[1].pid_speed = PID_location(Car_wheel[1].standard_speed, Car_wheel[1].current_speed,
                                              &Car_wheel[1].PID_AutoCar);//PID控制来达到左轮子目标速度设定

        motor_control(Car_wheel[0].pid_speed, Car_wheel[1].pid_speed);//驱动小车
    }
}

void Target_tracking_3() {
    //两个轮子的PID参数150 4 10
    Car_wheel[0].PID_AutoCar.kp = 150;
    Car_wheel[0].PID_AutoCar.ki = 4;
    Car_wheel[0].PID_AutoCar.kd = 10;
    Car_wheel[1].PID_AutoCar.kp = 150;
    Car_wheel[1].PID_AutoCar.ki = 4;
    Car_wheel[1].PID_AutoCar.kd = 10;

    //(左右)位置环PID参数
    PID_position_2.kp = 0.75;
    PID_position_2.kd = 6;//2
    if (pid_flag == 0) {//如果获得了当前速度（TIM2中断控制），进行一次PID控制
        pid_flag = 1;//消除标志位
        Differential_speed = -PID_location_2(80.0, dirction_distance, &PID_position_2, 3);//左右位置追踪

        Car_wheel[0].standard_speed = average_speed_34 + Differential_speed * 0.5;//右轮子目标速度设定
        Car_wheel[1].standard_speed = average_speed_34 - Differential_speed * 0.5;//左轮子目标速度设定

        Car_wheel[0].pid_speed = PID_location(Car_wheel[0].standard_speed, Car_wheel[0].current_speed,
                                              &Car_wheel[0].PID_AutoCar);//PID控制来达到右轮子目标速度设定
        Car_wheel[1].pid_speed = PID_location(Car_wheel[1].standard_speed, Car_wheel[1].current_speed,
                                              &Car_wheel[1].PID_AutoCar);//PID控制来达到左轮子目标速度设定

        motor_control(Car_wheel[0].pid_speed, Car_wheel[1].pid_speed);//驱动小车
    }
}
void Turn_1(int around, uint8_t angle) {

    Car_wheel[0].PID_AutoCar.kp = 150;
    Car_wheel[0].PID_AutoCar.ki = 4;
    Car_wheel[0].PID_AutoCar.kd = 10;
    Car_wheel[1].PID_AutoCar.kp = 150;
    Car_wheel[1].PID_AutoCar.ki = 4;
    Car_wheel[1].PID_AutoCar.kd = 10;

//位置角度环PID参数0.15，0.5
    PID_angle.kp = 0.12;
    PID_angle.kd = 0.5;

    PID_angle_1.kp = 0.12;
    PID_angle_1.kd = 0.5;

    uint32_t line,line_1;

    if (around == 1) {
        line = angle * 6.2 + EncCnt;
        line_1=-angle * 6.2 + EncCnt_2;
        while (abs(line*1.0-EncCnt*1.0) >=5||abs(line_1*1.0-EncCnt_2*1.0)>=5) {
            if (pid_flag == 0) {//如果获得了当前速度（TIM2中断控制），进行一次PID控制
                pid_flag = 1;//消除标志位

                average_speed = 0;
                Differential_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);;

                Car_wheel[0].standard_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);
                Car_wheel[1].standard_speed = PID_location((float) line_1 * 1.0f, (float) EncCnt_2 * 1.0f, &PID_angle_1);

                Car_wheel[0].pid_speed = PID_location(Car_wheel[0].standard_speed, Car_wheel[0].current_speed,
                                                      &Car_wheel[0].PID_AutoCar);//PID控制来达到右轮子目标速度设定
                Car_wheel[1].pid_speed = PID_location(Car_wheel[1].standard_speed, Car_wheel[1].current_speed,
                                                      &Car_wheel[1].PID_AutoCar);//PID控制来达到左轮子目标速度设定

                motor_control(Car_wheel[0].pid_speed, Car_wheel[1].pid_speed);//驱动小车
            }
        }
    } else {
        line = -angle * 6.2 + EncCnt;//5.8
        line_1 = angle * 6.2 + EncCnt_2;
        while (abs(line*1.0-EncCnt*1.0) >=5||abs(line_1*1.0-EncCnt_2*1.0)>=5) {
            if (pid_flag == 0) {//如果获得了当前速度（TIM2中断控制），进行一次PID控制
                pid_flag = 1;//消除标志位

                average_speed = 0;
                Differential_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);;

                Car_wheel[0].standard_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);
                Car_wheel[1].standard_speed = PID_location((float) line_1 * 1.0f, (float) EncCnt_2 * 1.0f,&PID_angle_1);

                Car_wheel[0].pid_speed = PID_location(Car_wheel[0].standard_speed, Car_wheel[0].current_speed,
                                                      &Car_wheel[0].PID_AutoCar);//PID控制来达到右轮子目标速度设定
                Car_wheel[1].pid_speed = PID_location(Car_wheel[1].standard_speed, Car_wheel[1].current_speed,
                                                      &Car_wheel[1].PID_AutoCar);//PID控制来达到左轮子目标速度设定

                motor_control(Car_wheel[0].pid_speed, Car_wheel[1].pid_speed);//驱动小车
            }
        }
    }

    motor_control(0,0);
}
void Turn_2(int around, uint8_t angle) {
    Car_wheel[0].PID_AutoCar.kp = 150;
    Car_wheel[0].PID_AutoCar.ki = 4;
    Car_wheel[0].PID_AutoCar.kd = 10;
    Car_wheel[1].PID_AutoCar.kp = 150;
    Car_wheel[1].PID_AutoCar.ki = 4;
    Car_wheel[1].PID_AutoCar.kd = 10;

    //位置角度环PID参数
    PID_angle.kp = 0.16;
    PID_angle.kd = 0.5;

    PID_angle_1.kp = 0.16;
    PID_angle_1.kd = 0.5;

    uint32_t line, line_1;

    if (around == 1) {
        HAL_UART_AbortReceive_IT(&huart2);
        line = angle * 6.2 + EncCnt;
        line_1 = -angle * 6.2 + EncCnt_2;
        while (abs(line*1.0-EncCnt*1.0) >=5||abs(line_1*1.0-EncCnt_2*1.0)>=5) {
            if (pid_flag == 0) {//如果获得了当前速度（TIM2中断控制），进行一次PID控制
                pid_flag = 1;//消除标志位

                average_speed = 0;
                Differential_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);;

                Car_wheel[0].standard_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);
                Car_wheel[1].standard_speed = PID_location((float) line_1 * 1.0f, (float) EncCnt_2 * 1.0f,
                                                           &PID_angle_1);

                Car_wheel[0].pid_speed = PID_location(Car_wheel[0].standard_speed, Car_wheel[0].current_speed,
                                                      &Car_wheel[0].PID_AutoCar);//PID控制来达到右轮子目标速度设定
                Car_wheel[1].pid_speed = PID_location(Car_wheel[1].standard_speed, Car_wheel[1].current_speed,
                                                      &Car_wheel[1].PID_AutoCar);//PID控制来达到左轮子目标速度设定

                motor_control(Car_wheel[0].pid_speed, Car_wheel[1].pid_speed);//驱动小车
            }
        }
    } else {
        HAL_UART_AbortReceive_IT(&huart2);
        line = -angle * 6.0 + EncCnt;
        line_1 = angle * 6.0 + EncCnt_2;
        while (abs(line*1.0-EncCnt*1.0) >=5||abs(line_1*1.0-EncCnt_2*1.0)>=5) {
            if (pid_flag == 0) {//如果获得了当前速度（TIM2中断控制），进行一次PID控制
                pid_flag = 1;//消除标志位

                average_speed = 0;
                Differential_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);;

                Car_wheel[0].standard_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);
                Car_wheel[1].standard_speed = PID_location((float) line_1 * 1.0f, (float) EncCnt_2 * 1.0f,
                                                           &PID_angle_1);

                Car_wheel[0].pid_speed = PID_location(Car_wheel[0].standard_speed, Car_wheel[0].current_speed,
                                                      &Car_wheel[0].PID_AutoCar);//PID控制来达到右轮子目标速度设定
                Car_wheel[1].pid_speed = PID_location(Car_wheel[1].standard_speed, Car_wheel[1].current_speed,
                                                      &Car_wheel[1].PID_AutoCar);//PID控制来达到左轮子目标速度设定

                motor_control(Car_wheel[0].pid_speed, Car_wheel[1].pid_speed);//驱动小车
            }
        }
    }

    motor_control(0, 0);

    HAL_UART_Receive_IT(&huart2, &Res, 1);
}
void Turn_1_3(int around, uint8_t angle) {

    Car_wheel[0].PID_AutoCar.kp = 150;
    Car_wheel[0].PID_AutoCar.ki = 4;
    Car_wheel[0].PID_AutoCar.kd = 10;
    Car_wheel[1].PID_AutoCar.kp = 150;
    Car_wheel[1].PID_AutoCar.ki = 4;
    Car_wheel[1].PID_AutoCar.kd = 10;

    //位置角度环PID参数0.15，0.5
    PID_angle.kp = 0.15;
    PID_angle.kd = 1;

    PID_angle_1.kp = 0.15;
    PID_angle_1.kd = 1;

    uint32_t line, line_1;

    if (around == 1) {
        HAL_UART_AbortReceive_IT(&huart2);
        line = angle * 6.2 + EncCnt;
        line_1 = -angle * 6.2 + EncCnt_2;
        while (abs(line * 1.0 - EncCnt * 1.0) >= 5 || abs(line_1 * 1.0 - EncCnt_2 * 1.0) >= 5) {
            if (pid_flag == 0) {//如果获得了当前速度（TIM2中断控制），进行一次PID控制
                pid_flag = 1;//消除标志位

                average_speed = 0;
                Differential_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);;

                Car_wheel[0].standard_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);
                Car_wheel[1].standard_speed = PID_location((float) line_1 * 1.0f, (float) EncCnt_2 * 1.0f,
                                                           &PID_angle_1);

                Car_wheel[0].pid_speed = PID_location(Car_wheel[0].standard_speed, Car_wheel[0].current_speed,
                                                      &Car_wheel[0].PID_AutoCar);//PID控制来达到右轮子目标速度设定
                Car_wheel[1].pid_speed = PID_location(Car_wheel[1].standard_speed, Car_wheel[1].current_speed,
                                                      &Car_wheel[1].PID_AutoCar);//PID控制来达到左轮子目标速度设定

                motor_control(Car_wheel[0].pid_speed, Car_wheel[1].pid_speed);//驱动小车
                UART_printf(&huart6, "%d,%d\n", line - EncCnt, line_1 - EncCnt_2);
            }
        }
    } else {
        HAL_UART_AbortReceive_IT(&huart2);
        line = -angle * 5.8 + EncCnt;//5.8
        line_1 = angle * 5.8 + EncCnt_2;
        while (abs(line * 1.0 - EncCnt * 1.0) >= 5 || abs(line_1 * 1.0 - EncCnt_2 * 1.0) >= 5) {
            if (pid_flag == 0) {//如果获得了当前速度（TIM2中断控制），进行一次PID控制
                pid_flag = 1;//消除标志位

                average_speed = 0;
                Differential_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);;

                Car_wheel[0].standard_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);
                Car_wheel[1].standard_speed = PID_location((float) line_1 * 1.0f, (float) EncCnt_2 * 1.0f,
                                                           &PID_angle_1);

                Car_wheel[0].pid_speed = PID_location(Car_wheel[0].standard_speed, Car_wheel[0].current_speed,
                                                      &Car_wheel[0].PID_AutoCar);//PID控制来达到右轮子目标速度设定
                Car_wheel[1].pid_speed = PID_location(Car_wheel[1].standard_speed, Car_wheel[1].current_speed,
                                                      &Car_wheel[1].PID_AutoCar);//PID控制来达到左轮子目标速度设定

                motor_control(Car_wheel[0].pid_speed, Car_wheel[1].pid_speed);//驱动小车
                UART_printf(&huart6, "%d,%d\n", line - EncCnt, line_1 - EncCnt_2);
            }
        }
    }

    motor_control(0, 0);

    HAL_UART_Receive_IT(&huart2, &Res, 1);
}
void Turn_2_4(int around, uint8_t angle) {
    Car_wheel[0].PID_AutoCar.kp = 150;
    Car_wheel[0].PID_AutoCar.ki = 4;
    Car_wheel[0].PID_AutoCar.kd = 10;
    Car_wheel[1].PID_AutoCar.kp = 150;
    Car_wheel[1].PID_AutoCar.ki = 4;
    Car_wheel[1].PID_AutoCar.kd = 10;

    //位置角度环PID参数
    if (weight_falg == 0) {
        PID_angle.kp = 0.18;
        PID_angle.kd = 1;

        PID_angle_1.kp = 0.18;
        PID_angle_1.kd = 1;

        uint32_t line, line_1;

        if (around == 1) {
            line = angle * 6.4 + EncCnt;
            line_1 = -angle * 6.4 + EncCnt_2;
            while (abs(line * 1.0 - EncCnt * 1.0) >= 3 || abs(line_1 * 1.0 - EncCnt_2 * 1.0) >= 3) {
                if (pid_flag == 0) {//如果获得了当前速度（TIM2中断控制），进行一次PID控制
                    pid_flag = 1;//消除标志位

                    average_speed = 0;
                    Differential_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);;

                    Car_wheel[0].standard_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);
                    Car_wheel[1].standard_speed = PID_location((float) line_1 * 1.0f, (float) EncCnt_2 * 1.0f,
                                                               &PID_angle_1);

                    Car_wheel[0].pid_speed = PID_location(Car_wheel[0].standard_speed, Car_wheel[0].current_speed,
                                                          &Car_wheel[0].PID_AutoCar);//PID控制来达到右轮子目标速度设定
                    Car_wheel[1].pid_speed = PID_location(Car_wheel[1].standard_speed, Car_wheel[1].current_speed,
                                                          &Car_wheel[1].PID_AutoCar);//PID控制来达到左轮子目标速度设定

                    motor_control(Car_wheel[0].pid_speed, Car_wheel[1].pid_speed);//驱动小车
                }
            }
        } else {
            line = -angle * 6.7 + EncCnt;
            line_1 = angle * 6.7 + EncCnt_2;
            while (abs(line * 1.0 - EncCnt * 1.0) >= 3 || abs(line_1 * 1.0 - EncCnt_2 * 1.0) >= 3) {
                if (pid_flag == 0) {//如果获得了当前速度（TIM2中断控制），进行一次PID控制
                    pid_flag = 1;//消除标志位

                    average_speed = 0;
                    Differential_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);;

                    Car_wheel[0].standard_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);
                    Car_wheel[1].standard_speed = PID_location((float) line_1 * 1.0f, (float) EncCnt_2 * 1.0f,
                                                               &PID_angle_1);

                    Car_wheel[0].pid_speed = PID_location(Car_wheel[0].standard_speed, Car_wheel[0].current_speed,
                                                          &Car_wheel[0].PID_AutoCar);//PID控制来达到右轮子目标速度设定
                    Car_wheel[1].pid_speed = PID_location(Car_wheel[1].standard_speed, Car_wheel[1].current_speed,
                                                          &Car_wheel[1].PID_AutoCar);//PID控制来达到左轮子目标速度设定

                    motor_control(Car_wheel[0].pid_speed, Car_wheel[1].pid_speed);//驱动小车
                }
            }
        }

    } else {
        PID_angle.kp = 0.14;
        PID_angle.kd = 1;

        PID_angle_1.kp = 0.14;
        PID_angle_1.kd = 1;

        uint32_t line, line_1;

        if (around == 1) {
            line = angle * 6.2 + EncCnt;
            line_1 = -angle * 6.2 + EncCnt_2;
            while (abs(line * 1.0 - EncCnt * 1.0) >= 3 || abs(line_1 * 1.0 - EncCnt_2 * 1.0) >= 3) {
                if (pid_flag == 0) {//如果获得了当前速度（TIM2中断控制），进行一次PID控制
                    pid_flag = 1;//消除标志位

                    average_speed = 0;
                    Differential_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);;

                    Car_wheel[0].standard_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);
                    Car_wheel[1].standard_speed = PID_location((float) line_1 * 1.0f, (float) EncCnt_2 * 1.0f,
                                                               &PID_angle_1);

                    Car_wheel[0].pid_speed = PID_location(Car_wheel[0].standard_speed, Car_wheel[0].current_speed,
                                                          &Car_wheel[0].PID_AutoCar);//PID控制来达到右轮子目标速度设定
                    Car_wheel[1].pid_speed = PID_location(Car_wheel[1].standard_speed, Car_wheel[1].current_speed,
                                                          &Car_wheel[1].PID_AutoCar);//PID控制来达到左轮子目标速度设定

                    motor_control(Car_wheel[0].pid_speed, Car_wheel[1].pid_speed);//驱动小车
                }
            }
        } else {
            line = -angle * 6.0 + EncCnt;
            line_1 = angle * 6.0 + EncCnt_2;
            while (abs(line * 1.0 - EncCnt * 1.0) >= 3 || abs(line_1 * 1.0 - EncCnt_2 * 1.0) >= 3) {
                if (pid_flag == 0) {//如果获得了当前速度（TIM2中断控制），进行一次PID控制
                    pid_flag = 1;//消除标志位

                    average_speed = 0;
                    Differential_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);;

                    Car_wheel[0].standard_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);
                    Car_wheel[1].standard_speed = PID_location((float) line_1 * 1.0f, (float) EncCnt_2 * 1.0f,
                                                               &PID_angle_1);

                    Car_wheel[0].pid_speed = PID_location(Car_wheel[0].standard_speed, Car_wheel[0].current_speed,
                                                          &Car_wheel[0].PID_AutoCar);//PID控制来达到右轮子目标速度设定
                    Car_wheel[1].pid_speed = PID_location(Car_wheel[1].standard_speed, Car_wheel[1].current_speed,
                                                          &Car_wheel[1].PID_AutoCar);//PID控制来达到左轮子目标速度设定

                    motor_control(Car_wheel[0].pid_speed, Car_wheel[1].pid_speed);//驱动小车
                }
            }
        }
    }


    motor_control(0, 0);

}

void Totation() {
    static int ii=0;
    float weight=0;

    motor_control(0, 0);
    HAL_Delay(1000);

    Car_wheel[0].PID_AutoCar.kp = 150;
    Car_wheel[0].PID_AutoCar.ki = 4;
    Car_wheel[0].PID_AutoCar.kd = 10;
    Car_wheel[1].PID_AutoCar.kp = 150;
    Car_wheel[1].PID_AutoCar.ki = 4;
    Car_wheel[1].PID_AutoCar.kd = 10;

//位置角度环PID参数
    PID_angle.kp = 0.05;
    PID_angle.kd = 0;

    PID_angle_1.kp = 0.05;
    PID_angle_1.kd = 0;

    uint32_t line, line_1;

    HAL_UART_AbortReceive_IT(&huart2);

    line = 180 * 6.2 + EncCnt;
    line_1 = -180 * 6.2 + EncCnt_2;

    while (abs(line * 1.0 - EncCnt * 1.0) >= 3 || abs(line_1 * 1.0 - EncCnt_2 * 1.0) >= 3) {
        if (pid_flag == 0) {//如果获得了当前速度（TIM2中断控制），进行一次PID控制
            pid_flag = 1;//消除标志位

            average_speed = 0;
            Differential_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);;

            Car_wheel[0].standard_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);
            Car_wheel[1].standard_speed = PID_location((float) line_1 * 1.0f, (float) EncCnt_2 * 1.0f, &PID_angle_1);

            Car_wheel[0].pid_speed = PID_location(Car_wheel[0].standard_speed, Car_wheel[0].current_speed,
                                                  &Car_wheel[0].PID_AutoCar);//PID控制来达到右轮子目标速度设定
            Car_wheel[1].pid_speed = PID_location(Car_wheel[1].standard_speed, Car_wheel[1].current_speed,
                                                  &Car_wheel[1].PID_AutoCar);//PID控制来达到左轮子目标速度设定

            motor_control(Car_wheel[0].pid_speed, Car_wheel[1].pid_speed);//驱动小车
        }
    }

    motor_control(0, 0);

    ii++;
    UART_printf(&huart1, "i=%d\n", ii);
    switch (ii) {
        case 1:
            //延迟一秒
            HAL_Delay(1000);
            weight_falg = 1;
            break;
        case 2:
            carstaion.car_date[0] = 0;
            break;
        default:
            break;
    }

    HAL_UART_Receive_IT(&huart2, &Res, 1);
}

void Totation_3() {
    static int ii = 0;
    float weight = 0;

    motor_control(0, 0);
    HAL_Delay(1000);

    Car_wheel[0].PID_AutoCar.kp = 150;
    Car_wheel[0].PID_AutoCar.ki = 4;
    Car_wheel[0].PID_AutoCar.kd = 10;
    Car_wheel[1].PID_AutoCar.kp = 150;
    Car_wheel[1].PID_AutoCar.ki = 4;
    Car_wheel[1].PID_AutoCar.kd = 10;

    //位置角度环PID参数
    PID_angle.kp = 0.05;
    PID_angle.kd = 0;

    PID_angle_1.kp = 0.05;
    PID_angle_1.kd = 0;

    uint32_t line, line_1;

    HAL_UART_AbortReceive_IT(&huart2);

    line = 180 * 6.6 + EncCnt;
    line_1 = -180 * 6.6 + EncCnt_2;

    while (abs(line * 1.0 - EncCnt * 1.0) >= 3 || abs(line_1 * 1.0 - EncCnt_2 * 1.0) >= 3) {
        if (pid_flag == 0) {//如果获得了当前速度（TIM2中断控制），进行一次PID控制
            pid_flag = 1;//消除标志位

            average_speed = 0;
            Differential_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);;

            Car_wheel[0].standard_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);
            Car_wheel[1].standard_speed = PID_location((float) line_1 * 1.0f, (float) EncCnt_2 * 1.0f, &PID_angle_1);

            Car_wheel[0].pid_speed = PID_location(Car_wheel[0].standard_speed, Car_wheel[0].current_speed,
                                                  &Car_wheel[0].PID_AutoCar);//PID控制来达到右轮子目标速度设定
            Car_wheel[1].pid_speed = PID_location(Car_wheel[1].standard_speed, Car_wheel[1].current_speed,
                                                  &Car_wheel[1].PID_AutoCar);//PID控制来达到左轮子目标速度设定

            motor_control(Car_wheel[0].pid_speed, Car_wheel[1].pid_speed);//驱动小车
        }
    }

    motor_control(0, 0);

    ii++;
    UART_printf(&huart1, "i=%d\n", ii);
    switch (ii) {
        case 1:
            //发送信息让K210亮红灯
            Date[0] = 0, Date[1] = 0, Date[2] = 0x20;
            HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);
            HAL_Delay(1000);
            //等待取掉药物
            do {weight = shoWeight();
            } while (weight >= 50);
            //发送信息让K210灭红灯
            Date[0] = 0, Date[1] = 0, Date[2] = 0x21;
            HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);
            HAL_Delay(1000);
            //延迟一秒
            HAL_Delay(1000);
            weight_falg=1;
            break;
        case 2:
            //发信息给K210亮绿灯
            Date[0] = 0, Date[1] = 0, Date[2] = 0x30;
            HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);

            carstaion.car_date[0]=0;
            break;
        default:
            break;
    }

    HAL_UART_Receive_IT(&huart2, &Res, 1);
}

void Totation_5() {
    static int i=0;
    float weight=0;

    motor_control(0, 0);
    HAL_Delay(1000);

    Car_wheel[0].PID_AutoCar.kp = 150;
    Car_wheel[0].PID_AutoCar.ki = 4;
    Car_wheel[0].PID_AutoCar.kd = 10;
    Car_wheel[1].PID_AutoCar.kp = 150;
    Car_wheel[1].PID_AutoCar.ki = 4;
    Car_wheel[1].PID_AutoCar.kd = 10;

    //位置角度环PID参数
    PID_angle.kp = 0.05;
    PID_angle.kd = 0;

    PID_angle_1.kp = 0.05;
    PID_angle_1.kd = 0;

    uint32_t line, line_1;

    HAL_UART_AbortReceive_IT(&huart2);

    line = 180 * 6.6 + EncCnt;
    line_1 = -180 * 6.6 + EncCnt_2;

    while (abs(line * 1.0 - EncCnt * 1.0) >= 3 || abs(line_1 * 1.0 - EncCnt_2 * 1.0) >= 3) {
        if (pid_flag == 0) {//如果获得了当前速度（TIM2中断控制），进行一次PID控制
            pid_flag = 1;//消除标志位

            average_speed = 0;
            Differential_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);;

            Car_wheel[0].standard_speed = PID_location((float) line * 1.0f, (float) EncCnt * 1.0f, &PID_angle);
            Car_wheel[1].standard_speed = PID_location((float) line_1 * 1.0f, (float) EncCnt_2 * 1.0f, &PID_angle_1);

            Car_wheel[0].pid_speed = PID_location(Car_wheel[0].standard_speed, Car_wheel[0].current_speed,
                                                  &Car_wheel[0].PID_AutoCar);//PID控制来达到右轮子目标速度设定
            Car_wheel[1].pid_speed = PID_location(Car_wheel[1].standard_speed, Car_wheel[1].current_speed,
                                                  &Car_wheel[1].PID_AutoCar);//PID控制来达到左轮子目标速度设定

            motor_control(Car_wheel[0].pid_speed, Car_wheel[1].pid_speed);//驱动小车
        }
    }

    motor_control(0, 0);

    i++;
    UART_printf(&huart1,"i=%d\n",i);
    switch (i) {
        case 1:
            //发送信息让K210亮红灯
            Date[0] = 0, Date[1] = 0, Date[2] = 0x20;
            HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);
            HAL_Delay(1000);
            //等待取掉药物
            do {weight = shoWeight();
            } while (weight >= 50);
            //延迟一秒
            HAL_Delay(1000);
            weight_falg=1;
            break;
        case 2:
            do {weight = shoWeight();
                UART_printf(&huart1,"%.2f\n",weight);
                HAL_Delay(100);
            } while (weight >= 5);
            //延迟一秒
            HAL_Delay(1000);
            break;
        case 3:
            //发信息给K210亮绿灯
            Date[0] = 0, Date[1] = 0, Date[2] = 0x30;
            HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);

            carstaion.car_date[0]=0;
            break;
        default:break;
    }

    HAL_UART_Receive_IT(&huart2, &Res, 1);
}

void ready_Totation(){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
}
