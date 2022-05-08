//
// Created by zengshuai on 2022/4/3.
//

#include "task.h"
#include "HX711.h"
uint8_t turn_flag_4=0;//限制K210摄像头只打开一次
uint8_t turn_flag_41=0;//限制K210摄像头只打开一次

void Task_1(void) {
    memset(carstaion.car_mode,0x00,sizeof(carstaion.car_mode));
    int my_turn_command=0;
    uint16_t last_crossroads=0;
    int Back_turn_around[2]={1,1};

    while (carstaion.car_date[0] == 1) {
        //根据路口变化来完成控制
        if (carstaion.car_date[4] != last_crossroads) {
            switch (carstaion.car_date[4]) {
                case 1:
                    my_turn_command=1;
                    Turn_around= turn_date;
                    Back_turn_around[0]=-Turn_around;
                    //发送准备自转的信号
                    ready_Totation();
                    break;
                case 2:
                    my_turn_command=1;
                    Turn_around= Back_turn_around[0];
                    ready_Totation();
                    break;
                default:break;
            }
            UART_printf(&huart1,"cro=%d,turn=%d\n",carstaion.car_date[4],Turn_around);
            last_crossroads = carstaion.car_date[4];
        }

        if (carstaion.car_mode[2] == 1) {
            mode = 3;
        }
        if (carstaion.car_mode[1] == 1) {
            mode = 4;
            carstaion.car_mode[1]=0;
        }
        if (carstaion.car_mode[3] == 1) {
            mode = 0;
            carstaion.car_date[0]=0;
        }
        if (my_turn_command == 1) {
            mode = 2;
        }

        switch (mode) {
            case 1:
                break;
            case 2:
                my_turn_command = 0;
                Turn_1(Turn_around, 90);
                break;
            case 3:
                Target_tracking();
                break;
            case 4:
                Totation();
                break;
            default:
                motor_control(0, 0);
                break;
        }
    }
}
void Task_2(void ){
    memset(carstaion.car_mode, 0x00, sizeof(carstaion.car_mode));
    int my_turn_command = 0;
    uint16_t last_crossroads = 0;
    int Back_turn_around[2] = {1, 1};

    while (carstaion.car_date[0] == 2) {

        //根据路口变化来完成控制
        if (carstaion.car_date[4] != last_crossroads) {
            switch (carstaion.car_date[4]) {
                case 3:
                    my_turn_command = 1;
                    Turn_around = turn_date;
                    Back_turn_around[0] = -Turn_around;
                    break;
                case 4:
                    my_turn_command = 1;
                    Turn_around = turn_date;
                    ready_Totation();
                    break;
                case 5:
                    my_turn_command = 1;
                    Turn_around=Back_turn_around[0];
                    break;
                case 6:
                    my_turn_command = 1;
                    Turn_around=Back_turn_around[0];
                    break;
                case 8:
                    ready_Totation();
                default:
                    break;
            }
            UART_printf(&huart1, "cro=%d,turn=%d\n", carstaion.car_date[4], Turn_around);
            last_crossroads = carstaion.car_date[4];
        }

        if (carstaion.car_mode[2] == 1) {
            mode = 3;
        }
        if (carstaion.car_mode[1] == 1) {
            mode = 4;
            carstaion.car_mode[1] = 0;
        }
        if (carstaion.car_mode[3] == 1) {
            mode = 0;
            carstaion.car_date[0] = 0;
        }
        if (my_turn_command == 1) {
            mode = 2;
        }

        switch (mode) {
            case 1:
                break;
            case 2:
                my_turn_command = 0;
                Turn_1(Turn_around, 90);
                break;
            case 3:
                Target_tracking();
                break;
            case 4:
                Totation();
                break;
            default:
                motor_control(0, 0);
                break;
        }
    }
}

void Task_3(void) {
    memset(carstaion.car_mode, 0x00, sizeof(carstaion.car_mode));
    int my_turn_command = 0;
    uint16_t last_crossroads = 0;
    int Back_turn_around[2] = {1, 1};

    //识别要去的病房1或2
    Date[0] = 1, Date[1] = 1, Date[2] = 0X0;
    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);   //串口发送三个字节数据，最大传输时间0xffff
    while (rev6_flag == 0) {}
    rev6_flag = 0;
    //处理
    if (carstaion.car_date[2] == 1) {
        Turn_around = -1;
    } else {
        Turn_around = 1;
    }

    //发送信息让K210亮黄灯
    Date[0] = 0, Date[1] = 0, Date[2] = 0x10;
    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);

    float weight;//药物重量
    do {
        weight = shoWeight();
    } while (weight <= 200);

    //发送信息让K210灭黄灯
    Date[0] = 0, Date[1] = 0, Date[2] = 0x11;
    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);

    while (carstaion.car_date[0] == 3) {

        if (carstaion.car_date[4] != last_crossroads) {
            switch (carstaion.car_date[4]) {
                case 1:
                    my_turn_command = 1;
                    Back_turn_around[0] = -Turn_around;
                    //发送准备自转的信号
                    ready_Totation();
                    break;
                case 2:
                    my_turn_command = 1;
                    Turn_around = Back_turn_around[0];
                    ready_Totation();
                    break;
                default:
                    break;
            }
            UART_printf(&huart1, "cro=%d,turn=%d\n", carstaion.car_date[4], Turn_around);
            last_crossroads = carstaion.car_date[4];
        }

        if (carstaion.car_mode[2] == 1) {
            mode = 3;
        }
        if (carstaion.car_mode[1] == 1) {
            mode = 4;
            carstaion.car_mode[1] = 0;
        }
        if (carstaion.car_mode[3] == 1) {
            mode = 0;
            carstaion.car_date[0] = 0;
        }
        if (my_turn_command == 1) {
            mode = 2;
        }

        switch (mode) {
            case 1:
                break;
            case 2:
                Turn_2_4(Turn_around, 90);
                my_turn_command = 0;
                break;
            case 3:
                Target_tracking_3();
                break;
            case 4:
                Totation_3();
                break;
            default:
                motor_control(0, 0);
                break;
        }
    }
}
void Task_4(void) {
    memset(carstaion.car_mode, 0x00, sizeof(carstaion.car_mode));
    int my_turn_command=0;
    uint16_t last_crossroads=0;
    int Back_turn_around[2]={1,1};

    //第一次识别要去的病房5678
    Date[0] = 1, Date[1] = 1, Date[2] = 0X0;
    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);   //串口发送三个字节数据，最大传输时间0xffff
    while (rev6_flag == 0) {}
    rev6_flag = 0;
    //处理
    uint8_t number=0;
    number=carstaion.car_date[2];

    //发送信息让K210亮黄灯
    Date[0] = 0, Date[1] = 0, Date[2] = 0x10;
    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);

    float weight;//药物重量
    do {
        weight = shoWeight();
    } while (weight <= 200);

    //发送信息让K210灭黄灯
    Date[0] = 0, Date[1] = 0, Date[2] = 0x11;
    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);

    while (carstaion.car_date[0] == 4) {

        //根据路口的变化来做出判断
        if (carstaion.car_date[4] != last_crossroads) {
            switch (carstaion.car_date[4]) {
                case 2:
                    //开启识别要去的病房5678
                    Date[0] = 1, Date[1] = 2, Date[2] = 0X0;
                    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);   //串口发送三个字节数据
                    average_speed_34=13;
                    break;
                case 3:
                    //转弯置位
                    my_turn_command=1;
                    //转弯方向判断
                    if (number == carstaion.car_date[2] || number == carstaion.car_date[3]) {
                        Turn_around = -1;
                    } else {
                        Turn_around = 1;
                    }
                    Back_turn_around[1] = -Turn_around;
                    //开启识别要去的病房56或78
                    Date[0] = 1, Date[1] = 1, Date[2] = 0X0;
                    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);//串口发送三个字节数据
                    average_speed_34=13;
                    break;
                case 4:
                    //转弯置位
                    my_turn_command=1;
                    //转弯方向判断
                    if (number == carstaion.car_date[2]) {
                        Turn_around = -1;
                    } else {
                        Turn_around = 1;
                    }
                    Back_turn_around[0] = -Turn_around;
                    ready_Totation();
                    break;
                case 5:
                    //转弯置位
                    my_turn_command=1;
                    Turn_around = Back_turn_around[0];
                    break;
                case 6:
                    //转弯置位
                    my_turn_command=1;
                    Turn_around = Back_turn_around[1];
                    break;
                case 8:
                    ready_Totation();
                    break;
                default:
                    break;
            }
            UART_printf(&huart1,"cro=%d,turn=%d\n",carstaion.car_date[4],Turn_around);
            last_crossroads = carstaion.car_date[4];
        }

        if (carstaion.car_mode[2] == 1) {
            mode = 3;
        }
        if (carstaion.car_mode[1] == 1) {
            carstaion.car_mode[1]=0;
            mode = 4;
        }
        if (carstaion.car_mode[3] == 1) {
            mode = 0;
        }
        if (my_turn_command == 1) {
            mode = 2;
        }
        //数字检测结束后开始加速
        if (rev6_flag == 1) {
            rev6_flag = 0;
            average_speed_34 = 19;
        }

        switch (mode) {
            case 1:
                break;
            case 2:
                Turn_2_4(Turn_around, 90);
                my_turn_command = 0;
                break;
            case 3:
                Target_tracking_3();
                break;
            case 4:
                Totation_3();
                break;
            default:
                motor_control(0, 0);
                break;
        }
    }
}

void Task_5(void) {
    memset(carstaion.car_mode, 0x00, sizeof(carstaion.car_mode));
    int my_turn_command = 0;
    uint16_t last_crossroads = 0;
    int Back_turn_around[3] = {1, 1, 1};
    int car_case = 0;

    //第一次识别要去的两个病房5678
    Date[0] = 1, Date[1] = 3, Date[2] = 0X0;
    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);   //串口发送三个字节数据，最大传输时间0xffff
    while (rev6_flag == 0) {}
    rev6_flag = 0;
    //处理
    uint8_t number_1 = 0, number_2 = 0;
    number_1 = carstaion.car_date[2];
    number_2 = carstaion.car_date[3];

    //发送信息让K210亮黄灯
    Date[0] = 0, Date[1] = 0, Date[2] = 0x10;
    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);

    float weight;//药物重量
    do {
        weight = shoWeight();
    } while (weight <= 200);

    //发送信息让K210灭黄灯
    Date[0] = 0, Date[1] = 0, Date[2] = 0x11;
    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);

    while (carstaion.car_date[0] == 5) {

        //根据路口的变化来做出判断
        if (carstaion.car_date[4] != last_crossroads) {
            //前三个路口
            switch (carstaion.car_date[4]) {
                case 2:
                    //开启识别要去的病房5678
                    Date[0] = 1, Date[1] = 2, Date[2] = 0X0;
                    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);   //串口发送三个字节数据
                    average_speed_34 = 13;
                    break;
                case 3:
                    //转弯置位
                    my_turn_command = 1;
                    //判断路口情况
                    if ((number_1 == carstaion.car_date[2] && number_2 == carstaion.car_date[3]) ||
                        (number_1 == carstaion.car_date[3] && number_2 == carstaion.car_date[2])) {
                        car_case = case_1100;
                        Turn_around = -1;
                    } else if ((number_1 != carstaion.car_date[2] && number_1 != carstaion.car_date[3]) &&
                               (number_2 != carstaion.car_date[2] && number_2 != carstaion.car_date[3])) {
                        car_case = case_0011;
                        Turn_around = 1;
                    } else if(number_1 == carstaion.car_date[2]||number_1 == carstaion.car_date[3]){
                        car_case = case_1010;
                        Turn_around = -1;
                    } else{
                        car_case = case_0101;
                        Turn_around = 1;
                    }
                    break;
                default:
                    break;
            }
            //分情况讨论
            switch (car_case) {
                case case_0011:
                case case_1100:
                    switch (carstaion.car_date[4]) {
                        case 3:
                            //开启识别要去的第一个病房
                            Date[0] = 1, Date[1] = 1, Date[2] = 0X0;
                            HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);   //串口发送三个字节数据
                            //降速
                            average_speed_34 = 13;
                            Back_turn_around[1] = -Turn_around;
                            break;
                        case 4:
                            //转弯置位
                            my_turn_command = 1;
                            //转弯方向判断
                            if (number_1 == carstaion.car_date[2]) {
                                Turn_around = -1;
                            } else {
                                Turn_around = 1;
                            }

                            Back_turn_around[0] = Turn_around;

                            //发送准备自转的信号
                            ready_Totation();
                            break;
                        case 5:
                            //发送准备自转的信号
                            ready_Totation();
                            break;
                        case 6:
                            my_turn_command = 1;
                            Turn_around = Back_turn_around[0];
                            break;
                        case 7:
                            my_turn_command = 1;
                            Turn_around = Back_turn_around[1];
                            break;
                        case 9:
                            //发送准备自转的信号
                            ready_Totation();
                            break;
                        default:
                            break;
                    }
                    break;
                case case_0101:
                case case_1010:
                    switch (carstaion.car_date[4]) {
                        case 3:
                            //开启识别要去的第一个病房
                            Date[0] = 1, Date[1] = 1, Date[2] = 0X0;
                            HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);   //串口发送三个字节数据
                            average_speed_34 = 13;
                            Back_turn_around[2] = Turn_around;
                            break;
                        case 4:
                            my_turn_command = 1;
                            if (number_1 == carstaion.car_date[2]) {
                                Turn_around = -1;
                            } else {
                                Turn_around = 1;
                            }
                            Back_turn_around[0] = -Turn_around;

                            //发送准备自转的信号
                            ready_Totation();
                            break;
                        case 5:
                            my_turn_command = 1;
                            Turn_around = Back_turn_around[0];
                            break;
                        case 6:
                            //开启识别要去的第二个病房
                            Date[0] = 1, Date[1] = 1, Date[2] = 0X0;
                            HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);   //串口发送三个字节数据
                            average_speed_34 = 13;

                            break;
                        case 7:
                            my_turn_command = 1;
                            if (number_2 == carstaion.car_date[2]) {
                                Turn_around = -1;
                            } else {
                                Turn_around = 1;
                            }
                            Back_turn_around[1] = -Turn_around;
                            //发送准备自转的信号
                            ready_Totation();
                            break;
                        case 8:
                            my_turn_command = 1;
                            Turn_around = Back_turn_around[1];
                            break;
                        case 9:
                            my_turn_command = 1;
                            Turn_around = Back_turn_around[2];
                            break;
                        case 11:
                            //发送准备自转的信号
                            ready_Totation();
                            break;
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            }
            UART_printf(&huart1, "cro=%d,turn=%d,case=%d\n", carstaion.car_date[4], Turn_around,car_case);
            last_crossroads = carstaion.car_date[4];
        }
        if (carstaion.car_mode[2] == 1) {
            mode = 3;
        }
        if (carstaion.car_mode[1] == 1) {
            carstaion.car_mode[1]=0;
            mode = 4;
        }
        if (carstaion.car_mode[3] == 1) {
            mode = 0;
            //发信息给K210亮绿灯
            Date[0] = 0, Date[1] = 0, Date[2] = 0x30;
            HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);
            //结束后定死
        }
        if (my_turn_command == 1) {
            mode = 2;
        }
        //数字检测结束后开始加速
        if (rev6_flag == 1) {
            rev6_flag = 0;
            average_speed_34 = 19;
        }

        switch (mode) {
            case 1:
                break;
            case 2:
                Turn_2_4(Turn_around, 90);
                my_turn_command = 0;
                break;
            case 3:
                Target_tracking_3();
                break;
            case 4:
                Totation_5();
                break;
            default:
                motor_control(0, 0);
                break;
        }
    }
}