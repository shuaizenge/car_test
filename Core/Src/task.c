//
// Created by zengshuai on 2022/4/3.
//

#include "task.h"
#include "HX711.h"
uint8_t turn_flag_4=0;//����K210����ͷֻ��һ��
uint8_t turn_flag_41=0;//����K210����ͷֻ��һ��

void Task_1(void) {
    memset(carstaion.car_mode,0x00,sizeof(carstaion.car_mode));
    int my_turn_command=0;
    uint16_t last_crossroads=0;
    int Back_turn_around[2]={1,1};

    while (carstaion.car_date[0] == 1) {
        //����·�ڱ仯����ɿ���
        if (carstaion.car_date[4] != last_crossroads) {
            switch (carstaion.car_date[4]) {
                case 1:
                    my_turn_command=1;
                    Turn_around= turn_date;
                    Back_turn_around[0]=-Turn_around;
                    //����׼����ת���ź�
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

        //����·�ڱ仯����ɿ���
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

    //ʶ��Ҫȥ�Ĳ���1��2
    Date[0] = 1, Date[1] = 1, Date[2] = 0X0;
    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);   //���ڷ��������ֽ����ݣ������ʱ��0xffff
    while (rev6_flag == 0) {}
    rev6_flag = 0;
    //����
    if (carstaion.car_date[2] == 1) {
        Turn_around = -1;
    } else {
        Turn_around = 1;
    }

    //������Ϣ��K210���Ƶ�
    Date[0] = 0, Date[1] = 0, Date[2] = 0x10;
    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);

    float weight;//ҩ������
    do {
        weight = shoWeight();
    } while (weight <= 200);

    //������Ϣ��K210��Ƶ�
    Date[0] = 0, Date[1] = 0, Date[2] = 0x11;
    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);

    while (carstaion.car_date[0] == 3) {

        if (carstaion.car_date[4] != last_crossroads) {
            switch (carstaion.car_date[4]) {
                case 1:
                    my_turn_command = 1;
                    Back_turn_around[0] = -Turn_around;
                    //����׼����ת���ź�
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

    //��һ��ʶ��Ҫȥ�Ĳ���5678
    Date[0] = 1, Date[1] = 1, Date[2] = 0X0;
    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);   //���ڷ��������ֽ����ݣ������ʱ��0xffff
    while (rev6_flag == 0) {}
    rev6_flag = 0;
    //����
    uint8_t number=0;
    number=carstaion.car_date[2];

    //������Ϣ��K210���Ƶ�
    Date[0] = 0, Date[1] = 0, Date[2] = 0x10;
    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);

    float weight;//ҩ������
    do {
        weight = shoWeight();
    } while (weight <= 200);

    //������Ϣ��K210��Ƶ�
    Date[0] = 0, Date[1] = 0, Date[2] = 0x11;
    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);

    while (carstaion.car_date[0] == 4) {

        //����·�ڵı仯�������ж�
        if (carstaion.car_date[4] != last_crossroads) {
            switch (carstaion.car_date[4]) {
                case 2:
                    //����ʶ��Ҫȥ�Ĳ���5678
                    Date[0] = 1, Date[1] = 2, Date[2] = 0X0;
                    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);   //���ڷ��������ֽ�����
                    average_speed_34=13;
                    break;
                case 3:
                    //ת����λ
                    my_turn_command=1;
                    //ת�䷽���ж�
                    if (number == carstaion.car_date[2] || number == carstaion.car_date[3]) {
                        Turn_around = -1;
                    } else {
                        Turn_around = 1;
                    }
                    Back_turn_around[1] = -Turn_around;
                    //����ʶ��Ҫȥ�Ĳ���56��78
                    Date[0] = 1, Date[1] = 1, Date[2] = 0X0;
                    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);//���ڷ��������ֽ�����
                    average_speed_34=13;
                    break;
                case 4:
                    //ת����λ
                    my_turn_command=1;
                    //ת�䷽���ж�
                    if (number == carstaion.car_date[2]) {
                        Turn_around = -1;
                    } else {
                        Turn_around = 1;
                    }
                    Back_turn_around[0] = -Turn_around;
                    ready_Totation();
                    break;
                case 5:
                    //ת����λ
                    my_turn_command=1;
                    Turn_around = Back_turn_around[0];
                    break;
                case 6:
                    //ת����λ
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
        //���ּ�������ʼ����
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

    //��һ��ʶ��Ҫȥ����������5678
    Date[0] = 1, Date[1] = 3, Date[2] = 0X0;
    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);   //���ڷ��������ֽ����ݣ������ʱ��0xffff
    while (rev6_flag == 0) {}
    rev6_flag = 0;
    //����
    uint8_t number_1 = 0, number_2 = 0;
    number_1 = carstaion.car_date[2];
    number_2 = carstaion.car_date[3];

    //������Ϣ��K210���Ƶ�
    Date[0] = 0, Date[1] = 0, Date[2] = 0x10;
    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);

    float weight;//ҩ������
    do {
        weight = shoWeight();
    } while (weight <= 200);

    //������Ϣ��K210��Ƶ�
    Date[0] = 0, Date[1] = 0, Date[2] = 0x11;
    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);

    while (carstaion.car_date[0] == 5) {

        //����·�ڵı仯�������ж�
        if (carstaion.car_date[4] != last_crossroads) {
            //ǰ����·��
            switch (carstaion.car_date[4]) {
                case 2:
                    //����ʶ��Ҫȥ�Ĳ���5678
                    Date[0] = 1, Date[1] = 2, Date[2] = 0X0;
                    HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);   //���ڷ��������ֽ�����
                    average_speed_34 = 13;
                    break;
                case 3:
                    //ת����λ
                    my_turn_command = 1;
                    //�ж�·�����
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
            //���������
            switch (car_case) {
                case case_0011:
                case case_1100:
                    switch (carstaion.car_date[4]) {
                        case 3:
                            //����ʶ��Ҫȥ�ĵ�һ������
                            Date[0] = 1, Date[1] = 1, Date[2] = 0X0;
                            HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);   //���ڷ��������ֽ�����
                            //����
                            average_speed_34 = 13;
                            Back_turn_around[1] = -Turn_around;
                            break;
                        case 4:
                            //ת����λ
                            my_turn_command = 1;
                            //ת�䷽���ж�
                            if (number_1 == carstaion.car_date[2]) {
                                Turn_around = -1;
                            } else {
                                Turn_around = 1;
                            }

                            Back_turn_around[0] = Turn_around;

                            //����׼����ת���ź�
                            ready_Totation();
                            break;
                        case 5:
                            //����׼����ת���ź�
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
                            //����׼����ת���ź�
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
                            //����ʶ��Ҫȥ�ĵ�һ������
                            Date[0] = 1, Date[1] = 1, Date[2] = 0X0;
                            HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);   //���ڷ��������ֽ�����
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

                            //����׼����ת���ź�
                            ready_Totation();
                            break;
                        case 5:
                            my_turn_command = 1;
                            Turn_around = Back_turn_around[0];
                            break;
                        case 6:
                            //����ʶ��Ҫȥ�ĵڶ�������
                            Date[0] = 1, Date[1] = 1, Date[2] = 0X0;
                            HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);   //���ڷ��������ֽ�����
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
                            //����׼����ת���ź�
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
                            //����׼����ת���ź�
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
            //����Ϣ��K210���̵�
            Date[0] = 0, Date[1] = 0, Date[2] = 0x30;
            HAL_UART_Transmit(&huart6, (uint8_t *) Date, 3, 0xffff);
            //��������
        }
        if (my_turn_command == 1) {
            mode = 2;
        }
        //���ּ�������ʼ����
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