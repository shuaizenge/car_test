/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  * @history:
  * version2.0 date4.7
  * K210烧录出现问题
  * 串口的不定长接收（研究一下）和串口接受数据重写（每次接受两个数据）
  * version2.1 date4.9
  * 下次做好开始时引脚命名工作，方便之后的代码对现实的分析
  * version2.2 date 4.10
  * 已完成：第一二三题，第四题计划用编码器解决
  *
  * bug:
  * 1.小车转向90度不准的问题：
  * 要求：转弯后速度要低，转弯停留时间不能太久，负重之后转弯不能变
  * 尝试解决1，转弯的精度要提高，pid降低
  * 尝试解决2，提高POD结算次数
  * 2.小车摇摆，幅度大
  * 尝试解决1，再调一下PID
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdarg.h>
#include <arithmetic.h>
#include <string.h>
#include "HX711.h"
#include "task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

Wheel Car_wheel[2];//两个轮子的结构体
Wheel Car_wheel[2] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
//两个轮子结构体的初始化（感觉这样0是不是太多了，应该有更简洁的写法=-=）
Car_station carstaion;
Car_station carstaion ={{0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0}};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define Target_distance 80 //80为160像素屏幕宽的中间值

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//串口调试
int UART_printf(UART_HandleTypeDef *huart, const char *fmt, ...);
//接受处理函数
void Receive1_treatment(uint8_t rev1_buff);
void Receive2_treatment(uint8_t rev2_buff[10]);
void Receive6_treatment(uint8_t rev6_buff[10]);

uint32_t solve_encode(TIM_HandleTypeDef *htim);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//蓝牙参数
uint8_t Ress=0;
uint8_t uart1_flag_rx_ok = 1;
uint8_t uart1_rx_con=0;
uint8_t uart1_rx_checksum=0;
uint8_t uart1_rx_buf[20];//0-3为校验位，4-9为数据位
//4、5为第一遥感控制，6、7为第二遥感控制
//4、5、6、7、8也为参数调整，方便小车PID调节,或者速度控制

//openmv摄像头数据传送
uint8_t Res=0;//因为串口每次传递一个数据，所以用RES来保存
uint8_t Video_rx_con=0;
uint8_t Video_rx_buf[10];//0-1为校验位，2-8为数据位，9为结尾

//K210数据传输
uint8_t uart6_Res=0;
uint8_t uart6_rx_con=0;
uint8_t uart6_rx_buf[10];//0-1为校验位，2-8为数据位，9为结尾
//16进制给K210发信息
uint8_t Date[3]={0,0,0x00};
int rev6_flag=0;//用来判断K210第一次数据发送到达
int rev6_flag_1=0;//用来判断K210第二次数据发送到达
int rev6_flag_2=0;//用来判断K210第二次数据发送到达

//设定初始值为目标值，保证开始的时候静止
float dirction_distance=Target_distance;

int Turn_around=1;
int turn_date=1;

//编码器参数，因为编码器只有16位，储存的数据容易溢出造成测不准，我们用算法来解决这个问题
uint32_t EncCnt = 100000; //解决TIM4的编码器ARR溢出
uint32_t EncCnt_2 = 100000; //解决TIM5的编码器ARR溢出

//进入PID状态获取
uint16_t pid_flag=1;

uint8_t mode = 0;//0,暂停；1，蓝牙遥控；2，转弯；3，巡线；
uint8_t task = 0;//0,默认停止；1-5，五道题目；

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

    //外设初始化
    //定时器中断打开
    HAL_TIM_Base_Start_IT(&htim2);//2ms一次，用来获取小车两轮子速度
//    HAL_TIM_Base_Start_IT(&htim10);//100ms一次
    //小车驱动PWM波输出
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    //编码器定时器打开
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2);

    //串口中断
    HAL_UART_Receive_IT(&huart1, &Ress, 1);//用来接受蓝牙数据;
    HAL_UART_Receive_IT(&huart2, &Res, 1);//用来接受openmv数据;
    HAL_UART_Receive_IT(&huart6, &uart6_Res, 1);//用来接受K210数据;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    motor_init();//电机初始化

    float weight;//药物重量
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    switch (carstaion.car_date[0]) {
            case 0:motor_control(0,0);break;
            case 1:Task_1();break;
            case 2:Task_2();break;
            case 3:Task_3();break;
            case 4:Task_4();break;
            case 5:Task_5();break;
            default:
                motor_control(0,0);
        }

    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//串口重定向
int UART_printf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int length;
    char buffer[128];
    length = vsnprintf(buffer, 128, fmt, ap);
    HAL_UART_Transmit(huart, (uint8_t *)buffer, length, HAL_MAX_DELAY);
    va_end(ap);
    return length;
}

//串口中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        Receive1_treatment(Ress);
        HAL_UART_Receive_IT(&huart1, &Ress, 1);
    }
    if (huart->Instance == USART2) {

        if (Video_rx_con >= 10)  //溢出判断
        {
            Video_rx_con = 0;
            memset(Video_rx_buf, 0x00, sizeof(Video_rx_buf));
        } else {
            Video_rx_buf[Video_rx_con++] = Res;
            if (Video_rx_buf[5] == 0x5B && Video_rx_buf[0] == 0X2C) {

                Receive2_treatment(Video_rx_buf);

                Video_rx_con = 0;
                memset(Video_rx_buf, 0x00, sizeof(Video_rx_buf));
            }
        }
        HAL_UART_Receive_IT(&huart2, &Res, 1);
    }
    if (huart->Instance == USART6) {
        if (uart6_rx_con >= 10)  //溢出判断
            {
            uart6_rx_con = 0;
            memset(uart6_rx_buf, 0x00, sizeof(uart6_rx_buf));
            } else {
            uart6_rx_buf[uart6_rx_con++] = uart6_Res;
            if (uart6_rx_buf[uart6_rx_con - 1] == 0x5B) {//结束位判断

                Receive6_treatment(uart6_rx_buf);

                uart6_rx_con = 0;
                memset(uart6_rx_buf, 0x00, sizeof(uart6_rx_buf));
            }
        }
        HAL_UART_Receive_IT(&huart6, &uart6_Res, 1);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim==&htim2){//2ms一次

        //解决TIM4编码器CNT溢出
        static int encoder_1[2] = {0, 0};// 两次编码器读值，用以计算溢出方向
        static int N_1 = 0;// 圈数
        encoder_1[1] = TIM5->CNT;
        if ((encoder_1[1] - encoder_1[0]) > 0x7FFF) {
            N_1--;
        } else if ((encoder_1[0] - encoder_1[1]) > 0x7FFF) {
            N_1++;
        }
        EncCnt = N_1 * 0xFFFF + encoder_1[1]+100000;
        encoder_1[0] = encoder_1[1];

        //解决TIM5编码器CNT溢出
        static int encoder_2[2] = {0, 0};// 两次编码器读值，用以计算溢出方向
        static int N_2 = 0;// 圈数
        encoder_2[1] = TIM4->CNT;
        if ((encoder_2[1] - encoder_2[0]) > 0x7FFF) {
            N_2--;
        } else if ((encoder_2[0] - encoder_2[1]) > 0x7FFF) {
            N_2++;
        }
        EncCnt_2 = N_2 * 0xFFFF + encoder_2[1] + 100000;
        encoder_2[0] = encoder_2[1];

        //每获得一次速度后进行一次pid解算
        pid_flag=0;

        //获得当前一号轮子的速度
        Car_wheel[0].CNT_prevalue=Car_wheel[0].value;
        Car_wheel[0].current_speed=(EncCnt*1.0f-Car_wheel[0].CNT_prevalue)*500/(13*4);
        Car_wheel[0].value=EncCnt*1.0f;

        //获得当前二号轮子的速度
        Car_wheel[1].CNT_prevalue=Car_wheel[1].value;
        Car_wheel[1].current_speed=(EncCnt_2*1.0f-Car_wheel[1].CNT_prevalue)*500/(13*4);
        Car_wheel[1].value=EncCnt_2*1.0f;
        //公式应该是：
        // （CNT2-CNT1）*计数器溢出频率/（编码器线数*采集边沿数）
        //PS：因为设置了TI1和TI2共同采样，所以采集边沿为4个。

//        UART_printf(&huart6, "%.2f\n",Car_wheel[0].current_speed);

    }
    if(htim==&htim10){//100ms一次

    }
}

uint32_t solve_encode(TIM_HandleTypeDef *htim){
    static uint32_t encoder_1[2] = {0, 0};// 两次编码器读值，用以计算溢出方向
    static int N_1 = 0;// 圈数
    encoder_1[1] = htim->Instance->CNT;
    if ((encoder_1[1] - encoder_1[0]) > 0x7FFF) {
        N_1--;
    } else if ((encoder_1[0] - encoder_1[1]) > 0x7FFF) {
        N_1++;
    }
    EncCnt = N_1 * 0xFFFF + encoder_1[1];
    encoder_1[0] = encoder_1[1];
    return EncCnt;
}

void Receive1_treatment(uint8_t rev1_buff){
    rev1_buff=rev1_buff-48;
    if(rev1_buff<=10){
        carstaion.car_date[0]= rev1_buff;
        turn_date= 1;
    }
    if(rev1_buff==6){
        carstaion.car_date[0]=1;
        turn_date=-1;
    }
    if(rev1_buff==7){
        carstaion.car_date[0]=2;
        turn_date=-1;
    }
}
void Receive2_treatment(uint8_t rev2_buff[10]){
    if (rev2_buff[2] == 2) {
        carstaion.car_mode[0] = 1;
    } else if (rev2_buff[2] == 4) {
        carstaion.car_mode[1] = 1;
    } else {
        carstaion.car_mode[2] = 1;
    }

    //获取中心距离差，并映射到0-160;
    carstaion.car_date[1] = rev2_buff[3];
    dirction_distance = 1.0f * carstaion.car_date[1] * 80 / 32;
    //获取十字路口数
    carstaion.car_date[4] = rev2_buff[4];
}
void Receive6_treatment(uint8_t rev6_buff[10]){
    rev6_flag=1;
    carstaion.car_date[2]=rev6_buff[0];
    carstaion.car_date[3]=rev6_buff[1];
    UART_printf(&huart1,"%d,%d\n",carstaion.car_date[2],carstaion.car_date[3]);
}
//if (huart->Instance == USART1) {
//    if (uart1_rx_con < 3) {
//        if (uart1_rx_con == 0) {
//            if (Ress == 0xAA) {
//                uart1_rx_buf[0] = Ress;
//                uart1_rx_con = 1;
//            } else {
//                uart1_rx_con = 0;
//            }
//        } else if (uart1_rx_con == 1) {
//            if (Ress == 0x55) {
//                uart1_rx_buf[1] = Ress;
//                uart1_rx_con = 2;
//            } else {
//                uart1_rx_con = 0;
//            }
//        } else {
//            uart1_rx_buf[2] = Ress;
//            uart1_rx_con = 3;
//            uart1_rx_checksum = (0xAA + 0x55) + Ress;
//        }
//    }
//    else {
//        if (uart1_rx_con < (uart1_rx_buf[2] - 1)) {
//            uart1_rx_buf[uart1_rx_con] = Ress;
//            uart1_rx_con++;
//            uart1_rx_checksum = uart1_rx_checksum + Ress;
//        }
//        else {
//            if (Ress == uart1_rx_checksum) {
//                uart1_flag_rx_ok = 1;
//
//                Receive1_treatment(uart1_rx_buf);
//
//                uart1_rx_con = 0;
//            }
//        }
//    }
//    HAL_UART_Receive_IT(&huart1, &Ress, 1);
//}
///* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

