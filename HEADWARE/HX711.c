/**
 * HX711驱动程序
 *
 * SCK:
 * 推挽输出;速率:High; output level:Hight; 无需上下拉
 *
 * DIO：
 * 上拉输入；
 */

#include "HX711.h"
#include "main.h"

uint32_t Read_Weight(void)
{
    uint32_t Data = 0;
    HAL_GPIO_WritePin(SCK_GPIO_Port,SCK_Pin,GPIO_PIN_RESET);

    while(HAL_GPIO_ReadPin(DIO_GPIO_Port,DIO_Pin) == 1);

    for(uint8_t i=0;i<24;i++)
    {
        HAL_GPIO_WritePin(SCK_GPIO_Port,SCK_Pin,GPIO_PIN_SET);
        Data = Data << 1;
        if(HAL_GPIO_ReadPin(DIO_GPIO_Port,DIO_Pin) == 1)
            Data++;
        Delay_us(20);
        HAL_GPIO_WritePin(SCK_GPIO_Port,SCK_Pin,GPIO_PIN_RESET);
        Delay_us(20);
    }
    HAL_GPIO_WritePin(SCK_GPIO_Port,SCK_Pin,GPIO_PIN_SET);
    Delay_us(10);
    HAL_GPIO_WritePin(SCK_GPIO_Port,SCK_Pin,GPIO_PIN_RESET);

    if((Data & 0x7fffff) == 0x7fffff)
        Read_Weight();

    return(Data);
}

float shoWeight(void)
{
    uint32_t valInit=456315,valCurrent;
    uint16_t GapValue = 409;
    float Weight;

/*    valInit = (Read_Weight() / 1000) * 1000;*/

    valCurrent = (Read_Weight() / 1000) * 1000;
    valCurrent = valCurrent - valInit;
    Weight = ((float)valCurrent / GapValue)-13.90;

    return Weight;
}

void Delay_us(uint32_t nus)
{
    uint32_t temp;
    SysTick->LOAD = nus * 100;      /* 时间加载-时钟频率 */
    SysTick->VAL = 0x00;            /* 清空计数�???????? */
    SysTick->CTRL |= 1 << 0 ;       /* �????????始�?�数 */

    do
    {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16))); /* CTRL.ENABLE位必须为1, 并等待时间到�???????? */

    SysTick->CTRL &= ~(1 << 0) ;    /* 关闭SYSTICK */
    SysTick->VAL = 0X00;            /* 清空计数�???????? */
}

void delay_ms(uint16_t nms)
{
    uint32_t repeat = nms / 1000;   /*  这里用1000,是考虑到可能有超频应用,
                                     *  比如128Mhz的时候, delay_us最大只能延时1048576us左右了
                                     */
    uint32_t remain = nms % 1000;

    while (repeat)
    {
        Delay_us(1000 * 1000);      /* 利用delay_us 实现 1000ms 延时 */
        repeat--;
    }

    if (remain)
    {
        Delay_us(remain * 1000);    /* 利用delay_us, 把尾数延时(remain ms)给做了 */
    }
}

void HAL_Delay(uint32_t Delay)
{
    delay_ms(Delay);
}