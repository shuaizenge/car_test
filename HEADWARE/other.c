#include "other.h"
#include "usart.h"
#include "main.h"
#include <stdio.h>
#include <math.h>
int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit (&huart1 ,(uint8_t *)&ch,1,HAL_MAX_DELAY );
	return ch;
}
void LIGHT()
{
//	HAL_GPIO_WritePin(,,1);
//	HAL_Delay(1000);
//	HAL_GPIO_WritePin(,,0);
}
