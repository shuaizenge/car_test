#ifndef HX711_HX711_H
#define HX711_HX711_H

#include "main.h"
#include "system_stm32f4xx.h"

uint32_t Read_Weight(void);
float shoWeight(void);
void Delay_us(uint32_t nus);
void delay_ms(uint16_t nms);

#endif //HX711_HX711_H
