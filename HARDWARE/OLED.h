#ifndef __OLED_H
#define __OLED_H
#include "stdlib.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_rcc.h"
//-----------------OLED IIC引脚定义----------------  	
#define OLED_PORT_SCL   GPIOA 								 //OLED SCL端口定义
#define OLED_SCL    	GPIO_PIN_12                           //定义SCL引脚  可任意更改为其他GPIO

#define OLED_PORT_SDA   GPIOA 								//OLED  SDA端口定义
#define OLED_SDA    	GPIO_PIN_15                           //定义SDA引脚  可任意更改为其他GPIO

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowFloatNum(uint8_t Line, uint8_t Column, float Fumber);
void OLED_DrawBMP(uint8_t x0, uint8_t y0,uint8_t x1, uint8_t y1,uint8_t BMP[]);

#endif
