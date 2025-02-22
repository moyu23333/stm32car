#ifndef __OLED_H
#define __OLED_H
#include "stdlib.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_rcc.h"
//-----------------OLED IIC���Ŷ���----------------  	
#define OLED_PORT_SCL   GPIOA 								 //OLED SCL�˿ڶ���
#define OLED_SCL    	GPIO_PIN_12                           //����SCL����  ���������Ϊ����GPIO

#define OLED_PORT_SDA   GPIOA 								//OLED  SDA�˿ڶ���
#define OLED_SDA    	GPIO_PIN_15                           //����SDA����  ���������Ϊ����GPIO

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
