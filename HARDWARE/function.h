#ifndef __FUNCTION_H
#define __FUNCTION_H

#include "f10x.h"
#include "stdint.h"
#include "stm32f1xx_hal.h"

void TIM_IT_START(uint16_t time_val);
void TIM_PWM_Int(TIM_HandleTypeDef *htim,uint32_t Channel,uint16_t freq,uint32_t duty);
void TIM1_START(void);
uint32_t GET_TIM1_CNT(void);
void ADC2_Init(uint32_t ADC_PIN,uint32_t ADC_CHANNEL);
uint16_t get_adc(uint32_t ADC_CHANNEL);
extern ADC_HandleTypeDef hadc2;	


#endif
