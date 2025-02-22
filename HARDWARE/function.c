#include "function.h"
#include "adc.h"
#include "tim.h"
#include "f10x.h"
#include "stdint.h"
#include "stm32f1xx_hal.h"
///////////////////////////////////////////////////////////
//													    //
//        需要用到的模块功能函数在此调用               //
//												      //
///////////////////////////////////////////////////////
//定时器模块

//启动定时器中断
//设置中断时间 单位：ms
//中断执行的回调函数在main.c中定义
void TIM_IT_START(uint16_t time_val)
{
	htim2.Instance->ARR=(time_val*10)-1;
	htim2.Init.Period=(time_val*10)-1;
	__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim2);
}

//启动定时器1
void TIM1_START(void)
{
	htim1.Instance->CNT=0;//清空计数器的值
	__HAL_TIM_ENABLE(&htim1);//启动定时器
}
//获取定时器计数到的值
uint32_t GET_TIM1_CNT(void)
{
	__HAL_TIM_DISABLE(&htim1);
	uint32_t time=0;
	time=htim1.Instance->CNT;//获取计数器的值
	htim1.Instance->CNT=0;//清空计数器的值
	return time;//返回获取到的值
}

//PWM模块
/*
PWM输出引脚通道对照
PB9 ----->TIM4_CH4  
PB8 ----->TIM4_CH3
PB7 ----->TIM4_CH2
PB6 ----->TIM4_CH1
PB5 ----->TIM3_CH2
PB4 ----->TIM3_CH1
注意PB4默认上电状态为高电平，所以电机一开始时会空转一会儿，正常现象，等带单片机初始化完成即可
*/
/*
PWM定时器初始化函数
htim:定时器模块
Channel：定时器通道
freq :预分频系数，通过修改系数来修改PWM输出频率，频率=7.2khz/freq, freq=0时，pwm输出频率为7.2khz.
duty：pwm输出占空比
例：TIM_PWM_Int(&htim4,TIM_CHANNEL_1,0,0);初始化PWM PB6引脚，定时器模块4，通道一，频率7.2khz,占空比0
*/
void TIM_PWM_Int(TIM_HandleTypeDef *htim,uint32_t Channel,uint16_t freq,uint32_t duty)
{	                
	htim->Instance->PSC=freq;//   修改预分频系数 频率=TIMx_CLK/( (ARR+1)*(PSC+1) ）ARR=9999
	__HAL_TIM_SET_COMPARE(htim,Channel,duty);//设定占空比
	HAL_TIM_PWM_Start(htim,Channel);//启动PWM输出
}
/*
pwm修改占空比
此函数是HAL库封装好的函数，直接调用即可
htim：定时器模块
Channel：定时器通道
duty：占空比的值 
注意：占空比的最大值为10000；设定时不要超过这个值

  __HAL_TIM_SET_COMPARE(&htim,Channel,duty)；
*/

//ADC采集

ADC_HandleTypeDef hadc2;//ADC2初始化的结构体
ADC_ChannelConfTypeDef ADC_sConfig = {0}; //ADC2通道的结构体
/*
函数名：ADC2_Init     功能：ADC引脚初始化
函数参数：
ADC_PIN: ADC转换引脚
ADC_CHANNEL：ADC转换通道
注意：ADC的转换通道大部分对应的是A端口的引脚，所以初始化中配置的只有A端口。
引脚初始化时选择PA0~PA5引脚作为ADC的转换的输入引脚
A0~A5的引脚与ADC的转换通道对应 
例：GPIO_PIN_1----->ADC_CHANNEL_1
	GPIO_PIN_2----->ADC_CHANNEL_2
	……
初始化时应注意选择与引脚对应的ADC转换通道。
例：ADC2_Init(GPIO_PIN_1,ADC_CHANNEL_1)；初始化ADC模块，PA1引脚，ADC通道1

初始化之后需执行ADC校准函数：
HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef* hadc);
*/
void ADC2_Init(uint32_t ADC_PIN,uint32_t ADC_CHANNEL)
{
	GPIO_InitTypeDef  GPIO_InitStructure={0};
	
	__HAL_RCC_ADC2_CLK_ENABLE();//使能ADC2时钟
	__HAL_RCC_GPIOA_CLK_ENABLE();//使能A端口时钟
	//GPIO引脚初始化
	GPIO_InitStructure.Pin = ADC_PIN;//初始化ADC引脚
	GPIO_InitStructure.Mode=GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOA,&GPIO_InitStructure);
	//初始化ADC模块
	hadc2.Instance = ADC2;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}
	//设置ADC转换通道
	ADC_sConfig.Channel = ADC_CHANNEL;
	ADC_sConfig.Rank = ADC_REGULAR_RANK_1;
	ADC_sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc2, &ADC_sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

//获取ADC转换的值
//参数：
//     ADC_CHANNEL：ADC转换通道
uint16_t get_adc(uint32_t ADC_CHANNEL)
{
	ADC_sConfig.Channel =ADC_CHANNEL;//修改ADC转换通道
	if (HAL_ADC_ConfigChannel(&hadc2, &ADC_sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2,10);	
	return HAL_ADC_GetValue(&hadc2);
}



