#include "function.h"
#include "adc.h"
#include "tim.h"
#include "f10x.h"
#include "stdint.h"
#include "stm32f1xx_hal.h"
///////////////////////////////////////////////////////////
//													    //
//        ��Ҫ�õ���ģ�鹦�ܺ����ڴ˵���               //
//												      //
///////////////////////////////////////////////////////
//��ʱ��ģ��

//������ʱ���ж�
//�����ж�ʱ�� ��λ��ms
//�ж�ִ�еĻص�������main.c�ж���
void TIM_IT_START(uint16_t time_val)
{
	htim2.Instance->ARR=(time_val*10)-1;
	htim2.Init.Period=(time_val*10)-1;
	__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim2);
}

//������ʱ��1
void TIM1_START(void)
{
	htim1.Instance->CNT=0;//��ռ�������ֵ
	__HAL_TIM_ENABLE(&htim1);//������ʱ��
}
//��ȡ��ʱ����������ֵ
uint32_t GET_TIM1_CNT(void)
{
	__HAL_TIM_DISABLE(&htim1);
	uint32_t time=0;
	time=htim1.Instance->CNT;//��ȡ��������ֵ
	htim1.Instance->CNT=0;//��ռ�������ֵ
	return time;//���ػ�ȡ����ֵ
}

//PWMģ��
/*
PWM�������ͨ������
PB9 ----->TIM4_CH4  
PB8 ----->TIM4_CH3
PB7 ----->TIM4_CH2
PB6 ----->TIM4_CH1
PB5 ----->TIM3_CH2
PB4 ----->TIM3_CH1
ע��PB4Ĭ���ϵ�״̬Ϊ�ߵ�ƽ�����Ե��һ��ʼʱ���תһ������������󣬵ȴ���Ƭ����ʼ����ɼ���
*/
/*
PWM��ʱ����ʼ������
htim:��ʱ��ģ��
Channel����ʱ��ͨ��
freq :Ԥ��Ƶϵ����ͨ���޸�ϵ�����޸�PWM���Ƶ�ʣ�Ƶ��=7.2khz/freq, freq=0ʱ��pwm���Ƶ��Ϊ7.2khz.
duty��pwm���ռ�ձ�
����TIM_PWM_Int(&htim4,TIM_CHANNEL_1,0,0);��ʼ��PWM PB6���ţ���ʱ��ģ��4��ͨ��һ��Ƶ��7.2khz,ռ�ձ�0
*/
void TIM_PWM_Int(TIM_HandleTypeDef *htim,uint32_t Channel,uint16_t freq,uint32_t duty)
{	                
	htim->Instance->PSC=freq;//   �޸�Ԥ��Ƶϵ�� Ƶ��=TIMx_CLK/( (ARR+1)*(PSC+1) ��ARR=9999
	__HAL_TIM_SET_COMPARE(htim,Channel,duty);//�趨ռ�ձ�
	HAL_TIM_PWM_Start(htim,Channel);//����PWM���
}
/*
pwm�޸�ռ�ձ�
�˺�����HAL���װ�õĺ�����ֱ�ӵ��ü���
htim����ʱ��ģ��
Channel����ʱ��ͨ��
duty��ռ�ձȵ�ֵ 
ע�⣺ռ�ձȵ����ֵΪ10000���趨ʱ��Ҫ�������ֵ

  __HAL_TIM_SET_COMPARE(&htim,Channel,duty)��
*/

//ADC�ɼ�

ADC_HandleTypeDef hadc2;//ADC2��ʼ���Ľṹ��
ADC_ChannelConfTypeDef ADC_sConfig = {0}; //ADC2ͨ���Ľṹ��
/*
��������ADC2_Init     ���ܣ�ADC���ų�ʼ��
����������
ADC_PIN: ADCת������
ADC_CHANNEL��ADCת��ͨ��
ע�⣺ADC��ת��ͨ���󲿷ֶ�Ӧ����A�˿ڵ����ţ����Գ�ʼ�������õ�ֻ��A�˿ڡ�
���ų�ʼ��ʱѡ��PA0~PA5������ΪADC��ת������������
A0~A5��������ADC��ת��ͨ����Ӧ 
����GPIO_PIN_1----->ADC_CHANNEL_1
	GPIO_PIN_2----->ADC_CHANNEL_2
	����
��ʼ��ʱӦע��ѡ�������Ŷ�Ӧ��ADCת��ͨ����
����ADC2_Init(GPIO_PIN_1,ADC_CHANNEL_1)����ʼ��ADCģ�飬PA1���ţ�ADCͨ��1

��ʼ��֮����ִ��ADCУ׼������
HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef* hadc);
*/
void ADC2_Init(uint32_t ADC_PIN,uint32_t ADC_CHANNEL)
{
	GPIO_InitTypeDef  GPIO_InitStructure={0};
	
	__HAL_RCC_ADC2_CLK_ENABLE();//ʹ��ADC2ʱ��
	__HAL_RCC_GPIOA_CLK_ENABLE();//ʹ��A�˿�ʱ��
	//GPIO���ų�ʼ��
	GPIO_InitStructure.Pin = ADC_PIN;//��ʼ��ADC����
	GPIO_InitStructure.Mode=GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOA,&GPIO_InitStructure);
	//��ʼ��ADCģ��
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
	//����ADCת��ͨ��
	ADC_sConfig.Channel = ADC_CHANNEL;
	ADC_sConfig.Rank = ADC_REGULAR_RANK_1;
	ADC_sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc2, &ADC_sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

//��ȡADCת����ֵ
//������
//     ADC_CHANNEL��ADCת��ͨ��
uint16_t get_adc(uint32_t ADC_CHANNEL)
{
	ADC_sConfig.Channel =ADC_CHANNEL;//�޸�ADCת��ͨ��
	if (HAL_ADC_ConfigChannel(&hadc2, &ADC_sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2,10);	
	return HAL_ADC_GetValue(&hadc2);
}



