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
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "bmp.h"
#include "stdint.h"
#include "function.h"
#include "OLED.h"

//�Լ�����ı���
uint16_t ADC_left,ADC_right=0;
int16_t erro=0;
//PID
int16_t adc_kp=600,adc_kd=3;//PID��ϵ��
uint16_t last_adc_erro=0;
int16_t adc_out=0;//PID���ֵ
void SystemClock_Config(void);

//������ƺ���������ʵ�ֵ������ת�Լ�ռ�ձȵ���
void CAR_GO(int16_t duty1,int16_t duty2)
{	
	if(duty1>8000)duty1=8000;//�޷�����ֹռ�ձȹ����ջ����
	else if(duty1<-8000)duty1=-8000;
	if(duty2>8000)duty2=8000;
	else if(duty2<-8000)duty2=-8000;
	
	if(duty1<0)//�ж������������������������������ת�����������ת
	{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,-duty1);//������ͨ����ռ�ձȣ������ת
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
	}
	else
	{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,duty1);//������ͨ��һռ�ձȣ������ת��
	}
	
	if(duty2<0)//���2ͬ��
	{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,-duty2);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
	}
	else
	{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,duty2);
	}
}
/*-------------------------------------------------------
 * ���Թ�һ��
 *
 ------------------------------------------------------*/
float MinMax_scaling(int date,int max,int min)
{
    int16_t datesum=0,datediff=0;
    datediff=max-min;
    datesum=date-min;
    return (float)datesum/datediff;
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
	
  //��Ӧ�ĳ�ʼ���������������
  OLED_Init();
  //ע��OLED����������������ţ����ź궨����OLED.H�У���ҿ����Զ����޸�Ϊ�Լ��õ�OLED����
  //���PWM��ʼ��
  TIM_PWM_Int(&htim4,TIM_CHANNEL_1,0,0);
  TIM_PWM_Int(&htim4,TIM_CHANNEL_2,0,0);
  TIM_PWM_Int(&htim4,TIM_CHANNEL_3,0,0);
  TIM_PWM_Int(&htim4,TIM_CHANNEL_4,0,0);
  //����źŲɼ���ʼ��
  //����ѭ�����
  ADC2_Init(GPIO_PIN_0,ADC_CHANNEL_0);
  ADC2_Init(GPIO_PIN_1,ADC_CHANNEL_1);
  //����ADC����Ҫ����ADC�ĵ�У׼
  HAL_ADCEx_Calibration_Start(&hadc1);//ADCУ׼
  //��ʱ���жϣ�һ��ʼѧ�Ȳ����˽⣩
  TIM_IT_START(5);//������5ms��PID�Ŀ��Ƴ���һ��ͨ����ʱ�ж�ִ��
  
  while (1)
  {
	 //С�����Ƴ���
	//�ɼ���е��ź�ֵ
	ADC_left=get_adc(ADC_CHANNEL_0);
	ADC_right=get_adc(ADC_CHANNEL_1);
	//ע�⣺ADC�ɼ�����ֵӦ�������˲�����Ч������ã�����û���ã��������ˣ������õĵ���˲��㷨�У�ƽ��ֵ�˲��������˲��������Թ������˲���Ч������Ҫ��ƽ��ֵ�˲���ƽ��
	erro=(ADC_left-ADC_right)*100/(ADC_left+ADC_right+1);//��Ⱥͼ���ƫ�� ��ɹ��ںŵ��������н��ܲ�Ⱥ��㷨
	  
	adc_out=adc_kp*erro+adc_kd*(erro-last_adc_erro);//PID�������������ֻ����P��D�Ŀ��ƣ�û�м�I��������
	last_adc_erro=erro;//����Dֵ
	  
	if((ADC_right+ADC_left)<100)//�߽籣������ֹС�����������ײ��
	CAR_GO(0,0);//���������С��ռ�ձȸ�0��Ҳ���Ը���ֵ��ʵ�ֵ�����Ч��
	else
	CAR_GO(4000+adc_out,4000-adc_out);//ѭ������ ��еĲ�ֵ��ΪС�����ֲ��ٵ�ֵ�����Դﵽ��˳���Ŀ���Ч��
	
	//OLED��ʾ
	OLED_ShowSignedNum(1,2,ADC_right,4);
	OLED_ShowSignedNum(2,2,ADC_left,4);
	OLED_ShowSignedNum(3,2,erro,4);
	OLED_ShowSignedNum(4,2,adc_out,4);
	
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//��ʱ���жϻص�����
{
if(htim->Instance==TIM2)//��ʱ��2���ж�
{
	
//	 //�ж�ִ�������ڴ˵���
//	  ADC_value_fliter();//�����˲�
//	  erro=(move_filter_average[1]-move_filter_average[0])*100/(move_filter_average[1]+move_filter_average[0]+1);
//	  adc_out=adc_kp*erro+adc_kd*(erro-last_adc_erro);
//	  last_adc_erro=erro;
//	if((move_filter_average[0]+move_filter_average[1])<100)
//		CAR_GO(0,0);
//	else
//      CAR_GO(4000+adc_out,4000-adc_out);
	
}

}

/* USER CODE END 4 */

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
