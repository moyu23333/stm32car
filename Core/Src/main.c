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

//自己定义的变量
uint16_t ADC_left,ADC_right=0;
int16_t erro=0;
//PID
int16_t adc_kp=600,adc_kd=3;//PID的系数
uint16_t last_adc_erro=0;
int16_t adc_out=0;//PID输出值
void SystemClock_Config(void);

//电机控制函数，可以实现电机正反转以及占空比调速
void CAR_GO(int16_t duty1,int16_t duty2)
{	
	if(duty1>8000)duty1=8000;//限幅，防止占空比过大烧坏电机
	else if(duty1<-8000)duty1=-8000;
	if(duty2>8000)duty2=8000;
	else if(duty2<-8000)duty2=-8000;
	
	if(duty1<0)//判断输入量的正负，如果是正数则电机正转，负数电机反转
	{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,-duty1);//负数给通道二占空比，电机反转
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
	}
	else
	{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,duty1);//正数给通道一占空比，电机正转，
	}
	
	if(duty2<0)//电机2同上
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
 * 线性归一化
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
	
  //相应的初始化函数在这里调用
  OLED_Init();
  //注意OLED如果不亮，请检查引脚，引脚宏定义在OLED.H中，大家可以自定义修改为自己用的OLED引脚
  //电机PWM初始化
  TIM_PWM_Int(&htim4,TIM_CHANNEL_1,0,0);
  TIM_PWM_Int(&htim4,TIM_CHANNEL_2,0,0);
  TIM_PWM_Int(&htim4,TIM_CHANNEL_3,0,0);
  TIM_PWM_Int(&htim4,TIM_CHANNEL_4,0,0);
  //电磁信号采集初始化
  //左右循迹电感
  ADC2_Init(GPIO_PIN_0,ADC_CHANNEL_0);
  ADC2_Init(GPIO_PIN_1,ADC_CHANNEL_1);
  //开启ADC后需要进行ADC的的校准
  HAL_ADCEx_Calibration_Start(&hadc1);//ADC校准
  //定时器中断（一开始学先不用了解）
  TIM_IT_START(5);//设置了5ms，PID的控制程序一般通过定时中断执行
  
  while (1)
  {
	 //小车控制程序
	//采集电感的信号值
	ADC_left=get_adc(ADC_CHANNEL_0);
	ADC_right=get_adc(ADC_CHANNEL_1);
	//注意：ADC采集到的值应当进行滤波处理效果会更好，这里没有用（懒得用了），常用的电磁滤波算法有：平均值滤波、滑动滤波，粗略试过滑动滤波的效果可能要比平均值滤波更平滑
	erro=(ADC_left-ADC_right)*100/(ADC_left+ADC_right+1);//差比和计算偏差 逐飞公众号的推文中有介绍差比和算法
	  
	adc_out=adc_kp*erro+adc_kd*(erro-last_adc_erro);//PID控制输出（这里只用了P、D的控制，没有加I控制量）
	last_adc_erro=erro;//更新D值
	  
	if((ADC_right+ADC_left)<100)//边界保护，防止小车冲出赛道后撞坏
	CAR_GO(0,0);//冲出赛道，小车占空比给0，也可以给负值，实现倒车的效果
	else
	CAR_GO(4000+adc_out,4000-adc_out);//循迹控制 电感的差值作为小车俩轮差速的值，可以达到很顺滑的控制效果
	
	//OLED显示
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//定时器中断回调函数
{
if(htim->Instance==TIM2)//定时器2的中断
{
	
//	 //中断执行内容在此调用
//	  ADC_value_fliter();//滑动滤波
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
