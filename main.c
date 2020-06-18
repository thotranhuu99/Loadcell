/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
double adc_value;
unsigned long int sample;
double sum;
//char adc_lcd;
//char *test=&adc_lcd;
char adc_lcd_memory;
char *adc_lcd=&adc_lcd_memory;
char offset_lcd_memory;
char *offset_lcd=&offset_lcd_memory;
double weight;
double weight_pre;
double adc_offset =300;
double pre;
double gain;
uint32_t adc_dma;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//if(hadc->Instance==hadc1.Instance)
		//sum+=HAL_ADC_GetValue(&hadc1);
		/*sum+=adc_dma;
		sample++;
		HAL_ADC_Stop_DMA(&hadc1);
		HAL_ADC_Start_DMA(&hadc1,&adc_dma,32);
		if (sample==1400)
		{
			HAL_ADC_Stop_DMA(&hadc1);
		}*/
}
void delay (uint16_t delay)
{__HAL_TIM_SET_COUNTER(&htim1,0);
while(__HAL_TIM_GET_COUNTER(&htim1)<delay);	
}
void delay1s(void)
{for(int i=1;i<40000;i++)
	delay(100);}
void clear_lcd(void)
{lcd_goto_XY(1,0);
	lcd_send_string("                ");
	lcd_goto_XY(2,0);
	lcd_send_string("                ");
}
void clear_warning(void)
{lcd_goto_XY(2,10);
	lcd_send_string("            ");
}
double lowpass(double pre, uint32_t now)
{
	return((1-0.00042094746)*pre+0.00042094746*(float)now);
}
void get_weight(void)
{ sum=0;
	sample=0;
	while(sample<3000)
	{//sum+=adc_dma;
		pre=lowpass(pre,adc_dma);
		sum+=pre;
		sample++;
	}
	adc_value=sum/(double)sample;
	weight=gain*(adc_value-adc_offset);
}
double average(void)
{double sum_average=0;
	double sample_average=0;
	while (sample_average<5)
	{sum_average+=(double) adc_dma;
		sample_average++;
	}
	return(sum_average/sample_average);}
void calculate_offset()
{ double sum=0;
	clear_lcd();
	for(int i=0;i<3;i++)
	{
	lcd_goto_XY(1,0);
	lcd_send_string ("Tinh toan diem 0 cua can: ");
	//get_weight();
	sum+=average();
	lcd_goto_XY(2,5*i-1);
	lcd_send_string (".......");
	}
	adc_offset=sum/3.0;
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string ("Gia tri Vref: ");
	sprintf(offset_lcd,"%3.8f",adc_offset*3.2/4095);
	lcd_goto_XY(2,0);
	lcd_send_string(offset_lcd);
	lcd_goto_XY(2,10);
	lcd_send_string ("V");
	lcd_goto_XY(2,12);
	sprintf(offset_lcd,"%d",(int) adc_offset);
	lcd_send_string(offset_lcd);
	for(int i=1;i<20000;i++)
	delay(100);
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string ("Dat vat len can ");
	for(int i=1;i<40000;i++)
	delay(100);
}
void warning_high_adc()
{ clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string("     Luu y:  ");
	delay1s();
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string("Vuot qua khoi   ");
	lcd_goto_XY(2,0);
	lcd_send_string("luong do duoc  ");
	delay1s();
	delay1s();
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string("   Huong dan: ");
	delay1s();
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string("Giam khoi luong");
	lcd_goto_XY(2,0);
	lcd_send_string("hoac van Vref ");
	delay1s();
	delay1s();
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string("nguoc chieu kim");
	lcd_goto_XY(2,0);
	delay1s();
	delay1s();
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string("   Nhac lai: ");
	delay1s();
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string("Giam khoi luong");
	lcd_goto_XY(2,0);
	lcd_send_string("hoac van Vref ");
	delay1s();
	delay1s();
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string("nguoc chieu kim");
	lcd_goto_XY(2,0);
	lcd_send_string("dong ho ");
	delay1s();
	delay1s();
	clear_lcd();
}
void warning_low_adc()
{ clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string("     Luu y:  ");
	delay1s();
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string ("Tang Vref len de");
	lcd_goto_XY(2,0);
	lcd_send_string ("co ket qua chuan");	
	delay1s();
	delay1s();
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string("   Huong dan: ");
	delay1s();
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string(" Van Vref cung ");
	lcd_goto_XY(2,0);
	lcd_send_string(" chieu dong ho");
	delay1s();
	delay1s();
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string("   Nhac lai: ");
	delay1s();
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string(" Van Vref cung ");
	lcd_goto_XY(2,0);
	lcd_send_string(" chieu dong ho");
	delay1s();
	delay1s();
	clear_lcd();
}
void main_warning_low__adc()
	{ 
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string("     Luu y:  ");
	HAL_Delay(1000);
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string ("Tang Vref len de");
	lcd_goto_XY(2,0);
	lcd_send_string ("co ket qua chuan");	
	HAL_Delay(2000);
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string("   Huong dan: ");
	HAL_Delay(1000);
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string(" Van Vref cung ");
	lcd_goto_XY(2,0);
	lcd_send_string(" chieu dong ho");
	HAL_Delay(2000);
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string("   Nhac lai: ");
	HAL_Delay(1000);
	clear_lcd();
	lcd_goto_XY(1,0);
	lcd_send_string(" Van Vref cung ");
	lcd_goto_XY(2,0);
	lcd_send_string(" chieu dong ho");
	HAL_Delay(2000);
	clear_lcd();
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
	if( HAL_ADC_Start(&hadc1) != HAL_OK)  
  return 0;  
	if (HAL_ADC_Start_DMA(&hadc1,&adc_dma, 1) != HAL_OK)  
  return 0;
	HAL_Delay(100);
	lcd_init();
	/*for(int i=0;i<4;i++)
	{
	lcd_goto_XY(1,0);
	lcd_send_string ("Lay vat khoi can");
	lcd_goto_XY(2,4*i);
	lcd_send_string ("....");
	HAL_Delay(100);}*/
	calculate_offset();
	pre=adc_offset;
	gain=0.58860863291;//0.5918*166/170*166/162*166.5/167.5;
	clear_lcd();
	if (adc_offset<400)
	warning_low_adc();
	//HAL_ADC_Start_DMA (&hadc1, &buffer,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
