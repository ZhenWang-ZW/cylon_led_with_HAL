/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

typedef struct{
	GPIO_TypeDef* GPIO_x;
	uint16_t GPIO_Pin;
}GPIO_TYPE;

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int DisplayMode = 1;
int DisplayCounter = 0;
int ReadS1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);

void TOGGLE_CYLON_DISPLAY(GPIO_TYPE* GPIO_ARRAY);
void RESET_ALL_LED(GPIO_TYPE* GPIO_ARRAY);
void SET_ALL_LED(GPIO_TYPE* GPIO_ARRAY);
void SET_PART_LED(GPIO_TYPE* GPIO_ARRAY, int* startpos);
void GET_PART_LET_POSITIONS_BY_ADC_VALUE(int* startpos, uint32_t adc_result);
//void USART_PRINT(char *msg);
//void USART2_write (int ch);
/* USER CODE BEGIN PFP */

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
	GPIO_TYPE PA10 	= {.GPIO_x = GPIOA, .GPIO_Pin=GPIO_PIN_10};
	GPIO_TYPE PB3 	= {.GPIO_x = GPIOB, .GPIO_Pin=GPIO_PIN_3};
	GPIO_TYPE PB5 	= {.GPIO_x = GPIOB, .GPIO_Pin=GPIO_PIN_5};
	GPIO_TYPE PB4	= {.GPIO_x = GPIOB, .GPIO_Pin=GPIO_PIN_4};
	GPIO_TYPE PB10	= {.GPIO_x = GPIOB, .GPIO_Pin=GPIO_PIN_10};
	GPIO_TYPE PA8	= {.GPIO_x = GPIOA, .GPIO_Pin=GPIO_PIN_8};
	GPIO_TYPE PC1	= {.GPIO_x = GPIOC, .GPIO_Pin=GPIO_PIN_1};
	GPIO_TYPE PC0	= {.GPIO_x = GPIOC, .GPIO_Pin=GPIO_PIN_0};

	GPIO_TYPE GPIO_ARRAY[8] = {PA10, PB3, PB5, PB4, PB10, PA8, PC1, PC0};

	uint32_t ADC_Status;
	uint32_t ADC_Result = 0;
	int startpos = 0;
	uint8_t cylon_msg[20] = "LED Cylon Display\r\n";
	uint8_t adc_msg[20]="";


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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  ReadS1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
	  if(ReadS1 == 1){
		  if(DisplayMode==1){
			  RESET_ALL_LED(GPIO_ARRAY);
		  }else{
			  SET_ALL_LED(GPIO_ARRAY);
		  }
		  TOGGLE_CYLON_DISPLAY(GPIO_ARRAY);
		  if(DisplayCounter++>=5){
			  DisplayCounter=0;
			  if(DisplayMode==1)
				  DisplayMode=0;
			  else
				  DisplayMode=1;
			  HAL_UART_Transmit(&huart2, (uint8_t *)cylon_msg, sizeof(cylon_msg), HAL_MAX_DELAY);
		  }
	  }else{
		  ADC_Status = HAL_ADC_Start(&hadc1);
		  ADC_Status = HAL_ADC_PollForConversion(&hadc1, 100);
		  if (ADC_Status == HAL_OK)
		  {
			  ADC_Result = HAL_ADC_GetValue(&hadc1);
		  }
		  GET_PART_LET_POSITIONS_BY_ADC_VALUE(&startpos, ADC_Result);
		  SET_PART_LED(GPIO_ARRAY, &startpos);
		  sprintf(adc_msg, "ADC Result = %d\r\n", ADC_Result);
		  HAL_UART_Transmit(&huart2, (uint8_t*)adc_msg, sizeof(adc_msg), HAL_MAX_DELAY);
//		  HAL_Delay(500);
	  }



    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void TOGGLE_CYLON_DISPLAY(GPIO_TYPE* GPIO_ARRAY){
	int i=0;
	int sizeofgpio = sizeof(*GPIO_ARRAY);
	for(i=0; i<sizeofgpio; i++){
		 HAL_GPIO_TogglePin(GPIO_ARRAY[i].GPIO_x, GPIO_ARRAY[i].GPIO_Pin);
		 HAL_Delay(250);
		 HAL_GPIO_TogglePin(GPIO_ARRAY[i].GPIO_x, GPIO_ARRAY[i].GPIO_Pin);
	 }

	 for(i=sizeofgpio-2; i>0; i--){
		 HAL_GPIO_TogglePin(GPIO_ARRAY[i].GPIO_x, GPIO_ARRAY[i].GPIO_Pin);
		 HAL_Delay(250);
		 HAL_GPIO_TogglePin(GPIO_ARRAY[i].GPIO_x, GPIO_ARRAY[i].GPIO_Pin);
	 }
}

void RESET_ALL_LED(GPIO_TYPE* GPIO_ARRAY){
	int sizeofgpio = sizeof(*GPIO_ARRAY);
	for(int i=0; i<sizeofgpio; i++){
		HAL_GPIO_WritePin(GPIO_ARRAY[i].GPIO_x, GPIO_ARRAY[i].GPIO_Pin, GPIO_PIN_RESET);
	}
}

void SET_ALL_LED(GPIO_TYPE* GPIO_ARRAY){
	int sizeofgpio = sizeof(*GPIO_ARRAY);
	for(int i=0; i<sizeofgpio; i++){
		HAL_GPIO_WritePin(GPIO_ARRAY[i].GPIO_x, GPIO_ARRAY[i].GPIO_Pin, GPIO_PIN_SET);
	}
}

void SET_PART_LED(GPIO_TYPE* GPIO_ARRAY, int* startpos){
	int sizeofgpio = sizeof(*GPIO_ARRAY);
	for(int i=0; i<sizeofgpio; i++){
		if(i==*startpos){
			HAL_GPIO_WritePin(GPIO_ARRAY[i].GPIO_x, GPIO_ARRAY[i].GPIO_Pin, GPIO_PIN_SET);
			*startpos=i+1;
		}
		else
			HAL_GPIO_WritePin(GPIO_ARRAY[i].GPIO_x, GPIO_ARRAY[i].GPIO_Pin, GPIO_PIN_RESET);
	}
}

void GET_PART_LET_POSITIONS_BY_ADC_VALUE(int* startpos, uint32_t adc_result){

	if(adc_result<=511){
		*startpos=7;
	}else if(adc_result>=512 && adc_result<=1023){
		*startpos=6;
	}else if(adc_result>=1024 && adc_result<=1535){
		*startpos=5;
	}else if(adc_result>=1536 && adc_result<=2047){
		*startpos=4;
	}else if(adc_result>=2048 && adc_result<=2559){
		*startpos=3;
	}else if(adc_result>=2560 && adc_result<=3071){
		*startpos=2;
	}else if(adc_result>=3072 && adc_result<=3583){
		*startpos=1;
	}else if(adc_result>=3584 && adc_result<=4095){
		*startpos=0;
	}else{
		*startpos=0;
	}
}

//void USART_PRINT(char *buffer){
//	for(int i=0; i<strlen(buffer);i++){
//		USART2_write(buffer[i]);
//	}
//	USART2_write('\r');
//	USART2_write('\n');
//}
//
//void USART2_write (int ch) {
//    while (!(USART2->ISR & 0x0080)) {}   // wait until Tx buffer empty
//    USART2->TDR = (ch & 0xFF);
//}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
