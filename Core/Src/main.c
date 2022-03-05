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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rx_buffer[20] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void UART_SendText(volatile char*);
void UART_SendNumber(uint32_t x);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t IC_Value1 = 0;
uint32_t IC_Value2 = 0;
uint32_t Difference = 0;
uint32_t Frequency = 0;
int test = 0;
uint8_t Is_First_Captured = 0;  // 0- not captured, 1- captured
char str_Freq[32];
int ALLUME_LED = 0;
uint16_t timer_val = 0;
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  //HAL_UART_Transmit(&huart2, (uint8_t *)" UART pOLLING METHOD\r\n", sizeof(" UART pOLLING METHOD\r\n"), 300);
  //HAL_UART_Transmit(&huart2, (uint8_t *)"UART INTERUpT\r\n", sizeof("UART INTERUpT\r\n"), 300);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim16);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  int f_test = 40000;
  timer_val = __HAL_TIM_GET_COUNTER(&htim16);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*
	  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_RESET);
	  HAL_Delay(5000);
	  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);
	  HAL_Delay(500);
	  */

//	  UART_SendText("Frequency ");
//	  UART_SendNumber(Frequency);
//	  UART_SendText("\r\n");
	  /*
	  if(Frequency < 10000)
	  {
		  ALLUME_LED =~ ALLUME_LED;
		  HAL_Delay(500);
//		  if(ALLUME_LED == 1)
//		  {
//			  ALLUME_LED = 0;
//		  }
//		  if(ALLUME_LED == 0)
//		  {
//			  ALLUME_LED = 1;
//		  }
	  }
	  */
	  //HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);
	  HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
	  HAL_Delay(500);
	  HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
	  HAL_Delay(500);
	  HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
	  HAL_Delay(500);
	  HAL_GPIO_TogglePin(LED_4_GPIO_Port, LED_4_Pin);
	  HAL_Delay(1000);
	  if(Frequency < 10000)
	  {
		  if(ALLUME_LED == 1)
		  {
			  ALLUME_LED = 0;
			  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		  }
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  /*
	  timer_val = __HAL_TIM_GET_COUNTER(&htim16);
	  HAL_Delay(1000);
	  timer_val = __HAL_TIM_GET_COUNTER(&htim16) - timer_val;
	  UART_SendNumber(timer_val);
	  HAL_Delay(1000);
	  UART_SendText("\r\n");
	  */

	  UART_SendText("Frequency ");
	  //HAL_Delay(500);
	  UART_SendNumber(Frequency);
	  //HAL_Delay(500);
	  UART_SendText("\r\n");
	  //HAL_Delay(500);
	  HAL_Delay(300);
	  Frequency = 0;



	  /*
	  char term_buffer_UART[100];
	  sprintf(str_Freq, "%ld", Frequency);
	  snprintf(term_buffer_UART, sizeof(term_buffer_UART), "Frequency = %s Hz Capacity %s pF\r\n", str_Freq, "50");
	  UART_SendText(term_buffer_UART);
	  */

	  //UART_SendText("Frequency = 4108 Hz Capacity = 24 pF\r\n");
	  //UART_SendText("4108,24\r\n");
	  //HAL_UART_Transmit(&huart2, (uint8_t *)"UART INTERUpT\r\n", sizeof("UART INTERUpT\r\n"), 300);
	  /*
	  HAL_UART_Transmit(&huart2, (uint8_t *)" UART pOLLING METHOD\r\n", sizeof(" UART pOLLING METHOD\r\n"), 300);
	  HAL_UART_Receive(&huart2, rx_buffer,20, 300);
	  HAL_UART_Transmit(&huart2, rx_buffer, 20, 300);
	  */
  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM16;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 8000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65536 - 1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  USART1->CR1 |= (USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE);
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_1_Pin|LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_3_Pin|LED_4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_1_Pin LED_2_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin|LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_3_Pin LED_4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LED_3_Pin|LED_4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void UART_SendText(volatile char* s)
{
	uint16_t i = 0;
	uint16_t size = 0;
	while(s[i])
	{
		size++;
		i++;
	}
	size++; // '\0'
	HAL_UART_Transmit(&huart2, (uint8_t *)s, size, 300);
}

void UART_SendNumber(uint32_t x)
{
	char temp[100];
	sprintf(temp, "%d", x);
	UART_SendText(temp);
	/*
	  char value[10]; //a temp array to hold results of conversion
	  int i = 0; //loop index
	  uint16_t size = 0;
	  do
	  {
		value[i++] = (char)(x % 10) + '0'; //convert integer to character
		x /= 10;
		size++;
	  } while(x);

	  while(i) //send data
	  {
		HAL_UART_Transmit(&huart2, (uint8_t *)value[--i], sizeof(char), 300);

	  }
	  */
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{


		  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if interrput source is channel 1
		   {
		    if (Is_First_Captured==0)  // is the first value captured ?
		    {
		     IC_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // capture the first value
		     Is_First_Captured =1;  // set the first value captured as true
		    }

		    else if (Is_First_Captured)  // if the first is captured
		    {
		     IC_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // capture second value

		     if (IC_Value2 > IC_Value1)
		     {
		      Difference = IC_Value2-IC_Value1;   // calculate the difference
		     }

		     else if (IC_Value2 < IC_Value1)
		     {
		      Difference = ((0xffff-IC_Value1)+IC_Value2) +1;
		     }

		     else
		     {
		      Error_Handler();
		     }

		     Frequency = HAL_RCC_GetPCLK1Freq()/Difference;  // calculate frequency
		     Is_First_Captured = 0;  // reset the first captured
		     if(__HAL_TIM_GET_COUNTER(&htim16) - timer_val >= 1000)
		     {
				  test += 1;
				  timer_val = __HAL_TIM_GET_COUNTER(&htim16);
				  if(Frequency < 10000)
				  	  {
//				  		  ALLUME_LED =~ ALLUME_LED;
//				  		  HAL_Delay(500);
//				  		  if(ALLUME_LED == 1)
//				  		  {
//				  			  ALLUME_LED = 0;
//				  		  }
				  		  if(ALLUME_LED == 0)
				  		  {
				  			  ALLUME_LED = 1;
				  			  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
				  		  }
				  	  }
		     }
		    }
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
