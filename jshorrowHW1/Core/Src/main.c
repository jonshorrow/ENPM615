/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

uint32_t upperDisplay = 12;
uint32_t lowerDisplay = 34;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
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
  MX_USART3_UART_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
#if Switch
  uint32_t check = 60000;
  uint32_t countOne = 0;
  uint32_t countTwo = check / 3;
  uint32_t countThree = countTwo * 2;

  uint16_t displayCount = 0;
  uint8_t displayIndex = 1;
  uint16_t displayCheck = 500;

  //toggleAndPrintGpioState(GPIOG, DisplayFour_Pin, (uint8_t*) "Display: 4");
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  	HAL_TIM_Base_Start_IT(&htim2);
#if Blinky
  	toggleAndPrintGpioState(GPIOB, LED_RED_Pin, (uint8_t*) "STM: RED LED");
  	HAL_Delay(175);
  	toggleAndPrintGpioState(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, (uint8_t*) "STM: YELLOW LED");
  	HAL_Delay(175);
  	toggleAndPrintGpioState(GPIOB, LED_GREEN_Pin, (uint8_t*) "STM: GREEN LED");
  	HAL_Delay(175);
  	toggleAndPrintGpioState(GPIOA, YellowLED_Pin, (uint8_t*) "EXTERNAL: YELLOW LED");
  	HAL_Delay(175);
  	toggleAndPrintGpioState(GPIOA, BlueLED_Pin, (uint8_t*) "EXTERNAL: BLUE LED");
  	HAL_Delay(175);
  	toggleAndPrintGpioState(GPIOA, RedLED_Pin, (uint8_t*) "EXTERNAL: RED LED");
  	HAL_Delay(175);
#endif
#if Switch
  	// Check switches to see if things need to flip
  	// This runs so fast and there isnt a need for acuracy (like latching)
  	// so no need to debounce
  	switchToggle(GPIOD,
								 SwitchOne_Pin,
								 GPIOA,
								 YellowLED_Pin,
								 (uint8_t*) "External: Yellow");
  	switchToggle(GPIOD,
								 SwtichTwo_Pin                                                                                                                                                                ,
								 GPIOA,
								 BlueLED_Pin,
								 (uint8_t*) "External: Blue");
  	switchToggle(GPIOG,
								 Switch1_Pin                                                                                                                                                                ,
								 GPIOA,
								 RedLED_Pin,
								 (uint8_t*) "External: RED");

  	// This blinks the STM leds
  	// Just do some general incrementing in main because i dont want to hold up anything else
  	// Print the temp whever you toggle the green LED
  	countOne += 1;
  	countTwo += 1;
  	countThree += 1;
  	if (countOne > check)
  	{
  		toggleAndPrintGpioState(GPIOB, LED_RED_Pin, (uint8_t*) "STM: RED LED");
  		countOne = 0;
  	}
  	if (countTwo > check)
		{
  		toggleAndPrintGpioState(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, (uint8_t*) "STM: YELLOW LED");
			countTwo = 0;
		}
  	if (countThree > check)
		{
  		toggleAndPrintGpioState(GPIOB, LED_GREEN_Pin, (uint8_t*) "STM: GREEN LED");
			countThree = 0;
			printTemp();
		}


  	// Logic to display different numbers on the display
  	// Could be better but it works
  	displayCount += 1;

  	if (displayCount >= displayCheck)
  	{
  		displayCount = 0;
  		displayIndex += 1;
  	}
  	if (displayIndex == 3)
  	{
  		setDecimal(1);
  	}
  	else
  	{
  		setDecimal(0);
  	}

  	if (displayIndex >= 5)
  	{
  		displayIndex = 0;
  	}

  	displayOneNumber((lowerDisplay + (100 * upperDisplay)), displayIndex);
#endif

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 275;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 549;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 549;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DisplayOne_Pin|Middle_Pin|LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, DisplayTwo_Pin|TopRight_Pin|BottomRight_Pin|Decimal_Pin
                          |BottomMid_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Trig_Pin|BlueLED_Pin|YellowLED_Pin|RedLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, TopMid_Pin|BottomLeft_Pin|DisplayFour_Pin|Switch1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_FS_PWR_EN_Pin|TopLeft_Pin|DisplayThree_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DisplayOne_Pin Middle_Pin LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = DisplayOne_Pin|Middle_Pin|LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DisplayTwo_Pin TopRight_Pin BottomRight_Pin Decimal_Pin
                           BottomMid_Pin */
  GPIO_InitStruct.Pin = DisplayTwo_Pin|TopRight_Pin|BottomRight_Pin|Decimal_Pin
                          |BottomMid_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Trig_Pin */
  GPIO_InitStruct.Pin = Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Trig_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BlueLED_Pin YellowLED_Pin RedLED_Pin */
  GPIO_InitStruct.Pin = BlueLED_Pin|YellowLED_Pin|RedLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TopMid_Pin BottomLeft_Pin DisplayFour_Pin Switch1_Pin */
  GPIO_InitStruct.Pin = TopMid_Pin|BottomLeft_Pin|DisplayFour_Pin|Switch1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS_PWR_EN_Pin TopLeft_Pin DisplayThree_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin|TopLeft_Pin|DisplayThree_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SwtichTwo_Pin SwitchOne_Pin */
  GPIO_InitStruct.Pin = SwtichTwo_Pin|SwitchOne_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 * @bried This will print the temp to console
 * 				This will also put temp on 7 seg display
 */
void printTemp()
{
	// Start using adc
	HAL_ADC_Start(&hadc1);

	// Poll for value
	if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK)
	{
		uint32_t adcValue = HAL_ADC_GetValue(&hadc1);

		// Running at 3.3 volts with 12 bits of resolution (4095)
		// then convert 10mv per celcius
		float temp = ((adcValue * 320) / 4095.0);

		unsigned int tempAsInt = (unsigned int) temp;

		// Makes it a bit less jumpy because of floating point conversion
		if (tempAsInt != ((unsigned int) (temp - 0.2)))
		{
			tempAsInt -= 1;
		}

		// Dont feel like importing float printing so just print as int,
		// makes displaying easier anyway
		printf("The Temp is: %u\r\n", tempAsInt);

		if (tempAsInt < 100)
		{
			upperDisplay = (uint32_t) tempAsInt;
		}
		else
		{
			upperDisplay = 0;
		}
	}
	HAL_ADC_Stop(&hadc1);
}

/*
 * @brief increments the number on the lower display
 * 				This is for interupt handler
 */
void incrementLowerDisplay()
{
	lowerDisplay += 1;
	if (lowerDisplay >= 100)
	{
		lowerDisplay = 0;
	}
}

/*
 * @brief This will setup a switch to toggle a LED if on/off
 * @param GPIOx_Switch which gpio bank the switch is in
 * @param GPIO_Pin_Switch which pin the switch is
 * @param GPIOx_LED which gpio bank the LED is in
 * @param GPIO_Pin_LED which pin led is
 * @param name The name to print of thats pin
 */
void switchToggle(GPIO_TypeDef *GPIOx_Switch,
										 uint16_t GPIO_Pin_Switch,
										 GPIO_TypeDef *GPIOx_LED,
										 uint16_t GPIO_Pin_LED,
										 uint8_t *name)
{
	if (HAL_GPIO_ReadPin(GPIOx_Switch, GPIO_Pin_Switch) && !HAL_GPIO_ReadPin(GPIOx_LED, GPIO_Pin_LED))
	{
		HAL_GPIO_WritePin(GPIOx_LED, GPIO_Pin_LED, GPIO_PIN_SET);
		printf("%s is set to HIGH\r\n", name);
		return;
	}
	if (!HAL_GPIO_ReadPin(GPIOx_Switch, GPIO_Pin_Switch) && HAL_GPIO_ReadPin(GPIOx_LED, GPIO_Pin_LED))
	{
		HAL_GPIO_WritePin(GPIOx_LED, GPIO_Pin_LED, GPIO_PIN_RESET);
		printf("%s is set to Low\r\n", name);
	}
}

/*
 * @brief Toggles the LED State and prints to uart which was set high
 * @param GPIOx Which bank the GPIO pins are in
 * @param GPIO_Pin The specific gpio pin you are toggling
 * @param name The name of the pin for uart
 */
void toggleAndPrintGpioState(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t *name)
{
	if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))
	{
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
#if printLED
		printf("%s is set to low\r\n", name);
#endif
	}
	else
	{
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
#if printLED
		printf("%s is set to high\r\n", name);
#endif
	}
}

/*
 * @brief Sets which single display you want to work with
 * @param display A number from 1-4 to indicate which display you want to use
 */
void setDisplay(uint8_t display)
{
	switch (display)
	{
		case 1:
			HAL_GPIO_WritePin(GPIOE, DisplayOne_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF, DisplayTwo_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, DisplayThree_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOG, DisplayFour_Pin, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOE, DisplayOne_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, DisplayTwo_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, DisplayThree_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOG, DisplayFour_Pin, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOE, DisplayOne_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, DisplayTwo_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, DisplayThree_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG, DisplayFour_Pin, GPIO_PIN_SET);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOE, DisplayOne_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, DisplayTwo_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, DisplayThree_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOG, DisplayFour_Pin, GPIO_PIN_RESET);
			break;
	}
}

/*
 * @brief This will take apart a 4 digit number and display 1 part on the display designated
 * @param number The 4 digit number you are wanting to display
 * @param display The display and index of the number you want to display on
 */
void displayOneNumber(uint16_t number, uint8_t display)
{
	if (number > 10000)
	{
		return;
	}

	// Does some moding to isolate values
	// Does divion to  bring value to ones place
	uint8_t digitOne = number % 10;
	uint8_t digitTwo = ((number % 100) - digitOne) / 10;
	uint8_t digitThree = ((number % 1000) - digitOne - digitTwo) / 100;
	uint8_t digitFour = (number - digitOne - digitTwo - digitThree) / 1000;

	// Only display One Number
	switch (display)
	{
		case 1:
			setDisplay(1);
				setNumber(digitOne);
			break;
		case 2:
			setDisplay(2);
			setNumber(digitTwo);
			break;
		case 3:
			setDisplay(3);
			setNumber(digitThree);
			break;
		case 4:
			setDisplay(4);
			setNumber(digitFour);
			break;
	}
}

/*
 * @brief Sets/resets decimal pin
 * @param flag Unset decimal if 0, set otherwise
 */
void setDecimal(uint8_t flag)
{
	if (flag == 0)
	{
		HAL_GPIO_WritePin(GPIOF, Decimal_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOF, Decimal_Pin, GPIO_PIN_SET);
	}
}

/*
 * @brief Sets up a number for 7 segment display
 * @param A number between 0-9 to set the 7 segment display to
 */
void setNumber(uint8_t number)
{
	switch(number)
	{
		case 0:
			HAL_GPIO_WritePin(GPIOG, BottomLeft_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, BottomMid_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, BottomRight_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOD, TopLeft_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOG, TopMid_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, TopRight_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOE, Middle_Pin, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOG, BottomLeft_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF, BottomMid_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF, BottomRight_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOD, TopLeft_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG, TopMid_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF, TopRight_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOE, Middle_Pin, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOG, BottomLeft_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, BottomMid_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, BottomRight_Pin, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(GPIOD, TopLeft_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG, TopMid_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, TopRight_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOE, Middle_Pin, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOG, BottomLeft_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF, BottomMid_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, BottomRight_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOD, TopLeft_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG, TopMid_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, TopRight_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOE, Middle_Pin, GPIO_PIN_SET);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOG, BottomLeft_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF, BottomMid_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF, BottomRight_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOD, TopLeft_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOG, TopMid_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF, TopRight_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOE, Middle_Pin, GPIO_PIN_SET);
			break;
		case 5:
			HAL_GPIO_WritePin(GPIOG, BottomLeft_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF, BottomMid_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, BottomRight_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOD, TopLeft_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOG, TopMid_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, TopRight_Pin, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(GPIOE, Middle_Pin, GPIO_PIN_SET);
			break;
		case 6:
			HAL_GPIO_WritePin(GPIOG, BottomLeft_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, BottomMid_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, BottomRight_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOD, TopLeft_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOG, TopMid_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, TopRight_Pin, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(GPIOE, Middle_Pin, GPIO_PIN_SET);
			break;
		case 7:
			HAL_GPIO_WritePin(GPIOG, BottomLeft_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF, BottomMid_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF, BottomRight_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOD, TopLeft_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG, TopMid_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, TopRight_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOE, Middle_Pin, GPIO_PIN_RESET);
			break;
		case 8:
			HAL_GPIO_WritePin(GPIOG, BottomLeft_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, BottomMid_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, BottomRight_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOD, TopLeft_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOG, TopMid_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, TopRight_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOE, Middle_Pin, GPIO_PIN_SET);
			break;
		case 9:
			HAL_GPIO_WritePin(GPIOG, BottomLeft_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF, BottomMid_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF, BottomRight_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOD, TopLeft_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOG, TopMid_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF, TopRight_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOE, Middle_Pin, GPIO_PIN_SET);
			break;
		default:
			break;
	}
}

/*
 * @brief This function will wait a set period of microseconds and return
 *        This will hang
 * @param us Amount of microseconds to wait
 */
void delayUs(uint32_t us)
{
	// get current count and count up to US, tim3 is ticking at 1 microsecond
	uint32_t start = __HAL_TIM_GET_COUNTER(&htim3);
  while ((__HAL_TIM_GET_COUNTER(&htim3) - start) < us);
}

/*
 * @brief This function for printf to print to UART2
 * @param ch The char to be printed
 * @retval int char printed
 */
int __io_putchar(int ch){
	// Transmit on UART3 which is though the USB
	// Transmits 1 char and will time out after 100 milliseconds
	HAL_UART_Transmit(&huart3, (uint8_t *) &ch, 1, 100);
	return ch;
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
