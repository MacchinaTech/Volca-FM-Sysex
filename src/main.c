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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sysex_fm.h"
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
ADC_HandleTypeDef hadc;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t sysexMsg[163];
uint8_t val = 0;
uint8_t button = 0;
uint8_t pot = 0;
uint8_t OP_active = 0b00111111;
uint8_t noteOff[3] = {0x80, 0x40, 0x00};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
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
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_ADC_Start(&hadc);
    	HAL_ADC_PollForConversion(&hadc, 1); 
    	pot = HAL_ADC_GetValue(&hadc)/2.5;
      if (pot>99) 
      {
        pot=99;
      }
      uint8_t algo = HAL_ADC_GetValue(&hadc)/8;
      
    //global.algorithm=TIM2->CNT; //

    sysexMsg[0] = device.status;
    sysexMsg[1] = device.ID;
    sysexMsg[2] = device.globalChannel;
    sysexMsg[3] = device.formatNumber;
    sysexMsg[4] = device.byteCountMSB;
    sysexMsg[5] = device.byteCountLSB;
    sysexMsg[6] = OP6.EG_rate_1;
    sysexMsg[7] = OP6.EG_rate_2;
    sysexMsg[8] = OP6.EG_rate_3;
    sysexMsg[9] = OP6.EG_rate_4;
    sysexMsg[10] = OP6.EG_level_1;
    sysexMsg[11] = OP6.EG_level_2;
    sysexMsg[12] = OP6.EG_level_3;
    sysexMsg[13] = OP6.EG_level_4;
    sysexMsg[14] = OP6.scale_breakPoint;
    sysexMsg[15] = OP6.scale_leftDepth;
    sysexMsg[16] = OP6.scale_rightDepth;
    sysexMsg[17] = OP6.scale_leftCurve;
    sysexMsg[18] = OP6.scale_rightCurve;
    sysexMsg[19] = OP6.scale_rate;
    sysexMsg[20] = OP6.amp_mod;
    sysexMsg[21] = OP6.key_velocity;
    sysexMsg[22] = OP6.output_level;
    sysexMsg[23] = OP6.osc_mode;
    sysexMsg[24] = OP6.freq_coarse;
    sysexMsg[25] = OP6.freq_fine;
    sysexMsg[26] = OP6.osc_detune;
    sysexMsg[27] = OP5.EG_rate_1;
    sysexMsg[28] = OP5.EG_rate_2;
    sysexMsg[29] = OP5.EG_rate_3;
    sysexMsg[30] = OP5.EG_rate_4;
    sysexMsg[31] = OP5.EG_level_1;
    sysexMsg[32] = OP5.EG_level_2;
    sysexMsg[33] = OP5.EG_level_3;
    sysexMsg[34] = OP5.EG_level_4;
    sysexMsg[35] = OP5.scale_breakPoint;
    sysexMsg[36] = OP5.scale_leftDepth;
    sysexMsg[37] = OP5.scale_rightDepth;
    sysexMsg[38] = OP5.scale_leftCurve;
    sysexMsg[39] = OP5.scale_rightCurve;
    sysexMsg[40] = OP5.scale_rate;
    sysexMsg[41] = OP5.amp_mod;
    sysexMsg[42] = OP5.key_velocity;
    sysexMsg[43] = OP5.output_level;
    sysexMsg[44] = OP5.osc_mode;
    sysexMsg[45] = OP5.freq_coarse;
    sysexMsg[46] = OP5.freq_fine;
    sysexMsg[47] = OP5.osc_detune;
    sysexMsg[48] = OP4.EG_rate_1;
    sysexMsg[49] = OP4.EG_rate_2;
    sysexMsg[50] = OP4.EG_rate_3;
    sysexMsg[51] = OP4.EG_rate_4;
    sysexMsg[52] = OP4.EG_level_1;
    sysexMsg[53] = OP4.EG_level_2;
    sysexMsg[54] = OP4.EG_level_3;
    sysexMsg[55] = OP4.EG_level_4;
    sysexMsg[56] = OP4.scale_breakPoint;
    sysexMsg[57] = OP4.scale_leftDepth;
    sysexMsg[58] = OP4.scale_rightDepth;
    sysexMsg[59] = OP4.scale_leftCurve;
    sysexMsg[60] = OP4.scale_rightCurve;
    sysexMsg[61] = OP4.scale_rate;
    sysexMsg[62] = OP4.amp_mod;
    sysexMsg[63] = OP4.key_velocity;
    sysexMsg[64] = OP4.output_level;
    sysexMsg[65] = OP4.osc_mode;
    sysexMsg[66] = OP4.freq_coarse;
    sysexMsg[67] = OP4.freq_fine;
    sysexMsg[68] = OP4.osc_detune;
    sysexMsg[69] = OP3.EG_rate_1;
    sysexMsg[70] = OP3.EG_rate_2;
    sysexMsg[71] = OP3.EG_rate_3;
    sysexMsg[72] = OP3.EG_rate_4;
    sysexMsg[73] = OP3.EG_level_1;
    sysexMsg[74] = OP3.EG_level_2;
    sysexMsg[75] = OP3.EG_level_3;
    sysexMsg[76] = OP3.EG_level_4;
    sysexMsg[77] = OP3.scale_breakPoint;
    sysexMsg[78] = OP3.scale_leftDepth;
    sysexMsg[79] = OP3.scale_rightDepth;
    sysexMsg[80] = OP3.scale_leftCurve;
    sysexMsg[81] = OP3.scale_rightCurve;
    sysexMsg[82] = OP3.scale_rate;
    sysexMsg[83] = OP3.amp_mod;
    sysexMsg[84] = OP3.key_velocity;
    sysexMsg[85] = OP3.output_level;
    sysexMsg[86] = OP3.osc_mode;
    sysexMsg[87] = OP3.freq_coarse;
    sysexMsg[88] = OP3.freq_fine;
    sysexMsg[89] = OP3.osc_detune;
    sysexMsg[90] = OP2.EG_rate_1;
    sysexMsg[91] = OP2.EG_rate_2;
    sysexMsg[92] = OP2.EG_rate_3;
    sysexMsg[93] = OP2.EG_rate_4;
    sysexMsg[94] = OP2.EG_level_1;
    sysexMsg[95] = OP2.EG_level_2;
    sysexMsg[96] = OP2.EG_level_3;
    sysexMsg[97] = OP2.EG_level_4;
    sysexMsg[98] = OP2.scale_breakPoint;
    sysexMsg[99] = OP2.scale_leftDepth;
    sysexMsg[100] = OP2.scale_rightDepth;
    sysexMsg[101] = OP2.scale_leftCurve;
    sysexMsg[102] = OP2.scale_rightCurve;
    sysexMsg[103] = OP2.scale_rate;
    sysexMsg[104] = OP2.amp_mod;
    sysexMsg[105] = OP2.key_velocity;
    sysexMsg[106] = OP2.output_level;
    sysexMsg[107] = OP2.osc_mode;
    sysexMsg[108] = OP2.freq_coarse;
    sysexMsg[109] = OP2.freq_fine;
    sysexMsg[110] = OP2.osc_detune;
    sysexMsg[111] = pot;//ßOP1.EG_rate_1;
    sysexMsg[112] = OP1.EG_rate_2;
    sysexMsg[113] = OP1.EG_rate_3;
    sysexMsg[114] = OP1.EG_rate_4;
    sysexMsg[115] = pot;//ßOP1.EG_level_1;
    sysexMsg[116] = OP1.EG_level_2;
    sysexMsg[117] = OP1.EG_level_3;
    sysexMsg[118] = OP1.EG_level_4;
    sysexMsg[119] = OP1.scale_breakPoint;
    sysexMsg[120] = OP1.scale_leftDepth;
    sysexMsg[121] = OP1.scale_rightDepth;
    sysexMsg[122] = OP1.scale_leftCurve;
    sysexMsg[123] = OP1.scale_rightCurve;
    sysexMsg[124] = OP1.scale_rate;
    sysexMsg[125] = OP1.amp_mod;
    sysexMsg[126] = OP1.key_velocity;
    sysexMsg[127] = pot;//OP1.output_level;
    sysexMsg[128] = button%2;//OP1.osc_mode;
    sysexMsg[129] = OP1.freq_coarse;
    sysexMsg[130] = OP1.freq_fine;
    sysexMsg[131] = OP1.osc_detune;
    sysexMsg[132] = global.pitch_EG_rate_1;
    sysexMsg[133] = global.pitch_EG_rate_2;
    sysexMsg[134] = global.pitch_EG_rate_3;
    sysexMsg[135] = global.pitch_EG_rate_4;
    sysexMsg[136] = global.pitch_EG_level_1;
    sysexMsg[137] = global.pitch_EG_level_2;
    sysexMsg[138] = global.pitch_EG_level_3;
    sysexMsg[139] = global.pitch_EG_level_4;
    sysexMsg[140] = algo;//val%33;// global.algorithm;
    sysexMsg[141] = global.feedback;
    sysexMsg[142] = global.osc_sync;
    sysexMsg[143] = global.LFO_speed;
    sysexMsg[144] = global.LFO_delay;
    sysexMsg[145] = global.LFO_pitchMod;
    sysexMsg[146] = global.LFO_ampMod;
    sysexMsg[147] = global.LFO_Sync;
    sysexMsg[148] = button%6;//global.LFO_Wave;
    sysexMsg[149] = global.pitch_mod;
    sysexMsg[150] = global.transpose;
    sysexMsg[151] = global.voice_name_1;
    sysexMsg[152] = global.voice_name_2;
    sysexMsg[153] = global.voice_name_3;
    sysexMsg[154] = global.voice_name_4;
    sysexMsg[155] = global.voice_name_5;
    sysexMsg[156] = global.voice_name_6;
    sysexMsg[157] = global.voice_name_7;
    sysexMsg[158] = global.voice_name_8;
    sysexMsg[159] = global.voice_name_9;
    sysexMsg[160] = global.voice_name_10;
    sysexMsg[161] = OP_active;
    sysexMsg[162] = device.end;
    HAL_UART_Transmit(&huart1, sysexMsg, 163, 100);
    HAL_UART_Transmit(&huart1, noteOff, sizeof(noteOff), 10);
    val++;
    HAL_Delay(2000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_8B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 32;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 32150;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

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
