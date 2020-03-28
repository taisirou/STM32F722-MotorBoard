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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
char msg2[] ="PushButton\r\n";
char rec1[] ="Receive1\r\n";
char rec2[] ="Receive2\r\n";
char rec3[] ="Receive3\r\n";
char rec4[] ="Forward\r\n";
char rec5[] ="Back\r\n";
char rec6[] ="Left\r\n";
char rec7[] ="Right\r\n";
char rec8[] ="Stop\r\n";

char nonans[] ="Not Understand \r\n";

char rev;

int cnt2 = 0; //
int cnt2_new = 0; //
int cnt2_old = 0; //
int cnt3 = 0; //
int cnt3_new = 0; //
int cnt3_old = 0; //
int cnt4 = 0; //
int cnt4_new = 0; //
int cnt4_old = 0; //
int cnt5 = 0; //
int cnt5_new = 0; //
int cnt5_old = 0; //

char scnt2[100]; //
char scnt3[100]; //
char scnt4[100]; //
char scnt5[100]; //

float adcval;
uint8_t adcout[12];
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start( &htim2, TIM_CHANNEL_ALL ); // encoder start
  HAL_TIM_Encoder_Start( &htim3, TIM_CHANNEL_ALL ); // encoder start
  HAL_TIM_Encoder_Start( &htim4, TIM_CHANNEL_ALL ); // encoder start
  HAL_TIM_Encoder_Start( &htim5, TIM_CHANNEL_ALL ); // encoder start

  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      if(HAL_UART_Receive_IT(&huart1, &rev, 1)==HAL_OK){
	            switch(rev)
	                    {
	            case '1':
	            	forward();
		            break;
	            case '2':
	            	 right();
	                 break;
	             case '3':
	            	 left();
	            	 break;
	             case '4':
	            	 back();
	                 break;
	             case '5':
	           	  stop();
	                 break;
	             case 'i':
	           	  forward();
	                 break;
	             case 'j':
	           	  left();
	                 break;
	             case 'l':
	            	 right();
	                 break;
	             case ',':
	           	  back();
	                 break;
	             case 'k':
	            	 stop();
	            	 break;
	             case '0':
	            	 test();
	            	 break;
	       /*      default:
	            	 stop();
	                 break;
	         */
	                    }
      }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Prescaler = 48000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 500;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_MultiProcessor_Init(&huart1, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR1_B_Pin|MOTOR1_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR1_P_Pin|MOTOR2_P_Pin|MOTOR2_A_Pin|MOTOR2_B_Pin 
                          |LED1_Pin|LED2_Pin|MOTOR3_A_Pin|MOTOR3_B_Pin 
                          |MOTOR4_P_Pin|MOTOR3_P_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR4_A_GPIO_Port, MOTOR4_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR4_B_GPIO_Port, MOTOR4_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SW2_Pin */
  GPIO_InitStruct.Pin = SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR1_B_Pin MOTOR1_A_Pin */
  GPIO_InitStruct.Pin = MOTOR1_B_Pin|MOTOR1_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR1_P_Pin MOTOR2_P_Pin MOTOR2_A_Pin MOTOR2_B_Pin 
                           LED1_Pin LED2_Pin MOTOR3_A_Pin MOTOR3_B_Pin 
                           MOTOR4_P_Pin MOTOR3_P_Pin */
  GPIO_InitStruct.Pin = MOTOR1_P_Pin|MOTOR2_P_Pin|MOTOR2_A_Pin|MOTOR2_B_Pin 
                          |LED1_Pin|LED2_Pin|MOTOR3_A_Pin|MOTOR3_B_Pin 
                          |MOTOR4_P_Pin|MOTOR3_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR4_A_Pin */
  GPIO_InitStruct.Pin = MOTOR4_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR4_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR4_B_Pin */
  GPIO_InitStruct.Pin = MOTOR4_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR4_B_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == SW1_Pin){
    	test();
 	   HAL_UART_Transmit( &huart1, rec8, strlen(rec8) + 1, 0xFFFF);
    }

    if(GPIO_Pin == SW2_Pin){
    	stop();
    	HAL_UART_Transmit( &huart1, rec8, strlen(rec8) + 1, 0xFFFF);}
}
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if((htim->Instance == TIM1) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)) {
  }
}
void forward()
{
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,1);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,0);
    HAL_GPIO_WritePin(MOTOR1_P_GPIO_Port,MOTOR1_P_Pin, 1);
    HAL_GPIO_WritePin(MOTOR1_A_GPIO_Port,MOTOR1_A_Pin, 1);
    HAL_GPIO_WritePin(MOTOR1_B_GPIO_Port,MOTOR1_B_Pin, 0);
    HAL_GPIO_WritePin(MOTOR2_P_GPIO_Port,MOTOR2_P_Pin, 1);
    HAL_GPIO_WritePin(MOTOR2_A_GPIO_Port,MOTOR2_A_Pin, 0);
    HAL_GPIO_WritePin(MOTOR2_B_GPIO_Port,MOTOR2_B_Pin, 1);
    HAL_GPIO_WritePin(MOTOR3_P_GPIO_Port,MOTOR3_P_Pin, 1);
    HAL_GPIO_WritePin(MOTOR3_A_GPIO_Port,MOTOR3_A_Pin, 1);
    HAL_GPIO_WritePin(MOTOR3_B_GPIO_Port,MOTOR3_B_Pin, 0);
    HAL_GPIO_WritePin(MOTOR4_P_GPIO_Port,MOTOR4_P_Pin, 1);
    HAL_GPIO_WritePin(MOTOR4_A_GPIO_Port,MOTOR4_A_Pin, 0);
    HAL_GPIO_WritePin(MOTOR4_B_GPIO_Port,MOTOR4_B_Pin, 1);
    HAL_UART_Transmit(&huart1,(uint8_t *)rec4,sizeof(rec4),3000);
}
void back()
{
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,0);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,1);
    HAL_GPIO_WritePin(MOTOR1_P_GPIO_Port,MOTOR1_P_Pin, 1);
    HAL_GPIO_WritePin(MOTOR1_A_GPIO_Port,MOTOR1_A_Pin, 0);
    HAL_GPIO_WritePin(MOTOR1_B_GPIO_Port,MOTOR1_B_Pin, 1);
    HAL_GPIO_WritePin(MOTOR2_P_GPIO_Port,MOTOR2_P_Pin, 1);
    HAL_GPIO_WritePin(MOTOR2_A_GPIO_Port,MOTOR2_A_Pin, 1);
    HAL_GPIO_WritePin(MOTOR2_B_GPIO_Port,MOTOR2_B_Pin, 0);
    HAL_GPIO_WritePin(MOTOR3_P_GPIO_Port,MOTOR3_P_Pin, 1);
    HAL_GPIO_WritePin(MOTOR3_A_GPIO_Port,MOTOR3_A_Pin, 0);
    HAL_GPIO_WritePin(MOTOR3_B_GPIO_Port,MOTOR3_B_Pin, 1);
    HAL_GPIO_WritePin(MOTOR4_P_GPIO_Port,MOTOR4_P_Pin, 1);
    HAL_GPIO_WritePin(MOTOR4_A_GPIO_Port,MOTOR4_A_Pin, 1);
    HAL_GPIO_WritePin(MOTOR4_B_GPIO_Port,MOTOR4_B_Pin, 0);
    HAL_UART_Transmit(&huart1,(uint8_t *)rec5,sizeof(rec5),3000);
}
void left()
{
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,0);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,1);
    HAL_GPIO_WritePin(MOTOR1_P_GPIO_Port,MOTOR1_P_Pin, 1);
    HAL_GPIO_WritePin(MOTOR1_A_GPIO_Port,MOTOR1_A_Pin, 0);
    HAL_GPIO_WritePin(MOTOR1_B_GPIO_Port,MOTOR1_B_Pin, 0);
    HAL_GPIO_WritePin(MOTOR2_P_GPIO_Port,MOTOR2_P_Pin, 1);
    HAL_GPIO_WritePin(MOTOR2_A_GPIO_Port,MOTOR2_A_Pin, 1);
    HAL_GPIO_WritePin(MOTOR2_B_GPIO_Port,MOTOR2_B_Pin, 0);
    HAL_GPIO_WritePin(MOTOR3_P_GPIO_Port,MOTOR3_P_Pin, 0);
    HAL_GPIO_WritePin(MOTOR3_A_GPIO_Port,MOTOR3_A_Pin, 0);
    HAL_GPIO_WritePin(MOTOR3_B_GPIO_Port,MOTOR3_B_Pin, 0);
    HAL_GPIO_WritePin(MOTOR4_P_GPIO_Port,MOTOR4_P_Pin, 0);
    HAL_GPIO_WritePin(MOTOR4_A_GPIO_Port,MOTOR4_A_Pin, 0);
    HAL_GPIO_WritePin(MOTOR4_B_GPIO_Port,MOTOR4_B_Pin, 0);
    HAL_UART_Transmit(&huart1,(uint8_t *)rec6,sizeof(rec6),3000);
}
void right()
{
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,0);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,1);
    HAL_GPIO_WritePin(MOTOR1_P_GPIO_Port,MOTOR1_P_Pin, 0);
    HAL_GPIO_WritePin(MOTOR1_A_GPIO_Port,MOTOR1_A_Pin, 0);
    HAL_GPIO_WritePin(MOTOR1_B_GPIO_Port,MOTOR1_B_Pin, 0);
    HAL_GPIO_WritePin(MOTOR2_P_GPIO_Port,MOTOR2_P_Pin, 1);
    HAL_GPIO_WritePin(MOTOR2_A_GPIO_Port,MOTOR2_A_Pin, 1);
    HAL_GPIO_WritePin(MOTOR2_B_GPIO_Port,MOTOR2_B_Pin, 0);
    HAL_GPIO_WritePin(MOTOR3_P_GPIO_Port,MOTOR3_P_Pin, 0);
    HAL_GPIO_WritePin(MOTOR3_A_GPIO_Port,MOTOR3_A_Pin, 0);
    HAL_GPIO_WritePin(MOTOR3_B_GPIO_Port,MOTOR3_B_Pin, 0);
    HAL_GPIO_WritePin(MOTOR4_P_GPIO_Port,MOTOR4_P_Pin, 0);
    HAL_GPIO_WritePin(MOTOR4_A_GPIO_Port,MOTOR4_A_Pin, 0);
    HAL_GPIO_WritePin(MOTOR4_B_GPIO_Port,MOTOR4_B_Pin, 0);
    HAL_UART_Transmit(&huart1,(uint8_t *)rec7,sizeof(rec7),3000);
}
void stop()
{
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,0);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,0);
    HAL_GPIO_WritePin(MOTOR1_P_GPIO_Port,MOTOR1_P_Pin, 0);
    HAL_GPIO_WritePin(MOTOR1_A_GPIO_Port,MOTOR1_A_Pin, 0);
    HAL_GPIO_WritePin(MOTOR1_B_GPIO_Port,MOTOR1_B_Pin, 0);
    HAL_GPIO_WritePin(MOTOR2_P_GPIO_Port,MOTOR2_P_Pin, 0);
    HAL_GPIO_WritePin(MOTOR2_A_GPIO_Port,MOTOR2_A_Pin, 0);
    HAL_GPIO_WritePin(MOTOR2_B_GPIO_Port,MOTOR2_B_Pin, 0);
    HAL_GPIO_WritePin(MOTOR3_P_GPIO_Port,MOTOR3_P_Pin, 0);
    HAL_GPIO_WritePin(MOTOR3_A_GPIO_Port,MOTOR3_A_Pin, 0);
    HAL_GPIO_WritePin(MOTOR3_B_GPIO_Port,MOTOR3_B_Pin, 0);
    HAL_GPIO_WritePin(MOTOR4_P_GPIO_Port,MOTOR4_P_Pin, 0);
    HAL_GPIO_WritePin(MOTOR4_A_GPIO_Port,MOTOR4_A_Pin, 0);
    HAL_GPIO_WritePin(MOTOR4_B_GPIO_Port,MOTOR4_B_Pin, 0);
    HAL_UART_Transmit(&huart1,(uint8_t *)rec8,sizeof(rec8),3000);
}
void test()
{
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,1);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,1);
    HAL_GPIO_WritePin(MOTOR1_P_GPIO_Port,MOTOR1_P_Pin, 1);
    HAL_GPIO_WritePin(MOTOR1_A_GPIO_Port,MOTOR1_A_Pin, 1);
    HAL_GPIO_WritePin(MOTOR1_B_GPIO_Port,MOTOR1_B_Pin, 0);
    HAL_GPIO_WritePin(MOTOR2_P_GPIO_Port,MOTOR2_P_Pin, 1);
    HAL_GPIO_WritePin(MOTOR2_A_GPIO_Port,MOTOR2_A_Pin, 0);
    HAL_GPIO_WritePin(MOTOR2_B_GPIO_Port,MOTOR2_B_Pin, 1);
    HAL_GPIO_WritePin(MOTOR3_P_GPIO_Port,MOTOR3_P_Pin, 1);
    HAL_GPIO_WritePin(MOTOR3_A_GPIO_Port,MOTOR3_A_Pin, 0);
    HAL_GPIO_WritePin(MOTOR3_B_GPIO_Port,MOTOR3_B_Pin, 1);
    HAL_GPIO_WritePin(MOTOR4_P_GPIO_Port,MOTOR4_P_Pin, 1);
    HAL_GPIO_WritePin(MOTOR4_A_GPIO_Port,MOTOR4_A_Pin, 1);
    HAL_GPIO_WritePin(MOTOR4_B_GPIO_Port,MOTOR4_B_Pin, 0);
    HAL_UART_Transmit(&huart1,(uint8_t *)rec4,sizeof(rec4),3000);
}
void  HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	   cnt2_old = cnt2_new;
	   cnt2_new = TIM2 -> CNT; //
	   cnt2 = cnt2_new - cnt2_old;
	   sprintf(scnt2, "TIM2 %d\r\n", cnt2);
	   HAL_UART_Transmit( &huart1, scnt2, strlen(scnt2) + 1, 0xFFFF);
	   cnt3_old = cnt3_new;
	   cnt3_new = TIM3 -> CNT; //
	   cnt3 = cnt3_new - cnt3_old;
	   sprintf(scnt3, "TIM3 %d\r\n", cnt3);
	   HAL_UART_Transmit( &huart1, scnt3, strlen(scnt3) + 1, 0xFFFF);
	   cnt4_old = cnt4_new;
	   cnt4_new = TIM4 -> CNT; //
	   cnt4 = cnt4_new - cnt4_old;
	   sprintf(scnt4, "TIM4 %d\r\n", cnt4);
	   HAL_UART_Transmit( &huart1, scnt4, strlen(scnt4) + 1, 0xFFFF);
	   cnt5_old = cnt5_new;
	   cnt5_new = TIM5 -> CNT; //
	   cnt5 = cnt5_new - cnt5_old;
	   sprintf(scnt5, "TIM5 %d\r\n", cnt5);
	   HAL_UART_Transmit( &huart1, scnt5, strlen(scnt5) + 1, 0xFFFF);

	   HAL_ADC_Start(&hadc1);
	   HAL_ADC_PollForConversion(&hadc1, 100);
	   HAL_ADC_Stop(&hadc1);
	   adcval= HAL_ADC_GetValue(&hadc1)/70;
	   sprintf((char*)adcout,"ADCVAL %u \n\r",(unsigned int)adcval);
	   HAL_UART_Transmit( &huart1, adcout, strlen(adcout) + 1, 0xFFFF);
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
