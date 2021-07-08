 /* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <math.h>

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

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t ADCin = 0;
uint64_t _micro = 0;
uint64_t t = 0;
uint16_t dataOut = 0;
uint8_t DACConfig = 0b0011;
uint8_t menu=1;
uint16_t F = 100 ;
uint16_t Vmax=33;
uint16_t Vmin=0;
uint16_t Dutycle=50;
uint32_t read=0;
char TxDataBuffer[32] =
{ 0 };
char RxDataBuffer[32] =
{ 0 };
char data[32]={0};
float slope=0;
uint8_t waveform=0;
double sin1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput);
uint64_t micros();
//void UARTRecieveAndResponsePolling();
int16_t UARTRecieveIT();
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

char temp[]="*********MENU*********\r\n\n a:SlopeUp Sawtooth Wave     s:SlopeDown Sawtooth Wave\r\n d:SinWave                   f:SquareWave\r\n g:adjustVmax                h:adjustVmin\r\n j:adjustF and Dutycycle\r\n";
char temp2[]="*********MENU AdjustVmax*********\r\n\n a:Voltage Up 0.1 v\r\n s:Voltage Down 0.1 v \r\n b:Back\r\n";
char temp3[]="*********MENU AdjustVmin*********\r\n\n a:Voltage Up 0.1 v\r\n s:Voltage Down 0.1 v \r\n b:Back\r\n";
char temp4[]="Input incorrect\r\n";
char temp5[]="*********MENU AdjustFrequency And Dutycycle*********\r\n a:Frequency Up 0.1 Hz\r\n s:Frequency Down 0.1 Hz \r\n d:Dutycycle up 1% \r\n f:Dutycycle Down 1%\r\n b:Back\r\n";
char temp6[]="Back";
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start_IT(&htim11);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADCin, 1);

	HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		/*Method 2 Interrupt Mode*/
		HAL_UART_Receive_IT(&huart2,  (uint8_t*)RxDataBuffer, 32);

		/*Method 2 W/ 1 Char Received*/
		int16_t inputchar = UARTRecieveIT();
		if(inputchar!=-1)
		{
			sprintf(TxDataBuffer, "ReceivedChar:[%c]\r\n", inputchar);
			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
		}

		switch(menu)
		{
		case 1:
			HAL_UART_Transmit(&huart2, (uint8_t*)temp, strlen(temp),100);
			menu=2;
			break;
		case 2:
			switch(inputchar)
			{
			case -1:
				break;
			case 'a':///sawup
				waveform=0;
				sprintf(data, "Sawtooth Slope Up\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), 100);
				menu=1;
				break;
			case 's':///sawdown
				waveform=1;
				sprintf(data, "Sawtooth Slope Down\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), 100);
				menu=1;
				break;

			case 'd'://sin
				waveform=2;
				sprintf(data, "Sin Wave\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), 100);
				menu=1;
				break;
			case 'f'://pwm
				sprintf(data, "Square Wave\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), 100);
				menu=1;
				waveform=3;
				break;
			case 'g'://vhig
				menu=3;
				break;
			case 'h'://vlow
				menu=5;
				break;
			case 'j'://F
				menu=7;
				break;
			default :
				menu=1;
				HAL_UART_Transmit(&huart2, (uint8_t*)temp4, strlen(temp4),100);
				break;
			}
			break;
		case 3://vhig menu
			HAL_UART_Transmit(&huart2, (uint8_t*)temp2, strlen(temp2),100);
			menu=4;
			break;
		case 4:
			switch(inputchar)
			{
			case 'a'://up
				if(Vmax<41)
				{
				Vmax+=1;
				}
				sprintf(data, "Vmax=%d.%d\r\n", Vmax/10,Vmax%10);
				HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), 100);
				menu=3;
				break;
			case 's'://down
				if(Vmax-1>Vmin)
				{
				Vmax-=1;
				}
				sprintf(data, "Vmax=%d.%d\r\n", Vmax/10,Vmax%10);
				HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), 100);
				menu=3;
				break;
			case 'b'://back
				menu=1;
				HAL_UART_Transmit(&huart2, (uint8_t*)temp6, strlen(temp6),100);
				break;
			case -1:
				break;
			default :
				HAL_UART_Transmit(&huart2, (uint8_t*)temp4, strlen(temp4),100);
				menu=3;
				break;
			}
			break;
		case 5://vlow menu
			menu=6;
			HAL_UART_Transmit(&huart2, (uint8_t*)temp3, strlen(temp3),100);
			break;
		case 6:
			switch(inputchar)
			{
			case 'a'://up
				if(Vmin+1<Vmax)
				{
				Vmin+=1;
				}
				sprintf(data, "Vmin=%d.%d\r\n", Vmin/10,Vmin%10);
				HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), 100);
				menu=5;
				break;
			case 's'://down
				if(Vmin>0)
				{
				Vmin-=1;
				}
				sprintf(data, "Vmin=%d.%d\r\n", Vmin/10,Vmin%10);
				HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), 100);
				menu=5;
				break;
			case 'b'://back
				menu=1;
				HAL_UART_Transmit(&huart2, (uint8_t*)temp6, strlen(temp6),100);
				break;
			case -1:
				break;
			default :
				HAL_UART_Transmit(&huart2, (uint8_t*)temp4, strlen(temp4),100);
				menu=5;
				break;
			}
			break;
		case 7://f menu
			menu=8;
			HAL_UART_Transmit(&huart2, (uint8_t*)temp5, strlen(temp5),100);
			break;
		case 8:
			switch(inputchar)
			{
			case 'a'://up
				F+=1;
				sprintf(data, "Frequency=%d.%d\r\n", F/10,F%10);
				HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), 100);
				menu=7;
				break;
			case 's'://down
				if(F>0)
				{
				F-=1;
				}
				sprintf(data, "Frequency=%d.%d\r\n", F/10,F%10);
				HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), 100);
				menu=7;
				break;
			case 'd':
				if (Dutycle<100)
				{
				Dutycle +=1;
				}
				sprintf(data, "Dutycle=%d percent\r\n", Dutycle);
				HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), 100);
				menu=7;
				break;
			case 'f':
				if (Dutycle>0)
				{
				Dutycle -=1;
				}
				sprintf(data, "Dutycle=%d percent\r\n", Dutycle);
				HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), 100);
				menu=7;
				break;
			case 'b'://back
				menu=1;
				HAL_UART_Transmit(&huart2, (uint8_t*)temp6, strlen(temp6),100);
				break;
			case -1:
				break;
			default :
				HAL_UART_Transmit(&huart2, (uint8_t*)temp4, strlen(temp4),100);
				menu=7;
				break;
			}
			break;
		}


		static uint64_t timestamp = 0;
		if (micros() - timestamp > 100)
		{	timestamp = micros();
			if(F>0)
			{
			t=timestamp%(10000000/F);
			if (waveform==0)
			{
				slope=(Vmax-Vmin)/(float)(10000000/F);
				dataOut=((slope*t)+Vmin)*4095/(41);
			}
			else if (waveform==1)
			{
				slope=(Vmax-Vmin)/(float)(10000000/F);
				dataOut=(-slope*t+Vmax)*4095/41;
			}
			else if (waveform==2)
			{
				sin1 = 2*3.141592*t/(float)(10000000/F);
				dataOut=(((Vmax-Vmin)/2)*(sin(sin1))+(Vmax+Vmin)/2)*4095/41;
			}
			else if (waveform==3)
			{
				if(t>=Dutycle*(10000000/F)/100)
				{
					dataOut=Vmin*4095/41;
				}
				else
				{
					dataOut=Vmax*4095/41;
				}
			}
			}
//			dataOut++;
//			dataOut %= 4096;
			if (hspi3.State == HAL_SPI_STATE_READY
					&& HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin)
							== GPIO_PIN_SET)
			{
				MCP4922SetOutput(DACConfig, dataOut);
			}
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 100;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SHDN_GPIO_Port, SHDN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LOAD_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LOAD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_SS_Pin */
  GPIO_InitStruct.Pin = SPI_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SHDN_Pin */
  GPIO_InitStruct.Pin = SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SHDN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int16_t UARTRecieveIT()
{
	static uint32_t dataPos =0;
	int16_t data=-1;
	if(huart2.RxXferSize - huart2.RxXferCount!=dataPos)
	{
		data=RxDataBuffer[dataPos];
		dataPos= (dataPos+1)%huart2.RxXferSize;
	}
	return data;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	sprintf(TxDataBuffer, "Received:[%s]\r\n", RxDataBuffer);
//	HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
}

void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput)
{
	uint32_t OutputPacket = (DACOutput & 0x0fff) | ((Config & 0xf) << 12);
	HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi3, &OutputPacket, 1);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi3)
	{
		HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim11)
	{
		_micro += 65535;
	}
}

inline uint64_t micros()
{
	return htim11.Instance->CNT + _micro;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
