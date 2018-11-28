
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include <string.h>
#include "stdbool.h"
#include "stdlib.h"
#include <stdio.h>
#include <math.h>
//#include "MQ2Config.h"

/* USER CODE BEGIN Includes */
#define WELCOME_CLASS "\r\n==========================================\r\nWelcome Electronics&Telecom. Project Class\r\n==========================================\r\n"
#define WELCOME_MSG "Nucleo-Board Management Console\r\n"
#define MAIN_MENU   "Select the option you are interested in:\r\n 1. Toggle LD2 LED\r\n 2. Read USER BUTTON status\r\n 3. Clear screen and print this message\r\n"
#define PROMPT "\r\n> "
#define BIEN "Tu valor rawValue es:\r\n"

#define RL_Value 10
#define RO_CLEAN_AIR_FACTOR 9.83

#define LPG 0
#define SMOKE 1
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
char readBuf[1];
char mandar[]= WELCOME_MSG;
char mandar2[]= BIEN;
__IO ITStatus UartReady = SET;
volatile bool status = 0;
int8_t opt;
UART_HandleTypeDef huart1;
char valor[50];
char var[40];
float LPGCurve[3]={2.3, 0.2, -0.45};

float SmokeCurve[3] ={2.3,0.53,-0.43};

float Ro =10;


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_USART1_UART_Init(void);
float  GetPercentage(float rs_ro_ratio, float *pcurve);
float GetGasPercentage(float rs_ro_ratio, int gas_id);
float ReadSensor(float valor);
float ResistanceCalculation(float raw_adc);
float SensorCalibration(float adcv);
float ReadMQ (float crudo,int cual, float di);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void performCriticalTasks(void);
void printWelcomeMessage(void);
volatile uint32_t rawValue;
volatile float volt;
volatile float calibrado;
volatile float calibra2;
//char valor[20];
//uint8_t processUserInput(int8_t opt);
//int8_t readUserInput(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  int contad=1;
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  float intermedio;
  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_SetPriority(USART1_IRQn,0,0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_ADC_Start_IT(&hadc);
 // printWelcomeMessage();
  //calibrado = ReadMQ((float)rawValue,1,true,0);
  /* USER CODE END 2 */
 // calibrado= 10.67;
  HAL_Delay(3000);
  HAL_ADC_Start_IT(&hadc);
  intermedio= 4096-(float)rawValue;
  calibrado = (10*(4096-rawValue)/rawValue)/9.83;
  gcvt(calibrado,6,valor);
  	  sprintf(var, "Valor de resistencia: %s\r\n",valor);
  	  HAL_UART_Transmit_IT(&huart1, (uint8_t*)var, strlen(var));
  HAL_Delay(3000);
  HAL_ADC_Start_IT(&hadc);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
	  HAL_Delay(1000);
	  HAL_ADC_Start_IT(&hadc);
	  if(contad==1)
	  {
	  volt = ReadMQ((float)rawValue,1,calibrado);
	  gcvt((float)volt,20,valor);
	  sprintf(var, "Gas flamable: %s ppm\r\n",&valor);
	  contad=0;
	  }
	  else
	  {
		  volt = ReadMQ((float)rawValue,2,calibrado);
		  gcvt((float)volt,20,valor);
		  sprintf(var, "Humo: %s ppm\r\n",&valor);
		  contad=1;
	  }
	  HAL_UART_Transmit_IT(&huart1, (uint8_t*)var, strlen(var));
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void performCriticalTasks(void){
	HAL_Delay(2000);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
//	char msg[20];
  //

  rawValue = (HAL_ADC_GetValue(hadc));

  //itoa((int)rawValue,valor,10);
 // valor= "Tu Raw Value es  \r\n";
  //printWelcomeMessage();
  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)WELCOME_MSG, strlen(mandar));
  //sprintf(msg, "rawValue: %hu\r\n",(uint8_t*)rawValue);
  //snprintf(valor,20,"%hu\n",(unsigned*)rawValue);
  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)mandar2,strlen(mandar2));

  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
	/* Set transmission flag: transfer complete*/
	UartReady = SET;
}



void printWelcomeMessage(void){
	char *strings[] = {PROMPT, WELCOME_MSG, MAIN_MENU, PROMPT};

	for (uint8_t i = 0; i < 4; i++)
	{
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)strings[i], strlen(strings[i]));

		while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX || HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX_RX);
	}
}
/*
int8_t readUserInput(void){
	int8_t retVal = -1;

	if(UartReady == SET)
	{
		UartReady = RESET;
		HAL_UART_Receive_IT(&huart1, (uint8_t*)readBuf, 1);
		retVal = atoi(readBuf);
	}
	return retVal;
}
*/


float ReadMQ (float crudo,int cual, float di)
{
		switch(cual)
		{
		case 1:
			return (GetGasPercentage(ReadSensor(crudo)/di,LPG));
		case 2:
			return (GetGasPercentage(ReadSensor(crudo)/di,SMOKE));
		default:
			return crudo;

		}
}


float SensorCalibration(float adcv)
{                                   // This function assumes that sensor is in clean air.
  //float val=0;

  return (float)((10*(4096-adcv)/adcv)/9.83);                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro
                                                        //according to the chart in the datasheet
}

float ReadSensor(float raw)
{                                 // take multiple readings and average it.
    return ResistanceCalculation((raw));   // rs changes according to gas concentration.
}

float ResistanceCalculation(float raw_adc)
{
	return (10*(4096-raw_adc)/raw_adc);
}

float GetGasPercentage(float rs_ro_ratio, int gas_id)
{
	if (gas_id == LPG)
	{
		return GetPercentage(rs_ro_ratio,LPGCurve);
	}
	else if (gas_id == SMOKE)
	{
		return GetPercentage(rs_ro_ratio,SmokeCurve);
	}
	return 0;
}

float GetPercentage(float rs_ro_ratio, float *curve)
{
	//return rs_ro_ratio;
	return ( pow(10,(((log10(rs_ro_ratio)+curve[1])/curve[2])+ curve[0])));
}



/*
uint8_t processUserInput(int8_t opt){
	  char msg[30];

	  if(!(opt >=1 && opt <= 3))
	    return 0;

	  sprintf(msg, "%d", opt);
	  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	  switch(opt) {
		  case 1:
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			break;
		  case 2:
			sprintf(msg, "\r\nUSER BUTTON status: %s",
				HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET ? "PRESSED" : "RELEASED");
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			break;
		  case 3:
			return 2;
	  };

	  HAL_UART_Transmit(&huart1, (uint8_t*)PROMPT, strlen(PROMPT), HAL_MAX_DELAY);
	  return 1;
}
*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
