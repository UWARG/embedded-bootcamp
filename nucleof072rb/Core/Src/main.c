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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MCP3004_CS_PORT         GPIOB
#define MCP3004_CS_PIN          GPIO_PIN_8
#define MCP3004_TRANSFER_BYTES  3U

// SPI command bits for MCP3004 channel 1
#define MCP3004_START_BIT       0x01
#define MCP3004_SINGLE_ENDED    0x80  // 1000 0000: start bit + single-ended mode
#define MCP3004_CHANNEL       0x00  // 0000 0000: channel 0 (D2=0, D1=0, D0=0)
//together, Sgl/diff and ch1 makes 0x90!

// For ADC-to-PWM conversion
#define ADC_MAX_VALUE        1023U       // 10-bit ADC maximum
#define PWM_MIN_COUNTS       3000U       // corresponds to 1ms pulse width
#define PWM_MAX_COUNTS       6000U       // corresponds to 2ms pulse width

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//set the txData and rxData for our SPI's MOSI and MISO pins
//this is because 0x90 = 0b10010000
//where 1 = sgl/diff, 0 = d2, 0 = d1, 1 = d0
uint8_t txData[MCP3004_TRANSFER_BYTES] = {
		MCP3004_START_BIT,
		MCP3004_SINGLE_ENDED,
		MCP3004_CHANNEL
};//send

uint8_t rxData[MCP3004_TRANSFER_BYTES] = {0};//receive
//This denotes how many bytes are sent out and how many are received
uint16_t transmit_receive_bytes = 3;

uint16_t Read_ADC_Channel1(void){
	//spiStatus will be checked every time
	HAL_StatusTypeDef spiStatus;

	//pull Chip Select to low, as the slave receives (master sends out) data on low edge
	HAL_GPIO_WritePin(MCP3004_CS_PORT, MCP3004_CS_PIN, GPIO_PIN_RESET);
	//now we use spi1 to transmit/receive data by HAL_SPI_TransmitReceive()
	//set spiStatus to the return value
	spiStatus = HAL_SPI_TransmitReceive(&hspi1, txData, rxData, transmit_receive_bytes, HAL_MAX_DELAY);
	//pull Chip Select back to high, as it is supposed to idle in high
	HAL_GPIO_WritePin(MCP3004_CS_PORT, MCP3004_CS_PIN, GPIO_PIN_SET);

	if(spiStatus != HAL_OK){
		printf("Error: %d\n", spiStatus);
	}

	//reconstruct the output into a 10 bit value of type uint16_t
	return ((rxData[1] & 0x03) << 8) | rxData[2];
}
//use this function to get the pwm read
uint32_t Convert_to_PWM(uint16_t adcin){
	//we begin with minimum 1ms, all the way up to 2ms by maxing out adcin as 1023.
	//notice the formula base value + (adcread * delta)/10-bit-max
	return PWM_MIN_COUNTS + (uint32_t)adcin * (PWM_MAX_COUNTS - PWM_MIN_COUNTS) / ADC_MAX_VALUE;
}
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //this is because we are using timer 1
  //begin the timer
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint16_t adcReturn = Read_ADC_Channel1();
	  uint32_t pwmReturn = Convert_to_PWM(adcReturn);
	  //compare the HIGH time
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwmReturn);

	  //printf("ADC value through CH1: %u\r\n", adcReturn);
	  //printf("PWM value output: %lu\r\n", pwmReturn);
	  HAL_Delay(10);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
