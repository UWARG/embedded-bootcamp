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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_GPIO_PORT GPIOB
#define ADC_GPIO_PIN GPIO_PIN_8
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
uint16_t readADC(uint8_t input_channel);
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
  uint16_t adc_reading = 0;
  uint16_t duty_counts = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // Procedure from docs page 475
  // Low-Level Initialization
  HAL_TIM_PWM_MspInit(&htim1);
  HAL_SPI_MspInit(&hspi1);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();

  // Handles high-level initialization by calling HAL_TIM_PWM_Init
  // as well as config channel
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  // Set the CS line to high so we de-select and don't accidently communicate to it
  HAL_GPIO_WritePin(ADC_GPIO_PORT, ADC_GPIO_PIN, GPIO_PIN_SET);

  // Start the PWM timer
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	// read ADC channel 1
	adc_reading = readADC(0x01);

	// 3200 - 6400 counts for a 5-10% duty cycle with 64000 counts period
	// The following converts a reading from range 0-1023 to range 3200-6400
	duty_counts = (adc_reading / 1023) * 3200 + 3200;

	// Load value into compare to set duty cycle
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_counts);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_Delay(10);
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
/**
 * @brief Function to read the ADC
 * @param inputChannelConfig input channel, only 3 bits are used
 * @note ADC is used in differential mode
 * @retval uint16_t representing the ADC reading
 */
uint16_t readADC(uint8_t input_channel) {
	// Initialize variables
	uint8_t transmit_bytes[3]= {0};
	uint8_t recieve_bytes[3] = {0};

	// Set transmit bytes
	// Start bit at the end of first byte
	transmit_bytes[0] = 0x01;

	// Input channel config, first bit is 0 for differential
	transmit_bytes[1] = (0x00 | (input_channel << 4));

	// Don't cares
	transmit_bytes[2] = 0x00;

	// Communicate through SPI
	// Put CS to low to select ADC
	HAL_GPIO_WritePin(ADC_GPIO_PORT, ADC_GPIO_PIN, GPIO_PIN_RESET);

	// Note: this is a blocking function!
	if(HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&transmit_bytes, (uint8_t *)&recieve_bytes, 3, 100) != HAL_OK) {
		Error_Handler();
	}

	// Put CS to high to de-select ADC
	HAL_GPIO_WritePin(ADC_GPIO_PORT, ADC_GPIO_PIN, GPIO_PIN_SET);

	// Format received bytes
	// Append last two bytes and then mask out the last 10
	uint16_t data = 0b1111111111 & ((recieve_bytes[2] << 8) | recieve_bytes[3]);

	return data;
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
