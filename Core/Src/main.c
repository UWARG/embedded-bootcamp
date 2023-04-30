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

// Declare constants
const uint8_t ADC_READ_TIMEOUT = 5; // Corresponds to 5 ms
const float MAX_DUTY_CYCLE = 0.05; // Corresponds to 5%
const float MIN_DUTY_CYCLE = 0.10; // Corresponds to 10%
const uint16_t COUNTER_PERIOD = 65535; // Corresponds to 2^16 - 1

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

  // Control Bit Selection to select Channel 1 (SGL/DIFF = 1 | D2 = 0 | D1 = 0 | D0 = 1)
  uint8_t tx_data = 0x00;
  uint8_t rx_data = 0x00;
  uint16_t adc_value = 0x0000;
  float adc_ratio = 0.0;
  float duty_cycle = 0.0;
  uint16_t compare_value = 0x0000;

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

  // Start the PWM signal
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Delay to ensure the MCU doesn't overload the ADC
	  HAL_Delay(10);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // Set Chip Select (CS) to LOW to enable communication

	  // Transmit the first byte (From Diagram 6-1 the first byte is 0000 0001)
	  tx_data = 0b00000001;
	  HAL_SPI_Transmit(&hspi1, &tx_data, sizeof(uint8_t), ADC_READ_TIMEOUT);
	  tx_data = 0;

	  /*
	   * Transmit the second byte (From Diagram 6-1 the second byte should be 1000 0000)
	   * Explanation: SGL/DIFF = 1 -> Want to select single-ended
	   * D2 = 1 | D1 = 0 | D0 = 0 -> Want to select Channel 0
	  */
	  tx_data = 0b10000000;
	  HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rx_data, sizeof(uint8_t), ADC_READ_TIMEOUT);
	  tx_data = 0;

	  // AND the rx_data with 0000 0011 to only keep bits 0 and 1
	  // LEFT SHIFT	by 8 bits to move bits 0 and 1 to bits 8 and 9 respectively
	  adc_value = (rx_data & 0x03) << 8;
	  rx_data = 0;

	  // Transmit the third byte (From Diagram 6-1 the third byte XXXX XXXX where X represents a don't care)
	  tx_data = 0b00000000;
	  HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rx_data, sizeof(uint8_t), ADC_READ_TIMEOUT);
	  tx_data = 0;

	  // OR the adc_value with rx_data to receive bits 7 to 0
	  adc_value |= rx_data;
	  rx_data = 0;

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); // Set CS to HIGH to disable communication

	  // The ADC ratio is the value received from the ADC divided by the maximum value 10 bits can store 2^10 - 1 = 1023
	  adc_ratio = (float)(adc_value / 1023);
	  duty_cycle = (adc_ratio * (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE)) + MIN_DUTY_CYCLE;
	  compare_value = (uint16_t)(duty_cycle * COUNTER_PERIOD);

	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare_value);

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
