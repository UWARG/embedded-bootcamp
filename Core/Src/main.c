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

#define ADC_VAL_TO_COUNTS 46.875
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Is extern from spi.c/h
extern SPI_HandleTypeDef hspi1;
// Is extern from tim.c/h
extern TIM_HandleTypeDef htim2;
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t spi_transmit_data_buf, spi_receive_data_buf = 0;
  // Need 10 bits to represent a reading
  uint16_t spi_reading = 0;
  const uint32_t adc_read_timeout = 5; // In ms
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // HAL_TIM_PWM_Start

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	  loop_start: HAL_Delay(10);
	  // Note size is in bits


	  // --- Take a reading ---
	  // Set CS to LOW
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
	  // (1) pipe in 0b0000 0001
	  spi_transmit_data_buf = 0x01;
	  HAL_SPI_Transmit(&hspi1, &spi_transmit_data_buf,
			           sizeof(uint8_t), adc_read_timeout);

	  // (2) pipe in/out 0b1000 0000 / 0b0000 00RR
	  spi_transmit_data_buf = 0xF0;
	  HAL_SPI_TransmitReceive(&hspi1, &spi_transmit_data_buf,
			  	  	  	  	  	  	  &spi_receive_data_buf,
									  sizeof(uint16_t), adc_read_timeout);

	  // Check we read in data properly
	  if (spi_receive_data_buf > 3) {
		  // We read data incorrectly, try to reset
		  goto loop_start;
	  }

	  spi_reading ^= spi_receive_data_buf;
	  spi_reading <<= 8; // Shift in in bits 9 and 8
	  // (3) Grab remaining 8 bits from SPI
	  spi_receive_data_buf = 0;
	  HAL_SPI_Receive(&hspi1, &spi_receive_data_buf,
			          sizeof(uint8_t), adc_read_timeout);

	  spi_reading ^= spi_receive_data_buf;
	  // Set CS to HIGH, reading is done
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
	  // Reset both buffers
	  spi_transmit_data_buf = 0;
	  spi_transmit_data_buf = 0;
	  // --- Reading is Done ---
	  // --- Set Timer Based on ADC Reading ---
	  	  // 0 to 3.3v goes into adc
	  	  // 0 to 1024 gets mapped to that range
	  // Timer counts up to 960000 over 20ms
	  // Max counts is (10%) * (960000) = 96000
	  // Min counts is (5%) * (960000) = 48000

	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,
			  	  	  	  	(uint32_t) ADC_VAL_TO_COUNTS * spi_reading + 48000);
	  // Reset ADC reading
	  spi_reading = 0;

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
