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
uint8_t read_bit(uint8_t data, int bit);
uint8_t set_bit(uint8_t data, int bit);
uint8_t unset_bit(uint8_t data, int bit);
void write_bit(uint8_t *data, int bit, bool value);
bool read_bit_from(uint8_t *data, int bit);

void write_chip_select(GPIO_PinState state);
void set_duty_cycle(int duty_cycle);

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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  uint8_t receiver_buffer[3] = {0, 0, 0};
  // the 7 MSB of the first byte are zeroed so that the final bit can act as the start bit and the received data is nicely byte aligned
  // the second byte is structured as SGL/DIFF D2 D1 D0 X X X X, with the 0s indicating single-ended mode and selecting channel 0
  uint8_t transceiver_buffer[3] = {0b00000001, 0b00000000, 0};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		// set the chip select pin, receive and transmit data, unset the chip select pin
		write_chip_select(GPIO_PIN_SET);
		if (HAL_SPI_TransmitReceive(&hspi1, transceiver_buffer, receiver_buffer, 3, 10) != HAL_OK) {
			Error_Handler();
		}
		write_chip_select(GPIO_PIN_RESET);

		// ensure that the 7th bit is null, as per the data sheet
		if (read_bit_from(receiver_buffer, 6) != 0) {
			Error_Handler();
		}

		// convert the remaining input bits into an integer, MSB first
		int adc_output = 0;
		for (int i = 0; i < 10; i++) {
			int bit_index = i + 7;
			adc_output = (adc_output << 1) + read_bit_from(receiver_buffer, bit_index);
		}

		// largest unsigned value representable with 10 bits = 2**10 - 1
		double max_adc = 1023.0;

		// linearly map a value in the range [0, max_adc] to the range [5, 10]
		int duty_cycle = (int) (adc_output / max_adc * 5 + 5);

		set_duty_cycle(duty_cycle);

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
uint8_t read_bit(uint8_t data, int bit) {
	return (data >> bit) & 1;
}

uint8_t set_bit(uint8_t data, int bit) {
	return data | (1 << bit);
}

uint8_t unset_bit(uint8_t data, int bit) {
	return data & ~(1 << bit);
}

void write_bit(uint8_t *data, int bit, bool value) {
	int bit_index = bit % 8;
	int byte_index = bit / 8;
	uint8_t *byte = data + byte_index;
	if (value) {
		*byte = set_bit(*byte, bit_index);
	} else {
		*byte = unset_bit(*byte, bit_index);
	}
}

bool read_bit_from(uint8_t *data, int bit) {
	int bit_index = bit % 8;
	int byte_index = bit / 8;
	uint8_t *byte = data + byte_index;
	return read_bit(*byte, bit_index);
}


void write_chip_select(GPIO_PinState state) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, state);
}

// 0 <= duty_cycle <= 100
void set_duty_cycle(int duty_cycle) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);
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
