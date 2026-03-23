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
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ADC
#define ADC_START_BIT   0x01
#define ADC_SIGNAL_BIT     0x01
#define ADC_DIFF_BIT    0x00

#define ADC_CH0   0b000
#define ADC_CH1   0b001
#define ADC_CH2   0b010
#define ADC_CH3   0b011

#define MAX_ADC   1023
#define MIN_PWM   3200
#define MAX_PWM   6400
#define TIMEOUT_MS  100

uint16_t pwm_range = MAX_PWM - MIN_PWM;
/* USER CODE END PD */


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Converts raw ADC value (0–1023) into PWM CCR value (3200–6400)
uint16_t convert_adc_to_pwm(uint16_t adc_value)
{
    if (adc_value > MAX_ADC)
        adc_value = MAX_ADC;

    // Multiply before divide to avoid truncating to 0
    uint16_t ccr_value =
        MIN_PWM +
        (uint16_t)((adc_value * pwm_range) / (float)MAX_ADC);

    return ccr_value;
}

// Extract 10 bit ADC result from MCP3004 SPI response
uint16_t extract_adc_result(uint8_t *rx_buffer)
{
    uint16_t upper_bits = (uint16_t)(rx_buffer[1] & 0x03) << 8;
    uint16_t lower_bits = rx_buffer[2];

    return (upper_bits | lower_bits);
}

/* USER CODE END 0 */


int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */

  // Start PWM once (we will only update duty cycle)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  uint8_t tx_buffer[3] = {0};
  uint8_t rx_buffer[3] = {0};

  // Build MCP3004 command (single ended, channel 0)
  tx_buffer[0] = ADC_START_BIT;
  tx_buffer[1] = (ADC_SIGNAL_BIT << 7) | (ADC_CH0 << 4);
  tx_buffer[2] = 0x00;

  /* USER CODE END 2 */

  while (1)
  {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

      HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
          &hspi1,
          tx_buffer,
          rx_buffer,
          sizeof(tx_buffer),
          TIMEOUT_MS
      );

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

      if (status != HAL_OK)
          continue;


      // Extract ADC value (0–1023)
      uint16_t adc_value = extract_adc_result(rx_buffer);

      // Convert ADC value into PWM CCR
      uint16_t pwm_ccr = convert_adc_to_pwm(adc_value);

      // Update PWM duty cycle
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_ccr);

      HAL_Delay(10);
  }
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
