/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

const double WARG_MAX_DUTY_CYCLE = 0.1;
const double WARG_MIN_DUTY_CYCLE = 0.05;
const uint32_t WARG_ADC_MAX_VALUE = 1023;
const uint32_t WARG_TIM1_COUNTER_PERIOD = 64000;
const uint32_t WARG_TIM1_PRESCALER = 14;
const uint32_t WARG_TIM1_PWM_CHANNEL = 1;
const uint32_t WARG_ADC_SELECT = 0x80;
const uint32_t WARG_GPIO_PIN = 8;

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

  // Initialize SPI1
  SPI_HandleTypeDef hspi1;
  HAL_SPI_Init(&hspi1);
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

  //Initialize TIM1
  TIM_HandleTypeDef htim1;
  HAL_TIM_PWM_Init(&htim1);
  htim1.Init.Prescaler = WARG_TIM1_PRESCALER;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = WARG_TIM1_COUNTER_PERIOD;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // No division?
  htim1.Init.RepetitionCounter = 0;
  TIM_OC_InitTypeDef octim1;
  octim1.OCMode = TIM_OCMODE_PWM1;
  octim1.Pulse = 0;
  octim1.OCPolarity = TIM_OCPOLARITY_HIGH;
  octim1.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  octim1.OCFastMode = TIM_OCFAST_DISABLE;
  octim1.OCIdleState = TIM_OCIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &octim1, WARG_TIM1_PWM_CHANNEL);

  const uint16_t size = 3;
  uint8_t pTxData[3] = {1, WARG_ADC_SELECT, 0}; // select ch0, single for ADC
  uint8_t pRxData[3];
  uint32_t onCount;
  uint32_t adcData;


  HAL_TIM_PWM_Start(&htim1, WARG_TIM1_PWM_CHANNEL);
  HAL_GPIO_WritePin(GPIOB, WARG_GPIO_PIN, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Set CS line low to begin transmission with ADC
	  HAL_GPIO_WritePin(GPIOB, WARG_GPIO_PIN, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData, size,
			  HAL_MAX_DELAY);
	  // Pull CS line high between conversations
	  HAL_GPIO_WritePin(GPIOB, WARG_GPIO_PIN, GPIO_PIN_SET);
	  // Format data from ADC, shift off last 6 garbage bits
	  adcData = (uint16_t)((pRxData[1] << 8) | (pRxData[2])) &
			  WARG_ADC_MAX_VALUE;
	  onCount = htim1.Init.Period * (adcData / WARG_ADC_MAX_VALUE *
			  (WARG_MAX_DUTY_CYCLE - WARG_MIN_DUTY_CYCLE) +
			  WARG_MIN_DUTY_CYCLE);
	  // Set compare register for TIM1
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, onCount);

	  HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  HAL_TIM_PWM_Stop(&htim1, WARG_TIM1_PWM_CHANNEL);
  HAL_SPI_DeInit(&hspi1);
  HAL_TIM_PWM_DeInit(&htim1);
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
