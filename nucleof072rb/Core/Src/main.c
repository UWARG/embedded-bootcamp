/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
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
#include <stdlib.h>

/* Private define ------------------------------------------------------------*/
#define PWM_MIN 3200
#define PWM_MAX 6400



// i just did some small changes mostly 



/* Private variables ---------------------------------------------------------*/
uint8_t pT_data[3] = {0x01, 0x80, 0x00};  // nothing changed here lol
uint8_t pR_data[3];
uint16_t size = 3;
uint32_t timeout = HAL_MAX_DELAY;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
double ADCToPWM(uint32_t ADC_val);

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_SPI1_Init();
    MX_TIM2_Init();  // Changed to use TIM2

    /* Start the PWM output */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    /* Main loop */
    while (1)
    {
        // Set CS low before transmission
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // i used write instead of toggle here
        HAL_SPI_TransmitReceive(&hspi1, pT_data, pR_data, size, timeout);
        // Set CS high after transmission
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

        /* Process ADC data */
        uint32_t ADC_val = ((pR_data[1] & 0x03) << 8) | pR_data[2]; 

        /* Set compare register for TIM2 */
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ADCToPWM(ADC_val));

        HAL_Delay(10);
    }
}

/**
  * @brief 
  * @retval 
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1;
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
double ADCToPWM(uint32_t ADC_val) {
    return ((ADC_val / 1023.0) * (PWM_MAX - PWM_MIN)) + PWM_MIN;
}
/* USER CODE END 4 */

/**
  * @brief  
  * @retval 
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        // Error handling code
    }
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
    // Assert failure handling code
}
#endif /* USE_FULL_ASSERT */
