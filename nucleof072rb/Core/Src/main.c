/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN PV */

// Define constants to replace magic numbers
#define ADC_MAX_VALUE 1023         // Maximum ADC value (10-bit ADC)
#define PWM_PERIOD 19999           // Timer period value
#define DUTY_CYCLE_MIN 5           // Minimum duty cycle percentage
#define DUTY_CYCLE_MAX 10          // Maximum duty cycle percentage
#define CHIP_SELECT_PIN GPIO_PIN_8  // Chip Select Pin
#define CHIP_SELECT_PORT GPIOB      // Chip Select Port

uint16_t adc_value = 0;  // Stores ADC reading
uint16_t pwm_value = 0;  // Stores PWM output

/* USER CODE END PV */

void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN INIT */

  // Set Chip Select High initially
  HAL_GPIO_WritePin(CHIP_SELECT_PORT, CHIP_SELECT_PIN, GPIO_PIN_SET);

  // Set Initial PWM Value (Start at the minimum duty cycle)
  pwm_value = (DUTY_CYCLE_MIN * PWM_PERIOD) / 100;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);

  /* USER CODE END INIT */

  /* Start PWM */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  while (1)
  {
    uint8_t spi_tx[3] = {0x01, 0x80, 0x00}; // Command to ADC
    uint8_t spi_rx[3] = {0};

    // Set Chip Select LOW (Start SPI)
    HAL_GPIO_WritePin(CHIP_SELECT_PORT, CHIP_SELECT_PIN, GPIO_PIN_RESET);

    // SPI Transfer
    HAL_SPI_TransmitReceive(&hspi1, spi_tx, spi_rx, 3, HAL_MAX_DELAY);

    // Set Chip Select HIGH (End SPI)
    HAL_GPIO_WritePin(CHIP_SELECT_PORT, CHIP_SELECT_PIN, GPIO_PIN_SET);

    // Convert received ADC data (10-bit value)
    adc_value = ((spi_rx[1] & 0x03) << 8) | spi_rx[2];

    // Convert ADC value to PWM (5-10% duty cycle) using defined constants
    pwm_value = ((adc_value * (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN)) / ADC_MAX_VALUE) + DUTY_CYCLE_MIN;

    // Scale PWM duty cycle to match timer period
    pwm_value = (pwm_value * PWM_PERIOD) / 100;

    // Set PWM Duty Cycle
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);

    HAL_Delay(10);  // Prevent ADC overload
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

  /* RCC Configuration */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) { Error_Handler(); }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief  Error Handler
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
