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
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>  // Include standard input-output header
#include <string.h> // Include string functions if needed

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Define speed of sound, unit in cm/us
#define SPEED_OF_SOUND 0.0343f

// Variable declarations
uint32_t start_time1 = 0;
uint32_t end_time1 = 0;
uint8_t distance1 = 0; // Modified to uint8_t

uint32_t start_time2 = 0;
uint32_t end_time2 = 0;
uint8_t distance2 = 0; // Modified to uint8_t
uint8_t average_distance = 0; // Modified to uint8_t

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
// Function to send uint8_t data
void USART_SendUint8(uint8_t data);
// Microsecond-level delay function
void delay_us(uint16_t us);
void calculate_distance(uint32_t start, uint32_t end, uint8_t *distance);
void calculate_average(uint8_t dist1, uint8_t dist2, uint8_t *avg);
void clear_sensor_data(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Microsecond-level delay function, using TIM2
void delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim2, 0); // Reset counter
    while (__HAL_TIM_GET_COUNTER(&htim2) < us); // Wait until counter reaches specified value
}

// Function to send uint8_t data
void USART_SendUint8(uint8_t data) {
    HAL_UART_Transmit(&huart2, &data, 4, HAL_MAX_DELAY);
}

// Calculate distance and convert to uint8_t
void calculate_distance(uint32_t start, uint32_t end, uint8_t *distance) {
    uint32_t time_diff = 0;
    if (end >= start) {
        time_diff = end - start;
    } else {
        // Handle counter overflow
        time_diff = (0xFFFFFFFF - start) + end;
    }

    // Calculate distance in cm, limit to 0-255 range
    float dist = (time_diff * SPEED_OF_SOUND) / 2.0f;
    if (dist > 255.0f) {
        dist = 255.0f; // Maximum value limit
    }
    *distance = (uint8_t)dist;
}

// Calculate average distance and convert to uint8_t
void calculate_average(uint8_t dist1, uint8_t dist2, uint8_t *avg) {
    uint16_t temp_avg = (dist1 + dist2) / 2;
    if (temp_avg > 255) {
        temp_avg = 255; // Maximum value limit
    }
    *avg = (uint8_t)temp_avg;
}

// Clear sensor data
void clear_sensor_data(void) {
    distance1 = 0;
    distance2 = 0;
    average_distance = 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* No additional variable declarations needed */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset all peripherals, initialize the Flash interface and the Systick */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* No additional initialization needed */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* No additional system initialization needed */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  // Start timer
  HAL_TIM_Base_Start(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    clear_sensor_data(); // Clear sensor data

    // ====== Sensor 1 ======
    // Trigger ultrasonic sensor 1 by setting Trig pin PA5 high for 10 microseconds
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // Set Trig pin high
    delay_us(10); // Delay 10 microseconds
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // Set Trig pin low

    // Wait for Echo1 pin to go high, set timeout to prevent infinite wait
    uint32_t timeout1 = 0;
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET) {
        if (++timeout1 > 1000000) { // Adjust timeout as needed
            break;
        }
    }

    if (timeout1 <= 1000000) { // Continue only if Echo1 goes high successfully
        start_time1 = __HAL_TIM_GET_COUNTER(&htim2); // Record start time

        // Wait for Echo1 pin to go low, set timeout
        timeout1 = 0;
        while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) {
            if (++timeout1 > 1000000) { // Adjust timeout as needed
                break;
            }
        }

        if (timeout1 <= 1000000) { // Continue only if Echo1 goes low successfully
            end_time1 = __HAL_TIM_GET_COUNTER(&htim2); // Record end time

            // Calculate distance and convert to uint8_t
            calculate_distance(start_time1, end_time1, &distance1);
        }
    }

    // ====== Sensor 2 ======
    // Trigger ultrasonic sensor 2 by setting Trig pin PA9 high for 10 microseconds
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); // Set Trig pin high
    delay_us(10); // Delay 10 microseconds
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); // Set Trig pin low

    // Wait for Echo2 pin to go high, set timeout to prevent infinite wait
    uint32_t timeout2 = 0;
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET) {
        if (++timeout2 > 1000000) { // Adjust timeout as needed
            break;
        }
    }

    if (timeout2 <= 1000000) { // Continue only if Echo2 goes high successfully
        start_time2 = __HAL_TIM_GET_COUNTER(&htim2); // Record start time

        // Wait for Echo2 pin to go low, set timeout
        timeout2 = 0;
        while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) {
            if (++timeout2 > 1000000) { // Adjust timeout as needed
                break;
            }
        }

        if (timeout2 <= 1000000) { // Continue only if Echo2 goes low successfully
            end_time2 = __HAL_TIM_GET_COUNTER(&htim2); // Record end time

            // Calculate distance and convert to uint8_t
            calculate_distance(start_time2, end_time2, &distance2);
        }
    }

    // ====== Calculate average distance ======
    calculate_average(distance1, distance2, &average_distance);

    // Send average_distance via USART
    USART_SendUint8(average_distance);

    HAL_Delay(100);  // Delay 100 milliseconds

    /* USER CODE END 3 */
  }
  /* USER CODE END 3 */
} // End of main function

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  /* System Clock Configuration code (generated by STM32CubeMX) */
  // Keep unchanged based on your project settings
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  /* Configure TIM2 for microsecond-level counting */
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = (SystemCoreClock / 1000000) - 1; // 1 MHz counting frequency
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF; // Maximum count value
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  /* Configure USART2 for communication at 115200 baud rate */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200; // Set baud rate
  huart2.Init.WordLength = UART_WORDLENGTH_8B; // 8 data bits
  huart2.Init.StopBits = UART_STOPBITS_1; // 1 stop bit
  huart2.Init.Parity = UART_PARITY_NONE; // No parity
  huart2.Init.Mode = UART_MODE_TX_RX; // Enable transmit and receive
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE; // No hardware flow control
  huart2.Init.OverSampling = UART_OVERSAMPLING_16; // 16x oversampling
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure GPIO pin output level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_9, GPIO_PIN_RESET); // Trig1 and Trig2 pins
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Vibration motor control pin (if used)

  /* Configure GPIO pin: PA5 (Trig1 pin) */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull output
  GPIO_InitStruct.Pull = GPIO_NOPULL; // No pull-up or pull-down
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Low speed
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure GPIO pin: PA6 (Echo1 pin) */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // Input mode
  GPIO_InitStruct.Pull = GPIO_NOPULL; // No pull-up or pull-down
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure GPIO pin: PA9 (Trig2 pin) */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull output
  GPIO_InitStruct.Pull = GPIO_NOPULL; // No pull-up or pull-down
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Low speed
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure GPIO pin: PA8 (Echo2 pin) */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // Input mode
  GPIO_InitStruct.Pull = GPIO_NOPULL; // No pull-up or pull-down
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure GPIO pin: PB0 (Vibration motor control pin) */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull output
  GPIO_InitStruct.Pull = GPIO_NOPULL; // No pull-up or pull-down
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Low speed
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Function executed in case of error.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add their own error handling code here, such as blinking an LED to indicate an error */
  __disable_irq();
  while (1)
  {
    // Error handling loop
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: Pointer to the name of the source file
  * @param  line: Line number where the assert_param error occurred
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* Users can add their own implementation here to report the file name
     and line number, for example:
     printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  */
}
#endif /* USE_FULL_ASSERT */
