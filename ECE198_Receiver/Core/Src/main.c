/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for Receiver MCU
  ******************************************************************************
  * @attention
  *
  * This is the complete code for the receiver MCU. It receives the
  * average_distance data sent from the sender MCU via USART1. If the received
  * value is less than or equal to 50, it sets PB0 to high level.
  *
  ******************************************************************************
  */
  /* USER CODE END Header */

  /* Includes ------------------------------------------------------------------*/
  #include "main.h"

  /* Private includes ----------------------------------------------------------*/
  /* USER CODE BEGIN Includes */
  #include <stdio.h>   // For printf function
  #include <string.h>  // For strlen function
  /* USER CODE END Includes */

  /* Private typedef -----------------------------------------------------------*/
  /* USER CODE BEGIN PTD */
  /* No additional typedefs needed */
  /* USER CODE END PTD */

  /* Private define ------------------------------------------------------------*/
  /* USER CODE BEGIN PD */
  #define CONDITION_THRESHOLD 50  // Threshold value for triggering PB0
  /* USER CODE END PD */

  /* Private macro -------------------------------------------------------------*/
  /* USER CODE BEGIN PM */
  /* No additional macros needed */
  /* USER CODE END PM */

  /* Private variables ---------------------------------------------------------*/
  UART_HandleTypeDef huart1;  // USART1 handle
  UART_HandleTypeDef huart2;  // USART2 handle (optional for debugging)

  /* USER CODE BEGIN PV */
  uint8_t received_data = 0;  // Variable to store received data
  /* USER CODE END PV */

  /* Private function prototypes -----------------------------------------------*/
  void SystemClock_Config(void);
  static void MX_GPIO_Init(void);
  static void MX_USART1_UART_Init(void);
  static void MX_USART2_UART_Init(void);  // Optional for debugging

  /* USER CODE BEGIN PFP */
  int _write(int file, char *data, int len);  // Function to redirect printf to USART2
  /* USER CODE END PFP */

  /* Private user code ---------------------------------------------------------*/
  /* USER CODE BEGIN 0 */
  // Function to redirect printf to USART2
  int _write(int file, char *data, int len)
  {
      HAL_UART_Transmit(&huart2, (uint8_t*)data, len, HAL_MAX_DELAY);
      return len;
  }
  /* USER CODE END 0 */

  /**
    * @brief  The application entry point.
    * @retval int
    */
  int main(void)
  {
    /* USER CODE BEGIN 1 */
    /* No additional variables needed */
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */
    /* No additional initialization needed */
    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();  // Optional for debugging
    /* USER CODE BEGIN 2 */

    // Start USART1 reception in interrupt mode
    HAL_UART_Receive_IT(&huart1, &received_data, 1);

    // Optional: Print initialization message
    printf("Receiver MCU Initialized.\r\n");

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
      // Main loop does nothing; data is processed in UART receive callback
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
      // Add any additional code here if needed
    }
    /* USER CODE END 3 */
  }

  /**
    * @brief System Clock Configuration
    * @retval None
    */
  void SystemClock_Config(void)
  {
    /* Configure the system clock as per your hardware and requirements */
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /** Initializes the RCC Oscillators */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  // Use external crystal
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;  // Enable HSE
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;  // Enable PLL
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;  // PLL source is HSE
    RCC_OscInitStruct.PLL.PLLM = 8;  // Adjust according to your HSE frequency
    RCC_OscInitStruct.PLL.PLLN = 336;  // PLLN value
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;  // PLLP value
    RCC_OscInitStruct.PLL.PLLQ = 7;  // PLLQ value
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();  // Initialization Error
    }

    /** Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;  // Configure clocks
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // Use PLL as SYSCLK source
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  // HCLK = SYSCLK
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  // APB1 = HCLK/2
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  // APB2 = HCLK

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
      Error_Handler();  // Initialization Error
    }
  }

  /**
    * @brief USART1 Initialization Function
    * @param None
    * @retval None
    */
  static void MX_USART1_UART_Init(void)
  {
    /* Configure USART1 for receiving data from the sender MCU */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;  // Baud rate matches the sender
    huart1.Init.WordLength = UART_WORDLENGTH_8B;  // 8 data bits
    huart1.Init.StopBits = UART_STOPBITS_1;  // 1 stop bit
    huart1.Init.Parity = UART_PARITY_NONE;  // No parity
    huart1.Init.Mode = UART_MODE_RX;  // Receive only
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;  // No hardware flow control
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;  // 16x oversampling
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
      Error_Handler();  // Initialization Error
    }
  }

  /**
    * @brief USART2 Initialization Function (Optional for debugging)
    * @param None
    * @retval None
    */
  static void MX_USART2_UART_Init(void)
  {
    /* Configure USART2 for debugging via serial port assistant */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;  // Baud rate
    huart2.Init.WordLength = UART_WORDLENGTH_8B;  // 8 data bits
    huart2.Init.StopBits = UART_STOPBITS_1;  // 1 stop bit
    huart2.Init.Parity = UART_PARITY_NONE;  // No parity
    huart2.Init.Mode = UART_MODE_TX_RX;  // Enable transmit and receive
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;  // No hardware flow control
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;  // 16x oversampling
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
      Error_Handler();  // Initialization Error
    }
  }

  /**
    * @brief GPIO Initialization Function
    * @param None
    * @retval None
    */
  static void MX_GPIO_Init(void)
  {
    /* Initialize GPIO for PB0 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();  // Enable GPIOB clock
    __HAL_RCC_GPIOA_CLK_ENABLE();  // Enable GPIOA clock

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // Set PB0 to low

    /*Configure GPIO pin : PB0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull output
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // Low speed
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configure USART1_RX Pin : PA10 */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;  // Alternate function Push-Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  // Very high speed
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;  // Set alternate function to USART1_RX
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure USART2_TX and USART2_RX Pins (Optional for debugging) */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;  // PA2 (USART2_TX), PA3 (USART2_RX)
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;  // Alternate function Push-Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  // Very high speed
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;  // Set alternate function to USART2
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }

  /* USER CODE BEGIN 4 */
  /**
    * @brief  UART receive complete callback function
    * @param  huart: UART handle
    * @retval None
    */
  void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
  {
    if (huart->Instance == USART1)
    {
      // Process the received data
      uint8_t data = received_data;

      // Optional: Send the received data to serial port assistant via USART2
      printf("Received Data: %d\r\n", data);

      // Check the condition and set PB0 high if condition is met
      if (data <= CONDITION_THRESHOLD)
      {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // Set PB0 high
        printf("PB0 set to HIGH\r\n");
      }
      else
      {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // Set PB0 low
        printf("PB0 set to LOW\r\n");
      }

      // Restart UART reception
      HAL_UART_Receive_IT(&huart1, &received_data, 1);
    }
  }
  /* USER CODE END 4 */

  /**
    * @brief  This function is executed in case of error occurrence.
    * @retval None
    */
  void Error_Handler(void)
  {
    /* User can add their own error handling code here */
    __disable_irq();
    while (1)
    {
      // Stay in this loop in case of error
    }
  }

  #ifdef USE_FULL_ASSERT
  /**
    * @brief  Reports the name of the source file and the source line number where the assert_param error has occurred.
    * @param  file: Pointer to the source file name
    * @param  line: Line number where the assert_param error has occurred
    * @retval None
    */
  void assert_failed(uint8_t *file, uint32_t line)
  {
    /* User can add their own implementation to report the file name and line number */
    printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  }
  #endif /* USE_FULL_ASSERT */
