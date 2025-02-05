/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GSM_BUFFER_SIZE 512
#define DEBUG_BUFFER_SIZE 512

#define RS485_ENABLE

#ifdef RS485_ENABLE
#   define MAX485_RECV do{HAL_GPIO_WritePin(MODBUS_REn_DE_GPIO_Port, MODBUS_REn_DE_Pin, GPIO_PIN_RESET);}while(0);
#   define MAX485_TRAN do{HAL_GPIO_WritePin(MODBUS_REn_DE_GPIO_Port, MODBUS_REn_DE_Pin, GPIO_PIN_SET);}while(0);
#else
#   define MAX485_RECV do{}while(0);
#   define MAX485_TRAN do{}while(0);
#endif


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1; // RS485 modbus
UART_HandleTypeDef huart3; // GSM module
UART_HandleTypeDef huart4; // Debug Uart

/* USER CODE BEGIN PV */
char GSMrxBuffer[GSM_BUFFER_SIZE]; // Buffer for UART3 reception
volatile char GSMrxData; // Single-byte buffer for ISR
uint16_t GSMrxIndex = 0;
volatile uint32_t sysTickCounter = 0;
char DebugBuffer[DEBUG_BUFFER_SIZE+11]; // Buffer for debug messages //offset 11 is for print response

// APN Configuration
#warning Add your APN and HTTP here
const char *apn = "your_apn";
const char *http_url = "http://example.com";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART4_UART_Init(void);
/* USER CODE BEGIN PFP */
void GSMSendCommand(const char *cmd);
void ProcessGSMResponse(void);
void HTTPGetRequest(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/


/**
 * @brief  SysTick Interrupt Handler
 */
void SysTick_Handler(void) {
    if (sysTickCounter > 0) {
        sysTickCounter--;
    }
}

/**
 * @brief  Delay function using SysTick
 * @param  ms: Milliseconds to wait
 */
void DelayMs(uint32_t ms) {
    sysTickCounter = ms;
    while (sysTickCounter);  // Wait until counter reaches zero
}

/* USER CODE BEGIN 0 */
void GSMUARTInit(void)
{
    HAL_UART_Receive_IT(&huart3, (uint8_t*)&GSMrxData, 1); // Start UART3 interrupt
}

/**
 * @brief  UART Rx Complete Callback (Interrupt Handler)
 * @param  huart: UART handle pointer
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {  // Check if interrupt is from UART3 (GSM)
        if (GSMrxIndex < GSM_BUFFER_SIZE - 1) {
            GSMrxBuffer[GSMrxIndex++] = GSMrxData;
        }
        GSMrxBuffer[GSMrxIndex] = '\0'; // Null-terminate the string
        HAL_UART_Receive_IT(&huart3, (uint8_t*)&GSMrxData, 1); // Re-enable interrupt
    }
}

/**
 * @brief  Sends AT command to EC200U
 * @param  cmd: Command string to send
 */
void GSMSendCommand(const char *cmd) {
    HAL_UART_Transmit(&huart3, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

    snprintf(DebugBuffer, sizeof(DebugBuffer), "Sent: %s\n", cmd);
    HAL_UART_Transmit(&huart4, (uint8_t*)DebugBuffer, strlen(DebugBuffer), HAL_MAX_DELAY);
}

/**
 * @brief  Processes GSM Response
 */
void ProcessGSMResponse(void) {
    if (strstr(GSMrxBuffer, "OK") || strstr(GSMrxBuffer, "ERROR")) {
        snprintf(DebugBuffer, sizeof(DebugBuffer), "Response: %s\n", GSMrxBuffer);
        HAL_UART_Transmit(&huart4, (uint8_t*)DebugBuffer, strlen(DebugBuffer), HAL_MAX_DELAY);
    }
    memset(GSMrxBuffer, 0, GSM_BUFFER_SIZE);
    GSMrxIndex = 0;
}



/**
 * @brief  Establishes a PDP context and sends an HTTP GET request
 */
void HTTPGetRequest(void) {
    GSMSendCommand("AT");               // Check module
    DelayMs(1000);
    ProcessGSMResponse();

    char tempBuff[64];
    snprintf(tempBuff, sizeof(tempBuff), "AT+CGDCONT=1,\"IP\",\"%s\"", apn);
    GSMSendCommand(tempBuff);  // Set APN
    DelayMs(2000);
    ProcessGSMResponse();

    GSMSendCommand("AT+QIACT=1");       // Activate PDP context
    DelayMs(5000);
    ProcessGSMResponse();

    GSMSendCommand("AT+QHTTPURL=20,80"); // Set HTTP URL length
    DelayMs(1000);
    GSMSendCommand(http_url);           // Send HTTP URL
    DelayMs(2000);
    ProcessGSMResponse();

    GSMSendCommand("AT+QHTTPGET=80");   // Perform HTTP GET
    DelayMs(5000);
    ProcessGSMResponse();
}



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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();
  /* USER CODE BEGIN 2 */
#if 0 //Ranjan
  //Welcome Messages
    printf("System Initializing Started...\n");
    HAL_Delay(2000);
    printf("System Initialization Done\n");
    HAL_Delay(2000);

    //Initialize 3.8V Regulator
    printf("Initializing 3.8V Regulator...\n");
    HAL_Delay(2000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    printf("3.8V Regulator Started\n");
    HAL_Delay(2000);

    //Initialize 4G modem power-key
    printf("Initializing 4G modem power-key...\n");
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(2000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    printf("4G Modem Started\n");
    HAL_Delay(2000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_Delay(2000);

    HAL_NVIC_SetPriority(USART3_4_LPUART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_4_LPUART1_IRQn);

    HAL_UART_Receive_IT(&huart4, &rx_buffer, 1);
        // Send an AT command to the modem via UART3
    	  printf("sending AT");
        Send_AT_Command("AT");  // For example, "AT" command to test if the modem responds
        HAL_Delay(2000);  // Wait for a response (you may want to implement a timeout here)
        printf("sending ATI");
        Send_AT_Command("ATI");
        HAL_Delay(2000);
#endif //ranjan
   //Welcome Messages
#warning why 4 sec delay???
    printf("System Initializing Started...\n");
    DelayMs(2000);
    printf("System Initialization Done\n");
    DelayMs(2000);

    //Initialize 3.8V Regulator
    printf("Initializing 3.8V Regulator...\n");
    DelayMs(2000);
    HAL_GPIO_WritePin(MODEM_3V8_EN_GPIO_Port, MODEM_3V8_EN_Pin, GPIO_PIN_SET);
    printf("3.8V Regulator Started\n");
    DelayMs(2000);

#warning according to datasheet ist 100ms why 2sec??
    //Initialize 4G modem power-key
    printf("Initializing 4G modem power-key...\n");
    HAL_GPIO_WritePin(MODEM_PWRKEY_GPIO_Port, MODEM_PWRKEY_Pin, GPIO_PIN_SET);
    DelayMs(2000);
    HAL_GPIO_WritePin(MODEM_PWRKEY_GPIO_Port, MODEM_PWRKEY_Pin, GPIO_PIN_RESET);
    printf("4G Modem Started\n");
    DelayMs(2000);
    HAL_GPIO_WritePin(MODEM_APREADY_GPIO_Port, MODEM_APREADY_Pin, GPIO_PIN_SET);
    DelayMs(2000);
    HTTPGetRequest();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GTVL_ON_Pin|GTVL_OFF_Pin|RELAY1_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RELAY2_OUT_Pin|MODBUS_REn_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MODEM_APREADY_Pin|MODEM_3V8_EN_Pin|MODEM_RESETn_Pin|MODEM_PWRKEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GTVL_ON_Pin GTVL_OFF_Pin RELAY1_OUT_Pin */
  GPIO_InitStruct.Pin = GTVL_ON_Pin|GTVL_OFF_Pin|RELAY1_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LIMIT_SW_IN1_Pin LIMIT_SW_IN2_Pin */
  GPIO_InitStruct.Pin = LIMIT_SW_IN1_Pin|LIMIT_SW_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : R_PH_OK_Pin Y_PH_OK_Pin B_PH_OK_Pin */
  GPIO_InitStruct.Pin = R_PH_OK_Pin|Y_PH_OK_Pin|B_PH_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY2_OUT_Pin MODBUS_REn_DE_Pin */
  GPIO_InitStruct.Pin = RELAY2_OUT_Pin|MODBUS_REn_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ZCD_IN_Pin */
  GPIO_InitStruct.Pin = ZCD_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ZCD_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MODEM_APREADY_Pin MODEM_3V8_EN_Pin MODEM_RESETn_Pin MODEM_PWRKEY_Pin */
  GPIO_InitStruct.Pin = MODEM_APREADY_Pin|MODEM_3V8_EN_Pin|MODEM_RESETn_Pin|MODEM_PWRKEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MODEM_RI_Pin */
  GPIO_InitStruct.Pin = MODEM_RI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MODEM_RI_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
