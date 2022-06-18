/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
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
 UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline void printout(char *str) {
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);

}

//void handle_command(char *cmd) {
//	switch (*(cmd +1)){
//	case 'n':
//		HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, 0);
//		printout("Relay On\r\n");
//		break;
//	case 'f':
//		HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, 1);
//		printout("Relay Off\r\n");
//		break;
//	}
//}

void handle_ctrl(char *cmd){
	switch(*cmd){
		case 'w':
			X_LEFT();
			break;
		case 's':
			X_RIGHT();
			break;
		case 'a':
			Y_LEFT();
			break;
		case 'd':
			Y_RIGHT();
			break;
		case 'q':
			Z_LEFT();
			break;
		case 'e':
			Z_RIGHT();
			break;
		case 'o':
			X_OFF();
			Y_OFF();
			Z_OFF();
			break;
		default:
			X_OFF();
			Y_OFF();
			Z_OFF();
			break;
	}
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t state;
  char cmd[64];
  char buf[64];
  char *ptr = cmd;
  printout("STM BOOTING\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_StatusTypeDef rc;
  while (1)
  {
	  rc = HAL_UART_Receive(&huart1, (uint8_t *)ptr, 1, 100);

	  handle_ctrl(cmd);
	  *ptr = 0;

	  // below is input state machine
//	  state = idle;
//	     while (1) {
//	         switch (state) {
//	         case idle:
//	             printout("\r:>> ");
//	             state = receiving;
//	             break;
//	         case receiving:;
//	             HAL_StatusTypeDef rc = HAL_UART_Receive(&huart1, (uint8_t *)ptr, 1, 100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	             /* Build up the command one byte at a time */
//	             if (rc != HAL_OK) {
//	                 if (rc != HAL_TIMEOUT) {
//	                     sprintf(buf, "UART read error: %x\r\n", rc);
//	                     printout(buf);
//	                 }
//	                 continue;
//	             }
//	             /* Command is complete when we get EOL of some sort */
//	             if (*ptr == '\n' || *ptr == '\r') {
//	                 *ptr = 0;
//	                 printout("\r\n");
//	                 handle_command(cmd);
//	                 ptr = cmd;
//	                 state = idle;
//
//	             } else {
//	                 *(ptr + 1) = 0;
//	                 printout(ptr);
//
//	                 if (*ptr == 0x7f) { // handle backspace
//	                     if (ptr > cmd)
//	                         --ptr;
//	                 } else
//	                     ++ptr;
//	             }
//	             break;
//	         }
//	     }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RELAYX1_Pin|RELAYX2_Pin|RELAYY1_Pin|RELAYY2_Pin
                          |RELAYZ1_Pin|RELAYZ2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAYX1_Pin RELAYX2_Pin RELAYY1_Pin RELAYY2_Pin
                           RELAYZ1_Pin RELAYZ2_Pin */
  GPIO_InitStruct.Pin = RELAYX1_Pin|RELAYX2_Pin|RELAYY1_Pin|RELAYY2_Pin
                          |RELAYZ1_Pin|RELAYZ2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void X_LEFT(){
	HAL_GPIO_WritePin(RELAYX1_GPIO_Port, RELAYX1_Pin, 1);
	HAL_GPIO_WritePin(RELAYX2_GPIO_Port, RELAYX2_Pin, 0);
	printout("X LEFT\r\n");
}
void Y_LEFT(){
	HAL_GPIO_WritePin(RELAYY1_GPIO_Port, RELAYY1_Pin, 1);
	HAL_GPIO_WritePin(RELAYY2_GPIO_Port, RELAYY2_Pin, 0);
	printout("Y LEFT\r\n");
}
void Z_LEFT(){
	HAL_GPIO_WritePin(RELAYZ1_GPIO_Port, RELAYZ1_Pin, 1);
	HAL_GPIO_WritePin(RELAYZ2_GPIO_Port, RELAYZ2_Pin, 0);
	printout("Z LEFT\r\n");

}
void X_RIGHT(){
	HAL_GPIO_WritePin(RELAYX1_GPIO_Port, RELAYX1_Pin, 0);
	HAL_GPIO_WritePin(RELAYX2_GPIO_Port, RELAYX2_Pin, 1);
	printout("X RIGHT\r\n");
}
void Y_RIGHT(){
	HAL_GPIO_WritePin(RELAYY1_GPIO_Port, RELAYY1_Pin, 0);
	HAL_GPIO_WritePin(RELAYY2_GPIO_Port, RELAYY2_Pin, 1);
	printout("Y RIGHT\r\n");
}
void Z_RIGHT(){
	HAL_GPIO_WritePin(RELAYZ1_GPIO_Port, RELAYZ1_Pin, 0);
	HAL_GPIO_WritePin(RELAYZ2_GPIO_Port, RELAYZ2_Pin, 1);
	printout("Z RIGHT\r\n");
}

void X_OFF(){
//	HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, 0);
	HAL_GPIO_WritePin(RELAYX1_GPIO_Port, RELAYX1_Pin, 1);
	HAL_GPIO_WritePin(RELAYX2_GPIO_Port, RELAYX2_Pin, 1);
}
void Y_OFF(){
	HAL_GPIO_WritePin(RELAYY1_GPIO_Port, RELAYY1_Pin, 1);
	HAL_GPIO_WritePin(RELAYY2_GPIO_Port, RELAYY2_Pin, 1);
}
void Z_OFF(){
	HAL_GPIO_WritePin(RELAYZ1_GPIO_Port, RELAYZ1_Pin, 1);
	HAL_GPIO_WritePin(RELAYZ2_GPIO_Port, RELAYZ2_Pin, 1);}


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
