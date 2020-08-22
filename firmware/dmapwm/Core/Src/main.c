/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BUFFER_SIZE 24

// Define LED driver state machine states
#define LED_RES 0 // Reset state - all values should be 0 for 2 buffers
#define LED_DAT 1 // Data state - values should be taken from led_values

#define LED_ROWS 1
#define LED_COLS 1
#define R 0
#define G 1
#define B 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

/* USER CODE BEGIN PV */

uint16_t buffer[BUFFER_SIZE * 4] = { 0 };

void *buffer_pointer;

uint8_t led_value[LED_ROWS][LED_COLS][3] = { 0 };

uint8_t led_state = LED_RES;
uint8_t led_col = 0;
uint8_t led_row = 0;

uint32_t cnt_full = 0;
uint32_t cnt_half = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len) {
	for (int i = 0; i < len; i++)
		ITM_SendChar((*ptr++));
	return len;
}

//// Done sending first half of the DMA buffer - this can now safely be updated
//void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim) {
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//	if (htim->Instance == TIM3) {
//		buffer_pointer = &buffer[BUFFER_SIZE];
//		for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
//			buffer_pointer = &buffer[BUFFER_SIZE + i];
//			*(uint16_t*) buffer_pointer = (uint16_t) 68;
//			buffer_pointer++;
//		}
//	}
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//}
//
//// Done sending the second half of the DMA buffer - this can now be safely updated
//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//	if (htim->Instance == TIM3) {
//		buffer_pointer = &buffer[0];
//		for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
//			buffer_pointer = &buffer[i];
//			*(uint16_t*) buffer_pointer = (uint16_t) 31;
//			buffer_pointer++;
//		}
//	}
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//}

void setLedValue(uint8_t row, uint8_t col, uint8_t r, uint8_t g, uint8_t b) {

//	led_value[row][col][R] = r;
//	led_value[row][col][G] = g;
//	led_value[row][col][B] = b;

// Green msb first
	buffer[48] = (g >> 7 && 1) * 33 + 34;
	buffer[49] = (g >> 6 && 1) * 33 + 34;
	buffer[50] = (g >> 5 && 1) * 33 + 34;
	buffer[51] = (g >> 4 && 1) * 33 + 34;
	buffer[52] = (g >> 3 && 1) * 33 + 34;
	buffer[53] = (g >> 2 && 1) * 33 + 34;
	buffer[54] = (g >> 1 && 1) * 33 + 34;
	buffer[55] = (g && 1) * 33 + 34;
	// Red msb first
	buffer[56] = (r >> 7 && 1) * 33 + 34;
	buffer[57] = (r >> 6 && 1) * 33 + 34;
	buffer[58] = (r >> 5 && 1) * 33 + 34;
	buffer[59] = (r >> 4 && 1) * 33 + 34;
	buffer[60] = (r >> 3 && 1) * 33 + 34;
	buffer[61] = (r >> 2 && 1) * 33 + 34;
	buffer[62] = (r >> 1 && 1) * 33 + 34;
	buffer[63] = (r && 1) * 33 + 34;
	// Blue msb first
	buffer[64] = (b >> 7 && 1) * 33 + 34;
	buffer[65] = (b >> 6 && 1) * 33 + 34;
	buffer[66] = (b >> 5 && 1) * 33 + 34;
	buffer[67] = (b >> 4 && 1) * 33 + 34;
	buffer[68] = (b >> 3 && 1) * 33 + 34;
	buffer[69] = (b >> 2 && 1) * 33 + 34;
	buffer[70] = (b >> 1 && 1) * 33 + 34;
	buffer[71] = (b && 1) * 33 + 34;

	// Green msb first
	buffer[72] = 34;
	buffer[73] = 34;
	buffer[74] = 34;
	buffer[75] = 34;
	buffer[76] = 34;
	buffer[77] = 34;
	buffer[78] = 34;
	buffer[79] = 34;
	// Red msb first
	buffer[80] = 34;
	buffer[81] = 34;
	buffer[82] = 34;
	buffer[83] = 34;
	buffer[84] = 34;
	buffer[85] = 34;
	buffer[86] = 34;
	buffer[87] = 34;
	// Blue msb first
	buffer[88] = 34;
	buffer[89] = 34;
	buffer[90] = 34;
	buffer[91] = 34;
	buffer[92] = 34;
	buffer[93] = 34;
	buffer[94] = 34;
	buffer[95] = 34;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_DMA_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	// Set buffer appropriately
	//setLedValue(0, 0, 5);
	setLedValue(0, 0, 0, 10, 0);

	// Start the timer to get the PWM going
	HAL_TIM_Base_Start(&htim3);

	// Start DMA to feed the PWM with values
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t*) buffer,
	BUFFER_SIZE * 4);

	HAL_Delay(100);

	printf("Firing up!\n");

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	setLedValue(0, 0, 50, 0, 0);
	uint32_t then = 0;

	while (1) {

		uint32_t now = HAL_GetTick();
		if (now % 1000 == 0 && now != then) {

			printf("tick %5lu\n", now);

			setLedValue(0, 0, 0, 0, 100);

			then = now;
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 104;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
