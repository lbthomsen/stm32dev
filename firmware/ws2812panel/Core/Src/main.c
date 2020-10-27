/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 Lars Boegild THomsen <lbthomsen@gmail.com>.
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
#include <math.h>
#include <arm_math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Configure the timer values
#define T3_PRE 0    // Timer 3 preload
#define T3_CNT 104  // Timer 3 counter - 104 result in 800 kHz PWM
#define T4_PRE 8399 // Timer 4 preload
#define T4_CNT 99   // Timer 4 counter - result in 100 interrupts per second

#define M_PI2 2 * M_PI   // Full circle
#define SAMPLE_FREQ 100  // Frequency of Timer 4

// Buffer allocated will be twice this
#define BUFFER_SIZE 24

// LED on/off counts.  PWM timer is running 104 counts.
//#define LED_PERIOD T3_CNT + 1
#define LED_OFF 33
#define LED_ON 71
#define LED_RESET_CYCLES 10

// Define LED driver state machine states
#define LED_RES 0
#define LED_DAT 1

#define LED_ROWS 8
#define LED_COLS 8
#define G 0
#define R 1
#define B 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

/* USER CODE BEGIN PV */

// Bunch of zeros used to latch the ws2812
const uint16_t zeros[24] = {0}; // Used to trigger latching

// Look up table for led color bit patterns.  "Waste" 4k of flash but is a
// lot faster (not measured accurately but I'd say about double).
const uint16_t color_value[256][8] = {
		{LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF},
		{LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON},
		{LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF},
		{LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON},
		{LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF},
		{LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON},
		{LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF},
		{LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON},
		{LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF},
		{LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON},
		{LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF},
		{LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON},
		{LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF},
		{LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON},
		{LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF},
		{LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON},
		{LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF},
		{LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON},
		{LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF},
		{LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON},
		{LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF},
		{LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON},
		{LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF},
		{LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON},
		{LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF},
		{LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON},
		{LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF},
		{LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON},
		{LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF},
		{LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON},
		{LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF},
		{LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON},
		{LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF},
		{LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON},
		{LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF},
		{LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON},
		{LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF},
		{LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON},
		{LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF},
		{LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON},
		{LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF},
		{LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON},
		{LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF},
		{LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON},
		{LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF},
		{LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON},
		{LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF},
		{LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON},
		{LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF},
		{LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON},
		{LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF},
		{LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON},
		{LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF},
		{LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON},
		{LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF},
		{LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON},
		{LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF},
		{LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON},
		{LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF},
		{LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON},
		{LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF},
		{LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON},
		{LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF},
		{LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON},
		{LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF},
		{LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON},
		{LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF},
		{LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON},
		{LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF},
		{LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON},
		{LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF},
		{LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON},
		{LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF},
		{LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON},
		{LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF},
		{LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON},
		{LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF},
		{LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON},
		{LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF},
		{LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON},
		{LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF},
		{LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON},
		{LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF},
		{LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON},
		{LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF},
		{LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON},
		{LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF},
		{LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON},
		{LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF},
		{LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON},
		{LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF},
		{LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON},
		{LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF},
		{LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON},
		{LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF},
		{LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON},
		{LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF},
		{LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON},
		{LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF},
		{LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON},
		{LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF},
		{LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON},
		{LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF},
		{LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON},
		{LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF},
		{LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON},
		{LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF},
		{LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON},
		{LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF},
		{LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON},
		{LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF},
		{LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON},
		{LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF},
		{LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON},
		{LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF},
		{LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON},
		{LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF},
		{LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON},
		{LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF},
		{LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON},
		{LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF},
		{LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON},
		{LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF},
		{LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON},
		{LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF},
		{LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON},
		{LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF},
		{LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON},
		{LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF},
		{LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON},
		{LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF},
		{LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON},
		{LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF},
		{LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON},
		{LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF},
		{LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON},
		{LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF},
		{LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON},
		{LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF},
		{LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON},
		{LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF},
		{LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON},
		{LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF},
		{LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON},
		{LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF},
		{LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON},
		{LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF},
		{LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON},
		{LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF},
		{LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON},
		{LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF},
		{LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON},
		{LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF},
		{LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON},
		{LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF},
		{LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON},
		{LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF},
		{LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON},
		{LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF},
		{LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON},
		{LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF},
		{LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON},
		{LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF},
		{LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON},
		{LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF},
		{LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON},
		{LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF},
		{LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON},
		{LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF},
		{LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON},
		{LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF},
		{LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON},
		{LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF},
		{LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON},
		{LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF},
		{LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON},
		{LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF},
		{LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON},
		{LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF},
		{LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON},
		{LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF},
		{LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON},
		{LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF},
		{LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON},
		{LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF},
		{LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON},
		{LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF},
		{LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON},
		{LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF},
		{LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON},
		{LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF},
		{LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON},
		{LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF},
		{LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON},
		{LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF},
		{LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON},
		{LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF},
		{LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON},
		{LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF},
		{LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON},
		{LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF},
		{LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON},
		{LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF},
		{LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON},
		{LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF},
		{LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON},
		{LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF},
		{LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON},
		{LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF},
		{LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON},
		{LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF},
		{LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON},
		{LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF},
		{LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON},
		{LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF},
		{LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON},
		{LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF},
		{LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON},
		{LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF},
		{LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON},
		{LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF},
		{LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON},
		{LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF},
		{LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON},
		{LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_OFF},
		{LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF, LED_ON},
		{LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF},
		{LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_ON},
		{LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF},
		{LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_ON},
		{LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_OFF},
		{LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON, LED_ON},
		{LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_OFF},
		{LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF, LED_ON},
		{LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_OFF},
		{LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON, LED_ON},
		{LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_OFF},
		{LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF, LED_ON},
		{LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_OFF},
		{LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON, LED_ON},
		{LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_OFF},
		{LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON},
		{LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_OFF},
		{LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON, LED_ON},
		{LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_OFF},
		{LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF, LED_ON},
		{LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_OFF},
		{LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON, LED_ON},
		{LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_OFF},
		{LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF, LED_ON},
		{LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_OFF},
		{LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON, LED_ON},
		{LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_OFF},
		{LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF, LED_ON},
		{LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_OFF},
		{LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON, LED_ON}
};

// The actual DMA buffer - contains two halves each container 24 byte
uint16_t dma_buffer[BUFFER_SIZE * 2] = { 0 };

// Pointer to above buffer.  This will be set to the start of the half way point
uint16_t *dma_buffer_pointer;

// Base for calculating RGB values
float led_angle[LED_ROWS][LED_COLS][3] = { 0 };
float led_velocity[LED_ROWS][LED_COLS][3] = { 0 };
uint8_t led_amplitude[LED_ROWS][LED_COLS][3] = { 0 };

// LED RGB values
uint8_t led_value[LED_ROWS][LED_COLS][3] = { 0 };

uint8_t led_state = LED_RES;
uint8_t res_cnt = 0;
uint8_t led_col = 0;
uint8_t led_row = 0;

uint32_t update_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * Update next 24 bits in the dma buffer - assume dma_buffer_pointer is pointing
 * to the buffer that is safe to update.
 *
 */
static inline void update_next_buffer() {

	// For debugging and measuring calculation time - toggle GPIO pin
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

	// A simple state machine - we're either resetting (two buffers worth of zeros) or
	// we are transmitting data

	if (led_state == LED_RES) { // Reset state - 10 or more full buffers of zeros

		// This one is simple - we got a bunch of zeros of the right size - just throw
		// that into the buffer
		memcpy(dma_buffer_pointer, zeros, 48);

//      Old version without the pre-filled buffer
//		for (uint8_t i = 0; i < BUFFER_SIZE; i++) { // Fill buffer with zeros
//			*(uint16_t*) dma_buffer_pointer = (uint16_t) 0;
//			dma_buffer_pointer++;
//		}

		res_cnt++;

		if (res_cnt >= LED_RESET_CYCLES) { // done enough reset cycles
			led_col = 0;	// prepare to send data
			led_row = 0;
			led_state = LED_DAT;
		}

	} else { // LED state

		// First let's deal with the current LED
		uint8_t *led = led_value[led_col][led_row];
		for (uint8_t c = 0; c < 3; c++) { // This is the bitch - need to be optimized!

			// Copy values from the pre-filled color_value buffer
			memcpy(dma_buffer_pointer, color_value[led[c]], 16);
			dma_buffer_pointer += 8;

			// Old version
//			uint8_t value = led[c];
//			// Now deal with each bit
//			for (uint8_t b = 0; b < 8; b++) {
//				*(uint16_t*) dma_buffer_pointer =
//						value >> (7 - b) != 0 ? LED_ON : LED_OFF;
//				dma_buffer_pointer++;
//			}
		}

		// Now move to next LED switching to reset state when all leds have been updated
		led_col++; // Next column
		if (led_col >= LED_COLS) { // reached top
			led_col = 0; // back to first
			led_row++; // and move on to next row
			if (led_row >= LED_ROWS) { // reached end

				res_cnt = 0;
				led_state = LED_RES;
			}
		}

	}

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

}

// Handle built-in blue led hanging off of C13
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM4) {

		// Toggle GPIO for time measurement
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

		// Update all led values - this is quite heavy for 8*8*3 it takes around 30-40 ms
		for (uint8_t col = 0; col < LED_COLS; col++) {

			for (uint8_t row = 0; row < LED_ROWS; row++) {

				for (uint8_t led = 0; led < 3; ++led) {

					led_value[row][col][led] = (uint8_t)(led_amplitude[row][col][led] - arm_cos_f32(led_angle[row][col][led]) * led_amplitude[row][col][led]);

					led_angle[row][col][led] += led_velocity[row][col][led];
					if (led_angle[row][col][led] > M_PI2) led_angle[row][col][led] -= M_PI2; // Positive wrap around
					if (led_angle[row][col][led] < M_PI2) led_angle[row][col][led] += M_PI2; // Negative wrap around

				}

			}

		}

		// Reset pin
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

	}

}

// Done sending first half of the DMA buffer - this can now safely be updated
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM3) {
		dma_buffer_pointer = &dma_buffer[0];
		update_next_buffer();
	}

}

// Done sending the second half of the DMA buffer - this can now be safely updated
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM3) {
		dma_buffer_pointer = &dma_buffer[BUFFER_SIZE];
		update_next_buffer();
	}

}

// Just throw values into led_value array - the dma interrupt will
// handle updating the dma buffer when needed
void setLedValue(uint8_t col, uint8_t row, uint8_t r, uint8_t g, uint8_t b) {

	led_value[col][row][R] = r;
	led_value[col][row][G] = g;
	led_value[col][row][B] = b;

}

void setLedAngle(uint8_t col, uint8_t row, float r, float g, float b) {

	led_angle[col][row][R] = r;
	led_angle[col][row][G] = g;
	led_angle[col][row][B] = b;

}

void setLedFreq(uint8_t col, uint8_t row, float r, float g, float b) {

	led_velocity[col][row][R] = M_PI2 / (SAMPLE_FREQ / r);
	led_velocity[col][row][G] = M_PI2 / (SAMPLE_FREQ / g);
	led_velocity[col][row][B] = M_PI2 / (SAMPLE_FREQ / b);

}

void setLedAmplitude(uint8_t col, uint8_t row, uint8_t r, uint8_t g, uint8_t b) {

	led_amplitude[col][row][R] = r;
	led_amplitude[col][row][G] = g;
	led_amplitude[col][row][B] = b;

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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim4);

	// Start DMA to feed the PWM with values
	// At this point the buffer should be empty - all zeros
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t*) dma_buffer,
	BUFFER_SIZE * 2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	// Outer ring


	for (int i = 0; i < 8; i++) {

		setLedAmplitude(i, 0, 50, 50, 50);
		setLedAngle(i, 0, i * M_PI2 / 28, 0, i * M_PI2 / 28);
		setLedFreq(i, 0, -1, 0, -1);

		setLedAmplitude(7 - i, 7, 50, 50, 50);
		setLedAngle(7 - i, 7, (14 + i) * M_PI2 / 28, 0, (14 + i) * M_PI2 / 28);
		setLedFreq(7 - i, 7, -1, 0, -1);

	}

	for (int i = 1; i < 7; i++) {

		setLedAmplitude(7, i, 50, 50, 50);
		setLedAngle(7, i, (8 + i) * M_PI2 / 28, 0, (8 + i) * M_PI2 / 28);
		setLedFreq(7, i, -1, 0, -1);

		setLedAmplitude(0, 7 - i, 50, 50, 50);
		setLedAngle(0, 7 - i, (22 + i) * M_PI2 / 28, 0, (22 + i) * M_PI2 / 28);
		setLedFreq(0, 7 - i, -1, 0, -1);

	}

	// Second ring

	for (int i = 1; i < 7; i++) {

		setLedAmplitude(i, 1, 50, 50, 50);
		setLedAngle(i, 1, 0, 0, (i - 1) * M_PI2 / 20);
		setLedFreq(i, 1, 0, 0, 1);

		setLedAmplitude(7 - i, 6, 50, 50, 50);
		setLedAngle(7 - i, 6, 0, 0, (10 + i) * M_PI2 / 20);
		setLedFreq(7 - i, 6, 0, 0, 1);

	}

	for (int i = 2; i < 6; i++) {

		setLedAmplitude(6, i, 50, 50, 50);
		setLedAngle(6, i, 0, 0, (5 + i) * M_PI2 / 20);
		setLedFreq(6, i, 0, 0, 1);

		setLedAmplitude(1, 7 - i, 50, 50, 50);
		setLedAngle(1, 7 - i, 0, 0, (14 + i) * M_PI2 / 20);
		setLedFreq(1, 7 - i, 0, 0, 1);

	}

	// Third ring

	for (int i = 2; i < 6; i++) {

		setLedAmplitude(i, 2, 50, 50, 50);
		setLedAngle(i, 2, 0, 0, (i - 2) * M_PI2 / 12);
		setLedFreq(i, 2, 0, 0, 1);

		setLedAmplitude(7 - i, 5, 50, 50, 50);
		setLedAngle(7 - i, 5, 0, 0, (4 + i) * M_PI2 / 12);
		setLedFreq(7 - i, 5, 0, 0, 1);

	}

	for (int i = 3; i < 5; i++) {

		setLedAmplitude(5, i, 50, 50, 50);
		setLedAngle(5, i, 0, 0, (1 + i) * M_PI2 / 12);
		setLedFreq(5, i, 0, 0, 1);

		setLedAmplitude(2, 7 - i, 50, 50, 50);
		setLedAngle(2, 7 - i, 0, 0, (7 + i) * M_PI2 / 12);
		setLedFreq(2, 7 - i, 0, 0, 1);

	}

	// Inner four

	for (int i = 3; i < 5; i++) {

		setLedAmplitude(i, 3, 50, 50, 50);
		setLedAngle(i, 3, 0, 0, (i - 3) * M_PI2 / 4);
		setLedFreq(i, 3, 0, 0, 1);

		setLedAmplitude(7 - i, 4, 50, 50, 50);
		setLedAngle(7 - i, 4, 0, 0, (-1 + i) * M_PI2 / 4);
		setLedFreq(7 - i, 4, 0, 0, 1);

	}

	uint32_t then = 0;

	while (1) {

		uint32_t now = HAL_GetTick();
		if (now % 500 == 0 && now != then) {

			// Not really doing anything as everything is driven by timers,
			// but let's toggle a GPIO pin just for the hell of it.
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = T3_PRE;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = T3_CNT;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = T4_PRE;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = T4_CNT;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
