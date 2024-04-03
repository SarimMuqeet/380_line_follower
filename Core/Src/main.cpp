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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Adafruit_TCS34725.h"
#include "helpers.h"
#include "statemachine.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cstring>
#include <cmath>
//#include <chrono>

using namespace std;
//using namespace std::chrono;

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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t debugStatus=0;
uint8_t state=1;

//potential cause of slow response (motors) when detecting colour
Adafruit_TCS34725 tcsFL = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcsFC = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcsFR = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

state_c currState = IDLE;
uint32_t leftFW = (uint32_t)(COUNTER_PERIOD*LEFT_FW);
uint32_t rightFW = (uint32_t)(COUNTER_PERIOD*RIGHT_FW);
uint32_t leftBW = (uint32_t)(COUNTER_PERIOD*LEFT_BW);
uint32_t rightBW = (uint32_t)(COUNTER_PERIOD*RIGHT_BW);

uint32_t leftTurn = (uint32_t)(COUNTER_PERIOD*LEFT_FW);
uint32_t rightTurn = (uint32_t)(COUNTER_PERIOD*RIGHT_FW);


//PD Global vars
//double prev_error = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
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
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  uint8_t tcsFL_addr = 1;
  uint8_t tcsFC_addr = 2;
  uint8_t tcsFR_addr = 2;

  if (tcsFL.begin(TCS34725_ADDRESS, &hi2c1, tcsFL_addr) && tcsFC.begin(TCS34725_ADDRESS, &hi2c3) && tcsFR.begin(TCS34725_ADDRESS, &hi2c1, tcsFR_addr)) {
	  debugStatus=0x55; //Found sensor
  } else {
	  debugStatus=0xAA; //Sensor not found
	  while(1);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
//  auto start = high_resolution_clock::now();

  moveForward(&leftFW, &rightFW);
  HAL_Delay(2000);

  moveLeft(&leftTurn);
  HAL_Delay(2000);

  moveBackward(&leftBW, &rightBW);
  HAL_Delay(2000);

  moveRight(&rightTurn);
  HAL_Delay(2000);

  stop();

//  release();
//  grab();


  while (1)
  {

//	  runStateMachine();

//	  uint16_t r1, g1, b1, c1;
//	  uint16_t r2, g2, b2, c2;
//	  uint16_t r3, g3, b3, c3;
//
//
//	  const uint16_t REDLINE[3] = {139, 46, 75};
//	  const uint16_t GREENZONE[3] = {43, 88, 125};
//	  const uint16_t BULLSEYE_BLUE[3] = {31, 70, 164};
//	  const uint16_t WOOD[3] = {77, 83, 88};
//
//
//    // Get RGB Values
//	  getRawData_noDelay(&tcsFL, &r1, &g1, &b1, &c1);
//	  getRawData_noDelay(&tcsFC, &r2, &g2, &b2, &c2);
//	  getRawData_noDelay(&tcsFR, &r3, &g3, &b3, &c3);
//
//	  /* New Format:
//	   * 	FL = 1
//	   * 	FC = 2
//	   * 	FR = 3
//	   */
////
////	  double colorReading1 = euclideanDistance(&GREEN_R, &GREEN_G, &GREEN_B, &r1, &g1, &b1); //goes from 0 to 441.67
////	  double colorReading2 = euclideanDistance(&GREEN_R, &GREEN_G, &GREEN_B, &r2, &g2, &b2); //goes from 0 to 441.67
////	  double colorReading3 = euclideanDistance(&GREEN_R, &GREEN_G, &GREEN_B, &r3, &g3, &b3); //goes from 0 to 441.67
//
//	  // Testing movement
//    uint32_t dutyCycle = 0.32*65535;
//
//    moveForward(&dutyCycle);
//    HAL_Delay(1000);
//    stop();
//
//    moveLeft(&dutyCycle);
//    HAL_Delay(1000);
//    stop();
//
//    moveRight(&dutyCycle);
//    HAL_Delay(1000);
//    stop();
//
//    moveBackward(&dutyCycle);
//    HAL_Delay(1000);
//    stop();
//
//    // Test Claw
//    grab();
//
//    release();
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 25;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13; //B1_Pin
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* tcs.getRawData() does a delay(Integration_Time) after the sensor readout.
We don't need to wait for the next integration cycle because we receive an interrupt when the integration cycle is complete*/
void getRawData_noDelay(Adafruit_TCS34725 *tcs, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = tcs->read16(TCS34725_CDATAL);
  *r = tcs->read16(TCS34725_RDATAL);
  *g = tcs->read16(TCS34725_GDATAL);
  *b = tcs->read16(TCS34725_BDATAL);

  HAL_Delay(1);

  uint32_t sum = *c;

    // Avoid divide by zero errors ... if clear = 0 return black
    if (c == 0) {
      *r = *g = *b = 0;
      return;
    }

    *r = (float)*r / sum * 255.0;
    *g = (float)*g / sum * 255.0;
    *b = (float)*b / sum * 255.0;
}

// If sensor reading is identical to RGB1 output is 100
int16_t euclideanDistance(uint16_t *r, uint16_t *g, uint16_t *b, const uint16_t RGB1[3], const uint16_t RGB2[3]) {
	int16_t deltaR1 = *r - RGB1[0];
	int16_t deltaG1 = *g - RGB1[1];
	int16_t deltaB1 = *b - RGB1[2];
	int16_t deltaR2 = *r - RGB2[0];
	int16_t deltaG2 = *g - RGB2[1];
	int16_t deltaB2 = *b - RGB2[2];

    double distance1 = sqrt(pow(deltaR1, 2)+ pow(deltaG1, 2) + pow(deltaB1, 2));
    double distance2 = sqrt(pow(deltaR2, 2)+ pow(deltaG2, 2) + pow(deltaB2, 2));

    double totalDistance = distance1+distance2;

    return ((distance2/totalDistance)*100);
}

// Movement Functions
void moveForward(uint32_t *dutyCycleL, uint32_t *dutyCycleR){
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, *dutyCycleL);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *dutyCycleR);
}

void moveBackward(uint32_t *dutyCycleL, uint32_t *dutyCycleR){
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, *dutyCycleL);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *dutyCycleR);
}

void moveLeft(uint32_t *dutyCycle){
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
//  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, *dutyCycle);
//  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, &0);

//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, *dutyCycle);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *dutyCycle);


}

void moveRight(uint32_t *dutyCycle){
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
//  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, *dutyCycle);
//  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *dutyCycle);

//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, *dutyCycle);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *dutyCycle);
}

void stop(){
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
}

// Claw Functions
void grab(){
  uint32_t pulseWidth = 0.075*65535;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulseWidth);
  HAL_Delay(500);


}

void release(){
  uint32_t pulseWidth = 0.05*65535;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulseWidth);
  HAL_Delay(1000);
}


//stateMachine
void runStateMachine(){
	switch(currState) {

	case IDLE:
		idle();
		break;

	case SEARCH_LEGO:
//		moveForward(&dutyCycleL, &dutyCycleR);
		search_lego();
		break;

	case SECURE_LEGO:
		stop();
		grab();
		currState = SEARCH_SAFE;
		break;

	case SEARCH_SAFE:
		search_safe();
		break;

	case DROPOFF_LEGO:
		stop();
		release();
		currState = RETURN_TO_START;
		break;

	case RETURN_TO_START:
		return_to_start();
		break;

	}


}

void idle() {
	int state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	//button is active low apparently
	if(state == GPIO_PIN_RESET) {
//		grab();
		release();
		currState = SEARCH_LEGO;
	}
}

void search_lego() {
	line_follow_fw();

	int16_t dist2 = euclideanDistance(&r2, &g2, &b2, BULLSEYE_BLUE, REDLINE);

	//detect blue, check bullseye
	if(dist2 > 75) {
		currState = SECURE_LEGO;
	}

    char str[64] = {0};
    sprintf(str, "Blue dist2 %d\n", dist2);
    HAL_UART_Transmit(&huart2, (uint8_t*)str, sizeof (str), 10);
}


void search_safe() {
//	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

//	char str[64] = {0};
//	sprintf(str, "SEARCH SAFE\n");
//	HAL_UART_Transmit(&huart2, (uint8_t*)str, sizeof (str), 10);

	line_follow_bw();

	//detect green
	int16_t dist1 = euclideanDistance(&r2, &g2, &b2, GREENZONE, REDLINE);
	int16_t dist3 = euclideanDistance(&r3, &g3, &b3, GREENZONE, REDLINE);

	//detect green, check for safe zone
	if(dist1 > 75 || dist3 > 75) {
		currState = DROPOFF_LEGO;
	}
}

void return_to_start() {
	line_follow_bw();

	int16_t dist1 = euclideanDistance(&r1, &g1, &b1, REDLINE, WOOD);
	int16_t dist2 = euclideanDistance(&r2, &g2, &b2, REDLINE, WOOD);
	int16_t dist3 = euclideanDistance(&r3, &g3, &b3, REDLINE, WOOD);

	if((dist1 > 70) && (dist2 > 70) && (dist3 > 70)){
		stop();
		currState = IDLE;
	}

}


//verified
void line_follow_fw() {
	

	getRawData_noDelay(&tcsFL, &r1, &g1, &b1, &c1);
	getRawData_noDelay(&tcsFC, &r2, &g2, &b2, &c2);
	getRawData_noDelay(&tcsFR, &r3, &g3, &b3, &c3);

  int16_t dist1 = euclideanDistance(&r1, &g1, &b1, REDLINE, WOOD);
//  int16_t dist2 = euclideanDistance(&r2, &g2, &b2, REDLINE, WOOD);
  int16_t dist3 = euclideanDistance(&r3, &g3, &b3, REDLINE, WOOD);

	//TODO: PID, euclidean distance
	if(dist1 > 70){
		//only until FC sees red again
		//vary DC based on PID
		moveLeft(&leftTurn);
//		HAL_Delay(10);
//		stop();
	} else if(dist3 > 70) {
		moveRight(&rightTurn);
//		HAL_Delay(10);
//		stop();
	} else{
//		stop();
		moveForward(&leftFW, &rightFW);
//		HAL_Delay(1000);
//		stop();
//		HAL_Delay(1000);
	}

//    char str[64] = {0};
//    sprintf(str, "Euclidean Distances: Dist1 %d\n Dist3 %d\n \n", dist1, dist3);
//    HAL_UART_Transmit(&huart2, (uint8_t*)str, sizeof (str), 10);
}

void line_follow_bw() {
	getRawData_noDelay(&tcsFL, &r1, &g1, &b1, &c1);
	getRawData_noDelay(&tcsFC, &r2, &g2, &b2, &c2);
	getRawData_noDelay(&tcsFR, &r3, &g3, &b3, &c3);

	int16_t dist1 = euclideanDistance(&r1, &g1, &b1, REDLINE, WOOD);
	int16_t dist3 = euclideanDistance(&r3, &g3, &b3, REDLINE, WOOD);

	if(dist1 > 70){
		moveRight(&rightTurn);
	} else if(dist3 > 70) {
		moveLeft(&leftTurn);
	} else{
		moveBackward(&leftBW, &rightBW);
	}

}

void print(char *str) {
//    sprintf(str, "Euclidean Distances: Dist1 %d\n Dist3 %d\n \n", dist1, dist3);
//    HAL_UART_Transmit(&huart2, (uint8_t*)str, sizeof (str), 10);
}

//// Line follow PD test
//void line_follow_fw_pd() {
//	double Kp = 0.1;
//  double Kd = 0.01;
//
//	getRawData_noDelay(&tcsFL, &r1, &g1, &b1, &c1);
//	getRawData_noDelay(&tcsFC, &r2, &g2, &b2, &c2);
//	getRawData_noDelay(&tcsFR, &r3, &g3, &b3, &c3);
//
//  int16_t dist1 = euclideanDistance(&r1, &g1, &b1, REDLINE, WOOD);
//  //int16_t dist2 = euclideanDistance(&r2, &g2, &b2, &WOOD, &REDLINE)
//  int16_t dist3 = euclideanDistance(&r3, &g3, &b3, REDLINE, WOOD);
//
//  double error = 0 - dist3 + dist1;
//
//  auto end = high_resolution_clock::now();
//
//  double P = error * Kp;
//  double D = Kd*(error - prev_error)/(end - start);
//
//  double control = P + D;
//
//  double dutyL = dutyCycle + control;
//  double dutyR = dutyCycle - control;
//
//	if(dist1 > 80){
//		moveRight(&dutyR);
//	} else if(dist3 > 80) {
//		moveLeft(&dutyL);
//	} else{
//    moveForward(&dutyCycle);
//  }
//
//  auto start = high_resolution_clock::now();
//  prev_error = error;
//
//}



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
