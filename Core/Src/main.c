/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define coeff_temp 5
#define coeff_speed 0.2
#define MAX_CAP 5000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart6;

osThreadId TaskReceiveDataHandle;
osThreadId TaskHeaterHandle;
osThreadId TaskFanHandle;
osThreadId TaskSendDataHandle;
/* USER CODE BEGIN PV */

//uint32_t adc_value, adc_buffer, sum, adc_average;
//uint16_t count = 0;
//uint8_t flag_avg = 0, flag_adc = 0;
uint8_t flag_value = 0, flag_speed = 0, request = 0;

float temp, speed, temp_set, speed_set;
uint16_t pwm_temp, pwm_speed;

extern float last_error_pres, integrated_error_pres;
extern uint32_t timerPID_pres;

//uint8_t flag_heater,flag_fan, flag_fc;
//uint8_t heater_failure,fan_failure, fc_failure;

char Txdata[50];
char rx_buffer[30];
//char pressureRx[30];
uint8_t rx_index = 0;
uint8_t rx_data;

PidParameter PID_TEMP = { 15, 5, 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART6_UART_Init(void);
void StartTaskReceiveData(void const *argument);
void StartTaskHeater(void const *argument);
void StartTaskFan(void const *argument);
void StartTaskSendData(void const *argument);

/* USER CODE BEGIN PFP */
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//	// time Conv = (480+15)/180Mhz = 2.75 us
//	adc_value = adc_buffer;
//	temp = (float) adc_value * coeff_temp;
//
//	if (count == 50) {
//		adc_average = sum / count;
//		temp = (float) adc_average * coeff_temp;
//		sum = 0;
//		count = 0;
////		flag_avg = 1;
//	} else {
//		sum += adc_buffer;
//		count++;
//	}
////	flag_adc = 1;
//}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == Thermostat_Pin) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
		HAL_GPIO_WritePin(Heater_Ctrl_GPIO_Port, Heater_Ctrl_Pin, 0);
	}
	if (GPIO_Pin == Relay_Pin) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
		HAL_GPIO_WritePin(Fan_Ctrl_GPIO_Port, Fan_Ctrl_Pin, 0);
	}
	if (GPIO_Pin == FC_Failure_Pin) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
		HAL_GPIO_WritePin(FC_Ctrl_GPIO_Port, FC_Ctrl_Pin, 0);
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	if (huart->Instance == huart6.Instance) {
//		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
		if (!rx_index) {
			for (int i = 0; i < 20; i++) {
				rx_buffer[i] = '\0';
			}
		}
		if (rx_data != '\n' && rx_data != '\r' && rx_data != '/') {
			rx_buffer[rx_index++] = rx_data;
		} else {
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//			HAL_UART_Transmit(&huart6, (uint8_t*)rx_buffer, strlen(rx_buffer), HAL_MAX_DELAY);
			flag_value = 1;
			rx_index = 0;
		}

		HAL_UART_Receive_IT(&huart6, &rx_data, 1);
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_USART6_UART_Init();
	/* USER CODE BEGIN 2 */
//	HAL_ADC_Start_DMA(&hadc1, &adc_buffer, 1);
	HAL_UART_Receive_IT(&huart6, &rx_data, 1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of TaskReceiveData */
	osThreadDef(TaskReceiveData, StartTaskReceiveData, osPriorityNormal, 0,
			128);
	TaskReceiveDataHandle = osThreadCreate(osThread(TaskReceiveData), NULL);

	/* definition and creation of TaskHeater */
	osThreadDef(TaskHeater, StartTaskHeater, osPriorityNormal, 0, 128);
	TaskHeaterHandle = osThreadCreate(osThread(TaskHeater), NULL);

	/* definition and creation of TaskFan */
	osThreadDef(TaskFan, StartTaskFan, osPriorityNormal, 0, 128);
	TaskFanHandle = osThreadCreate(osThread(TaskFan), NULL);

	/* definition and creation of TaskSendData */
	osThreadDef(TaskSendData, StartTaskSendData, osPriorityNormal, 0, 128);
	TaskSendDataHandle = osThreadCreate(osThread(TaskSendData), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
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
	RCC_OscInitStruct.PLL.PLLM = 15;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
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
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 9 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Heater_Ctrl_GPIO_Port, Heater_Ctrl_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
			GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | FC_Ctrl_Pin | Fan_Ctrl_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : Thermostat_Pin Relay_Pin */
	GPIO_InitStruct.Pin = Thermostat_Pin | Relay_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : Heater_Ctrl_Pin */
	GPIO_InitStruct.Pin = Heater_Ctrl_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Heater_Ctrl_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PD13 PD14 PD15 FC_Ctrl_Pin
	 Fan_Ctrl_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | FC_Ctrl_Pin
			| Fan_Ctrl_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : FC_Failure_Pin */
	GPIO_InitStruct.Pin = FC_Failure_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(FC_Failure_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
float readTemp(uint32_t timeout) {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, timeout);
	float Temp = (float) HAL_ADC_GetValue(&hadc1) / 4095 * 3.3 * coeff_temp;
	HAL_ADC_Stop(&hadc1);
	return Temp;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTaskReceiveData */
/**
 * @brief  Function implementing the TaskReceiveData thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskReceiveData */
void StartTaskReceiveData(void const *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		if (flag_value) {
			char num[30];
			uint8_t num_index = 0;
			while (rx_buffer[num_index] != '\0') {
				num[num_index] = rx_buffer[num_index + 1];
				num_index++;
			}
			switch (rx_buffer[0]) {
			case 't': {
				temp_set = atof(num);
				last_error_pres = 0;
				integrated_error_pres = 0;
				timerPID_pres = HAL_GetTick();
				break;
			}
			case 'v': {
				flag_speed = 1;
				speed_set = atof(num);
				break;
			}
			case 'h': //heater control
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, atoi(num)); //green
				HAL_GPIO_WritePin(Heater_Ctrl_GPIO_Port, Heater_Ctrl_Pin,
						atoi(num));
			}
			case 'c': //frequency converter control
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, atoi(num)); //yellow
				HAL_GPIO_WritePin(FC_Ctrl_GPIO_Port, FC_Ctrl_Pin, atoi(num));
			}
			case 'f': //fan control
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, atoi(num)); //red
				HAL_GPIO_WritePin(Fan_Ctrl_GPIO_Port, Fan_Ctrl_Pin, atoi(num));
			}
			case 'p': {
				PID_TEMP.Kp = atof(num);
				break;
			}
			case 'i': //
			{
				PID_TEMP.Ki = atof(num);
				break;
			}
			case 'd': //
			{
				PID_TEMP.Kd = atof(num);
				break;
			}
			case 'r': {
				request = 1;
			}
			}
			flag_value = 0;
		}

		osDelay(500);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskHeater */
/**
 * @brief Function implementing the TaskHeater thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskHeater */
void StartTaskHeater(void const *argument) {
	/* USER CODE BEGIN StartTaskHeater */
	/* Infinite loop */
	for (;;) {
		if (HAL_GPIO_ReadPin(Heater_Ctrl_GPIO_Port, Heater_Ctrl_Pin)
				&& HAL_GPIO_ReadPin(Fan_Ctrl_GPIO_Port, Fan_Ctrl_Pin)) {
			if (!HAL_GPIO_ReadPin(Thermostat_GPIO_Port, Thermostat_Pin)) {
				float sum = 0;
				for (uint8_t i = 0; i < 10; i++) {
					sum += readTemp(1000);
				}
				temp = sum / 10;
				if (abs(temp - temp_set) > 2) {
					pwm_temp = PID_Calc(PID_TEMP, temp, temp_set);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_temp);
//					osDelay(1000);
				} else {
					osDelay(60000);
				}

			} else {
				osDelay(300000);
			}
		}
		osDelay(1000);
	}
	/* USER CODE END StartTaskHeater */
}

/* USER CODE BEGIN Header_StartTaskFan */
/**
 * @brief Function implementing the TaskFan thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskFan */
void StartTaskFan(void const *argument) {
	/* USER CODE BEGIN StartTaskFan */
	/* Infinite loop */
	for (;;) {
		if (HAL_GPIO_ReadPin(Fan_Ctrl_GPIO_Port, Fan_Ctrl_Pin)) {
			if (!HAL_GPIO_ReadPin(Relay_GPIO_Port, Relay_Pin)
					&& !HAL_GPIO_ReadPin(FC_Failure_GPIO_Port,
					FC_Failure_Pin)) {
				if (flag_speed) {
					pwm_speed = (uint32_t) (speed_set * coeff_speed);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_speed);
					flag_speed = 0;
				}
			}
		}
		osDelay(5000);
	}
	/* USER CODE END StartTaskFan */
}

/* USER CODE BEGIN Header_StartTaskSendData */
/**
 * @brief Function implementing the TaskSendData thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskSendData */
void StartTaskSendData(void const *argument) {
	/* USER CODE BEGIN StartTaskSendData */
	/* Infinite loop */
	for (;;) {
		if (request) {
			uint8_t state_heater = HAL_GPIO_ReadPin(Thermostat_GPIO_Port,
			Thermostat_Pin);
			uint8_t state_fan = HAL_GPIO_ReadPin(Relay_GPIO_Port, Relay_Pin);
			uint8_t state_fc = HAL_GPIO_ReadPin(FC_Failure_GPIO_Port,
			FC_Failure_Pin);

			float sum = 0;
			for (uint8_t i = 0; i < 10; i++) {
				sum += readTemp(1000);
			}
			temp = sum / 10;

//			temp = readTemp(1000);

			snprintf(Txdata, sizeof(Txdata), "%.1f %d %d %d %d\n", temp,
					(uint16_t) (pwm_temp / PWM_MAX * MAX_CAP), state_heater,
					state_fan, state_fc);
			HAL_UART_Transmit(&huart6, (uint8_t*) Txdata, strlen(Txdata),
			HAL_MAX_DELAY);
			request = 0;
		}
		osDelay(1000);
	}
	/* USER CODE END StartTaskSendData */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
