/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum State {
	STOP,
	FAN_IDLE,
	FAN_ON,
	BLOCK,
	WORKING,
	HEATER_BLOWING,
	FAN_OFF
};
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_Temp_Pin GPIO_PIN_5
#define ADC_Temp_GPIO_Port GPIOA
#define Thermostat_Pin GPIO_PIN_8
#define Thermostat_GPIO_Port GPIOE
#define Thermostat_EXTI_IRQn EXTI9_5_IRQn
#define Relay_Pin GPIO_PIN_11
#define Relay_GPIO_Port GPIOE
#define Heater_Ctrl_Pin GPIO_PIN_12
#define Heater_Ctrl_GPIO_Port GPIOE
#define PWM_Heater_Pin GPIO_PIN_13
#define PWM_Heater_GPIO_Port GPIOE
#define PWM_Fan_Pin GPIO_PIN_14
#define PWM_Fan_GPIO_Port GPIOE
#define FC_Ctrl_Pin GPIO_PIN_1
#define FC_Ctrl_GPIO_Port GPIOD
#define Fan_Ctrl_Pin GPIO_PIN_3
#define Fan_Ctrl_GPIO_Port GPIOD
#define FC_Failure_Pin GPIO_PIN_4
#define FC_Failure_GPIO_Port GPIOD
#define FC_Failure_EXTI_IRQn EXTI4_IRQn

/* USER CODE BEGIN Private defines */
#define coeff_temp 5
#define coeff_speed 0.2
#define MAX_CAP 5000
#define CRC16_POLYNOMIAL 0x1021
#define CRC8_POLYNOMIAL 0x31
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
