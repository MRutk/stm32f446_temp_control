/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TimerCheck_Pin GPIO_PIN_3
#define TimerCheck_GPIO_Port GPIOC
#define BT0_Pin GPIO_PIN_0
#define BT0_GPIO_Port GPIOA
#define BT0_EXTI_IRQn EXTI0_IRQn
#define BT1_Pin GPIO_PIN_1
#define BT1_GPIO_Port GPIOA
#define BT1_EXTI_IRQn EXTI1_IRQn
#define BT2_Pin GPIO_PIN_4
#define BT2_GPIO_Port GPIOA
#define BT2_EXTI_IRQn EXTI4_IRQn
#define SENSOR_Pin GPIO_PIN_5
#define SENSOR_GPIO_Port GPIOA
#define RedLED_Pin GPIO_PIN_8
#define RedLED_GPIO_Port GPIOA
#define BT3_Pin GPIO_PIN_3
#define BT3_GPIO_Port GPIOB
#define BT3_EXTI_IRQn EXTI3_IRQn
#define IN1_Pin GPIO_PIN_8
#define IN1_GPIO_Port GPIOB
#define IN2_Pin GPIO_PIN_9
#define IN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define Critical_temp 120
//int16_t Critical_temp = 120; 	//temp sensors critical temperature max. operational temperature is 125C

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
