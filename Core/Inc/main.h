/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f405xx.h"
#include "stm32f4xx_hal_dac.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
void SystemClock_Config(void);
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
#define TIM2_PERIOD_CLOCKS 1680
#define APB1_TIM2_TIM14_FREQ 84000000
#define TIM2_REPETITION 5
#define M_RESET_Pin GPIO_PIN_13
#define M_RESET_GPIO_Port GPIOC
#define M_FF2_Pin GPIO_PIN_14
#define M_FF2_GPIO_Port GPIOC
#define M_FF1_Pin GPIO_PIN_15
#define M_FF1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_1
#define LED4_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOA
#define M_REF_Pin GPIO_PIN_5
#define M_REF_GPIO_Port GPIOA
#define M_CSOUT_Pin GPIO_PIN_6
#define M_CSOUT_GPIO_Port GPIOA
#define ADDRESS2_Pin GPIO_PIN_4
#define ADDRESS2_GPIO_Port GPIOC
#define ADDRESS1_Pin GPIO_PIN_5
#define ADDRESS1_GPIO_Port GPIOC
#define M_MODE_Pin GPIO_PIN_12
#define M_MODE_GPIO_Port GPIOC
#define M_PWM_Pin GPIO_PIN_3
#define M_PWM_GPIO_Port GPIOB
#define M_DIRO_Pin GPIO_PIN_4
#define M_DIRO_GPIO_Port GPIOB
#define M_DIR_Pin GPIO_PIN_8
#define M_DIR_GPIO_Port GPIOB
#define M_TACHO_Pin GPIO_PIN_9
#define M_TACHO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define CURRENT_SENSE_MIN_VOLT  0.0f
#define CURRENT_SENSE_MAX_VOLT  3.3f

#define SHUNT_RESISTANCE 0.075f
#define CURRENT_MEAS_GAIN 19.0f
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
