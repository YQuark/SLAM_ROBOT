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

/* USER CODE BEGIN Private defines */
#define R2_A_Pin GPIO_PIN_0
#define R2_A_GPIO_Port GPIOA
#define R2_B_Pin GPIO_PIN_1
#define R2_B_GPIO_Port GPIOA
#define ADC_Pin GPIO_PIN_2
#define ADC_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_4
#define LED_GPIO_Port GPIOA
#define PS2_CLK_Pin GPIO_PIN_5
#define PS2_CLK_GPIO_Port GPIOA
#define PS2_DAT_Pin GPIO_PIN_6
#define PS2_DAT_GPIO_Port GPIOA
#define PS2_CMD_Pin GPIO_PIN_7
#define PS2_CMD_GPIO_Port GPIOA
#define PS2_CS_Pin GPIO_PIN_4
#define PS2_CS_GPIO_Port GPIOC
#define L1_DIR_Pin GPIO_PIN_8
#define L1_DIR_GPIO_Port GPIOE
#define L1_PWM_Pin GPIO_PIN_9
#define L1_PWM_GPIO_Port GPIOE
#define L2_DIR_Pin GPIO_PIN_10
#define L2_DIR_GPIO_Port GPIOE
#define L2_PWM_Pin GPIO_PIN_11
#define L2_PWM_GPIO_Port GPIOE
#define R1_DIR_Pin GPIO_PIN_12
#define R1_DIR_GPIO_Port GPIOE
#define R1_PWM_Pin GPIO_PIN_13
#define R1_PWM_GPIO_Port GPIOE
#define R2_PWM_Pin GPIO_PIN_14
#define R2_PWM_GPIO_Port GPIOE
#define R2_DIR_Pin GPIO_PIN_15
#define R2_DIR_GPIO_Port GPIOE
#define R1_A_Pin GPIO_PIN_12
#define R1_A_GPIO_Port GPIOD
#define R1_B_Pin GPIO_PIN_13
#define R1_B_GPIO_Port GPIOD
#define L2_A_Pin GPIO_PIN_6
#define L2_A_GPIO_Port GPIOC
#define L2_B_Pin GPIO_PIN_7
#define L2_B_GPIO_Port GPIOC
#define L1_A_Pin GPIO_PIN_15
#define L1_A_GPIO_Port GPIOA
#define L1_B_Pin GPIO_PIN_3
#define L1_B_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_0
#define IMU_INT_GPIO_Port GPIOE
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
