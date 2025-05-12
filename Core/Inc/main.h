/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
extern I2C_HandleTypeDef hi2c1;
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
#define LED01_Pin GPIO_PIN_13
#define LED01_GPIO_Port GPIOC
#define LED02_Pin GPIO_PIN_14
#define LED02_GPIO_Port GPIOC
#define RC_CH01_Pin GPIO_PIN_0
#define RC_CH01_GPIO_Port GPIOA
#define RC_CH02_Pin GPIO_PIN_1
#define RC_CH02_GPIO_Port GPIOA
#define RC_CH03_Pin GPIO_PIN_2
#define RC_CH03_GPIO_Port GPIOA
#define RC_CH04_Pin GPIO_PIN_3
#define RC_CH04_GPIO_Port GPIOA
#define FLSH_CS_Pin GPIO_PIN_4
#define FLSH_CS_GPIO_Port GPIOA
#define WIFI_INT_Pin GPIO_PIN_0
#define WIFI_INT_GPIO_Port GPIOB
#define WIFI_CS_Pin GPIO_PIN_1
#define WIFI_CS_GPIO_Port GPIOB
#define BTN_01_Pin GPIO_PIN_2
#define BTN_01_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_10
#define BUZZER_GPIO_Port GPIOB
#define BTN_02_Pin GPIO_PIN_12
#define BTN_02_GPIO_Port GPIOB
#define BTN_03_Pin GPIO_PIN_13
#define BTN_03_GPIO_Port GPIOB
#define BTN_04_Pin GPIO_PIN_14
#define BTN_04_GPIO_Port GPIOB
#define ENC01_BTN_Pin GPIO_PIN_8
#define ENC01_BTN_GPIO_Port GPIOA
#define FSW_01_Pin GPIO_PIN_9
#define FSW_01_GPIO_Port GPIOA
#define FSW_02_Pin GPIO_PIN_10
#define FSW_02_GPIO_Port GPIOA
#define FSW_03_Pin GPIO_PIN_11
#define FSW_03_GPIO_Port GPIOA
#define FSW_04_Pin GPIO_PIN_12
#define FSW_04_GPIO_Port GPIOA
#define ENC02_BTN_Pin GPIO_PIN_15
#define ENC02_BTN_GPIO_Port GPIOA
#define BTN_05_Pin GPIO_PIN_3
#define BTN_05_GPIO_Port GPIOB
#define ENC01_DATA_Pin GPIO_PIN_4
#define ENC01_DATA_GPIO_Port GPIOB
#define ENC01_CLK_Pin GPIO_PIN_5
#define ENC01_CLK_GPIO_Port GPIOB
#define ENC02_DATA_Pin GPIO_PIN_6
#define ENC02_DATA_GPIO_Port GPIOB
#define ENC02_CLK_Pin GPIO_PIN_7
#define ENC02_CLK_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_8
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_9
#define I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
