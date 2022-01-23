/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define TRUE 1
#define FALSE 0
#define timeAdge 2
#define Z1Receive 1
#define Z2Receive 2
#define Z3Receive 10
#define Z4Receive 11
#define setZ1transmit GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF8_0)) | (GPIO_CRH_CNF8_1 | GPIO_CRH_MODE8_1)
#define setZ2transmit GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF9_0)) | (GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_1)
#define setZ3transmit GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF10_0)) | (GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_1)
#define setZ4transmit GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF11_0)) | (GPIO_CRH_CNF11_1 | GPIO_CRH_MODE11_1)
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
#define senserInput_Pin GPIO_PIN_0
#define senserInput_GPIO_Port GPIOA
#define Z1Receive_Pin GPIO_PIN_1
#define Z1Receive_GPIO_Port GPIOB
#define Z2Receive_Pin GPIO_PIN_2
#define Z2Receive_GPIO_Port GPIOB
#define Z3Receive_Pin GPIO_PIN_10
#define Z3Receive_GPIO_Port GPIOB
#define Z4Receive_Pin GPIO_PIN_11
#define Z4Receive_GPIO_Port GPIOB
#define Z1_Pin GPIO_PIN_8
#define Z1_GPIO_Port GPIOA
#define Z2_Pin GPIO_PIN_9
#define Z2_GPIO_Port GPIOA
#define Z3_Pin GPIO_PIN_10
#define Z3_GPIO_Port GPIOA
#define Z4_Pin GPIO_PIN_11
#define Z4_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
uint16_t currentMode;
uint16_t Z12, Z21, Z23, Z32, Z34, Z43, Z41, Z14;
uint32_t sumCounter2, fastCounter;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
bool readyFlag, firstFlag;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
