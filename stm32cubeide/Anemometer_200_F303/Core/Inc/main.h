/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
//#include <math.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern bool readyData, readyCapture ;
#define CONVERSION_COUNT 400
#define REF_COUNT 100
#define MEASURE_COUNT 100
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern uint16_t adcBuffer[CONVERSION_COUNT];
extern uint16_t maxLevel, minLevel, ajustCount, haftConf, maxIndex, mesCount;
extern uint32_t captureTIM2, finishCapture;
extern float measArray[CONVERSION_COUNT];
extern char SndBuffer[200];
extern double maxLev, curLev, avgLevel, maxAmp, maxIdxAmp;
void ADC_complite(DMA_HandleTypeDef * hdma);
#define SHOW_DATA 100
#define START_TEXT "\n\rAnemometr200 init\n\r"
#define FINISH_TEXT "Init finish.\n\r"
#define ERROR_TEXT "Error, system reset\n\r"
#define AD5245 /* Автоматическая регулировка уровня от 0 до 254 */
#ifdef AD5245
	void AD5245level(uint8_t lev);
	#define AJUST_DELAY 1
	#define AD5245_I2C_ADDR (0x2C << 1)
	#define AD5245_WRITE    0x00
	#define AD5245_RESET    0x40
	#define AD5245_SHUTDOWN 0x20
	#define AD5245_I2C_PORT hi2c1
	#ifndef MY_DEFINES
	#define MY_DEFINES 1
	extern uint8_t currentLevel;
	#endif
#endif
//#define X9CXXX
#ifdef X9CXXX
	void levelUp(uint8_t channel, uint8_t lev, bool updn);
	#define AJUST_DELAY 5
	#define UP true
	#define DOWN false
#endif
#define ACURACY_LEVEL 100
#define NOMINAL_LEVEL 1800
#define MEASURMENT_DALAY 35000
//#define RAW_DATA_OUT

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
#define AnIn1_Pin GPIO_PIN_0
#define AnIn1_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOA
#define Z1Sel_Pin GPIO_PIN_0
#define Z1Sel_GPIO_Port GPIOB
#define Z2Sel_Pin GPIO_PIN_1
#define Z2Sel_GPIO_Port GPIOB
#define Z3Sel_Pin GPIO_PIN_2
#define Z3Sel_GPIO_Port GPIOB
#define Z4Sel_Pin GPIO_PIN_10
#define Z4Sel_GPIO_Port GPIOB
#define EthRst_Pin GPIO_PIN_11
#define EthRst_GPIO_Port GPIOB
#define Z1_Pin GPIO_PIN_8
#define Z1_GPIO_Port GPIOA
#define Z2_Pin GPIO_PIN_9
#define Z2_GPIO_Port GPIOA
#define Z3_Pin GPIO_PIN_10
#define Z3_GPIO_Port GPIOA
#define Z4_Pin GPIO_PIN_11
#define Z4_GPIO_Port GPIOA
#define Zero_Pin GPIO_PIN_15
#define Zero_GPIO_Port GPIOA
#define CS4_Pin GPIO_PIN_5
#define CS4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LED_PULSE HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
