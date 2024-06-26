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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
//#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <arm_math.h>
#include <arm_const_structs.h>
#include <arm_chilbert_f32.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define CONVERSION_COUNT 512
#define MEASURE_COUNT 16
#define REF_COUNT 60

extern float refArray[];
extern bool readyData, readyCapture ;
extern uint16_t adcBuffer[CONVERSION_COUNT], maxLevel, minLevel, ajustCount, haftConf, mesCount;
extern float32_t measArray[CONVERSION_COUNT], convArray[CONVERSION_COUNT + REF_COUNT];
extern float curLev, avgLevel, maxAmp;
extern uint32_t maxIdxAmp;

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
	extern uint8_t currentLevel;
#endif
#define ACURACY_LEVEL 100
#define NOMINAL_LEVEL 1800
#define MEASURMENT_DALAY 38000
#define SAMPLE_RATE 32.258f

#define RAW_DATA_OUT

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define LED_PULSE HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define AIn_Pin GPIO_PIN_0
#define AIn_GPIO_Port GPIOA
#define selZ1_Pin GPIO_PIN_11
#define selZ1_GPIO_Port GPIOB
#define selZ2_Pin GPIO_PIN_12
#define selZ2_GPIO_Port GPIOB
#define selZ3_Pin GPIO_PIN_13
#define selZ3_GPIO_Port GPIOB
#define selZ4_Pin GPIO_PIN_14
#define selZ4_GPIO_Port GPIOB
#define Z1_Pin GPIO_PIN_8
#define Z1_GPIO_Port GPIOA
#define Z2_Pin GPIO_PIN_9
#define Z2_GPIO_Port GPIOA
#define Z3_Pin GPIO_PIN_10
#define Z3_GPIO_Port GPIOA
#define Z4_Pin GPIO_PIN_11
#define Z4_GPIO_Port GPIOA
#define SCL_Pin GPIO_PIN_15
#define SCL_GPIO_Port GPIOA
#define capture_Pin GPIO_PIN_3
#define capture_GPIO_Port GPIOB
#define TX_Pin GPIO_PIN_6
#define TX_GPIO_Port GPIOB
#define RX_Pin GPIO_PIN_7
#define RX_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
