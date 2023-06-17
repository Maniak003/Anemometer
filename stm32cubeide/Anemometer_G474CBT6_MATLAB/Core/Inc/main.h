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
#include <stdlib.h>
#include <stdio.h>
#include "maxEnvHilbert.h"
#include "sntp.h"
#include "w5500.h"
#include "socket.h"
#include "dhcp.h"
#define BME280_ENABLE
#ifdef BME280_ENABLE
#include "BME280.h"
#endif

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define CONVERSION_COUNT 1024
#define MEASURE_COUNT 16
#define REF_COUNT 140

extern float refArray[], measArray[CONVERSION_COUNT], convArray[CONVERSION_COUNT + REF_COUNT], curLev, avgLevel, maxAmp, maxIdxAmp;
extern bool readyData, readyCapture ;
extern uint16_t adcBuffer[CONVERSION_COUNT], maxLevel, minLevel, ajustCount, haftConf, mesCount;
extern uint32_t sumCaptureTIM2;

#define START_TEXT "\n\rAnemometr200 init\n\r"
#define FINISH_TEXT "Init finish.\n\r"
#define ERROR_TEXT "Error, system reset\n\r"
#define BME280_INIT_TEXT "BM280 init.."
#define AD5245_INIT_TEXT "AD5245 init.\n\r"
#define W5500_INIT_TEXT "W5500 init.."
#define ADC_TMR_INIT_TEXT "ADC, TMRs init.."
#define INIT_OK "Ok\n\r"
#define AD5245 /* Автоматическая регулировка уровня от 0 до 254 */
#ifdef AD5245
	void AD5245level(uint8_t lev);
	#define AJUST_DELAY 0
	#define AD5245_I2C_ADDR (0x2C << 1)
	#define AD5245_WRITE    0x00
	#define AD5245_RESET    0x40
	#define AD5245_SHUTDOWN 0x20
	#define AD5245_I2C_PORT hi2c1
	extern uint8_t currentLevel;
#endif
#define ACURACY_LEVEL 50
#define ADC_RESOLUTION 1024
#define NOMINAL_LEVEL 450
#define MEASURMENT_DELAY 40200
#define DISTANCE 203.5f
#define ABS_ZERRO 273.15f
/*
  Расчет интервала АЦП преобразования в uS
  1 / (timer clock / clock prescaler) * (sampling time + conversion time)
*/
#define SAMPLE_RATE 1.0f / (170.0f / 4.0f) * (2.5f + 10.5f)


//#define RAW_DATA_OUT

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
#define W5500_RST_Pin GPIO_PIN_3
#define W5500_RST_GPIO_Port GPIOA
#define W5500_CS_Pin GPIO_PIN_4
#define W5500_CS_GPIO_Port GPIOA
#define W5500_SCK_Pin GPIO_PIN_5
#define W5500_SCK_GPIO_Port GPIOA
#define W5500_MISO_Pin GPIO_PIN_6
#define W5500_MISO_GPIO_Port GPIOA
#define W5500_MOSI_Pin GPIO_PIN_7
#define W5500_MOSI_GPIO_Port GPIOA
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
#define TX_Pin GPIO_PIN_6
#define TX_GPIO_Port GPIOB
#define RX_Pin GPIO_PIN_7
#define RX_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* For W5500*/
#define DHCP_SOCKET     0
#define DNS_SOCKET      1
#define TCP_SOCKET		2
#define _DHCP_DEBUG_

//#define ZABBIX_DEBUG
#define DATA_BUF_SIZE   1024
#define ZABBIX_ENABLE
#define ZABBIXAGHOST	"Anemometer04"  // Default hostname.
#define ZABBIXPORT		10051
#define ZABBIXMAXLEN	128
#define MAC_ADDRESS		0x00, 0x11, 0x22, 0x33, 0x44, 0xf0

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
