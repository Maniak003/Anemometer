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
#include "wizchip_conf.h"
#include "dhcp.h"
#include "dns.h"
#include "socket.h"
#include <stdlib.h>
#include "stdarg.h"
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
uint32_t runFlag;
uint16_t front_sum, resulMeass;
bool readyFlag;

uint16_t C_13, C_24;

union {
	float f;
	uint32_t u;
} DX1;
union {
	float f;
	uint32_t u;
} DX2;
union {
	float f;
	uint32_t u;
} DY1;
union {
	float f;
	uint32_t u;
} DY2;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define SYSTICK_DISABLE
#define TRUE 1
#define FALSE 0
#define START_CAPTURE HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
#define STOP_CAPTURE HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1); HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
#define CALIBRATE_START 35000
#define Z1Receive 1
#define Z2Receive 2
#define Z3Receive 3
#define Z4Receive 4
#define Z12Comparator 14
#define Z34Comparator 13
#define receiversOff GPIOB->ODR &= ~((1 << Z1Receive) | (1 << Z2Receive) | (1 << Z3Receive) | (1 << Z4Receive)); \
	GPIOC->ODR |= ((1 << Z34Comparator) |  (1 << Z12Comparator));
#define setZ1receive GPIOB->ODR |= (1 << Z1Receive); GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8)) | GPIO_CRH_CNF8_0;\
	GPIOC->ODR &= ~(1 << Z12Comparator);
#define setZ2receive GPIOB->ODR |= (1 << Z2Receive); GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9)) | GPIO_CRH_CNF9_0;\
	GPIOC->ODR &= ~(1 << Z12Comparator);
#define setZ3receive GPIOB->ODR |= (1 << Z3Receive); GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10)) | GPIO_CRH_CNF10_0;\
	GPIOC->ODR &= ~(1 << Z34Comparator);
#define setZ4receive GPIOB->ODR |= (1 << Z4Receive); GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF11 | GPIO_CRH_MODE11)) | GPIO_CRH_CNF11_0;\
	GPIOC->ODR &= ~(1 << Z34Comparator);

//#define LED_PULSE LED_GPIO_Port->BSRR = (uint32_t)LED_Pin; LED_GPIO_Port->BSRR = (uint32_t)LED_Pin << 16u;
#define LED_PULSE HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);;
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Z34_Pin GPIO_PIN_13
#define Z34_GPIO_Port GPIOC
#define Z12_Pin GPIO_PIN_14
#define Z12_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOA
#define Z1_Pin GPIO_PIN_1
#define Z1_GPIO_Port GPIOB
#define Z2_Pin GPIO_PIN_2
#define Z2_GPIO_Port GPIOB
#define SCSN_Pin GPIO_PIN_12
#define SCSN_GPIO_Port GPIOB
#define OUTZ1_Pin GPIO_PIN_8
#define OUTZ1_GPIO_Port GPIOA
#define OUTZ2_Pin GPIO_PIN_9
#define OUTZ2_GPIO_Port GPIOA
#define OUTZ3_Pin GPIO_PIN_10
#define OUTZ3_GPIO_Port GPIOA
#define OUTZ4_Pin GPIO_PIN_11
#define OUTZ4_GPIO_Port GPIOA
#define nRst_Pin GPIO_PIN_12
#define nRst_GPIO_Port GPIOA
#define Z3_Pin GPIO_PIN_3
#define Z3_GPIO_Port GPIOB
#define Z4_Pin GPIO_PIN_4
#define Z4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define INIT_FINISH_TEXT "\n\rFinish init...\n\r"
#define INIT_START_TEXT "\n\rStart init...\n\r"
#define DHCP_ERROR_ASSIGN "\r\nIP was not assigned :(\r\n"
#define COUNT_FRONT 40

/* For W5500*/
#define DHCP_SOCKET     0
#define DNS_SOCKET      1
#define TCP_SOCKET		2
#define DHCP_TRY_CNT	1000
#define _DHCP_DEBUG_

//#define ZABBIX_DEBUG
//#define ZABBIX_ENABLE
#define ZABBIXAGHOST	"Anemometer"  // Default hostname.
#define ZABBIXPORT		10051
#define ZABBIXMAXLEN	128
#define MAC_ADDRESS		0x00, 0x11, 0x22, 0x33, 0x44, 0xEA
char ZabbixHostName[255];
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
