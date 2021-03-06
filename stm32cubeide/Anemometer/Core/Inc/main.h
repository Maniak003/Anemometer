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
//#define TMP117_ENABLE
#define BME280_ENABLE
#include <stm32f1xx_hal_i2c.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "wizchip_conf.h"
#include "dhcp.h"
#include "dns.h"
#include "socket.h"
#include "stdarg.h"
#ifdef TMP117_ENABLE
#include "tmp117.h"
#endif
#ifdef BME280_ENABLE
#include "BME280.h"
#endif
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80
#define TRUE 1
#define FALSE 0
#define timeAdge 2
#define Z1Receive 1
#define Z2Receive 2
#define Z3Receive 10
#define Z4Receive 11
//#define setZ1transmit GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF8_0)) | (GPIO_CRH_CNF8_1 | GPIO_CRH_MODE8_1);
//#define setZ2transmit GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF9_0)) | (GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_1);
//#define setZ3transmit GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF10_0)) | (GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_1);
//#define setZ4transmit GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF11_0)) | (GPIO_CRH_CNF11_1 | GPIO_CRH_MODE11_1);
#define setZ1receive GPIOB->ODR |= (1 << Z1Receive); GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8)) | GPIO_CRH_CNF8_0;
#define setZ2receive GPIOB->ODR |= (1 << Z2Receive); GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9)) | GPIO_CRH_CNF9_0;
#define setZ3receive GPIOB->ODR |= (1 << Z3Receive); GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10)) | GPIO_CRH_CNF10_0;
#define setZ4receive GPIOB->ODR |= (1 << Z4Receive); GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF11 | GPIO_CRH_MODE11)) | GPIO_CRH_CNF11_0;
#define LED_PULSE GPIOA->BSRR = (uint32_t)LED_Pin; GPIOA->BSRR = (uint32_t)LED_Pin << 16u;
#define TP_PULSE GPIOA->BSRR = (uint32_t)GPIO_PIN_15; GPIOA->BSRR = (uint32_t)GPIO_PIN_15 << 16u;
//#define START_CAPTURE HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*) &fastCounter, 2); HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);
//#define STOP_CAPTURE HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_1); HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_2);
#define START_CAPTURE HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
#define STOP_CAPTURE HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1); HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
#define SYSTICK_DISABLE
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
#define Eth_CS_Pin GPIO_PIN_12
#define Eth_CS_GPIO_Port GPIOB
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
#define TP_Pin GPIO_PIN_15
#define TP_GPIO_Port GPIOA
#define Eth_int_Pin GPIO_PIN_3
#define Eth_int_GPIO_Port GPIOB
#define Eth_rst_Pin GPIO_PIN_4
#define Eth_rst_GPIO_Port GPIOB
#define TMP117_SCL_Pin GPIO_PIN_8
#define TMP117_SCL_GPIO_Port GPIOB
#define TMP117_SDA_Pin GPIO_PIN_9
#define TMP117_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
uint16_t currentMode, startCount, measCount, test_cnt, front_sum;
//uint16_t Z12, Z21, Z23, Z32, Z34, Z43, Z41, Z14;
uint16_t C_12, C_34, C_14, C_23, BC_12, BC_34, BC_14, BC_23;
float ZX1, ZX2, ZX3, ZX4, ZY1, ZY2, ZY3, ZY4;
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
uint16_t calibrateCount, calibrateMode, runFlag;
#define	COUNT_FRONT 10
#define CALIBRATE_ACURACY 1
/*
 * // 24000
 * ?????????????????? ???? ?????????????????? ??????????????????. ?????? ???????????? ??????????????????, ?????? ???????????? jitter
 * */
#define CALIBRATE_START 44000// 34000, 24000
#define MEASSURE_COUNT 100
#define PREFETCH 20
#define SPEED_CALIBRATE 42.19f // cos(atg(17/18)) * (49.52/330000 - 49.52/331000) / (1/64000000) * 2
#define MAX_DIFFERENT 21		// ???????????????????? ???????????????? ?????????? ????????????????.
#define MAX_SPEED 25
#define CALIBRATE_TIMES 5
#define CALIBRATE_MAX_COUNT 1600
#define BODY_CALIBRATE_START 1000
#define MEDIAN_FILTER_ENABLE
//#define DIFFERENT_ANALIS
#define TEST_TEXT "\r\nStart test.\r\n"
#define TEST_TERMINATE "\r\nTerminate test or calibrate.\r\n"
#define CALIBRATE_TEXT "\r\nStart callibrate \r\n"
#define CALIBRATE_ERROR_RANGE "\r\nCalibrate ERROR.\r\nDelta out of range.\r\n"
#define CALIBRATE_ERROR_TOUT "\r\nCalibrate ERROR.\r\nTime out\r\n"
#define INIT_FINISH_TEXT "Init finish.\r\n"
#define RESTART_TOUT "\r\nTime out, restart.\r\n"
#define START_TEXT "\r\nAnemometer start.\r\n"
double X, Y, V, Vmaxfin, Xmaxfin, Ymaxfin, A, Xsum, Ysum, Vmax, Xmax, Ymax, Xsum1, Xsum2, Ysum1, Ysum2;
float resul_arrayX1[MEASSURE_COUNT], resul_arrayX2[MEASSURE_COUNT],resul_arrayX3[MEASSURE_COUNT], resul_arrayX4[MEASSURE_COUNT];
float resul_arrayY1[MEASSURE_COUNT], resul_arrayY2[MEASSURE_COUNT],resul_arrayY3[MEASSURE_COUNT], resul_arrayY4[MEASSURE_COUNT];
float temperature, pressure, humidity;
uint32_t sumCounter2, fastCounter;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
bool readyFlag, firstTime, calibrate12, calibrate34, calibrate14, calibrate23, test_flag;

/* For W5500*/
#define DHCP_SOCKET     0
#define DNS_SOCKET      1
#define TCP_SOCKET		2
#define W5500_RST_Pin	GPIO_PIN_4
#define W5500_CS_Pin	GPIO_PIN_12
#define _DHCP_DEBUG_

//#define ZABBIX_DEBUG
#define ZABBIX_ENABLE
#define ZABBIXAGHOST	"Anemometer"  // Default hostname.
#define ZABBIXPORT		10051
#define ZABBIXMAXLEN	128
#define MAC_ADDRESS		0x00, 0x11, 0x22, 0x33, 0x44, 0xEB
char ZabbixHostName[255];

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
