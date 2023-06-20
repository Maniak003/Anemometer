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
#ifdef TMP117_ENABLE
#include "tmp117.h"
#endif
#ifdef BME280_ENABLE
#include "BME280.h"
#endif
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
uint32_t runFlag, front_sum;
uint16_t resulMeass, test_cnt, calibrateCount, calibrateMode, currentMode, measCount, ADCValue;
bool readyFlag, test_flag, calibrate1, calibrate3, calibrate2, calibrate4, firstTime;
double Xsum, Ysum, Vmax, A, V, Vmaxfin, Xmaxfin, Ymaxfin, X, Y, Xmax, Ymax, Xsum1, Ysum1;
float ZX1, ZX2, ZY1, ZY2, temperature, pressure, humidity, front_sumf;
uint16_t C_1, C_3, C_2, C_4, C_X, C_Y;

union {
	float f;
	uint32_t u;
} t0;
union {
	float f;
	uint32_t u;
} DX1;
//union {
//	float f;
//	uint32_t u;
//} DX2;
union {
	float f;
	uint32_t u;
} DY1;
//union {
//	float f;
//	uint32_t u;
//} DY2;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
//#define SYSTICK_DISABLE
#define TRUE 1
#define FALSE 0
/*
 *   Количество тактов на 1м/с
 *
 *  cos(arctg(L/2 / H)) * (sqrt((L/2)^2 + H^2) / 330000 - (L/2)^2 + H^2) / 331000) / (1/64000000) * 2
 * L = 40; L/2 = 20
 * H = 20
 * a = 45
 * cos(atg(20/20)) * (56.57/330000 - 56.57/331000) / (1/64000000) * 2 = 46.87
 *  */
#define SPEED_CALIBRATE 46.87f
#define CHANNELS 1
#define PREFETCH 2					/* Количество игнорируемых измерений. Для устранения шума. */
#define CALIBRATE_START 40000		/* Начальное смещение начала измерения */
#define TIM1_PERIOD 179				/* Количество тактов для одного периода ~40kHz */
#define MAX_SPD 40.00f				/* Допустимая максимальная скорость, все, что выше -- игнорируется */
#define MEDIAN_FILTER_ENABLE
#define COUNT_FRONT 28				/* Количество переходов в одном цикле измерения */
#define MEASSURE_COUNT 100			/* Количество циклов в стандартном измерении */
#define CALIBRATE_MAX_COUNT 1600	/* Диапазон перебора смещения задержки измерения */
#define CALIBRATE_TIMES 5			/* Количество стандартных измерений измерений для точной калибровки */
#define CALIBRATE_ACURACY 1.0f		/* Максимальное отклонение при переборе смещения измерения */
#define AUTO_CALIBRATE				/* Подстройка средней точки диапазона для компенсации температуры и деформации корпуса */
#define FAST_CALIBRATE 50			/* Порог для ускоренной калибровки */
#define FAST_CALIBRATE_STEP 30		/* Шаг быстрой калибровки */

#define Z1Receive 1
#define Z2Receive 2
#define Z3Receive 3
#define Z4Receive 4
//#define START_CAPTURE HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
//#define STOP_CAPTURE HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1); HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
#define receiversOff GPIOB->ODR |= (1 << Z1Receive) | (1 << Z2Receive) | (1 << Z3Receive) | (1 << Z4Receive);
#define setZ1receive GPIOB->ODR &= ~((1 << Z1Receive)); GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8)) | GPIO_CRH_CNF8_0;
#define setZ2receive GPIOB->ODR &= ~((1 << Z2Receive)); GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9)) | GPIO_CRH_CNF9_0;
#define setZ3receive GPIOB->ODR &= ~((1 << Z3Receive)); GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10)) | GPIO_CRH_CNF10_0;
#define setZ4receive GPIOB->ODR &= ~((1 << Z4Receive)); GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF11 | GPIO_CRH_MODE11)) | GPIO_CRH_CNF11_0;

//#define LED_PULSE LED_GPIO_Port->BSRR = (uint32_t)LED_Pin; LED_GPIO_Port->BSRR = (uint32_t)LED_Pin << 16u;
#define LED_PULSE HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
#define TP_PULSE HAL_GPIO_WritePin(TP_GPIO_Port, TP_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(TP_GPIO_Port, TP_Pin, GPIO_PIN_RESET);
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TP_Pin GPIO_PIN_2
#define TP_GPIO_Port GPIOA
#define AnalogInput_Pin GPIO_PIN_3
#define AnalogInput_GPIO_Port GPIOA
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
#define CALIBRATE_TEXT "\r\nStart callibrate \r\n"
#define CALIBRATE_ERROR_RANGE "\r\nCalibrate ERROR.\r\nDelta out of range.\r\n"
#define CALIBRATE_ERROR_TOUT "\r\nCalibrate ERROR.\r\nTime out\r\n"
#define TEST_TEXT "\r\nStart test.\r\n"
#define TEST_TERMINATE "\r\nTerminate test or calibrate.\r\n"
#define WAIT_BME280 "Wait BME280."
#define OK "Ok.\r\n"
#define WAIT_W5500  "Wait W5500."

float resul_arrayX1[MEASSURE_COUNT], resul_arrayY1[MEASSURE_COUNT], resul_arrayX2[MEASSURE_COUNT], resul_arrayY2[MEASSURE_COUNT];
float avg_X1, avg_X2, avg_Y1, avg_Y2;
/* For W5500*/
#define DHCP_SOCKET     0
#define DNS_SOCKET      1
#define TCP_SOCKET		2
#define DHCP_TRY_CNT	1000
#define _DHCP_DEBUG_

//#define ZABBIX_DEBUG					/* Вывод хода подключения и записи данных в zabbix */
//#define ZABBIX_ENABLE					/* Включает сетевой интерфейс и инициирует запись данных в zabbix */
#define ZABBIX_RAW_DATA					/* Сохранение исходных значений, для температурной калибровки. */
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