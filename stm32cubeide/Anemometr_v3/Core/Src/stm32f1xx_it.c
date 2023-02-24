/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim1;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	static uint16_t ticks = 0;
	ticks++;
	if(ticks >= 1000) {
		DHCP_time_handler();
		ticks = 0;
	}

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	//HAL_TIM_Base_Stop_IT(&htim3);
	runFlag = COUNT_FRONT;		// Сработал таймер сброса таймера захвата, начинаем измерение.
	//LED_PULSE
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	/*
	* currentMode
	*    Z1
	* Z4    Z2
	*    Z3
	*
	* 0 - Z1 >> Z3
	* 1 - Z3 >> Z1
	* 2 - Z2 >> Z4
	* 3 - Z4 >> Z2
	*/
	double XX1, YY1;
	#ifdef MEDIAN_FILTER_ENABLE
	double X1m[3], Y1m[3];
	uint8_t countX1, countY1;
	#else
	double Vm[3];
	uint8_t countV;
	#endif
	#ifdef SYSTICK_DISABLE
		SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  // Включение SysTick
	#endif

		front_sum = 0;
		runFlag = 0;
		HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_3);
		HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_4);

		/* Отключим все мультиплексоры */
		receiversOff
		STOP_CAPTURE	// If not stop in callback.
		/* Set all timer channels for output mode */
		GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF8_0 | GPIO_CRH_CNF9_0 | GPIO_CRH_CNF10_0 | GPIO_CRH_CNF11_0))
				| (GPIO_CRH_CNF8_1 | GPIO_CRH_MODE8_1 | GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_1 | GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_1 | GPIO_CRH_CNF11_1 | GPIO_CRH_MODE11_1);

		/* Управление циклом опроса */
		if (currentMode >= CHANNELS) {
			currentMode = 0;
			measCount++;  // Следующее измерение.
		}
		//LED_PULSE
		if ((measCount == MEASSURE_COUNT) && (calibrateMode == 0)) {
			//LED_PULSE
			#ifdef SYSTICK_DISABLE
			SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  // Включение SysTick
			#endif
			HAL_TIM_Base_Stop_IT(&htim4);  // Остановим измерения на время обработки
			Vmax = 0;
			Xmax = 0;
			Ymax = 0;
			Xsum1 = 0;
			Ysum1 = 0;
			#ifdef MEDIAN_FILTER_ENABLE
			/* Фильтр для всех значений */
			X1m[0] = 0; X1m[1] = 0; X1m[2] = 0; countX1 = 0;
			Y1m[0] = 0; Y1m[1] = 0; Y1m[2] = 0; countY1 = 0;
			#else
			/* Фильтр только для максимальной скорости */
			Vm[0] = 0; Vm[1] = 0; Vm[2] = 0; countV = 0;
			#endif
			//LED_PULSE
			for (int ii = PREFETCH; ii < MEASSURE_COUNT; ii++) {
				// Медианный фильтр для X
				#ifdef MEDIAN_FILTER_ENABLE
				X1m[countX1] = resul_arrayX1[ii] - resul_arrayX2[ii] * DX1.f;
				if (++countX1 >= 3) countX1 = 0;
				XX1 = (X1m[0] < X1m[1]) ? ((X1m[1] < X1m[2]) ? X1m[1] : ((X1m[2] < X1m[0]) ? X1m[0] : X1m[2])) : ((X1m[0] < X1m[2]) ? X1m[0] : ((X1m[2] < X1m[1]) ? X1m[1] : X1m[2]));
				#else
				XX1 = resul_arrayX1[ii] - resul_arrayX2[ii] * DX1.f;
				#endif
				Xsum1 = Xsum1 + XX1;

				// Медианный фильтр для Y
				#ifdef MEDIAN_FILTER_ENABLE
				Y1m[countY1] = resul_arrayY1[ii] - resul_arrayY2[ii] * DY1.f;
				if (++countY1 >= 3) countY1 = 0;
				YY1 = (Y1m[0] < Y1m[1]) ? ((Y1m[1] < Y1m[2]) ? Y1m[1] : ((Y1m[2] < Y1m[0]) ? Y1m[0] : Y1m[2])) : ((Y1m[0] < Y1m[2]) ? Y1m[0] : ((Y1m[2] < Y1m[1]) ? Y1m[1] : Y1m[2]));
				#else
				YY1 = resul_arrayY1[ii] - resul_arrayY2[ii] * DY1.f;
				#endif
				Ysum1 = Ysum1 + YY1;

				X = XX1;
				Y = YY1;
				V = sqrt(pow(X, 2) + pow(Y, 2));

				/* Медианный фильтр для максимальных значений */
				#ifndef MEDIAN_FILTER_ENABLE
				Vm[countV] = V;
				if (++countV >= 3) countV = 0;
				V = (Vm[0] < Vm[1]) ? ((Vm[1] < Vm[2]) ? Vm[1] : ((Vm[2] < Vm[0]) ? Vm[0] : Vm[2])) : ((Vm[0] < Vm[2]) ? Vm[0] : ((Vm[2] < Vm[1]) ? Vm[1] : Vm[2]));
				#endif

				if ( V > Vmax) {
					Vmax = V;
				}
				if (abs(X) > Xmax) {
					Xmax = abs(X);
				}
				if (abs(Y) > Ymax) {
					Ymax = abs(Y);
				}
				resul_arrayX1[ii] = 0;
				resul_arrayX2[ii] = 0;
				resul_arrayY1[ii] = 0;
				resul_arrayY2[ii] = 0;
			}
			Xsum = Xsum1;
			Xsum = Xsum / ((MEASSURE_COUNT - PREFETCH));		// Среднее количество тактов по X
			Xsum = Xsum / SPEED_CALIBRATE;	// Скорость по X

			Ysum = Ysum1;
			Ysum = Ysum / ((MEASSURE_COUNT - PREFETCH));		// Среднее количество тактов по Y
			Ysum = Ysum / SPEED_CALIBRATE;	// Скорость по Y

			Vmaxfin = Vmax / SPEED_CALIBRATE;	// Максимальная скорость за время MEASSURE_COUNT
			Xmaxfin = Xmax / SPEED_CALIBRATE;
			Ymaxfin = Ymax / SPEED_CALIBRATE;
			V = sqrt(pow(Xsum, 2) + pow(Ysum, 2));  // Скалярное значение скорости
			if ( V == 0) {
			  A = 0;
			} else {
				A = acos( Ysum / V ) * 180 / 3.1415926; // Угол
				if (Xsum < 0) {
					A = 360 - A; // III, IV квадранты
				}
			}
			measCount = 0;
			readyFlag = TRUE;  // Разрешаем обработку в основном цикле.
		} else {
			if ((calibrateMode > 0) && (measCount == 1)) {  // Режим калибровки/тестирования
				HAL_TIM_Base_Stop_IT(&htim4);  // Остановим измерения на время обработки
				measCount = 0;
				#ifdef SYSTICK_DISABLE
					SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  // Включение SysTick
				#endif
				readyFlag = TRUE;  // Разрешаем обработку в основном цикле.
			} else {
				switch (currentMode++) {
					case 0: { 					// Z1 (transmit) > Z3 (receive) Y1
						//TP_PULSE
						TIM3->ARR = C_1; 		// Коррекция для таймера запуска измерения Z13
						setZ3receive 			// Turn on multiplexer for input Z2 channel.
						HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1); // Генерация для пьезокристалла в 1 канале
						break;
					}
					case 1: { 					// Z3 (transmit) > Z1 (receive) Y2
						TP_PULSE
						TIM3->ARR = C_4; 		// Коррекция для таймера запуска измерения Z31
						setZ1receive 			// Turn on multiplexer for input Z1 channel.
						HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3); // Генерация для пьезокристалла в 3 канале
						break;
					}
					case 2: { 					// Z2 (transmit) > Z4 (receive) X1
						//LED_PULSE
						TIM3->ARR = C_2; 		// Коррекция для таймера запуска измерения Z24
						setZ4receive 			// Turn on multiplexer for input Z4 channel.
						HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2);	// Генерация для пьезокристалла в 2 канале
						break;
					}
					case 3: { 					// Z4 (transmit) > Z3 (receive) X2
						//LED_PULSE
						TIM3->ARR = C_3; 		// Коррекция для таймера запуска измерения Z42
						setZ2receive 			// Turn on multiplexer for input Z3 channel.
						HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);	// Генерация для пьезокристалла в 4 канале
						break;
					}
				}
				/* Запускаем таймер захвата */
				START_CAPTURE
			}
		}
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
