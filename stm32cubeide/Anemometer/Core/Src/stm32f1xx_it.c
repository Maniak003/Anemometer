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
	//HAL_NVIC_SystemReset();
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
	//HAL_NVIC_SystemReset();
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
	//HAL_NVIC_SystemReset();
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
	//HAL_NVIC_SystemReset();
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
//	if (startCount-- == 0) {
	runFlag = COUNT_FRONT;		// Сработал таймер сброса таймера захвата, начинаем измерение.
		//LED_PULSE
//	}

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
	double XX1, XX2, YY1, YY2;
	#ifdef MEDIAN_FILTER_ENABLE
	double X1m[3], X2m[3], Y1m[3], Y2m[3];
	uint8_t countX1, countX2, countY1, countY2;
	#else
	double Vm[3];
	uint8_t countV;
	#endif
	/*
	* currentMode
	* Z1--Z2
	* |	  | > N
	* Z4__Z3
	*
	* 0 - Z1 >> Z2
	* 1 - Z2 >> Z1
	* 2 - Z2 >> Z3
	* 3 - Z3 >> Z2
	* 4 - Z3 >> Z4
	* 5 - Z4 >> Z3
	* 6 - Z4 >> Z1
	* 7 - Z1 >> Z4
	*/
	front_sum = 0;

	/* Turn off all multiplexer */
	GPIOB->ODR &= ~((1 << Z1Receive) | (1 << Z2Receive) | (1 << Z3Receive) | (1 << Z4Receive));

	STOP_CAPTURE	// If not stop in callback.
	runFlag = 0;		// Запрещаем запись результата захвата
	/* Restart timers */
	HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_4);

	/* Set all timer channels for input mode */
	/*
	GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8
			  | GPIO_CRH_CNF9 | GPIO_CRH_MODE9
			  | GPIO_CRH_CNF10 | GPIO_CRH_MODE10
			  | GPIO_CRH_CNF11 | GPIO_CRH_MODE11))
			  | (GPIO_CRH_CNF8_0 | GPIO_CRH_CNF9_0 | GPIO_CRH_CNF10_0 | GPIO_CRH_CNF11_0);
	*/
	/* Set all timer channels for output mode */
	GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF8_0 | GPIO_CRH_CNF9_0 | GPIO_CRH_CNF10_0 | GPIO_CRH_CNF11_0))
			| (GPIO_CRH_CNF8_1 | GPIO_CRH_MODE8_1 | GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_1 | GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_1 | GPIO_CRH_CNF11_1 | GPIO_CRH_MODE11_1);

	if (currentMode >= 8) {
		currentMode = 0;
		measCount++;
	}
	//LED_PULSE
	if ((measCount == MEASSURE_COUNT) && (calibrateMode == 0)) {
		//LED_PULSE
		//HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
		#ifdef SYSTICK_DISABLE
		SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  // Включение SysTick
		#endif
		HAL_TIM_Base_Stop_IT(&htim4);  // Остановим измерения на время обработки
		Vmax = 0;
		Xmax = 0;
		Ymax = 0;
		Xsum1 = 0;
		Xsum2 = 0;
		Ysum1 = 0;
		Ysum2 = 0;
		#ifdef MEDIAN_FILTER_ENABLE
		X1m[0] = 0; X1m[1] = 0; X1m[2] = 0; countX1 = 0;
		X2m[0] = 0; X2m[1] = 0; X2m[2] = 0; countX2 = 0;
		Y1m[0] = 0; Y1m[1] = 0; Y1m[2] = 0; countY1 = 0;
		Y2m[0] = 0; Y2m[1] = 0; Y2m[2] = 0; countY2 = 0;
		#else
		Vm[0] = 0; Vm[1] = 0; Vm[2] = 0; countV = 0;
		#endif
		//LED_PULSE
		for (int ii = PREFETCH; ii < MEASSURE_COUNT; ii++) {
			// Медианный фильтр для X
			#ifdef MEDIAN_FILTER_ENABLE
			X1m[countX1] = resul_arrayX1[ii] - resul_arrayX2[ii] * DX1.f;
			if (++countX1 >= 3) countX1 = 0;
			XX1 = (X1m[0] < X1m[1]) ? ((X1m[1] < X1m[2]) ? X1m[1] : ((X1m[2] < X1m[0]) ? X1m[0] : X1m[2])) : ((X1m[0] < X1m[2]) ? X1m[0] : ((X1m[2] < X1m[1]) ? X1m[1] : X1m[2]));
			X2m[countX2] = resul_arrayX4[ii] - resul_arrayX3[ii] * DX2.f;
			if (++countX2 >= 3) countX2 = 0;
			XX2 = (X2m[0] < X2m[1]) ? ((X2m[1] < X2m[2]) ? X2m[1] : ((X2m[2] < X2m[0]) ? X2m[0] : X2m[2])) : ((X2m[0] < X2m[2]) ? X2m[0] : ((X2m[2] < X2m[1]) ? X2m[1] : X2m[2]));
			#else
			XX1 = resul_arrayX1[ii] - resul_arrayX2[ii] * DX1.f;
			XX2 = resul_arrayX4[ii] - resul_arrayX3[ii] * DX2.f;
			#endif
			#ifdef DIFFERENT_ANALIS
			if (abs(XX1 - XX2) > MAX_DIFFERENT) {
				if (abs(XX1) > abs(XX2)) {
					XX1 = XX2;
				} else {
					XX2 = XX1;
				}
			}
			#endif
			Xsum1 = Xsum1 + XX1;
			Xsum2 = Xsum2 + XX2;

			// Медианный фильтр для Y
			#ifdef MEDIAN_FILTER_ENABLE
			Y1m[countY1] = resul_arrayY1[ii] - resul_arrayY2[ii] * DY1.f;
			if (++countY1 >= 3) countY1 = 0;
			YY1 = (Y1m[0] < Y1m[1]) ? ((Y1m[1] < Y1m[2]) ? Y1m[1] : ((Y1m[2] < Y1m[0]) ? Y1m[0] : Y1m[2])) : ((Y1m[0] < Y1m[2]) ? Y1m[0] : ((Y1m[2] < Y1m[1]) ? Y1m[1] : Y1m[2]));
			X2m[countY2] = resul_arrayY4[ii] - resul_arrayY3[ii] * DY2.f;
			if (++countY2 >= 3) countY2 = 0;
			YY2 = (Y2m[0] < Y2m[1]) ? ((Y2m[1] < Y2m[2]) ? Y2m[1] : ((Y2m[2] < Y2m[0]) ? Y2m[0] : Y2m[2])) : ((Y2m[0] < Y2m[2]) ? Y2m[0] : ((Y2m[2] < Y2m[1]) ? Y2m[1] : Y2m[2]));
			#else
			YY1 = resul_arrayY1[ii] - resul_arrayY2[ii] * DY1.f;
			YY2 = resul_arrayY4[ii] - resul_arrayY3[ii] * DY2.f;
			#endif
			#ifdef DIFFERENT_ANALIS
			if (abs(YY1 - YY2) > MAX_DIFFERENT) {
				if (abs(YY1) > abs (YY2)) {
					YY1 = YY2;
				} else {
					YY2 = YY1;
				}
			}
			#endif
			Ysum1 = Ysum1 + YY1;
			Ysum2 = Ysum2 + YY2;

			X = (XX1 + XX2) / 2;
			Y = (YY1 + YY2) / 2;
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
			resul_arrayX3[ii] = 0;
			resul_arrayX4[ii] = 0;
			resul_arrayY1[ii] = 0;
			resul_arrayY2[ii] = 0;
			resul_arrayY3[ii] = 0;
			resul_arrayY4[ii] = 0;
		}
		Xsum = (Xsum1 + Xsum2);
		Xsum = Xsum / ((MEASSURE_COUNT - PREFETCH) * 2);		// Среднее количество тактов по X
		Xsum = Xsum / SPEED_CALIBRATE;	// Скорость по X

		Ysum = (Ysum1 + Ysum2);
		Ysum = Ysum / ((MEASSURE_COUNT - PREFETCH) * 2);		// Среднее количество тактов по Y
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
		if ((calibrateMode > 0) && (measCount == 1)) {  // Режим калибровки
			HAL_TIM_Base_Stop_IT(&htim4);  // Остановим измерения на время обработки
			measCount = 0;
			#ifdef SYSTICK_DISABLE
				SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  // Включение SysTick
			#endif
			readyFlag = TRUE;  // Разрешаем обработку в основном цикле.
		} else {
			switch (currentMode++) {
				case 0: { 					// Z1 (transmit) > Z2 (receive) Y1
					LED_PULSE
					//setZ1transmit; 			// Set Z1 port to output mode
					setZ2receive 			// Turn on multiplexer for input Z2 channel.
					HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1); // Генерация для пьезокристалла в первом канале
					break;
				}
				case 1: { 					// Z2 (transmit) > Z1 (receive) Y2
					//LED_PULSE
					TIM3->ARR = C_23; 		// Коррекция для таймера запуска измерения Z23, Z32
					//setZ2transmit; 			// Set Z2 port to output mode
					setZ1receive 			// Turn on multiplexer for input Z1 channel.
					HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2); // Генерация для пьезокристалла во втором канале
					break;
				}
				case 2: { 					// Z2 (transmit) > Z3 (receive) X1
					//LED_PULSE
					//setZ2transmit; 			// Set Z2 port to output mode
					setZ3receive 			// Turn on multiplexer for input Z3 channel.
					HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2);	// Генерация для пьезокристалла во втором канале
					break;
				}
				case 3: { 					// Z3 (transmit) > Z2 (receive) X2
					//LED_PULSE
					TIM3->ARR = C_34; 		// Коррекция для таймера запуска измерения Z34, Z43
					//setZ3transmit; 			// Set Z3 port to output mode
					setZ2receive 			// Turn on multiplexer for input Z2 channel.
					HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3);	// Генерация для пьезокристалла в третьем канале
					break;
				}
				case 4: { 					// Z3 (transmit) > Z4 (receive) Y3
					//LED_PULSE
					//setZ3transmit; 			// Set Z3 port to output mode
					setZ4receive 			// Turn on multiplexer for input Z4 channel.
					HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3);	// Генерация для пьезокристалла в третьем канале
					break;
				}
				case 5: { 					// Z4 (transmit) > Z3 (receive) Y4
					//LED_PULSE
					TIM3->ARR = C_14; 		// Коррекция для таймера запуска измерения Z14, Z41
					//setZ4transmit; 			// Set Z4 port to output mode
					setZ3receive 			// Turn on multiplexer for input Z3 channel.
					HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);	// Генерация для пьезокристалла в четвертом канале
					break;
				}
				case 6: { 					// Z4 (transmit) > Z1 (receive) X3
					//LED_PULSE
					//setZ4transmit;			// Set Z4 port to output mode
					setZ1receive 			// Turn on multiplexer for input Z1 channel.
					HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);	// Генерация для пьезокристалла в четвертом канале
					break;
				}
				case 7: { 					// Z1 (transmit) > Z4 (receive) X4
					//LED_PULSE
					TIM3->ARR = C_12; 		// Коррекция для таймера запуска измерения Z12, Z21
					//setZ1transmit;			// Set Z1 port to output mode
					setZ4receive 			// Turn on multiplexer for input Z4 channel.
					HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);	// Генерация для пьезокристалла в первом канале
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
