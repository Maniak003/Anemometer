/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
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
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
uint16_t adcBuffer[CONVERSION_COUNT];
uint16_t haftConf, mesCount, ajustCount, maxLevel, minLevel;
bool readyData, readyCapture, levelNominal;
float maxAmp, curLev, avgLevel, measArray[CONVERSION_COUNT], maxIdxAmp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
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
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
	if ( (haftConf--) == 0) { // Половина преобразования
		LED_PULSE
		HAL_ADC_Stop_DMA(&hadc1);
		if (ajustCount++ > AJUST_DELAY) {
			/* Автоматическая регулировка уровня сигнала */
			ajustCount = 0;
			maxLevel = 0, minLevel = 4096, avgLevel = 0;
			for (int ii = 0; ii < CONVERSION_COUNT; ii++) {
				avgLevel = avgLevel + adcBuffer[ii];
				if (maxLevel < adcBuffer[ii]) {
					maxLevel = adcBuffer[ii];
				}
				if (minLevel > adcBuffer[ii]) {
					minLevel = adcBuffer[ii];
				}
			}
			avgLevel = avgLevel / CONVERSION_COUNT;
			if (abs(minLevel - avgLevel) > abs(maxLevel - avgLevel)) {
				maxLevel = abs(minLevel - avgLevel);
			} else {
				maxLevel = abs(maxLevel - avgLevel);
			}
			if (abs(maxLevel - NOMINAL_LEVEL) > ACURACY_LEVEL) {
				if (maxLevel - NOMINAL_LEVEL > 0) {
					levelNominal = false;
					/* Сигнал сильный, понижаем уровень */
					#ifdef AD5245
					AD5245level(currentLevel--);
					#endif
				} else {
					/* Сигнал слабый, повышаем уровень */
					#ifdef AD5245
					AD5245level(currentLevel++);
					#endif
				}
			} else {  /* Уровень в норме. Накапливаем данные для усреднения */
				levelNominal = true;
			}
		}
		if (levelNominal) {
			if (mesCount < MEASURE_COUNT - 1) {
				for (int ii = 0; ii < CONVERSION_COUNT; ii++) {
					measArray[ii] = measArray[ii] + adcBuffer[ii];
				}
				mesCount++;
			} else { /* Можно выполнять свертку */
				HAL_TIM_Base_Stop_IT(&htim4);
				mesCount = 0;
				readyData = true;
			}
		}
	}

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	LED_PULSE
	readyCapture = true;
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
	HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
	//HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
	//HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(selZ1_GPIO_Port, selZ1_Pin, GPIO_PIN_SET);
	haftConf = 1;
	readyCapture = true;
	LED_PULSE
	if (! readyData) {
		//HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcBuffer, CONVERSION_COUNT);
	}

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
