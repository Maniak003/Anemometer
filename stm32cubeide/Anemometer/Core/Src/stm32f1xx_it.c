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
extern DMA_HandleTypeDef hdma_tim2_ch1;
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
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */
	HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_1);
	/* Turn off all multiplexer */
	GPIOB->ODR &= ~((1 << Z1Receive) | (1 << Z2Receive) | (1 << Z3Receive) | (1 << Z4Receive));
	switch (currentMode) {
		case 1: { // Z1 > Z2
			Z12 = fastCounter & 0x0FFFF;
			break;
		}
		case 2: { // Z2 > Z1
			Z21 = fastCounter & 0x0FFFF;
			break;
		}
		case 3: { // Z2 > Z3
			Z23 = fastCounter & 0x0FFFF;
			break;
		}
		case 4: { // Z3 > Z2
			Z32 = fastCounter & 0x0FFFF;
			break;
		}
		case 5: { // Z3 > Z4
			Z34 = fastCounter & 0x0FFFF;
			break;
		}
		case 6: { // Z4 > Z3
			Z43 = fastCounter & 0x0FFFF;
			break;
		}
		case 7: { // Z4 > Z1
			Z41 = fastCounter & 0x0FFFF;
			break;
		}
		case 8: { // Z1 > Z4
			Z14 = fastCounter & 0x0FFFF;
			break;
		}
	}
	HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim2_ch1);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*) &fastCounter, 1);
	HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);

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

	  /* Turn off all multiplexer */
	  GPIOB->ODR &= ~((1 << Z1Receive) | (1 << Z2Receive) | (1 << Z3Receive) | (1 << Z4Receive));

	  HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_1);	// If not stop in DMA callback.
	  /* Restart timers */
	  HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_2);
	  HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_3);
	  HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_4);

	  /* Set all timer channels for input mode */
	  GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8
			  	  | GPIO_CRH_CNF9 | GPIO_CRH_MODE9
				  | GPIO_CRH_CNF10 | GPIO_CRH_MODE10
				  | GPIO_CRH_CNF11 | GPIO_CRH_MODE11))
				  | (GPIO_CRH_CNF8_0 | GPIO_CRH_CNF9_0 | GPIO_CRH_CNF10_0 | GPIO_CRH_CNF11_0);

	  firstFlag = TRUE;

	  switch (currentMode++) {
		  case 0: { // Z1 (transmit) > Z2 (receive)
			  setZ1transmit; // Set Z1 port to output mode
			  GPIOB->ODR |= (1 << Z2Receive); // Turn on multiplexer for input Z2 channel.
			  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
			  break;
		  }
		  case 1: { // Z2 (transmit) > Z1 (receive)
			  setZ2transmit; // Set Z2 port to output mode
			  GPIOB->ODR |= (1 << Z1Receive); // Turn on multiplexer for input Z1 channel.
			  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2);
			  break;
		  }
		  case 2: { // Z2 (transmit) > Z3 (receive)
			  //readyFlag = TRUE;
			  //currentMode = 0;
			  setZ2transmit; // Set Z2 port to output mode
			  GPIOB->ODR |= (1 << Z3Receive); // Turn on multiplexer for input Z3 channel.
			  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2);
			  break;
		  }
		  case 3: { // Z3 (transmit) > Z2 (receive)
			  setZ3transmit; // Set Z3 port to output mode
			  GPIOB->ODR |= (1 << Z2Receive); // Turn on multiplexer for input Z2 channel.
			  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3);
			  break;
		  }
		  case 4: { // Z3 (transmit) > Z4 (receive)
			  setZ3transmit; // Set Z3 port to output mode
			  GPIOB->ODR |= (uint16_t) (1 << Z4Receive); // Turn on multiplexer for input Z4 channel.
			  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3);
			  break;
		  }
		  case 5: { // Z4 (transmit) > Z3 (receive)
			  setZ4transmit; // Set Z4 port to output mode
			  GPIOB->ODR |= (1 << Z3Receive); // Turn on multiplexer for input Z4 channel.
			  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
			  break;
		  }
		  case 6: { // Z4 (transmit) > Z1 (receive)
			  setZ4transmit;	// Set Z4 port to output mode
			  GPIOB->ODR |= (1 << Z1Receive); // Turn on multiplexer for input Z4 channel.
			  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
			  break;
		  }
		  case 7: { // Z1 (transmit) > Z4 (receive)
			  setZ1transmit;	// Set Z1 port to output mode
			  GPIOB->ODR |= (1 << Z4Receive); // Turn on multiplexer for input Z4 channel.
			  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
			  break;
		  }
		  case 8: { // All data complete.
			  readyFlag = TRUE;
			  currentMode = 0;
			  break;
		  }
	  }
	  //HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*) &fastCounter, 1);

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
