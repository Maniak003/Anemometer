/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef	flash_ok;
char uart_buffer[10] = {0,};
char SndBuffer[200] = {0,};
wiz_NetInfo net_info = {
	.mac  = { MAC_ADDRESS },
	.dhcp = NETINFO_DHCP
};
uint32_t errCnt, errMin, errMax, err;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void rwFlash(uint8_t rwFlag) {
	uint32_t pageAdr = 0x800FC00; //.
	uint32_t magicKey;
	uint64_t dataForSave;
	magicKey = *(__IO uint32_t*) pageAdr;
	if ((magicKey != 0x12349876) || (rwFlag == 1)) { // rwFlag == 1 for wrtite data to flash
		magicKey = 0x12349876;
		if (rwFlag == 0) { // For first initial
			C_13 = CALIBRATE_START;
			C_24 = CALIBRATE_START;
			DX1.f = 1;
			//DX2.f = 1;
			DY1.f = 1;
			//DY2.f = 1;
		}
		FLASH_EraseInitTypeDef EraseInitStruct;
		uint32_t PAGEError = 0;
		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.PageAddress = pageAdr; //
		EraseInitStruct.NbPages     = 1;

		flash_ok = HAL_ERROR;
		// Unlock flash
		while(flash_ok != HAL_OK) {
		  flash_ok = HAL_FLASH_Unlock();
		}
		if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) == HAL_OK) {
			dataForSave = (uint64_t) magicKey;
			flash_ok = HAL_ERROR;
			while(flash_ok != HAL_OK){
				flash_ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, pageAdr, dataForSave); // Write  magic key
			}
			dataForSave = (uint64_t) (C_13 | ((uint64_t) C_24 << 16));
			flash_ok = HAL_ERROR;
			while(flash_ok != HAL_OK){
				flash_ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, pageAdr + 16, dataForSave); // Write C_12 C_34 C_14 C_23
			}
			flash_ok = HAL_ERROR;
			while(flash_ok != HAL_OK){
				flash_ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, pageAdr + 24, DX1.u); // Write DX1
			}
			//flash_ok = HAL_ERROR;
			//while(flash_ok != HAL_OK){
			//	flash_ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, pageAdr + 28, DX2.u); // Write DX2
			//}
			flash_ok = HAL_ERROR;
			while(flash_ok != HAL_OK){
				flash_ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, pageAdr + 32, DY1.u); // Write DY1
			}
			//flash_ok = HAL_ERROR;
			//while(flash_ok != HAL_OK){
			//	flash_ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, pageAdr + 36, DY2.u); // Write DY2
			//}
		}
		// Lock flash
		flash_ok = HAL_ERROR;
		while(flash_ok != HAL_OK){
			flash_ok = HAL_FLASH_Lock();
		}
	} else {
		/* Задержки измерения в каналах */
		C_13 = *(__IO uint16_t*) (pageAdr + 16);
		C_24 = *(__IO uint16_t*) (pageAdr + 18);
		//C_31 = *(__IO uint16_t*) (pageAdr + 20);
		//C_42 = *(__IO uint16_t*) (pageAdr + 22);
		memset(SndBuffer, 0, sizeof(SndBuffer));
		sprintf(SndBuffer, "C_13: %5d, C_24: %5d  \r\n", C_13, C_24);
		HAL_UART_Transmit(&huart1, (uint8_t *) SndBuffer, sizeof(SndBuffer), 1000);

		/* Поправочные коэффициенты */
		DX1.u = *(__IO uint32_t*) (pageAdr + 24);
		//DX2.u = *(__IO uint32_t*) (pageAdr + 28);
		DY1.u = *(__IO uint32_t*) (pageAdr + 32);
		//DY2.u = *(__IO uint32_t*) (pageAdr + 36);
		memset(SndBuffer, 0, sizeof(SndBuffer));
		sprintf(SndBuffer, "DX1: %7.6f, DY1: %7.6f \r\n", DX1.f, DY1.f);
		HAL_UART_Transmit(&huart1, (uint8_t *) SndBuffer, sizeof(SndBuffer), 1000);
	}
}

#ifdef ZABBIX_ENABLE

#ifdef ZABBIX_DEBUG
void UART_Printf(const char* fmt, ...) {
	char buff[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buff, sizeof(buff), fmt, args);
	HAL_UART_Transmit(&huart1, (uint8_t*) buff, strlen(buff), HAL_MAX_DELAY);
	va_end(args);
}
#endif


void W5500_Select(void) {
	HAL_GPIO_WritePin(SCSN_GPIO_Port, SCSN_Pin, GPIO_PIN_RESET);
}

void W5500_Unselect(void) {
	HAL_GPIO_WritePin(SCSN_GPIO_Port, SCSN_Pin, GPIO_PIN_SET);
}

void W5500_ReadBuff(uint8_t* buff, uint16_t len) {
	HAL_SPI_Receive(&hspi2, buff, len, HAL_MAX_DELAY);
}

void W5500_WriteBuff(uint8_t* buff, uint16_t len) {
	HAL_SPI_Transmit(&hspi2, buff, len, HAL_MAX_DELAY);
}

uint8_t W5500_ReadByte(void) {
	uint8_t byte;
	W5500_ReadBuff(&byte, sizeof(byte));
	return byte;
}

void W5500_WriteByte(uint8_t byte) {
	W5500_WriteBuff(&byte, sizeof(byte));
}

volatile bool ip_assigned = false;

void Callback_IPAssigned(void) {
#ifdef ZABBIX_DEBUG
	UART_Printf("Callback: IP assigned! Leased time: %d sec\r\n", getDHCPLeasetime());
#endif
    ip_assigned = true;
}

void Callback_IPConflict(void) {
#ifdef ZABBIX_DEBUG
    UART_Printf("Callback: IP conflict!\r\n");
#endif
}

// 1K should be enough, see https://forum.wiznet.io/t/topic/1612/2
uint8_t dhcp_buffer[1024];
// 1K seems to be enough for this buffer as well
//uint8_t dns_buffer[1024];

/*
 * addr -- IP address zabbix server, dhcp option 224.
 * Settings for isc-dhcp-server:
 * 		option zabbix-server-ip code 224 = ip-address ;
 * 		option zabix-host-name code 225 = string ;
 * 		host anemometr {
 *      	hardware ethernet 00:11:22:33:44:xx;
 *      	fixed-address 192.168.1.24;
 *       	option zabbix-server-ip 192.168.1.6;
 *       	option zabix-host-name "Ed";
 * 		}
 *
 * key -- Zabbix key: {ALTIM_DIRECT, ALTIM_SPEED}
 *
 * value -- Float value of key.
*/
uint8_t sendToZabbix(uint8_t * addr, char * host, char * key, float value) {
#ifdef ZABBIX_DEBUG
    UART_Printf("Creating socket...\r\n");
#endif
    uint8_t tcp_socket = TCP_SOCKET;
    uint8_t code = socket(tcp_socket, Sn_MR_TCP, 10888, 0);
    if(code != tcp_socket) {
	#ifdef ZABBIX_DEBUG
        UART_Printf("socket() failed, code = %d\r\n", code);
	#endif
        return(-1);
    }
#ifdef ZABBIX_DEBUG
    UART_Printf("Socket created, connecting...\r\n");
#endif
    code = connect(tcp_socket, addr, ZABBIXPORT);
    if(code != SOCK_OK) {
	#ifdef ZABBIX_DEBUG
        UART_Printf("connect() failed, code = %d\r\n", code);
	#endif
        close(tcp_socket);
        return(-2);
    }
#ifdef ZABBIX_DEBUG
    UART_Printf("Connected, sending ZABBIX request...\r\n");
#endif
    {
    	char req[ZABBIXMAXLEN];
    	char str[ZABBIXMAXLEN - 13];
    	sprintf(str, "{\"request\":\"sender data\",\"data\":[{\"host\":\"%.20s\",\"key\":\"%s\",\"value\":\"%f\"}]}", host, key, value);
    	req[0] = 'Z';
    	req[1] = 'B';
		req[2] = 'X';
		req[3] = 'D';
		req[4] = 0x01;
		req[5] = strlen(str);
		req[6] = 0;
		req[7] = 0;
		req[8] = 0;
		req[9] = 0;
		req[10] = 0;
		req[11] = 0;
		req[12] = 0;
		strcpy(req + 13, str);
        //char req[] = "ZBXD\1\0\0\0\0\0\0\0\0{\"request\":\"sender data\",\"data\":[{\"host\":\"Ed\",\"key\":\"ALTIM_DIRECT\",\"value\":\"10\"}]}";
        uint8_t len = req[5] + 13;
        uint8_t* buff = (uint8_t*)&req;
        while(len > 0) {
		#ifdef ZABBIX_DEBUG
            UART_Printf("Sending %d bytes, data length %d bytes...\r\n", len, req[5]);
		#endif
            int32_t nbytes = send(tcp_socket, buff, len);
            if(nbytes <= 0) {
			#ifdef ZABBIX_DEBUG
                UART_Printf("send() failed, %d returned\r\n", nbytes);
			#endif
                close(tcp_socket);
                return(-3);
            }
			#ifdef ZABBIX_DEBUG
            UART_Printf("%d b sent!\r\n", nbytes);
			#endif
            len -= nbytes;
        }
    }
    /* Read data from Zabbix */
	#ifdef ZABBIX_DEBUG
		UART_Printf("Read.\r\n");
		{
			char buff[32];
			for(;;) {
				int32_t nbytes = recv(tcp_socket, (uint8_t*)&buff, sizeof(buff)-1);
				if(nbytes == SOCKERR_SOCKSTATUS) {
					UART_Printf("\r\nDisconnect.\r\n");
					break;
				}

				if(nbytes <= 0) {
					UART_Printf("\r\nrecv() failed, %d\r\n", nbytes);
					break;
				}

				buff[nbytes] = '\0';
				UART_Printf("%s", buff);
			}
		}

		UART_Printf("Closing socket.\r\n");
	#endif
    close(tcp_socket);
    return(0);
}



void init_w5500() {
	#ifdef ZABBIX_DEBUG
    UART_Printf("\r\ninit() called!\r\n");
    UART_Printf("Registering W5500 callbacks...\r\n");
	#endif
    reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
    reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
    reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);
	#ifdef ZABBIX_DEBUG
    UART_Printf("Calling wizchip_init()...\r\n");
	#endif
    uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
    wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);
	#ifdef ZABBIX_DEBUG
    UART_Printf("Calling DHCP_init()...\r\n");
	#endif

    // set MAC address before using DHCP
    setSHAR(net_info.mac);
    DHCP_init(DHCP_SOCKET, dhcp_buffer);
	#ifdef ZABBIX_DEBUG
    UART_Printf("Registering DHCP callbacks...\r\n");
	#endif
    reg_dhcp_cbfunc(
        Callback_IPAssigned,
        Callback_IPAssigned,
        Callback_IPConflict
    );
	#ifdef ZABBIX_DEBUG
    UART_Printf("Calling DHCP_run()...\r\n");
	#endif
    // actually should be called in a loop, e.g. by timer
    uint32_t ctr = DHCP_TRY_CNT;
    while((!ip_assigned) && (ctr > 0)) {
        DHCP_run();
        ctr--;
        HAL_Delay(100);
    }
    if(!ip_assigned) {
		#ifdef ZABBIX_DEBUG
        UART_Printf(DHCP_ERROR_ASSIGN);
		#endif
        NVIC_SystemReset();
        return;
    }

    getIPfromDHCP(net_info.ip);
    getGWfromDHCP(net_info.gw);
    getSNfromDHCP(net_info.sn);
    getZABBIXfromDHCP(net_info.zabbix);
    getHostNamefromDHCP(net_info.hostname);
    if (net_info.hostname[0] == '\0') {
    	sprintf(ZabbixHostName, "%s", ZABBIXAGHOST);
    } else {
    	sprintf(ZabbixHostName, "%s", net_info.hostname);
    }

    //uint8_t dns[4];
    //getDNSfromDHCP(dns);
	#ifdef ZABBIX_DEBUG
    UART_Printf("IP:  %d.%d.%d.%d\r\nGW:  %d.%d.%d.%d\r\nNet: %d.%d.%d.%d\r\nZabbix: %d.%d.%d.%d\r\nHostName:%s\r\n",
        net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3],
        net_info.gw[0], net_info.gw[1], net_info.gw[2], net_info.gw[3],
        net_info.sn[0], net_info.sn[1], net_info.sn[2], net_info.sn[3],
        net_info.zabbix[0], net_info.zabbix[1], net_info.zabbix[2], net_info.zabbix[3],
		ZabbixHostName
    );
    UART_Printf("Calling wizchip_setnetinfo()...\r\n");
	#endif
    wizchip_setnetinfo(&net_info);

}
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	sprintf(SndBuffer, "\r\nLow power: %s, WWD: %s, IWD: %s, Soft: %s, Pin: %s, Power: %s\r\n",
			(__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) ? "Yes" : "No",
			(__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) ? "Yes" : "No",
			(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) ? "Yes" : "No",
			(__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) ? "Yes" : "No",
			(__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) ? "Yes" : "No",
			(__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) ? "Yes" : "No");

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  readyFlag = TRUE;
  //sumCounter2 = 0;
  HAL_UART_Transmit(&huart1, (uint8_t *) SndBuffer, sizeof(SndBuffer), HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart1, (uint8_t *) INIT_START_TEXT, sizeof(INIT_START_TEXT), HAL_MAX_DELAY);
#ifdef ZABBIX_ENABLE
  HAL_GPIO_WritePin(nRst_GPIO_Port, nRst_Pin, GPIO_PIN_RESET);	// Reset W5500
  HAL_GPIO_WritePin(nRst_GPIO_Port, nRst_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SCSN_GPIO_Port, SCSN_Pin, GPIO_PIN_SET);
  HAL_Delay(2000);
  init_w5500();
#else
  HAL_GPIO_WritePin(nRst_GPIO_Port, nRst_Pin, GPIO_PIN_RESET);	// Reset W5500
#endif
  rwFlash(0);		// Чтение параметров калибровки из Flash.

  /* Таймер задержки запуска измерения */
  receiversOff
  //C_13 = CALIBRATE_START;
  TIM3->ARR = C_13; 		// Коррекция для таймера запуска измерения Z13
  /*
   * calibrateMode == 0 -- Нормальный режим
   * calibrateMode > 0 -- Режим калибровки
   */
  calibrateMode = 0;
  test_flag = FALSE;
  Xsum = 0;
  Ysum = 0;
  Vmax = 0;
  firstTime = TRUE;
  currentMode = 0;
  STOP_CAPTURE
  measCount = 0;
  /*
   *	Очистка массива результатов.
   */
  for (int ii = 0; ii < MEASSURE_COUNT; ii++) {
	  resul_arrayX1[ii] = 0;
	  resul_arrayY1[ii] = 0;
  }
  calibrate13 = FALSE;
  calibrate24 = FALSE;

  HAL_UART_Transmit(&huart1, (uint8_t *) INIT_FINISH_TEXT, sizeof(INIT_FINISH_TEXT), HAL_MAX_DELAY);


  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  V = 0;
  Xsum = 0;
  Ysum = 0;
  Vmaxfin = 0;
  Xmaxfin = 0;
  Ymaxfin = 0;
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  while (1)
  {
	  if (readyFlag) {
		  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  // Включение SysTick
		  HAL_IWDG_Refresh(&hiwdg);
		  readyFlag = FALSE;
		  if (calibrateMode > 0) {
			  /* Процедура калибровки */
			  if (( calibrate13 || calibrate24 ) && (calibrateCount < CALIBRATE_MAX_COUNT)) {
				  memset(SndBuffer, 0, sizeof(SndBuffer));
				  if (test_flag) {
					  sprintf(SndBuffer, "Z13(%4.0f)-Z31(%4.0f):%5.0f, Z42(%4.0f)-Z24(%4.0f):%5.0f   \r",
							  resul_arrayY1[0], resul_arrayY2[0], resul_arrayY1[0] - resul_arrayY2[0] * DY1.f,
							  resul_arrayX1[0], resul_arrayX2[0], resul_arrayX1[0] - resul_arrayX2[0] * DX1.f);
				  } else {
					  sprintf(SndBuffer, "Z13-Z31:%5.0f-%5.0f, Z42-Z24:%5.0f-%5.0f   \r",
							  resul_arrayY1[0], resul_arrayY2[0], resul_arrayX1[0], resul_arrayX2[0]);
				  }
				  HAL_UART_Transmit(&huart1, (uint8_t *) SndBuffer, sizeof(SndBuffer), 1000);
				  /* Y */
				  if (! test_flag) {
					  if ( calibrate13 && (abs(resul_arrayY1[0] + resul_arrayY2[0] - 1600) > CALIBRATE_ACURACY) ) {
						  if (resul_arrayY1[0] + resul_arrayY2[0] > 1600) {
							  C_13++;
						  } else {
							  C_13--;
						  }
					  } else {
						  calibrate13 = FALSE;	// Закончена калибровка таймера запуска измерения в канале Y1
					  }
					  /* X */
					  if ( calibrate24 && (abs(resul_arrayX1[0] + resul_arrayX2[0] - 1600) > CALIBRATE_ACURACY) ) {
						  if (resul_arrayX1[0] + resul_arrayX2[0] > 1600) {
							  C_24++;
						  } else {
							  C_24--;
						  }
					  } else {
						  calibrate24 = FALSE;	// Закончена калибровка таймера запуска измерения в канале Y2
					  }
					  calibrateCount++;
				  }
				#ifdef SYSTICK_DISABLE
				  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;  // Выключение SysTick
				#endif
				  HAL_TIM_Base_Start_IT(&htim4);  // Перезапуск для начала измерений
			  } else {
				  if (calibrateCount >= CALIBRATE_MAX_COUNT) {
					  HAL_UART_Transmit(&huart1, (uint8_t *) CALIBRATE_ERROR_TOUT, sizeof(CALIBRATE_ERROR_TOUT), 1000);
					  /* System restart if calibrate error. */
					  HAL_NVIC_SystemReset();
				  }
				  if (calibrateMode > 0) {
					  ZX1 = ZX1 + (float) resul_arrayX1[0];
					  ZX2 = ZX2 + (float) resul_arrayX2[0];
					  ZY1 = ZY1 + (float) resul_arrayY1[0];
					  ZY2 = ZY2 + (float) resul_arrayY2[0];
					  calibrateMode--;
					  if (calibrateMode == 0) {
						  /* Вычисление поправок */
						  DX1.f = ZX1 / ZX2;
						  DY1.f = ZY1 / ZY2;
						  memset(SndBuffer, 0, sizeof(SndBuffer));
						  sprintf(SndBuffer, "\r\nCalibrate complite.\r\nC_13:%5d, C_24:%5d\r\n", C_13, C_24);
						  HAL_UART_Transmit(&huart1, (uint8_t *) SndBuffer, sizeof(SndBuffer), 1000);
						  memset(SndBuffer, 0, sizeof(SndBuffer));
						  sprintf(SndBuffer, "DY1:%5.4f, DX1:%5.4f\r\n\r\n", DY1.f, DX1.f);
						  HAL_UART_Transmit(&huart1, (uint8_t *) SndBuffer, sizeof(SndBuffer), 1000);
						  if (abs(DX1.f) < 2 && abs(DY1.f) < 2) {
							  rwFlash(1);  // Запись данных калибровки во Flash.
						  } else {
							  HAL_UART_Transmit(&huart1, (uint8_t *) CALIBRATE_ERROR_RANGE, sizeof(CALIBRATE_ERROR_RANGE), 1000);
						  }
						  calibrateCount = 0;
						  firstTime = TRUE;
					  }
				  }
				#ifdef SYSTICK_DISABLE
				  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;  // Выключение SysTick
				#endif
				  HAL_TIM_Base_Start_IT(&htim4);  // Перезапуск для начала измерений
			  }
			  //HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
		  } else {
			  //HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
			#ifdef TMP117_ENABLE
			  temperature = TMP117_get_Temperature(hi2c1);
			#endif
			#ifdef BME280_ENABLE
			  temperature = BME280_ReadTemperature();
			  pressure = BME280_ReadPressure() * 0.00750063755419211f; //0.00750063755419211
			  humidity = BME280_ReadHumidity();
			#endif
			  /*
			  memset(SndBuffer, 0, sizeof(SndBuffer));
			  for (int iii = 0; iii < MEASSURE_COUNT; iii++) {
				  sprintf(SndBuffer, "X3:%5.2f, X4:%5.2f                 \r\n", resul_arrayX3[iii], resul_arrayX4[iii]);
				  HAL_UART_Transmit(&huart1, (uint8_t *) SndBuffer, sizeof(SndBuffer), 1000);
			  }*/
			#ifdef ZABBIX_ENABLE
				#if defined(TMP117_ENABLE) || defined(BME280_ENABLE)
			  	  if ((temperature < 60.0) && (temperature > -40.0)) {
			  		  sendToZabbix(net_info.zabbix, ZabbixHostName, "ALTIM_TEMPERATURE", temperature);
					#ifdef BME280_ENABLE
					  sendToZabbix(net_info.zabbix, ZabbixHostName, "ALTIM_PRESSURE", pressure);
					  sendToZabbix(net_info.zabbix, ZabbixHostName, "ALTIM_HUMIDITY", humidity);
					#endif
			  	  } else {
					#ifdef BME280_ENABLE
			  		HAL_I2C_DeInit(&hi2c1);
			  		MX_I2C1_Init();
			  		BME280_Init();  // Сбой датчика bme280, пробуем исправить.
					#endif
			  	  }
				#endif
			  if ( V != 0 ) {
				  if ( (! firstTime) && (V < MAX_SPEED) && (Vmaxfin < MAX_SPEED) ) {  // Первый раз пропускаем для инициализации переменных.
					  sendToZabbix(net_info.zabbix, ZabbixHostName, "ALTIM_SPEED", V);
					  sendToZabbix(net_info.zabbix, ZabbixHostName, "ALTIM_DIRECT", A);
					  sendToZabbix(net_info.zabbix, ZabbixHostName, "ALTIM_MAXSPEED", Vmaxfin);
				  }
			  } else {
				  if ( (! firstTime) && (Vmaxfin < MAX_SPEED) ) {
					  sendToZabbix(net_info.zabbix, ZabbixHostName, "ALTIM_SPEED", 0);
					  sendToZabbix(net_info.zabbix, ZabbixHostName, "ALTIM_MAXSPEED", Vmaxfin);
				  }
			  }
			#endif
			  //HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
			  if ( ! firstTime ) {
				  sprintf(SndBuffer, "V:%5.2f, X:%5.2f, Y:%5.2f, Vmax:%5.2f, Xmax:%5.2f, Ymax:%5.2f, A:%3.0f, T:%5.2f, P:%8.3f, H:%5.2f   \r",
						  V, Xsum, Ysum, Vmaxfin, Xmaxfin, Ymaxfin, A, temperature, pressure, humidity);
				  HAL_UART_Transmit(&huart1, (uint8_t *) SndBuffer, sizeof(SndBuffer), 1000);
			  }
			  firstTime = FALSE;
				#ifdef SYSTICK_DISABLE
				  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;  // Выключение SysTick
				#endif
			  HAL_TIM_Base_Start_IT(&htim4);  // Перезапуск для начала измерений
		  }
	  }


	  /*
	   * Подготовка запуска процедуры калибровки
	   */
	  if(HAL_UART_Receive(&huart1, (uint8_t *) uart_buffer, 1, 10) ) {
		  if (uart_buffer[0] == 'c' ) {  // Клавиша c нажата ?
			  HAL_UART_Transmit(&huart1, (uint8_t *) CALIBRATE_TEXT, sizeof(CALIBRATE_TEXT), 1000);
			  HAL_TIM_Base_Stop_IT(&htim4); // Остановим измерения
			  STOP_CAPTURE
			  memset(SndBuffer, 0, sizeof(SndBuffer));
			  calibrate13 = TRUE;
			  calibrate24 = TRUE;
			  test_flag = FALSE;
			  calibrateCount = 0;
			  C_13 = CALIBRATE_START;
			  C_24 = CALIBRATE_START;
			  ZX1 = 0;
			  ZX2 = 0;
			  ZY1 = 0;
			  ZY2 = 0;
			  DX1.f = 0;
			  DY1.f = 0;
			  test_cnt = 0;
			  calibrateMode = MEASSURE_COUNT * CALIBRATE_TIMES;
			  currentMode = 0;
			  measCount = 0;
			  HAL_TIM_Base_Start_IT(&htim4); // Запуск измерения
		  } else {
			  if (uart_buffer[0] == 't' ) {		// Test
				  HAL_TIM_Base_Stop_IT(&htim4); // Остановим измерения
				  STOP_CAPTURE
				  HAL_UART_Transmit(&huart1, (uint8_t *) TEST_TEXT, sizeof(TEST_TEXT), 1000);
				  calibrateMode = 1;
				  calibrateCount = 0;
				  test_flag = TRUE;
				  calibrate13 = TRUE;
				  calibrate24 = TRUE;
				  measCount = 0;
				  HAL_TIM_Base_Start_IT(&htim4); // Запуск измерения
			  } else {
				  if (uart_buffer[0] == 'r' ) {		// Terminate calibration && test
					  HAL_TIM_Base_Stop_IT(&htim4); // Остановим измерения
					  STOP_CAPTURE
					  HAL_UART_Transmit(&huart1, (uint8_t *) TEST_TERMINATE, sizeof(TEST_TERMINATE), 1000);
					  test_flag = 0;
					  calibrate13 = TRUE;
					  calibrate24 = TRUE;
					  calibrateMode = 0;
					  calibrateCount = 0;
					  measCount = 0;
					  rwFlash(0);		// Чтение параметров калибровки из Flash.
					  HAL_TIM_Base_Start_IT(&htim4); // Запуск измерения
				  }
			  }
		  }
		  uart_buffer[0] = 0x00;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 810;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 91;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_1);
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_2);
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_4);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 40000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|nRst_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Z1_Pin|Z2_Pin|SCSN_Pin|Z3_Pin
                          |Z4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin nRst_Pin */
  GPIO_InitStruct.Pin = LED_Pin|nRst_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Z1_Pin Z2_Pin SCSN_Pin Z3_Pin
                           Z4_Pin */
  GPIO_InitStruct.Pin = Z1_Pin|Z2_Pin|SCSN_Pin|Z3_Pin
                          |Z4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
	if (runFlag > 0) {								// Разрешено измерение ?
		if ((htim->Instance == TIM2) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 || htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)) {
			//if ((runFlag < COUNT_FRONT) || ((GPIOA->IDR & GPIO_PIN_0) != 0) ) {  // Ждем фронт первого импульса, дальше обрабатываем все импульсы.
			if ((runFlag < COUNT_FRONT) || (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) ) {  // Ждем фронт первого импульса, дальше обрабатываем все импульсы.
				if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 ) {  // Активен фронт
					front_sum = front_sum + HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
				} else {   // Активен спад
					front_sum = front_sum + HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
				}
				runFlag--;
				if (runFlag == 0) {
					//LED_PULSE
					STOP_CAPTURE
					front_sumf = (float) front_sum / ((COUNT_FRONT + COUNT_FRONT * COUNT_FRONT) / 2);  // Расчитываем задержку от средины импульсов
					if (front_sumf > 1500) {	// Ошибка измерения.
						front_sumf = 1500;		// Значение необходимое для калибровки.
					}
					/* Отключим все мультиплексоры */
					receiversOff
					//runFlag = 0;
					#ifdef SYSTICK_DISABLE
						SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  // Включение SysTick
					#endif
					switch (currentMode) {
						case 1: { // Z1 > Z3, Z13
							resul_arrayY1[measCount] = front_sumf;
							break;
						}
						case 2: { // Z3 > Z1, Z31
							resul_arrayY2[measCount] = front_sumf;
							break;
						}
						case 3: { // Z2 > Z4 Z24
							resul_arrayX1[measCount] = front_sumf;
							break;
						}
						case 4: { // Z4 > Z2 Z42
							resul_arrayX2[measCount] = front_sumf;
							break;
						}
					}
				}
			}
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
