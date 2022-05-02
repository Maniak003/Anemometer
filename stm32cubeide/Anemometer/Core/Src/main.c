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

  Clock timers must be 64MHz.

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

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef	flash_ok;
char uart_buffer[10] = {0,};
char SndBuffer[200] = {0,};
uint32_t fastCounter;
wiz_NetInfo net_info = {
	.mac  = { 0x00, 0x11, 0x22, 0x33, 0x44, 0xEA },
	.dhcp = NETINFO_DHCP
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


	void rwFlash(uint8_t rwFlag) {
		uint32_t pageAdr = 0x800FC00; //.
		uint32_t magicKey;
		uint64_t dataForSave;
		magicKey = *(__IO uint32_t*) pageAdr;
		if ((magicKey != 0x12349876) || (rwFlag == 1)) { // rwFlag == 1 for wrtite data to flash
			magicKey = 0x12349876;
			//sprintf(SndBuffer, "\r\nWrite FLASH.\r\n");
			//HAL_UART_Transmit(&huart1, (uint8_t *) SndBuffer, sizeof(SndBuffer), 1000);
			if (rwFlag == 0) { // For first initial
				C_12 = CALIBRATE_START;
				C_34 = CALIBRATE_START;
				C_14 = CALIBRATE_START;
				C_23 = CALIBRATE_START;
				DX1.f = 0;
				DX2.f = 0;
				DY1.f = 0;
				DY2.f = 0;
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
				dataForSave = (uint64_t) (C_12 | ((uint64_t) C_34 << 16) | ((uint64_t) C_14 << 32) | ((uint64_t) C_23 << 48));
				flash_ok = HAL_ERROR;
				while(flash_ok != HAL_OK){
					flash_ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, pageAdr + 16, dataForSave); // Write C_12 C_34 C_14 C_23
				}
				flash_ok = HAL_ERROR;
				while(flash_ok != HAL_OK){
					flash_ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, pageAdr + 24, DX1.u); // Write DX1
				}
				flash_ok = HAL_ERROR;
				while(flash_ok != HAL_OK){
					flash_ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, pageAdr + 28, DX2.u); // Write DX2
				}
				flash_ok = HAL_ERROR;
				while(flash_ok != HAL_OK){
					flash_ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, pageAdr + 32, DY1.u); // Write DY1
				}
				flash_ok = HAL_ERROR;
				while(flash_ok != HAL_OK){
					flash_ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, pageAdr + 36, DY2.u); // Write DY2
				}
			}
			// Lock flash
			flash_ok = HAL_ERROR;
			while(flash_ok != HAL_OK){
				flash_ok = HAL_FLASH_Lock();
			}
		} else {
			//sprintf(SndBuffer, "\r\nMagick key: %x.\r\n", (uint) magicKey);
			//HAL_UART_Transmit(&huart1, (uint8_t *) SndBuffer, sizeof(SndBuffer), 1000);
			C_12 = *(__IO uint16_t*) (pageAdr + 16);
			C_34 = *(__IO uint16_t*) (pageAdr + 18);
			C_14 = *(__IO uint16_t*) (pageAdr + 20);
			C_23 = *(__IO uint16_t*) (pageAdr + 22);
			sprintf(SndBuffer, "C_12: %5d, C_34: %5d, C_14: %5d, C_23: %5d.\r\n", C_12, C_34, C_14, C_23);
			HAL_UART_Transmit(&huart1, (uint8_t *) SndBuffer, sizeof(SndBuffer), 1000);
			DX1.u = *(__IO uint32_t*) (pageAdr + 24);
			DX2.u = *(__IO uint32_t*) (pageAdr + 28);
			DY1.u = *(__IO uint32_t*) (pageAdr + 32);
			DY2.u = *(__IO uint32_t*) (pageAdr + 36);
			sprintf(SndBuffer, "DX1: %5.2f, DX2: %5.2f, DY1: %5.2f, DY2: %5.2f.\r\n", DX1.f, DX2.f, DY1.f, DY2.f);
			HAL_UART_Transmit(&huart1, (uint8_t *) SndBuffer, sizeof(SndBuffer), 1000);
		}
	}


/*
int  writeSector(uint32_t Address, void * values, uint16_t size) {
	uint16_t *AddressPtr;
	uint16_t *valuePtr;
	AddressPtr = (uint16_t *)Address;
	valuePtr=(uint16_t *)values;
	size = size / 2;  // incoming value is expressed in bytes, not 16 bit words
	while(size) {
		// unlock the flash
		// Key 1 : 0x45670123
		// Key 2 : 0xCDEF89AB
		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
		FLASH->CR &= ~BIT1; // ensure PER is low
		FLASH->CR |= BIT0;  // set the PG bit
		*(AddressPtr) = *(valuePtr);
		while(FLASH->SR & BIT0); // wait while busy
		if (FLASH->SR & BIT2)
			return -1; // flash not erased to begin with
		if (FLASH->SR & BIT4)
			return -2; // write protect error
		AddressPtr++;
		valuePtr++;
		size--;
	}
	return 0;
}
void eraseSector(uint32_t SectorStartAddress)
{
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;
	FLASH->CR &= ~BIT0;  // Ensure PG bit is low
	FLASH->CR |= BIT1; // set the PER bit
	FLASH->AR = SectorStartAddress;
	FLASH->CR |= BIT6; // set the start bit
	while(FLASH->SR & BIT0); // wait while busy
}
void readSector(uint32_t SectorStartAddress, void * values, uint16_t size)
{
	uint16_t *AddressPtr;
	uint16_t *valuePtr;
	AddressPtr = (uint16_t *)SectorStartAddress;
	valuePtr=(uint16_t *)values;
	size = size/2; // incoming value is expressed in bytes, not 16 bit words
	while(size)
	{
		*((uint16_t *)valuePtr)=*((uint16_t *)AddressPtr);
		valuePtr++;
		AddressPtr++;
		size--;
	}
}
*/


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
	HAL_GPIO_WritePin(GPIOB, W5500_CS_Pin, GPIO_PIN_RESET);
}

void W5500_Unselect(void) {
	HAL_GPIO_WritePin(GPIOB, W5500_CS_Pin, GPIO_PIN_SET);
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
 *      	hardware ethernet 00:11:22:33:44:ea;
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
    /* Read data from Zabbix
#ifdef ZABBIX_DEBUG
    UART_Printf("Read.\r\n");
#endif
    {
        char buff[32];
        for(;;) {
            int32_t nbytes = recv(tcp_socket, (uint8_t*)&buff, sizeof(buff)-1);
            if(nbytes == SOCKERR_SOCKSTATUS) {
			#ifdef ZABBIX_DEBUG
                UART_Printf("\r\nDisconnect.\r\n");
			#endif
                break;
            }

            if(nbytes <= 0) {
			#ifdef ZABBIX_DEBUG
                UART_Printf("\r\nrecv() failed, %d\r\n", nbytes);
			#endif
                break;
            }

            buff[nbytes] = '\0';
			#ifdef ZABBIX_DEBUG
            UART_Printf("%s", buff);
			#endif
        }
    }
    */
	#ifdef ZABBIX_DEBUG
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
//    wiz_NetInfo net_info = {
//        .mac  = { 0x00, 0x11, 0x22, 0x33, 0x44, 0xEA },
//        .dhcp = NETINFO_DHCP
//    };
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
    uint32_t ctr = 10000;
    while((!ip_assigned) && (ctr > 0)) {
        DHCP_run();
        ctr--;
        HAL_Delay(100);
    }
    if(!ip_assigned) {
		#ifdef ZABBIX_DEBUG
        UART_Printf("\r\nIP was not assigned :(\r\n");
		#endif
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

    // Test request.
    //sendToZabbix(net_info.zabbix, "Ed", "ALTIM_DIRECT", 11);
    //sendToZabbix(net_info.zabbix, "Ed", "ALTIM_SPEED", 2.5);
/*
    UART_Printf("Calling DNS_init()...\r\n");
    DNS_init(DNS_SOCKET, dns_buffer);

    uint8_t addr[4];
    {
        char domain_name[] = "eax.me";
        UART_Printf("Resolving domain name \"%s\"...\r\n", domain_name);
        int8_t res = DNS_run(dns, (uint8_t*)&domain_name, addr);
        if(res != 1) {
            UART_Printf("DNS_run() failed, res = %d", res);
            return;
        }
        UART_Printf("Result: %d.%d.%d.%d\r\n", addr[0], addr[1], addr[2], addr[3]);
    }
*/
}


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
  readyFlag = TRUE;
  sumCounter2 = 0;
  /* Turn off all multiplexer */
  GPIOB->ODR &= ~((1 << Z1Receive) | (1 << Z2Receive) | (1 << Z3Receive) | (1 << Z4Receive));
  /*
   * currentMode
   * Z1--Z2
   * |	  |
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
  HAL_UART_Transmit(&huart1, (uint8_t *) START_TEXT, sizeof(START_TEXT), HAL_MAX_DELAY);
#ifdef ZABBIX_ENABLE
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);	// Reset W5500
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, W5500_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(2000);
  init_w5500();
#endif
  rwFlash(0);		// Чтение параметров калибровки из Flash.
//  C_12 = 37640;
//  C_34 = 38450;
//  C_14 = 37600;
//  C_23 = 38500;
  firstTime = TRUE;
  currentMode = 0;
  STOP_CAPTURE
  measCount = 0;
  /*
   * calibrateMode == 0 -- Нормальный режим
   * calibrateMode > 0 -- Режим калибровки
   */
  calibrateMode = 0;
  Xsum = 0;
  Ysum = 0;
  Vmax = 0;
  /*
   *	Очистка массива результатов.
   */
  for (int ii = 0; ii < MEASSURE_COUNT; ii++) {
	  resul_arrayX[ii] = 0;
	  resul_arrayY[ii] = 0;
  }
  calibrate12 = FALSE;
  calibrate34 = FALSE;
  calibrate14 = FALSE;
  calibrate23 = FALSE;
#ifdef BME280_ENABLE
  BME280_Init();
#endif
#ifdef TMP117_ENABLE
  //TMP117_Initialization_DEFAULT(hi2c1);
#endif
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4); // Запуск измерения
  HAL_UART_Transmit(&huart1, (uint8_t *) INIT_FINISH_TEXT, sizeof(INIT_FINISH_TEXT), HAL_MAX_DELAY);
  readyFlag = FALSE;
  currentMode = 0;
  HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);	// LED off

  while (1) {
	  //__HAL_TIM_SET_COUNTER(&htim2, 0x0000);
	  //HAL_Delay(1);
	  if (readyFlag) {
		  readyFlag = FALSE;
		  if (calibrateMode > 0) {
			  //HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
			  //sprintf(SndBuffer, "Y1:%7d, Y2:%7d   \r", Z14 - Z41, Z23 - Z32);
			  //sprintf(SndBuffer, "Z14:%7d, Z41:%7d, Z23:%7d, Z32:%7d   \r", Z14, Z41, Z23, Z32);
			  // X
			  //sprintf(SndBuffer, "Z12:%7d, Z21:%7d, Z43:%7d, Z34:%7d   \r", Z12, Z21, Z43, Z34);
			  //sprintf(SndBuffer, "X:%7.0f, Y:%7.0f   \r", X, Y);
			  // X + Y
			  //sprintf(SndBuffer, "Z12:%5d, Z21:%5d, Z43:%5d, Z34:%5d, Z14:%5d, Z41:%5d, Z23:%5d, Z32:%5d   \r", Z12, Z21, Z43, Z34, Z14, Z41, Z23, Z32);
			  //HAL_UART_Transmit(&huart1, (uint8_t *) SndBuffer, sizeof(SndBuffer), 1000);

			  /* Процедура калибровки */
			  if ((calibrate12 || calibrate34 || calibrate14 || calibrate23) && (calibrateCount < 800)) {
				  sprintf(SndBuffer, "Z12-Z21:%5d-%5d, Z43-Z34:%5d-%5d, Z14-Z41:%5d-%5d, Z23-Z32:%5d-%5d   \r", Z12, Z21, Z43, Z34, Z14, Z41, Z23, Z32);
				  HAL_UART_Transmit(&huart1, (uint8_t *) SndBuffer, sizeof(SndBuffer), 1000);
				  if ( calibrate12 && (abs(Z12 + Z21 - 1600) > CALIBRATE_ACURACY) ) {
					  if (Z12 > 800) {
						  C_12++;
					  } else {
						  C_12--;
					  }
				  } else {
					  calibrate12 = FALSE;
				  }
				  if ( calibrate34 && (abs(Z34 + Z43 - 1600) > CALIBRATE_ACURACY) ) {
					  if (Z34 > 800) {
						  C_34++;
					  } else {
						  C_34--;
					  }
				  } else {
					  calibrate34 = FALSE;
				  }
				  if ( calibrate14 && (abs(Z14 + Z41 - 1600) > CALIBRATE_ACURACY) ) {
					  if(Z14 > 800) {
						  C_14++;
					  } else {
						  C_14--;
					  }
				  } else {
					  calibrate14 = FALSE;
				  }
				  if ( calibrate23 && (abs(Z23 + Z32 - 1600) > CALIBRATE_ACURACY) ) {
					  if(Z23 > 800) {
					  C_23++;
					  } else {
						  C_23--;
					  }
				  } else {
					  calibrate23 = FALSE;
				  }
				  calibrateCount++;
				  HAL_TIM_Base_Start_IT(&htim4);  // Перезапуск для начала измерений
			  } else {
				  if (/*(calibrate12 || calibrate34 || calibrate14 || calibrate23) &&*/ (calibrateCount >= 1600)) {
					  HAL_UART_Transmit(&huart1, (uint8_t *) CALIBRATE_ERROR_TOUT, sizeof(CALIBRATE_ERROR_TOUT), 1000);
					  /* System restart if calibrate error. */
					  HAL_NVIC_SystemReset();
				  }
				  if (calibrateMode > 0) {
					  DX1.f = DX1.f + (float) (Z12 - Z21);
					  DX2.f = DX2.f + (float) (Z43 - Z34);
					  DY1.f = DY1.f + (float) (Z14 - Z41);
					  DY2.f = DY2.f + (float) (Z23 - Z32);
					  calibrateMode--;
					  if (calibrateMode == 0) {
						  /* Вычисление поправок */
						  DX1.f = DX1.f / (float) MEASSURE_COUNT;
						  DX2.f = DX2.f / (float) MEASSURE_COUNT;
						  DY1.f = DY1.f / (float) MEASSURE_COUNT;
						  DY2.f = DY2.f / (float) MEASSURE_COUNT;
						  memset(SndBuffer, 0, sizeof(SndBuffer));
						  sprintf(SndBuffer, "\r\nCalibrate complite.\r\nC_12:%5d, C_34:%5d, C_14:%5d, C_23:%5d\r\n", C_12, C_34, C_14, C_23);
						  HAL_UART_Transmit(&huart1, (uint8_t *) SndBuffer, sizeof(SndBuffer), 1000);
						  memset(SndBuffer, 0, sizeof(SndBuffer));
						  sprintf(SndBuffer, "\r\nDX1:%5.2f, DX2:%5.2f, DY1:%5.2f, DY2:%5.2f\r\n", DX1.f, DX2.f, DY1.f, DY2.f);
						  HAL_UART_Transmit(&huart1, (uint8_t *) SndBuffer, sizeof(SndBuffer), 1000);
						  if (abs(DX1.f) < 30 && abs(DX2.f) < 30 && abs(DY1.f) < 30 && abs(DY2.f) < 30) {
							  rwFlash(1);  // Запись данных калибровки во Flash.
						  } else {
							  HAL_UART_Transmit(&huart1, (uint8_t *) CALIBRATE_ERROR_RANGE, sizeof(CALIBRATE_ERROR_RANGE), 1000);
						  }
						  calibrateCount = 0;
						  firstTime = TRUE;
					  }
				  }
				  HAL_TIM_Base_Start_IT(&htim4);  // Перезапуск для начала измерений
			  }
			  //HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
		  } else {
			#ifdef TMP117_ENABLE
			  temperature = TMP117_get_Temperature(hi2c1);
			#endif
			#ifdef BME280_ENABLE
			  temperature = BME280_ReadTemperature();
			  pressure = BME280_ReadPressure() * 0.000750061683f;
			  humidity = BME280_ReadHumidity();
			#endif
			#ifdef ZABBIX_ENABLE
				#if defined(TMP117_ENABLE) || defined(BME280_ENABLE)
				  sendToZabbix(net_info.zabbix, ZabbixHostName, "ALTIM_TEMPERATURE", temperature);
					#ifdef BME280_ENABLE
					  sendToZabbix(net_info.zabbix, ZabbixHostName, "ALTIM_PRESSURE", pressure);
					  sendToZabbix(net_info.zabbix, ZabbixHostName, "ALTIM_HUMIDITY", humidity);
					#endif
				#endif
			  if ( A != 0 ) {
				  if ( (! firstTime) && (V < 40) && (Vmax < 40) ) {  // Первый раз пропускаем для инициализации переменных.
					  sendToZabbix(net_info.zabbix, ZabbixHostName, "ALTIM_SPEED", V);
					  sendToZabbix(net_info.zabbix, ZabbixHostName, "ALTIM_DIRECT", A);
					  sendToZabbix(net_info.zabbix, ZabbixHostName, "ALTIM_MAXSPEED", Vmax);
				  }
			  } else {
				  if ( (! firstTime) && (Vmax < 40) ) {
					  sendToZabbix(net_info.zabbix, ZabbixHostName, "ALTIM_SPEED", 0);
					  sendToZabbix(net_info.zabbix, ZabbixHostName, "ALTIM_MAXSPEED", Vmax);
				  }
			  }
			#endif
			  if ( ! firstTime ) {
				  sprintf(SndBuffer, "X:%5.2f, Y:%5.2f, V:%5.2f, Vmax:%5.2f, A:%3.0f, T:%5.2f, P:%8.3f, H:%5.2f   \r", Xsum, Ysum, V, Vmax, A, temperature, pressure, humidity);
				  HAL_UART_Transmit(&huart1, (uint8_t *) SndBuffer, sizeof(SndBuffer), 1000);
			  }
			  firstTime = FALSE;
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
			  HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_1);
			  calibrate12 = TRUE;
			  calibrate34 = TRUE;
			  calibrate14 = TRUE;
			  calibrate23 = TRUE;
			  calibrateCount = 0;
			  C_12 = CALIBRATE_START;
			  C_34 = CALIBRATE_START;
			  C_14 = CALIBRATE_START;
			  C_23 = CALIBRATE_START;
			  DX1.f = 0;
			  DX2.f = 0;
			  DY1.f = 0;
			  DY2.f = 0;
			  calibrateMode = MEASSURE_COUNT;
			  currentMode = 0;
			  HAL_TIM_Base_Start_IT(&htim4); // Запуск измерения
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  htim1.Init.Period = 799;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 7;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
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
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  htim3.Init.Period = 30800;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
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
  htim4.Init.Prescaler = 2;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

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
  HAL_GPIO_WritePin(GPIOB, Z1Receive_Pin|Z2Receive_Pin|Z3Receive_Pin|Z4Receive_Pin
                          |Eth_CS_Pin|Eth_int_Pin|Eth_rst_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|TP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Z1Receive_Pin Z2Receive_Pin Z3Receive_Pin Z4Receive_Pin */
  GPIO_InitStruct.Pin = Z1Receive_Pin|Z2Receive_Pin|Z3Receive_Pin|Z4Receive_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Eth_CS_Pin Eth_int_Pin Eth_rst_Pin */
  GPIO_InitStruct.Pin = Eth_CS_Pin|Eth_int_Pin|Eth_rst_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin TP_Pin */
  GPIO_InitStruct.Pin = LED_Pin|TP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* Процедура измерения */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
	if (runFlag > 0) {								// Разрешено измерение ?
	  if ((htim->Instance == TIM2) && (HAL_TIM_ACTIVE_CHANNEL_1 || htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)) {
			if ( (runFlag < COUNT_FRONT) || ((GPIOA->IDR & GPIO_PIN_0) != 0) ) {  // Ждем фронт первого импульса, дальше пропускаем все импульсы.
				  //LED_PULSE
					runFlag--;
					if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 ) {  // Активен фронт
						front_sum = front_sum + (uint16_t) (HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1) & 0x0FFFF);
					} else {   // Активен спад
						front_sum = front_sum + (uint16_t) (HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2) & 0x0FFFF);
					}
					if (runFlag == 0) {  // Измерения закончены ?
						STOP_CAPTURE  // Таймер больше не нужен, выключаем
						#ifdef SYSTICK_DISABLE
							SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  // Включение SysTick
						#endif
						front_sum = front_sum / 4 - 1200;  // Расчитываем задержку от средины импульсов
						/* Turn off all multiplexer */
						GPIOB->ODR &= ~((1 << Z1Receive) | (1 << Z2Receive) | (1 << Z3Receive) | (1 << Z4Receive));
						switch (currentMode) {
							case 1: { // Z1 > Z2
								Z12 = front_sum;
								break;
							}
							case 2: { // Z2 > Z1
								Z21 = front_sum;
								break;
							}
							case 3: { // Z2 > Z3
								Z23 = front_sum;
								break;
							}
							case 4: { // Z3 > Z2
								Z32 = front_sum;
								break;
							}
							case 5: { // Z3 > Z4
								Z34 = front_sum;
								break;
							}
							case 6: { // Z4 > Z3
								Z43 = front_sum;
								break;
							}
							case 7: { // Z4 > Z1
								Z41 = front_sum;
								break;
							}
							case 8: { // Z1 > Z4
								Z14 = front_sum;
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
