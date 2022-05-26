#ifndef soft_i2c_f
#define soft_i2c_f

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "stm32f1xx_ll_gpio.h"



#define I2C_SDA_PORT  GPIOB
#define I2C_SDA_PIN   LL_GPIO_PIN_9 

#define I2C_SCL_PORT  GPIOB
#define I2C_SCL_PIN   LL_GPIO_PIN_8 

#define TimeX 15

#define SDA_up    I2C_SDA_PORT->BSRR = (I2C_SDA_PIN >> GPIO_PIN_MASK_POS) & 0x0000FFFFU 
#define SDA_down  I2C_SDA_PORT->BRR = (I2C_SDA_PIN >> GPIO_PIN_MASK_POS) & 0x0000FFFFU

#define SCL_up    I2C_SCL_PORT->BSRR = (I2C_SCL_PIN >> GPIO_PIN_MASK_POS) & 0x0000FFFFU
#define SCL_down  I2C_SCL_PORT->BRR = (I2C_SCL_PIN >> GPIO_PIN_MASK_POS) & 0x0000FFFFU


void _delay_us ( uint32_t delay);
void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_Write_Byte(uint8_t data);
uint8_t I2C_Write_Block(uint8_t address_ic, uint8_t address_reg, uint8_t* I2C_Buffer, uint16_t length);
uint8_t I2C_Read_Byte(uint8_t ACK);
uint8_t I2C_Read_Block(uint8_t address_ic, uint8_t address_reg, uint8_t* I2C_Buffer, uint16_t length);






#endif /* soft_i2c */