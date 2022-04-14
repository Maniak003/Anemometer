#ifndef LCD_H_
#define LCD_H_
//------------------------------------------------
#include "stm32f4xx_hal.h"
//------------------------------------------------
#define e_set() LCD_WriteByteI2CLCD(portlcd|=0x04)  //��������� ����� � � 1
#define e_reset() LCD_WriteByteI2CLCD(portlcd&=~0x04) //��������� ����� � � 0
#define rs_set() LCD_WriteByteI2CLCD(portlcd|=0x01) //��������� ����� RS � 1
#define rs_reset() LCD_WriteByteI2CLCD(portlcd&=~0x01) //��������� ����� RS � 0
#define setled() LCD_WriteByteI2CLCD(portlcd|=0x08) //��������� ����� RS � 1
#define setwrite() LCD_WriteByteI2CLCD(portlcd&=~0x02) //��������� ����� RS � 0
//------------------------------------------------
void LCD_ini(void);
void LCD_Clear(void);
void LCD_SendChar(char ch);
void LCD_String(char* st);
void LCD_SetPos(uint8_t x, uint8_t y);
//------------------------------------------------
#endif /* LCD_H_ */
