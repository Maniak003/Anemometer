#include "lcd.h"
//------------------------------------------------
uint8_t buf[1]={0};
extern I2C_HandleTypeDef hi2c3;
uint8_t portlcd; //€чейка дл€ хранени€ данных порта микросхемы расширени€
//------------------------------------------------
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
	micros *=(SystemCoreClock / 1000000) / 5;
	while (micros--);
}
//------------------------------------------------
void LCD_WriteByteI2CLCD(uint8_t bt)
{
	buf[0]=bt;
	HAL_I2C_Master_Transmit(&hi2c3,(uint16_t) 0x4E,buf,1,1000);
}
//------------------------------------------------
void sendhalfbyte(uint8_t c)
{
  c<<=4;
  LCD_WriteByteI2CLCD(portlcd|c);
  LCD_WriteByteI2CLCD((portlcd|=0x04)|c);//включаем линию E
  DelayMicro(1);
  LCD_WriteByteI2CLCD((portlcd&=~0x04)|c);//выключаем линию E
  DelayMicro(50);
}
//------------------------------------------------
void sendbyte(uint8_t c, uint8_t mode)
{
	if(mode==0) rs_reset();
	else rs_set();
	uint8_t hc=0;
	hc=c>>4;
	sendhalfbyte(hc);sendhalfbyte(c);
}
//------------------------------------------------
void LCD_Clear(void)
{
	sendbyte(0x01,0);
	HAL_Delay(2);
}
//------------------------------------------------
void LCD_SendChar(char ch)
{
	sendbyte(ch,1);
}
//------------------------------------------------
void LCD_String(char* st)
{
	uint8_t i=0;
	while(st[i]!=0)
	{
		sendbyte(st[i],1);
		i++;
	}
}
//------------------------------------------------
void LCD_SetPos(uint8_t x, uint8_t y)
{
	switch(y)
	{
		case 0:
			sendbyte(x|0x80,0);
			break;
		case 1:
			sendbyte((0x40+x)|0x80,0);
			break;
		case 2:
			sendbyte((0x14+x)|0x80,0);
			break;
		case 3:
			sendbyte((0x54+x)|0x80,0);
			break;
	}
}
//------------------------------------------------
void LCD_ini(void)
{
  HAL_Delay(50);
  LCD_WriteByteI2CLCD(0);
  HAL_Delay(100);
  sendhalfbyte(0x03);
  DelayMicro(4500);
  sendhalfbyte(0x03);
  DelayMicro(4500);
  sendhalfbyte(0x03);
  DelayMicro(200);
  sendhalfbyte(0x02);
  sendbyte(0x28,0);//режим 4 бит, 2 линии (дл€ нашего большого диспле€ это 4 линии, шрифт 5х8
  sendbyte(0x0C,0);//дисплей включаем (D=1), курсоры никакие не нужны
  HAL_Delay(1);
  sendbyte(0x01,0);// уберем мусор
  HAL_Delay(2);
  sendbyte(0x06,0);// пишем влево
  HAL_Delay(1);
  sendbyte(0x02,0);//курсор на место
  HAL_Delay(2);
  setled();//подсветка
  setwrite();//запись
}

