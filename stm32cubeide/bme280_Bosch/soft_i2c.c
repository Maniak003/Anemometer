#include "soft_i2c.h"




void _delay_us ( uint32_t delay)
{
      volatile uint32_t i;
      for (i=0;i<delay;i++)
      {
       i++;       
      }
}

void I2C_Init(void)
{
  LL_GPIO_SetPinMode (I2C_SDA_PORT, I2C_SDA_PIN, LL_GPIO_MODE_OUTPUT );
  LL_GPIO_SetPinMode (I2C_SCL_PORT, I2C_SCL_PIN, LL_GPIO_MODE_OUTPUT );

  LL_GPIO_SetPinOutputType (I2C_SDA_PORT, I2C_SDA_PIN, LL_GPIO_OUTPUT_OPENDRAIN  );
  LL_GPIO_SetPinOutputType (I2C_SCL_PORT, I2C_SCL_PIN, LL_GPIO_OUTPUT_OPENDRAIN );
  
  LL_GPIO_SetPinSpeed     (I2C_SDA_PORT, I2C_SDA_PIN, LL_GPIO_SPEED_FREQ_HIGH   );
  LL_GPIO_SetPinSpeed     (I2C_SCL_PORT, I2C_SCL_PIN, LL_GPIO_SPEED_FREQ_HIGH  );
    
  LL_GPIO_SetOutputPin  ( I2C_SDA_PORT, I2C_SDA_PIN );
  LL_GPIO_SetOutputPin  ( I2C_SCL_PORT, I2C_SCL_PIN );

}


void I2C_Start(void)
{
  SDA_up;
  _delay_us(TimeX);
  SCL_up;
  _delay_us(TimeX);
  while( !LL_GPIO_IsInputPinSet(I2C_SDA_PORT, I2C_SDA_PIN))
  {
    SCL_down;
    _delay_us(TimeX);
    SCL_up;
    _delay_us(TimeX);
  }
  SDA_down;
  _delay_us(TimeX);
  SCL_down;
  _delay_us(TimeX);
}


void I2C_Stop(void)
{
        SDA_down;
        _delay_us(TimeX);
        SCL_up;
        _delay_us(TimeX);
        SDA_up;
        _delay_us(TimeX);
}



uint8_t I2C_Write_Byte(uint8_t data)
{
       uint8_t i;
       volatile uint8_t ACK;
       
       for(i=0;i<8;i++)
       {
       if(data & 0x80)
       {
         SDA_up;
       }
       else
       {
         SDA_down;
       }
       _delay_us(TimeX);
       SCL_up;
       _delay_us(TimeX); 
       SCL_down;       
       data=data<<1;    
       }
       _delay_us(TimeX);
       SCL_up;
       _delay_us(TimeX);        
       ACK = LL_GPIO_IsInputPinSet(I2C_SDA_PORT, I2C_SDA_PIN);
       SCL_down;
       SDA_down;
       return ACK;
}


uint8_t I2C_Write_Block(uint8_t address_ic, uint8_t address_reg, uint8_t* I2C_Buffer, uint16_t length)
{
        uint16_t i ;
        volatile uint8_t res = 1;
        
        I2C_Start();
        res = I2C_Write_Byte(address_ic & ~0x01);
        if (res ==0) res = I2C_Write_Byte(address_reg);
        
        if (res == 0)
        {
          for(i=0;i<length;i++)
          {
            res = I2C_Write_Byte(I2C_Buffer[i]);
            if (res != 0) break;
          }
        }        
        I2C_Stop();
        return res;       
}


uint8_t I2C_Read_Byte(uint8_t ACK)
{
        uint8_t i;
        uint8_t data;
        
         SDA_up;
         for(i=0;i<8;i++)
         {
           _delay_us(TimeX);        
           SCL_up;        
           _delay_us(TimeX);       
           data<<=1;
           if(LL_GPIO_IsInputPinSet(I2C_SDA_PORT, I2C_SDA_PIN))
            data++; 
           SCL_down;
         }
      if (ACK) SDA_down;
        _delay_us(TimeX); 
        SCL_up;
        _delay_us(TimeX);
        SCL_down;
        SDA_up;
        return data;
}


uint8_t I2C_Read_Block(uint8_t address_ic, uint8_t address_reg, uint8_t* I2C_Buffer, uint16_t length)
{
        uint16_t i ;
        volatile uint8_t res = 1;
        
        I2C_Start();
        res = I2C_Write_Byte(address_ic & ~0x01);
        if (res == 0) res = I2C_Write_Byte(address_reg);
        I2C_Start();
        if (res == 0) res = I2C_Write_Byte(address_ic | 0x01);
        if (res == 0)
        {
          for(i=0;i<length;i++)
          {
            I2C_Buffer[i] = I2C_Read_Byte(i!=(length-1));      
          }
        }
        I2C_Stop();
        return res;               
}
