/*
  @
  @   Date               :        30.11.2018 / Sunday
  @
  @   Contact            :        Writing by Muhammet Rasit KIYAK                    @https://www.linkedin.com/in/mrstkyk/
  @                               mrstkyk@gmail.com 
  @
  @   Description        :        This Library for TMP117/TMP116 Temperature Sensor on Texas Instruments
  @                               Dependency library is HAL for STM32 series (__STM32xx_HAL_I2C_H)
  @
*/

#include "tmp117.h"

uint8_t TMP117_DeviceID=TMP117_DeviceID1;

/*  
   @Brief         Get temperature basically
   @Description   Function gives to us ambient temperature
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  Float
 */
float TMP117_get_Temperature             (I2C_HandleTypeDef i2c)
{
      static uint8_t buf[3];
      buf[0]=TMP117_TemperatureRegister;
      
      HAL_I2C_Master_Transmit(&i2c,TMP117_DeviceID,buf,0,100);
      HAL_Delay(1);
      HAL_I2C_Master_Receive(&i2c,TMP117_DeviceID,buf,1,100);
      
      return ((((buf[0]<<8)|buf[1]))*0.0078125);
}

/*  
   @Brief         Get Configuration
   @Description   Get Configuration Register Value
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  uint16_t
 */
uint16_t TMP117_get_Configuration     (I2C_HandleTypeDef i2c)
{
      static uint8_t buf[3];
      buf[0]=TMP117_ConfigurationRegister;
      
      HAL_I2C_Master_Transmit(&i2c,TMP117_DeviceID,buf,0,100);
      HAL_Delay(1);
      HAL_I2C_Master_Receive(&i2c,TMP117_DeviceID,buf,1,100);
      return ((buf[0]<<8)|buf[1]);
}

/*  
   @Brief         Set Configuration
   @Description   Set Configuration Register for Features
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
                  uint8_t first     ->  [15:8]  
                  uint8_t second    ->  [7:0]
   @Return value  void
 */
void TMP117_set_Configuration            (I2C_HandleTypeDef i2c,uint8_t first,uint8_t second)
{
      static uint8_t buf[3];
      buf[0]=TMP117_ConfigurationRegister;
      buf[1]=first;     
      buf[2]=second;    

      HAL_I2C_Master_Transmit(&i2c,TMP117_DeviceID,buf,2,100);
      HAL_Delay(1);
}

/*  
   @Brief         Set HighLimit
   @Description   Set HighLimit for Alert
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
                  uint8_t first     ->  [15:8]  
                  uint8_t second    ->  [7:0]
   @Return value  void
 */
void TMP117_set_HighLimit                (I2C_HandleTypeDef i2c,uint8_t first,uint8_t second)
{
      static uint8_t buf[3];
      buf[0]=TMP117_TemperatureHighLimit;
      buf[1]=first;     
      buf[2]=second;    

      HAL_I2C_Master_Transmit(&i2c,TMP117_DeviceID,buf,2,100);
      HAL_Delay(1);
      
}

/*  
   @Brief         Get Highlimit
   @Description   Get Highlimit Register Value
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  uint16_t
 */
uint16_t TMP117_get_HighLimit         (I2C_HandleTypeDef i2c)
{
      static uint8_t buf[3];
      buf[0]=TMP117_TemperatureHighLimit;
      
      HAL_I2C_Master_Transmit(&i2c,TMP117_DeviceID,buf,0,100);
      HAL_Delay(1);
      HAL_I2C_Master_Receive(&i2c,TMP117_DeviceID,buf,1,100);
      return ((buf[0]<<8)|buf[1]);
      
}

/*  
   @Brief         Set LowLimit
   @Description   Set LowLimit for Alert
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
                  uint8_t first     ->  [15:8]
                  uint8_t second    ->  [7:0]
   @Return value  void
 */
void TMP117_set_LowLimit                 (I2C_HandleTypeDef i2c,uint8_t first,uint8_t second)
{
      static uint8_t buf[3];
      buf[0]=TMP117_TemperatureLowLimit;
      buf[1]=first;     //Reset Value
      buf[2]=second;    //Reset Value

      HAL_I2C_Master_Transmit(&i2c,TMP117_DeviceID,buf,2,100);
      HAL_Delay(1);
      
}

/*  
   @Brief         Get LowLimit
   @Description   Get Lowlimit Register Value
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  uint16_t
 */
uint16_t TMP117_get_LowLimit          (I2C_HandleTypeDef i2c)
{
      static uint8_t buf[3];
      buf[0]=TMP117_TemperatureLowLimit;
      
      HAL_I2C_Master_Transmit(&i2c,TMP117_DeviceID,buf,0,100);
      HAL_Delay(1);
      HAL_I2C_Master_Receive(&i2c,TMP117_DeviceID,buf,1,100);
      return ((buf[0]<<8)|buf[1]);
      
}

/*  
   @Brief         Get EEPROM Unlock Register Value
   @Description   Check EEPROM for Unlock/Lock
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  uint16_t
 */
uint16_t TMP117_get_EEPROM_Unclock    (I2C_HandleTypeDef i2c)
{
      static uint8_t buf[3];
      buf[0]=TMP117_EEPROM_Uclock;
      
      HAL_I2C_Master_Transmit(&i2c,TMP117_DeviceID,buf,0,100);
      HAL_Delay(1);
      HAL_I2C_Master_Receive(&i2c,TMP117_DeviceID,buf,1,100);
      return ((buf[0]<<8)|buf[1]);
      
}

/*  
   @Brief         Set EEPROM Unlock Register Value
   @Description   Active/Inactive for EEPROM read/write situation
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
                  uint8_t first     ->  [15:8]
                  uint8_t second    ->  [7:0]
   @Return value  void
 */
void TMP117_set_EEPROM_Unlock            (I2C_HandleTypeDef i2c,uint8_t first,uint8_t second)
{
      static uint8_t buf[3];
      buf[0]=TMP117_EEPROM_Uclock;
      buf[1]=first;     
      buf[2]=second;    

      HAL_I2C_Master_Transmit(&i2c,TMP117_DeviceID,buf,2,100);
      HAL_Delay(1);
      
}

/*  
   @Brief         
   @Description   
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
                  uint8_t first     ->  [15:8]
                  uint8_t second    ->  [7:0]
   @Return value  void
 */
void TMP117_set_EEPROM1                  (I2C_HandleTypeDef i2c,uint8_t first,uint8_t second)
{
      static uint8_t buf[3];
      buf[0]=TMP117_EEPROM1;
      buf[1]=first;     //Reset Value
      buf[2]=second;    //Reset Value

      HAL_I2C_Master_Transmit(&i2c,TMP117_DeviceID,buf,2,100);
      HAL_Delay(1); 
}

/*  
   @Brief         
   @Description   
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  uint16_t
 */
uint16_t TMP117_get_EEPROM1           (I2C_HandleTypeDef i2c)
{
      static uint8_t buf[3];
      buf[0]=TMP117_EEPROM1;
      
      HAL_I2C_Master_Transmit(&i2c,TMP117_DeviceID,buf,0,100);
      HAL_Delay(1);
      HAL_I2C_Master_Receive(&i2c,TMP117_DeviceID,buf,1,100);
      return ((buf[0]<<8)|buf[1]);
      
}

/*  
   @Brief         
   @Description   
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
                  uint8_t first     ->  [15:8]
                  uint8_t second    ->  [7:0]
   @Return value  void
 */
void TMP117_set_EEPROM2                  (I2C_HandleTypeDef i2c,uint8_t first,uint8_t second)
{
      static uint8_t buf[3];
      buf[0]=TMP117_EEPROM2;
      buf[1]=first;     
      buf[2]=second;    

      HAL_I2C_Master_Transmit(&i2c,TMP117_DeviceID,buf,2,100);
      HAL_Delay(1);     
}

/*  
   @Brief         
   @Description   
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  uint16_t
 */
uint16_t TMP117_get_EEPROM2           (I2C_HandleTypeDef i2c)
{
      static uint8_t buf[3];
      buf[0]=TMP117_EEPROM2;
      
      HAL_I2C_Master_Transmit(&i2c,TMP117_DeviceID,buf,0,100);
      HAL_Delay(1);
      HAL_I2C_Master_Receive(&i2c,TMP117_DeviceID,buf,1,100);
      return ((buf[0]<<8)|buf[1]);
}

/*  
   @Brief         
   @Description   
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
                  uint8_t first     ->  [15:8]
                  uint8_t second    ->  [7:0]
   @Return value  void
 */
void TMP117_set_EEPROM3                  (I2C_HandleTypeDef i2c,uint8_t first,uint8_t second)
{
      static uint8_t buf[3];
      buf[0]=TMP117_EEPROM3;
      buf[1]=first;     
      buf[2]=second;    

      HAL_I2C_Master_Transmit(&i2c,TMP117_DeviceID,buf,2,100);
      HAL_Delay(1);
}

/*  
   @Brief         Get EEPROM3 Value
   @Description   
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  uint16_t
 */
uint16_t TMP117_get_EEPROM3           (I2C_HandleTypeDef i2c)
{
      static uint8_t buf[3];
      buf[0]=TMP117_EEPROM3;
      
      HAL_I2C_Master_Transmit(&i2c,TMP117_DeviceID,buf,0,100);
      HAL_Delay(1);
      HAL_I2C_Master_Receive(&i2c,TMP117_DeviceID,buf,1,100);
      return ((buf[0]<<8)|buf[1]);
      
}

/*  
   @Brief         Set Temperature Offset Value 
   @Description   Set Temperature Offset Value for Calibrating
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
                  uint8_t first     ->  [15:8]
                  uint8_t second    ->  [7:0]
   @Return value  void
 */
void TMP117_set_Temperature_Offset       (I2C_HandleTypeDef i2c,uint8_t first,uint8_t second)
{
      static uint8_t buf[3];
      buf[0]=TMP117_Temperature_Offset;
      buf[1]=first;     
      buf[2]=second;    

      HAL_I2C_Master_Transmit(&i2c,TMP117_DeviceID,buf,2,100);
      HAL_Delay(1);
      
}

/*  
   @Brief         Get Temperature Offset Value
   @Description   Get Temperature Offset Value for Calibrating
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  uint16_t
 */
uint16_t TMP117_get_Temperature_Offset(I2C_HandleTypeDef i2c)
{
      static uint8_t buf[3];
      buf[0]=TMP117_Temperature_Offset;
      
      HAL_I2C_Master_Transmit(&i2c,TMP117_DeviceID,buf,0,100);
      HAL_Delay(1);
      HAL_I2C_Master_Receive(&i2c,TMP117_DeviceID,buf,1,100);
      return ((buf[0]<<8)|buf[1]);
      
}

/*  
   @Brief         Get ID Register
   @Description   Check Device ID for Error Handler
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  uint16_t
 */
uint16_t TMP117_get_ID_Register       (I2C_HandleTypeDef i2c)
{
      static uint8_t buf[3];
      buf[0]=TMP117_ID_Register;
      
      HAL_I2C_Master_Transmit(&i2c,TMP117_DeviceID,buf,0,100);
      HAL_Delay(1);
      HAL_I2C_Master_Receive(&i2c,TMP117_DeviceID,buf,1,100);
      return ((buf[0]<<8)|buf[1]);
      
}

/*  
   @Brief         Custom Initialization
   @Description   Custom Parameters for Sensor
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  void
 */
void TMP117_Initialization               (I2C_HandleTypeDef i2c)
{
      TMP117_set_Configuration(i2c,0x02,0x20);
      TMP117_set_Temperature_Offset(i2c,0x00,0x00); //Default Value
      TMP117_set_LowLimit(i2c,0x12,0x80);           //Set 10 Celcius
      TMP117_set_HighLimit(i2c,0x51,0x20);          //Set 40 Celcius
}

/*  
   @Brief         Default Initialization
   @Description   Default Parameters for Sensor
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  void
 */
void TMP117_Initialization_DEFAULT       (I2C_HandleTypeDef i2c)
{
      TMP117_set_Configuration(i2c,0x02,0x20);
      TMP117_set_Temperature_Offset(i2c,0x00,0x00);
      TMP117_set_LowLimit(i2c,0x80,0x00);
      TMP117_set_HighLimit(i2c,0x60,0x00);
      TMP117_set_EEPROM_Unlock(i2c,0x00,0x00);
}