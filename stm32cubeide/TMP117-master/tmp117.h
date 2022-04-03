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

#ifndef      _TMP117_H_	
#define      _TMP117_H_	

/*
Check the model of your MCU model
*/
#ifndef     __STM32L0xx_HAL_I2C_H
#define     __STM32L0xx_HAL_I2C_H
#endif	

#define     TMP117_DeviceID1 0x48<<1    //	GND
#define     TMP117_DeviceID2 0x49<<1    //	Vcc
#define     TMP117_DeviceID3 0x4A<<1    //	SDA
#define     TMP117_DeviceID4 0x4B<<1    //	SCL


/*
Pointer Registers
*/
#define     TMP117_TemperatureRegister     0x00
#define     TMP117_ConfigurationRegister   0x01
#define     TMP117_TemperatureHighLimit    0x02
#define     TMP117_TemperatureLowLimit     0x03

#define     TMP117_EEPROM_Uclock           0x04
#define     TMP117_EEPROM1                 0x05
#define     TMP117_EEPROM2                 0x06
#define     TMP117_EEPROM3                 0x08

#define     TMP117_Temperature_Offset      0x07
#define     TMP117_ID_Register             0x0F

/*  
   @Brief         Get temperature basically
   @Description   Function gives to us ambient temperature
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  Float
 */
float TMP117_get_Temperature             (I2C_HandleTypeDef i2c);

/*  
   @Brief         Get Configuration
   @Description   Get Configuration Register Value
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  uint16_t
 */
uint16_t TMP117_get_Configuration     (I2C_HandleTypeDef i2c);

/*  
   @Brief         Set Configuration
   @Description   Set Configuration Register for Features
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
                  uint8_t first     ->  [15:8]  
                  uint8_t second    ->  [7:0]
   @Return value  void
 */
void TMP117_set_Configuration            (I2C_HandleTypeDef i2c,uint8_t first,uint8_t second);

/*  
   @Brief         Set HighLimit
   @Description   Set HighLimit for Alert
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
                  uint8_t first     ->  [15:8]  
                  uint8_t second    ->  [7:0]
   @Return value  void
 */
void TMP117_set_HighLimit                (I2C_HandleTypeDef i2c,uint8_t first,uint8_t second);

/*  
   @Brief         Get Highlimit
   @Description   Get Highlimit Register Value
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  uint16_t
 */
uint16_t TMP117_get_HighLimit         (I2C_HandleTypeDef i2c);

/*  
   @Brief         Set LowLimit
   @Description   Set LowLimit for Alert
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
                  uint8_t first     ->  [15:8]
                  uint8_t second    ->  [7:0]
   @Return value  void
 */
void TMP117_set_LowLimit                 (I2C_HandleTypeDef i2c,uint8_t first,uint8_t second);

/*  
   @Brief         Get LowLimit
   @Description   Get Lowlimit Register Value
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  uint16_t
 */
uint16_t TMP117_get_LowLimit          (I2C_HandleTypeDef i2c);

/*  
   @Brief         Get EEPROM Unlock Register Value
   @Description   Check EEPROM for Unlock/Lock
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  uint16_t
 */
uint16_t TMP117_get_EEPROM_Unclock    (I2C_HandleTypeDef i2c);

/*  
   @Brief         Set EEPROM Unlock Register Value
   @Description   Active/Inactive for EEPROM read/write situation
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
                  uint8_t first     ->  [15:8]
                  uint8_t second    ->  [7:0]
   @Return value  void
 */
void TMP117_set_EEPROM_Unlock            (I2C_HandleTypeDef i2c,uint8_t first,uint8_t second);

/*  
   @Brief         
   @Description   
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
                  uint8_t first     ->  [15:8]
                  uint8_t second    ->  [7:0]
   @Return value  void
 */
void TMP117_set_EEPROM1                  (I2C_HandleTypeDef i2c,uint8_t first,uint8_t second);

/*  
   @Brief         
   @Description   
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  uint16_t
 */
uint16_t TMP117_get_EEPROM1           (I2C_HandleTypeDef i2c);

/*  
   @Brief         
   @Description   
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
                  uint8_t first     ->  [15:8]
                  uint8_t second    ->  [7:0]
   @Return value  void
 */
void TMP117_set_EEPROM2                  (I2C_HandleTypeDef i2c,uint8_t first,uint8_t second);

/*  
   @Brief         
   @Description   
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  uint16_t
 */
uint16_t TMP117_get_EEPROM2           (I2C_HandleTypeDef i2c);

/*  
   @Brief         
   @Description   
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
                  uint8_t first     ->  [15:8]
                  uint8_t second    ->  [7:0]
   @Return value  void
 */
void TMP117_set_EEPROM3                  (I2C_HandleTypeDef i2c,uint8_t first,uint8_t second);

/*  
   @Brief         Get EEPROM3 Value
   @Description   
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  uint16_t
 */
uint16_t TMP117_get_EEPROM3           (I2C_HandleTypeDef i2c);

/*  
   @Brief         Set Temperature Offset Value 
   @Description   Set Temperature Offset Value for Calibrating
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
                  uint8_t first     ->  [15:8]
                  uint8_t second    ->  [7:0]
   @Return value  void
 */
void TMP117_set_Temperature_Offset       (I2C_HandleTypeDef i2c,uint8_t first,uint8_t second);

/*  
   @Brief         Get Temperature Offset Value
   @Description   Get Temperature Offset Value for Calibrating
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  uint16_t
 */
uint16_t TMP117_get_Temperature_Offset (I2C_HandleTypeDef i2c);

/*  
   @Brief         Get ID Register
   @Description   Check Device ID for Error Handler
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  uint16_t
 */
uint16_t TMP117_get_ID_Register       (I2C_HandleTypeDef i2c);

/*  
   @Brief         Custom Initialization
   @Description   Custom Parameters for Sensor
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  void
 */
void TMP117_Initialization               (I2C_HandleTypeDef i2c);

/*  
   @Brief         Default Initialization
   @Description   Default Parameters for Sensor
   @Parameter     I2C_HandleTypeDef ->  HAL_I2C Handle
   @Return value  void
 */
void TMP117_Initialization_DEFAULT       (I2C_HandleTypeDef i2c);

#endif