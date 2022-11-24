
#include "bme280.h"
#include "soft_i2c.h"


struct bme280_dev sensor = {0};
struct bme280_data sensorData = {0};



// **************************** Функции чтения софтовым I2C данных **********************

int8_t read_bme280  (uint8_t devID, uint8_t addr, uint8_t* data, uint16_t count)//(uint8_t reg_adr, uint8_t *reg_data, uint16_t len, uint8_t dev_adr)
{
  int8_t result = BME280_E_NVM_COPY_FAILED;
  
  result = I2C_Read_Block(devID, addr, data, count); //функция софтового I2C (файл soft_i2c.c)
 
     return result;
}

int8_t write_bme280 (uint8_t devID, uint8_t addr, uint8_t* data, uint16_t count)
{
   int8_t result = BME280_E_NVM_COPY_FAILED;
  
  result = I2C_Write_Block(devID, addr, data, count);  //функция софтового I2C (файл soft_i2c.c)

     return result;
}

void delay_bme280 (uint16_t time)
{
  osDelay(time);
}

//**********************************************************************












// **************************** Функции чтения данных из BME280  **********************

int8_t read_bme280  (uint8_t devID, uint8_t addr, uint8_t* data, uint16_t count)//(uint8_t reg_adr, uint8_t *reg_data, uint16_t len, uint8_t dev_adr)
{
  int8_t result = BME280_E_NVM_COPY_FAILED;
  
//  return_i2c = HAL_I2C_Mem_Read(&hi2c1, devID, addr, I2C_MEMADD_SIZE_8BIT, data, count, 100);
  result = I2C_Read_Block(devID, addr, data, count);
  
//  if (return_i2c == HAL_OK) return 0;
//   else 
     return result;
}

int8_t write_bme280 (uint8_t devID, uint8_t addr, uint8_t* data, uint16_t count)
{
   int8_t result = BME280_E_NVM_COPY_FAILED;
  
  result = I2C_Write_Block(devID, addr, data, count);

     return result;
}

void delay_bme280 (uint16_t time)
{
  osDelay(time);
}
//****************************************************************************










 //******************************** BME280 Init *****************************************
uint8_t measureSet = 0;
	
	 I2C_Init();

	sensor.dev_id = (BME280_I2C_ADDR_PRIM << 1);
	sensor.intf = BME280_I2C_INTF;
	sensor.read = read_bme280;
	sensor.write = write_bme280;
	sensor.delay_ms = delay_bme280;

	bme280_init(&sensor);

	//	set the temperature, pressure and humidity settings
	sensor.settings.osr_h = BME280_OVERSAMPLING_1X;
	sensor.settings.osr_p = BME280_OVERSAMPLING_16X;
	sensor.settings.osr_t = BME280_OVERSAMPLING_2X;
	sensor.settings.filter = BME280_FILTER_COEFF_16;
	sensor.settings.standby_time = BME280_STANDBY_TIME_125_MS;

	// set the required sensor settings needed
	measureSet = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

	//	set the desired sensor configuration
	bme280_set_sensor_settings(measureSet, &sensor);

	//	set sensor mode
	bme280_set_sensor_mode(BME280_NORMAL_MODE, &sensor);

//*******************************************************************************************





bme280_get_sensor_data(BME280_PRESS|BME280_TEMP, &sensorData, &sensor); //функция получения данных из BME280 (сейчас только давление и температура)












