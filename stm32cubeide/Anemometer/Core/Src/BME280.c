#include "BME280.h"
#include "main.h"
//------------------------------------------------
extern I2C_HandleTypeDef hi2c1;
extern char str1[100];
BME280_CalibData CalibData;
int32_t t_fine;
//------------------------------------------------
void Error(void)
{
}
//------------------------------------------------
static void I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Write(&hi2c1, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 0x10000);
  if(status != HAL_OK) Error();
}
//------------------------------------------------
static uint8_t I2Cx_ReadData(uint16_t Addr, uint8_t Reg)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0;
  status = HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 0x10000);
  if(status != HAL_OK) Error();
  return value;
}
//------------------------------------------------
static void I2Cx_ReadData16(uint16_t Addr, uint8_t Reg, uint16_t *Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)Value, 2, 0x10000);
  if(status != HAL_OK) Error();
}
//------------------------------------------------
static void I2Cx_ReadData24(uint16_t Addr, uint8_t Reg, uint32_t *Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)Value, 3, 0x10000);
  if(status != HAL_OK) Error();
}
//------------------------------------------------
void BME280_WriteReg(uint8_t Reg, uint8_t Value)
{
  I2Cx_WriteData(BME280_ADDRESS, Reg, Value);
}
//------------------------------------------------
uint8_t BME280_ReadReg(uint8_t Reg)
{
  uint8_t res = I2Cx_ReadData(BME280_ADDRESS,Reg);
  return res;
}
//------------------------------------------------
void BME280_ReadReg_U16(uint8_t Reg, uint16_t *Value)
{
  I2Cx_ReadData16(BME280_ADDRESS,Reg,Value);
}
//------------------------------------------------
void BME280_ReadReg_S16(uint8_t Reg, int16_t *Value)
{
  I2Cx_ReadData16(BME280_ADDRESS,Reg, (uint16_t*) Value);
}
//------------------------------------------------
uint16_t BME280_ReadReg_BE_S16(uint8_t Reg)
{
	uint8_t buffer_8[2];
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(&hi2c1, BME280_ADDRESS, Reg, I2C_MEMADD_SIZE_8BIT, buffer_8, 2, 0x10000);
	if(status != HAL_OK) {
		Error();
	}
	return (uint16_t)(buffer_8[0]) << 8 | (uint16_t)(buffer_8[1]);
}
//------------------------------------------------
void BME280_ReadReg_U24(uint8_t Reg, uint32_t *Value)
{
  I2Cx_ReadData24(BME280_ADDRESS,Reg,Value);
  *(uint32_t *) Value &= 0x00FFFFFF;
}
//------------------------------------------------
uint32_t BME280_ReadReg_BE_U24(uint8_t Reg) {
	uint8_t buffer_8[3];
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(&hi2c1, BME280_ADDRESS, Reg, I2C_MEMADD_SIZE_8BIT, buffer_8, 3, 0x10000);
	if(status != HAL_OK) {
		Error();
	}
	return (uint32_t) (buffer_8[0]) << 16 | (uint32_t) (buffer_8[1]) << 8 | (uint32_t) (buffer_8[2]);
}
//------------------------------------------------
float BME280_ReadTemperature(void)
{
	int32_t var1, var2;
	int32_t adc_T = BME280_ReadReg_BE_U24(BME280_REGISTER_TEMPDATA);

	if ( adc_T == 0x800000) {
		return NAN;
	}
	adc_T >>= 4;
	var1 = (int32_t)((adc_T / 8) - ((int32_t)CalibData.dig_T1 * 2));
  	var1 = (var1 * ((int32_t)CalibData.dig_T2)) / 2048;
  	var2 = (int32_t)((adc_T / 16) - ((int32_t)CalibData.dig_T1));
  	var2 = (((var2 * var2) / 4096) * ((int32_t)CalibData.dig_T3)) / 16384;
    t_fine = var1 + var2;
	int32_t T = (t_fine * 5 + 128) / 256;

	return (float) T / 100;
}
//------------------------------------------------
float BME280_ReadPressure(void)
{
	int64_t var1, var2, var3, var4;
	BME280_ReadTemperature(); // must be done first to get t_fine
	int32_t adc_P = BME280_ReadReg_BE_U24(BME280_REGISTER_PRESSUREDATA);
	adc_P >>= 4;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)CalibData.dig_P6;
	var2 = var2 + ((var1 * (int64_t)CalibData.dig_P5) * 131072);
	var2 = var2 + (((int64_t)CalibData.dig_P4) * 34359738368);
	var1 = ((var1 * var1 * (int64_t)CalibData.dig_P3) / 256) + ((var1 * ((int64_t)CalibData.dig_P2) * 4096));
	var3 = ((int64_t)1) * 140737488355328;
	var1 = (var3 + var1) * ((int64_t)CalibData.dig_P1) / 8589934592;
	if (var1 == 0) {
		return 0; // avoid exception caused by division by zero
	}
	var4 = 1048576 - adc_P;
	var4 = (((var4 * 2147483648) - var2) * 3125) / var1;
	var1 = (((int64_t)CalibData.dig_P9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
	var2 = (((int64_t)CalibData.dig_P8) * var4) / 524288;
	var4 = ((var4 + var1 + var2) / 256) + (((int64_t)CalibData.dig_P7) * 16);
	float P = var4 / 256.0;
  return P;
}
//------------------------------------------------
float BME280_ReadHumidity(void)
{
  int32_t var1, var2, var3, var4, var5;
	BME280_ReadTemperature(); 	// must be done first to get t_fine
	int32_t adc_H = BME280_ReadReg_BE_S16(BME280_REGISTER_HUMIDDATA);
	if (adc_H == 0x8000) {  	// value in case humidity measurement was disabled
		return NAN;
	}
	var1 = t_fine - ((int32_t)76800);
	var2 = (int32_t)(adc_H * 16384);
	var3 = (int32_t)(((int32_t)CalibData.dig_H4) * 1048576);
	var4 = ((int32_t)CalibData.dig_H5) * var1;
	var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
	var2 = (var1 * ((int32_t)CalibData.dig_H6)) / 1024;
	var3 = (var1 * ((int32_t)CalibData.dig_H3)) / 2048;
	var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
	var2 = ((var4 * ((int32_t)CalibData.dig_H2)) + 8192) / 16384;
	var3 = var5 * var2;
	var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
	var5 = var3 - ((var4 * ((int32_t)CalibData.dig_H1)) / 16);
	var5 = (var5 < 0 ? 0 : var5);
	var5 = (var5 > 419430400 ? 419430400 : var5);
	uint32_t H = (uint32_t)(var5 / 4096);
  return (float)H / 1024.0;
}

//------------------------------------------------
void BME280_Init(void)
{
  uint8_t value=0;
	value = BME280_ReadReg(BME280_REG_ID);
	if(value != BME280_ID)
	{
		Error();
		return;
	}
	BME280_WriteReg(BME280_REG_SOFTRESET,BME280_SOFTRESET_VALUE);
	while ((BME280_ReadReg(BME280_REGISTER_STATUS) & 0x09) & BME280_STATUS_IM_UPDATE) {} ;
	BME280_ReadReg_U16(BME280_REGISTER_DIG_T1,&CalibData.dig_T1);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_T2,&CalibData.dig_T2);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_T3,&CalibData.dig_T3);
	BME280_ReadReg_U16(BME280_REGISTER_DIG_P1,&CalibData.dig_P1);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_P2,&CalibData.dig_P2);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_P3,&CalibData.dig_P3);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_P4,&CalibData.dig_P4);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_P5,&CalibData.dig_P5);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_P6,&CalibData.dig_P6);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_P7,&CalibData.dig_P7);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_P8,&CalibData.dig_P8);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_P9,&CalibData.dig_P9);
	CalibData.dig_H1 = BME280_ReadReg(BME280_REGISTER_DIG_H1);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_H2,&CalibData.dig_H2);
	CalibData.dig_H3 = BME280_ReadReg(BME280_REGISTER_DIG_H3);
	CalibData.dig_H4 = (BME280_ReadReg(BME280_REGISTER_DIG_H4) << 4) | (BME280_ReadReg(BME280_REGISTER_DIG_H4 + 1) & 0xF);
	CalibData.dig_H5 = (BME280_ReadReg(BME280_REGISTER_DIG_H5 + 1) << 4) | (BME280_ReadReg(BME280_REGISTER_DIG_H5) >> 4);
	CalibData.dig_H6 = (int8_t)BME280_ReadReg(BME280_REGISTER_DIG_H6);

	uint8_t reg = BME280_ReadReg(BME280_REG_CONFIG) & ~BME280_STBY_MSK;
	reg |= BME280_STBY_1000 & BME280_STBY_MSK;
	BME280_WriteReg(BME280_REG_CONFIG, reg);

	reg = BME280_ReadReg(BME280_REG_CONFIG) & ~BME280_FILTER_MSK;
	reg |= BME280_FILTER_4 & BME280_FILTER_MSK;
	BME280_WriteReg(BME280_REG_CONFIG, reg);

	reg = BME280_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_OSRS_T_MSK;
	reg |= BME280_OSRS_T_x4 & BME280_OSRS_T_MSK;
	BME280_WriteReg(BME280_REG_CTRL_MEAS,reg);

	reg = BME280_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_OSRS_P_MSK;
	reg |= BME280_OSRS_P_x4 & BME280_OSRS_P_MSK;
	BME280_WriteReg(BME280_REG_CTRL_MEAS, reg);

	reg = BME280_ReadReg(BME280_REG_CTRL_HUM) & ~BME280_OSRS_H_MSK;
	reg |= BME280_OSRS_H_x4 & BME280_OSRS_H_MSK;
	BME280_WriteReg(BME280_REG_CTRL_HUM,reg);
	reg = BME280_ReadReg(BME280_REG_CTRL_MEAS);
	BME280_WriteReg(BME280_REG_CTRL_MEAS,reg);

	reg = BME280_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_MODE_MSK;
	reg |= BME280_MODE_NORMAL & BME280_MODE_MSK;
	BME280_WriteReg(BME280_REG_CTRL_MEAS,reg);
}
//------------------------------------------------
