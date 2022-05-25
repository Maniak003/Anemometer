#include "BME280.h"
#include "main.h"
//------------------------------------------------
extern I2C_HandleTypeDef hi2c1;
//extern UART_HandleTypeDef huart2;
extern char str1[100];
BME280_CalibData CalibData;
int32_t temper_int;
//------------------------------------------------
void Error(void)
{
  //LED_OFF;
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
void BME280_ReadReg_BE_S16(uint8_t Reg, int16_t *Value)
{
  I2Cx_ReadData16(BME280_ADDRESS,Reg,(uint16_t*)Value);
  *(uint16_t *) Value = be16toword(*(uint16_t *) Value);
}
//------------------------------------------------
void BME280_ReadReg_U24(uint8_t Reg, uint32_t *Value)
{
  I2Cx_ReadData24(BME280_ADDRESS,Reg,Value);
  *(uint32_t *) Value &= 0x00FFFFFF;
}
//------------------------------------------------
void BME280_ReadReg_BE_U24(uint8_t Reg, uint32_t *Value)
{
  I2Cx_ReadData24(BME280_ADDRESS, Reg, Value);
  *(uint32_t *) Value &= 0x00FFFFFF;
  *(uint32_t *) Value = be24toword(*(uint32_t *) Value) & 0x00FFFFFF;
}
//------------------------------------------------
float BME280_ReadTemperature(void)
{
	uint32_t temper_raw;
	int32_t var1, var2, adc_T,  T;
	BME280_ReadReg_BE_U24(BME280_REGISTER_TEMPDATA, &temper_raw);
	adc_T = (int32_t) temper_raw;

	if ( adc_T == 0x800000) {
		return NAN;
	}
	adc_T >>= 4;
	var1 = (int32_t)((adc_T / 8) - ((int32_t)CalibData.dig_T1 * 2));
	var1 = (var1 * ((int32_t)CalibData.dig_T2)) / 2048;
	var2 = (int32_t)((adc_T / 16) - ((int32_t)CalibData.dig_T1));
	var2 = (((var2 * var2) / 4096) * ((int32_t)CalibData.dig_T3)) / 16384;
	T = ((var1 + var2) * 5 + 128) / 256;

  return (float) T / 100;
}
//------------------------------------------------
float BME280_ReadPressure(void)
{
  float press_float = 0.0f;
	uint32_t press_raw, pres_int;
	int64_t val1, val2, p;
	BME280_ReadTemperature(); // must be done first to get t_fine
	BME280_ReadReg_BE_U24(BME280_REGISTER_PRESSUREDATA, &press_raw);
	press_raw >>= 4;
	val1 = ((int64_t) temper_int) - 128000;
	val2 = val1 * val1 * (int64_t)CalibData.dig_P6;
	val2 = val2 + ((val1 * (int64_t)CalibData.dig_P5) << 17);
	val2 = val2 + ((int64_t)CalibData.dig_P4 << 35);
	val1 = ((val1 * val1 * (int64_t)CalibData.dig_P3) >> 8) + ((val1 * (int64_t)CalibData.dig_P2) << 12);
	val1 = (((((int64_t)1) << 47) + val1)) * ((int64_t)CalibData.dig_P1) >> 33;
	if (val1 == 0) {
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - press_raw;
	p = (((p << 31) - val2) * 3125) / val1;
	val1 = (((int64_t)CalibData.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	val2 = (((int64_t)CalibData.dig_P8) * p) >> 19;
	p = ((p + val1 + val2) >> 8) + ((int64_t)CalibData.dig_P7 << 4);
	pres_int = ((p >> 8) * 1000) + (((p & 0xff) * 390625) / 100000);
	press_float = pres_int / 100.0f;
  return press_float;
}
//------------------------------------------------
float BME280_ReadHumidity(void)
{
  float hum_float = 0.0f;
	int16_t hum_raw;
	int32_t hum_raw_sign, v_x1_u32r;
	BME280_ReadTemperature(); // must be done first to get t_fine
	BME280_ReadReg_BE_S16(BME280_REGISTER_HUMIDDATA,&hum_raw);
	hum_raw_sign = ((int32_t)hum_raw)&0x0000FFFF;
	v_x1_u32r = (temper_int - ((int32_t)76800));
	v_x1_u32r = (((((hum_raw_sign << 14) - (((int32_t)CalibData.dig_H4) << 20) - (((int32_t)CalibData.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
		(((((((v_x1_u32r * ((int32_t)CalibData.dig_H6)) >> 10) *
		(((v_x1_u32r * ((int32_t)CalibData.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)CalibData.dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)CalibData.dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
	v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
	hum_float = (v_x1_u32r>>12);
	hum_float /= 1024.0f;
  return hum_float;
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
	CalibData.dig_H4 = (BME280_ReadReg(BME280_REGISTER_DIG_H4) << 4) | (BME280_ReadReg(BME280_REGISTER_DIG_H4+1) & 0xF);
	CalibData.dig_H5 = (BME280_ReadReg(BME280_REGISTER_DIG_H5+1) << 4) | (BME280_ReadReg(BME280_REGISTER_DIG_H5) >> 4);
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
