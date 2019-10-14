#include "spi.h"
int16_t gyro_offset = 0;
void imuSetting(){
  writeReg(0x6B,0x80);
  HAL_Delay(100);
  writeReg(0x6B,0x00);
  HAL_Delay(100);
  writeReg(0x1A,0x00);
  HAL_Delay(100);
  writeReg(0x1B,0x18);
  HAL_Delay(100);
}
int16_t readXAccel(){
	  int16_t data;
	  data = (int16_t)(((uint16_t)readReg(0x3B)<<8) | ((uint16_t)readReg(0x3C)));
	  return data;
}
int16_t readZGYRO(){
  int16_t data;
  data = (int16_t)(((uint16_t)readReg(0x47)<<8) | ((uint16_t)readReg(0x48)));
  return data;
}
uint8_t readReg(uint8_t reg){
  uint8_t data;
  uint8_t address = reg | 0x80;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
  HAL_SPI_Transmit(&hspi2, &address, 1, 1000);
  HAL_SPI_Receive(&hspi2, &data, 1, 1000);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
  return data;
}
void writeReg(uint8_t reg, uint8_t data){
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
  HAL_SPI_Transmit(&hspi2, &reg, 1, 1000);
  HAL_SPI_Transmit(&hspi2, &data, 1, 1000);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
}
void imuPing(){
  SEGGER_RTT_printf(0, "%d\n",readReg(0x75));
}
void imu_calibration(){
	float32_t tmp = 0;
	for(int i = 0; i < 1000; i++){
		tmp += readZGYRO();
		HAL_Delay(1);
	}
	tmp /= 1000.0;
	gyro_offset = (int16_t)tmp;
}
#if 0
void gyro_task(float32_t* degree){
	float32_t add_deg = 0;
	if(robotFlag.gyro == true){
		add_deg = (float32_t)(readZGYRO() - gyro_offset) / 16.4 / 1000.0;
		robotFlag.gyro = false;
	}
	*degree += add_deg;
}
#endif
