#include "spi.h"
int16_t gyro_offset = GPIO_PIN_RESET;
GyroData offset_data;

void imuSetting(){
  spi_write_reg(ICM20602_PWR_MGMT_1, 0x80);
  HAL_Delay(100);
  spi_write_reg(ICM20602_PWR_MGMT_1, 0x00);
  HAL_Delay(100);
  spi_write_reg(ICM20602_CONFIG, 0x00);
  HAL_Delay(100);
  spi_write_reg(ICM20602_GYRO_CONFIG, 0x18); //+-2000dps
  HAL_Delay(100);
  spi_write_reg(ICM20602_ACCEL_CONFIG, 0x00); //+-2g
  HAL_Delay(100);
  spi_write_reg(ICM20602_ACCEL_CONFIG2, 0x08); //lowpass 1kHz
  HAL_Delay(100);
}
int16_t readXAccel(){
	int16_t data;
  uint8_t buffer[2];
  spi_read_reg(ICM20602_ACCEL_XOUT_H, buffer, 2);
  data = (int16_t)(((uint16_t)buffer[0] << 8) | ((uint16_t)buffer[1]));
  return data;
}
int16_t readZGYRO(){
  int16_t data;
  uint8_t buffer[2];
  spi_read_reg(ICM20602_GYRO_ZOUT_H, buffer, 2);
  data = (int16_t)(((uint16_t)buffer[0] << 8) | ((uint16_t)buffer[1]));
  return data;
}
void spi_read_reg(uint8_t reg, uint8_t* buffer, size_t size){
	uint8_t send_data[size+1];
	uint8_t receive_data[size+1];
  send_data[0] = reg | 0x80;
  SPI_CHIP_SELECT(GPIO_PIN_RESET);
  for(size_t i = 0; i < size + 1; i++){
    HAL_SPI_TransmitReceive(&hspi2, &send_data[i], &receive_data[i], 1, 1000);
  }
	memcpy(buffer, receive_data+1, size);
  SPI_CHIP_SELECT(GPIO_PIN_SET);
}
void spi_write_reg(uint8_t reg, uint8_t data){
  SPI_CHIP_SELECT(GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, &reg, 1, 1000);
  HAL_SPI_Transmit(&hspi2, &data, 1, 1000);
  SPI_CHIP_SELECT(GPIO_PIN_SET);
}
void imuPing(){
	uint8_t data;
	spi_read_reg(0x75, &data, 1);
  SEGGER_RTT_printf(0, "%d\n", data);
}
void imu_calibration(){
	float32_t tmp = GPIO_PIN_RESET;
	for(int i = 0; i < 1000; i++){
		tmp += readZGYRO();
		HAL_Delay(1);
	}
	tmp /= 1000.0;
	gyro_offset = (int16_t)tmp;
}

void spi_transfer(uint8_t& address, uint8_t& data){
	address |= 0x80;
  HAL_SPI_Transmit(&hspi2, &address, 1, 1000);
  HAL_SPI_Receive(&hspi2, &data, 1, 1000);
}
void get_all_data(GyroData& data){
	uint8_t buf[14];
  uint8_t address;
  address = ICM20602_ACCEL_XOUT_H | 0x80;
  spi_read_reg(address, buf, 14);
  data.x_accel = (int16_t)(((uint16_t)buf[2] << 8)
  		| ((uint16_t)buf[3])) / ACCEL_FACTOR * 9.8;
  data.y_accel = -(int16_t)(((uint16_t)buf[0] << 8)
  		| ((uint16_t)buf[1])) / ACCEL_FACTOR * 9.8;
  data.z_accel = (int16_t)(((uint16_t)buf[4] << 8)
  		| ((uint16_t)buf[5])) / ACCEL_FACTOR * 9.8;

  data.x_gyro = (int16_t)(((uint16_t)buf[8] << 8)
  		| ((uint16_t)buf[9])) / GYRO_FACTOR / 180.f * PI;
  data.y_gyro = (int16_t)(((uint16_t)buf[10] << 8)
  		| ((uint16_t)buf[11])) / GYRO_FACTOR / 180.f * PI;
  data.z_gyro = (int16_t)(((uint16_t)buf[12] << 8)
  		| ((uint16_t)buf[13])) / GYRO_FACTOR / 180.f * PI;

}
void ICM20602::setCalibration(){
	GyroData sumData;
	for(int i = 0; i < 1000; i++){
		GyroData tmpData;
		get_all_data(tmpData);
		sumData = sumData + tmpData;
	}
	sumData = sumData / 1000.0;
 	offset_data = GyroData(sumData.x_accel, sumData.y_accel, sumData.z_accel,
			sumData.x_gyro, sumData.y_gyro, sumData.z_gyro);
 	printf("z is%f\n", sumData.z_gyro);
}
GyroData ICM20602::update(){
	GyroData get_data;
	get_all_data(get_data);
  return get_data - offset_data;
}

