/*
 * spi.h
 *
 *  Created on: Apr 13, 2019
 *      Author: dango
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#include "config.h"

#define ICM20602_CONFIG 0x1A
#define ICM20602_GYRO_CONFIG 0x1B
#define ICM20602_ACCEL_CONFIG 0x1C
#define ICM20602_ACCEL_CONFIG2 0x1D

#define ICM20602_ACCEL_XOUT_H 0x3B
#define ICM20602_ACCEL_XOUT_L 0x3C
#define ICM20602_ACCEL_YOUT_H 0x3D
#define ICM20602_ACCEL_YOUT_L 0x3E
#define ICM20602_ACCEL_ZOUT_H 0x3F
#define ICM20602_ACCEL_ZOUT_L 0x40

#define ICM20602_GYRO_XOUT_H 0x43
#define ICM20602_GYRO_XOUT_L 0x44
#define ICM20602_GYRO_YOUT_H 0x45
#define ICM20602_GYRO_YOUT_L 0x46
#define ICM20602_GYRO_ZOUT_H 0x47
#define ICM20602_GYRO_ZOUT_L 0x48

#define ICM20602_PWR_MGMT_1 0x6B
#define ICM20602_PWR_MGMT_2 0x6C

#define SPI_CHIP_SELECT(STATUS) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, STATUS)
constexpr float GYRO_FACTOR = 16.4f;
constexpr float ACCEL_FACTOR = 16384.0f;

extern int16_t gyro_offset;
void imuSetting();
int16_t readZGYRO();
int16_t readXAccel();
uint8_t spiExchange(uint8_t send_data);
void spi_read_reg(uint8_t reg, uint8_t* buffer, size_t size);
void spi_write_reg(uint8_t reg, uint8_t data);
void imuPing();
void imu_calibration();


struct GyroData{
  volatile float x_accel;
  volatile float y_accel;
  volatile float z_accel;
  volatile float x_gyro;
  volatile float y_gyro;
  volatile float z_gyro;
  GyroData(){}
	GyroData(float _x_accel, float _y_accel, float _z_accel,
					 float _x_gyro, float _y_gyro, float _z_gyro) :
						 x_accel(_x_accel),
						 y_accel(_y_accel),
						 z_accel(_z_accel),
						 x_gyro(_x_gyro),
						 y_gyro(_y_gyro),
						 z_gyro(_z_gyro){}
	inline GyroData operator+(const GyroData &obj) const{
		return GyroData(
				x_accel + obj.x_accel,
				y_accel + obj.y_accel,
				z_accel + obj.z_accel,
				x_gyro + obj.x_gyro,
			  y_gyro + obj.y_gyro,
				z_gyro + obj.z_gyro);
	}
	inline GyroData operator-(const GyroData &obj) const{
		return GyroData(
				x_accel - obj.x_accel,
				y_accel - obj.y_accel,
				z_accel - obj.z_accel,
				x_gyro - obj.x_gyro,
				y_gyro - obj.y_gyro,
				z_gyro - obj.z_gyro);
	}
	inline GyroData operator*(const float &obj) const{
		return GyroData(x_accel * obj, y_accel * obj, z_accel * obj,
				x_gyro * obj, y_gyro * obj, z_gyro * obj);
	}
	inline GyroData operator/(const float &obj) const{
		return GyroData(x_accel / obj, y_accel / obj, z_accel / obj,
				x_gyro / obj, y_gyro / obj, z_gyro / obj);
	}
};
class ICM20602{
private:
public:
	ICM20602(){}
	void initialSetting(){}
	void setCalibration();
	GyroData update();
};
void get_all_data(GyroData& data);
#endif /* INC_SPI_H_ */
