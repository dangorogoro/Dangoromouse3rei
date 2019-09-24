/*
 * spi.h
 *
 *  Created on: Apr 13, 2019
 *      Author: dango
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_
#include "main.h"
#include "arm_math.h"
extern int16_t gyro_offset;
void imuSetting();
int16_t readZGYRO();
int16_t readXAccel();
uint8_t spiExchange(uint8_t send_data);
uint8_t readReg(uint8_t reg);
void writeReg(uint8_t reg, uint8_t data);
void imuPing();
void imu_calibration();
void gyro_task(float32_t* degree);
#endif /* INC_SPI_H_ */
