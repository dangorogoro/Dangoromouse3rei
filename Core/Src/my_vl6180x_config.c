/*
 * my_vl6180x_config.c
 *
 *  Created on: Oct 19, 2019
 *      Author: dango
 */
#include "my_vl6180x_config.h"
#include "vl6180x_api.h"
#include "config.h"

int VL6180x_I2CWrite(VL6180xDev_t addr, uint8_t *buff, uint8_t len) {
  int status;
  status = HAL_I2C_Master_Transmit(i2c_bus, addr, buff, len, def_i2c_time_out);
  if (status) {
  		printf("write error\n");
  }
  return status;
}

int VL6180x_I2CRead(VL6180xDev_t addr, uint8_t *buff, uint8_t len) {
  int status;
  status = HAL_I2C_Master_Receive(i2c_bus, addr, buff, len, def_i2c_time_out);
  if (status) {
		printf("read error\n");
  }

  return status;
}


