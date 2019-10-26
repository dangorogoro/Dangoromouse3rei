/*
 * my_vl6180x_config.h
 *
 *  Created on: Oct 25, 2019
 *      Author: dango
 */

#ifndef INC_MY_VL6180X_CONFIG_H_
#define INC_MY_VL6180X_CONFIG_H_


#include "vl6180x_api.h"
#define i2c_bus      (&hi2c3)
#define def_i2c_time_out 10
extern int VL6180x_I2CWrite(VL6180xDev_t addr, uint8_t *buff, uint8_t len);
extern int VL6180x_I2CRead(VL6180xDev_t addr, uint8_t *buff, uint8_t len);


#endif /* INC_MY_VL6180X_CONFIG_H_ */
