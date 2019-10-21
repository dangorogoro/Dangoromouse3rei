/*
 * my_vl6180x.h
 *
 *  Created on: May 19, 2019
 *      Author: dango
 */

#ifndef INC_MY_VL6180X_H_
#define INC_MY_VL6180X_H_
#include "config.h"
#include "vl6180x_api.h"
#define i2c_bus      (&hi2c3)
#define def_i2c_time_out 1000
#define SET_VL6180X_LEFT_GPIO0(status)   HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, status);
#define SET_VL6180X_FRONT_GPIO0(status)   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, status);
#define SET_VL6180X_RIGHT_GPIO0(status)   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, status);
void ToF_sampling_test();
void Sample_SimpleRanging();
void Sample_SimpleAls();
void Sample_FreeRunningRanging(void);
#endif /* INC_MY_VL6180X_H_ */
