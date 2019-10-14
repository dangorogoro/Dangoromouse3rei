/*
 * motor.h
 *
 *  Created on: May 25, 2019
 *      Author: dango
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_
#include "main.h"
void set_left_motor_pulse(int16_t left_pulse);
void set_right_motor_pulse(int16_t right_pulse);
void set_motor_pulse(int16_t left_pulse, int16_t right_pulse);
void start_motor();
void stop_motor();
#endif /* INC_MOTOR_H_ */
