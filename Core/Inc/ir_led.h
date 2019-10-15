/*
 * ir_led.c
 *
 *  Created on: Jun 30, 2019
 *      Author: dango
 */

#ifndef INC_IR_LED_C_
#define INC_IR_LED_C_
#include "config.h"

void IR_start();
void IR_stop();
void IR_sampling();
void IR_reset();
void IR_task();
void IR_select();

struct IRUnit{
	uint16_t left_ir_sensor, right_ir_sensor;
	IRUnit(uint16_t left, uint16_t right) : left_ir_sensor(left), right_ir_sensor(right){}
};
class IRSensor{
private:
	uint16_t left_ir_sensor, right_ir_sensor;
	IRUnit unit;
public:
	IRSensor() : left_ir_sensor(2048), right_ir_sensor(2048), unit(2048,2048) {}
	void createTask(const char*name, const uint16_t& stack_size,
			const UBaseType_t& task_priority);
	void task();
	inline uint16_t const getLeftSensor(){return left_ir_sensor;}
	inline uint16_t const getRightSensor(){return right_ir_sensor;}
};
#endif /* INC_IR_LED_C_ */
