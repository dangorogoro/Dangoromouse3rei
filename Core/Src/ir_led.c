/*
 * ir_led.c
 *
 *  Created on: Jun 30, 2019
 *      Author: dango
 */
#include "ir_led.h"

void IR_start(){
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
}
void IR_stop(){
	HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
}
void IR_sampling(){
	if (robotFlag.ir_sample == true) {
		if (left_IR_tmp_value < g_ADCBuffer[0])
			left_IR_tmp_value = g_ADCBuffer[0];
		if (right_IR_tmp_value < g_ADCBuffer[1])
			right_IR_tmp_value = g_ADCBuffer[1];
		robotFlag.ir_sample = false;
	}
}
void IR_reset(){
	if(robotFlag.ir_reset == true){
		left_IR_value = left_IR_tmp_value;
		right_IR_value = right_IR_tmp_value;
		left_IR_tmp_value = 0;
		right_IR_tmp_value = 0;
		robotFlag.ir_reset = false;
	}
}
void IR_task(){
	IR_sampling();
	IR_reset();
}
void IR_select(){
	bool flag = false;
	IR_start();
	while(flag != true);
	IR_stop();
}