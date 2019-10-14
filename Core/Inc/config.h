/*
 * config.h
 *
 *  Created on: May 25, 2019
 *      Author: dango
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "stm32f4xx_hal.h"
#define ARM_MATH_CM4
#include "SEGGER_RTT.h"
#include "arm_math.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "list.h"

typedef struct{
	bool gyro;
	bool ir_reset;
	bool ir_sample;
	bool encoder;
	bool control_sequence;
	bool debug;
} Flag;
extern Flag robotFlag;
extern uint16_t left_IR_value, right_IR_value;
extern uint16_t left_IR_tmp_value, right_IR_tmp_value;
extern uint16_t g_ADCBuffer[];

extern uint8_t mode_select();
extern void debug_task();

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern I2C_HandleTypeDef hi2c3;

extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim11;
#endif /* INC_CONFIG_H_ */
