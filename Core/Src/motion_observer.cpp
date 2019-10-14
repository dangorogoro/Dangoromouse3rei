/*
 * motion_observer.cpp
 *
 *  Created on: Oct 13, 2019
 *      Author: dango
 */

#include "motion_observer.h"

void MotionObserver::createTask(const char*name, const uint16_t& stack_size,
		const UBaseType_t& task_priority){
	xTaskCreate([](void* obj){
		static_cast<MotionObserver*>(obj)->task();},
		name, stack_size, NULL, task_priority, NULL);
}

void MotionObserver::init(){
	imu.setCalibration();
}
void MotionObserver::task(){
	portTickType xLastWakeTime;
	const portTickType xFrequency = 100;
	xLastWakeTime = xTaskGetTickCount();
	while(1){

		auto observed_gyro = imu.update();
		printf("gyro %f %f\n",observed_gyro.x_accel, observed_gyro.z_gyro);
		auto observed_encoder = encoder.update();
		printf("TASK LEFT %f RIGHT %f\n", observed_encoder.left_encoder_velocity,
				observed_encoder.right_encoder_velocity);

		//printf("ADC %d, %d, %d\n", g_ADCBuffer[0], g_ADCBuffer[1], g_ADCBuffer[2]);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		//vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}
