/*
 * motion_observer.cpp
 *
 *  Created on: Oct 13, 2019
 *      Author: dango
 */
#include "motion_observer.h"
float target_velocity = 300.0;
void MotionObserver::createTask(const char*name, const uint16_t& stack_size,
		const UBaseType_t& task_priority){
	xTaskCreate([](void* obj){
		static_cast<MotionObserver*>(obj)->task();},
		name, stack_size, NULL, task_priority, NULL);
}

void MotionObserver::init(){
	imu.setCalibration();
	motor_control.pidInit();
}
void MotionObserver::task(){
	portTickType xLastWakeTime;
	const uint32_t calledFrequency = 1000;
	const portTickType xFrequency = configTICK_RATE_HZ / calledFrequency;
	xLastWakeTime = xTaskGetTickCount();
	uint16_t count = 0;
	bool flag = false;
	RobotState robot_state;
	MotionInput state_input;
	while(1){
		count = (count + 1) % 10000;
		auto observed_gyro = imu.update();
		auto observed_encoder = encoder.update();
		float v = (observed_encoder.left_encoder_velocity +
				observed_encoder.right_encoder_velocity) / 2.0 * calledFrequency;
		float w = observed_gyro.z_gyro;

		/*  uint32_t data;
		 *
		if(count >= 1000 && flag == false){
			if(count >= 1300){
				stop_motor();
				flag = true;
			}
			else	set_motor_pulse(100, 100);
			//printf("gyro %f %f\n",observed_gyro.x_accel, observed_gyro.z_gyro);
			if(count % 10 == 0){
				printf("%f, %f, %f, %f %f\n",
						o  uint32_t data;
						bserved_encoder.left_encoder_velocity * calledFrequency,
						observed_encoder.right_encoder_velocity * calledFrequency,
						observed_gyro.x_accel,
						observed_gyro.y_accel,
						observed_gyro.z_gyro);
			}
		}
		*/
		state_input = MotionInput(v, w);
		robot_state.update(state_input);

		//motor_control.pidControlUpdate(target_velocity - v, 0 - w);
		if(count % 100 == 0){
			MotionInput state_input;
	printf("motion -> %f, %f \n", state_input.v, state_input.w);
			printf("robot_state %f, %f, %f\n", robot_state.x, robot_state.y, robot_state.theta);
		}
		if(robot_state.y > 90)
	    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_SET);

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
