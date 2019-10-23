/*
 * motion_observer.cpp
 *
 *  Created on: Oct 13, 2019
 *      Author: dango
 */
#include "motion_observer.h"
float target_velocity = 300.0;
float target_w = 5;

float search_velocity_accel = 5.0;

float rad_accel = 0.4;

float alpha = 0.65;
float last_velocity = 0.0, present_velocity = 0.0;
float last_sum_accel = 0.0, sum_accel = 0.0;
float get_fusioned_velocity(const float& raw_velocity, const float& raw_accel){
	last_sum_accel = sum_accel;
	last_velocity = present_velocity;
	sum_accel += raw_accel * 1000.0 / 1000.0;
	present_velocity = last_velocity * alpha + (alpha * sum_accel - alpha * last_sum_accel)
			+ (1 - alpha) * raw_velocity;
	return present_velocity;
}

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
	bool flag = false;
	RobotState robot_state;
	MotionInput state_input;
	float recent_target_velocity = 0.0;
	float recent_target_w = 0.0;
	while(1){
		auto observed_gyro = imu.update();
		auto observed_encoder = encoder.update();

		float raw_velocity = (observed_encoder.left_encoder_velocity +
				observed_encoder.right_encoder_velocity) / 2.0 * calledFrequency;

		float v = get_fusioned_velocity(raw_velocity, observed_gyro.y_accel);
		float w = observed_gyro.z_gyro;

		state_input = MotionInput(v, w);
		robot_state.update(state_input);

		/*
		/ount = (count + 1) % 10000;
		if(count % 100 == 0){
			printf("motion -> %f, %f \n", state_input.v, state_input.w);
			printf("robot_state %f, %f, %f\n", robot_state.x, robot_state.y, robot_state.theta);
		}
		*/
		//if(target_velocity > recent_target_velocity)	recent_target_velocity += 5.0;
		if(target_w > recent_target_w) recent_target_w += rad_accel;
		if(robot_state.theta > 2 * PI){
			flag = true;
			stop_motor();
		}
		else if(flag == false){
			motor_control.pidControlUpdate(recent_target_velocity - v, recent_target_w - w);
		}

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

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
		printf("%f, %f, %f, %f, %f\n",
				o  uint32_t data;
				bserved_encoder.left_encoder_velocity * calledFrequency,
				observed_encoder.right_encoder_velocity * calledFrequency,
				observed_gyro.x_accel,
				observed_gyro.y_accel,
				observed_gyro.z_gyro);
	}
}
*/
