/*
 * motor.c
 *
 *  Created on: May 25, 2019
 *      Author: dango
 */
#include "motor.h"

arm_pid_instance_f32 left_motor_pid;
arm_pid_instance_f32 right_motor_pid;

void System_Identification(){

}
void set_left_motor_pulse(int16_t left_pulse){
	int16_t max_pulse = htim5.Init.Period - 1;
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if(left_pulse > 0){
		if(left_pulse >= max_pulse)	left_pulse = max_pulse;
		sConfigOC.Pulse = max_pulse;
		HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1);
		sConfigOC.Pulse = max_pulse - left_pulse;
		HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2);
	}
	else{
		if(left_pulse <= -max_pulse)	left_pulse = -max_pulse;
		sConfigOC.Pulse = max_pulse + left_pulse;
		HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1);
		sConfigOC.Pulse = max_pulse;
		HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2);
	}
}
void set_right_motor_pulse(int16_t right_pulse){
	int16_t max_pulse = htim5.Init.Period - 1;
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if(right_pulse > 0){
		if(right_pulse >= max_pulse)	right_pulse = max_pulse;
		sConfigOC.Pulse = max_pulse;
		HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3);
		sConfigOC.Pulse = max_pulse - right_pulse;
		HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4);
	}
	else{
		if(right_pulse <= -max_pulse)	right_pulse = -max_pulse;
		sConfigOC.Pulse = max_pulse + right_pulse;
		HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3);
		sConfigOC.Pulse = max_pulse;
		HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4);
	}
}
void set_motor_pulse(int16_t left_pulse, int16_t right_pulse){
	set_left_motor_pulse(left_pulse);
	set_right_motor_pulse(right_pulse);
	start_motor();
}
void start_motor(){
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
}
void stop_motor(){
  HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);
	set_motor_pulse(0,0);
}
void motor_control_init(){
	left_motor_pid.Kp = 0.5;
	left_motor_pid.Ki = 0.0;
	left_motor_pid.Kd = 0.0;
	right_motor_pid.Kp = 0.5;
	right_motor_pid.Ki = 0.0;
	right_motor_pid.Kd = 0.0;
	arm_pid_init_f32(&left_motor_pid, 1);
	arm_pid_init_f32(&right_motor_pid, 1);
	TIM3->CNT = 0;
	TIM4->CNT = 0;
}
#if 0
void motor_task(float32_t left_target_velocity, float32_t right_target_velocity){
	if(robotFlag.encoder == true){
		float32_t left_velocity = TIM4->CNT / 1024 * CIRC_LEN * 1000;
		float32_t right_velocity = TIM3->CNT / 1024 * CIRC_LEN * 1000;
		TIM3->CNT = 0;
		TIM4->CNT = 0;
		float32_t left_error = left_target_velocity - left_velocity;
		float32_t right_error = right_target_velocity - right_velocity;
		float32_t left_input = arm_pid_f32(&left_motor_pid, left_error);
		float32_t right_input = arm_pid_f32(&right_motor_pid, right_error);
		set_motor_pulse(left_input, right_input);
		robotFlag.encoder = false;
	}
}
#endif
