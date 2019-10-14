/*
 * encoder.cpp
 *
 *  Created on: Oct 13, 2019
 *      Author: dango
 */
#include "encoder.h"
EncoderUnit Encoder::update(){
	uint16_t fs = 1000;
	EncoderUnit uni;
	uni.left_encoder_velocity = (int16_t)TIM3->CNT / ENCODER_RESOLUTION * CIRCUMFERENCE * fs;
	uni.right_encoder_velocity = (int16_t)TIM4->CNT / ENCODER_RESOLUTION * CIRCUMFERENCE * fs;
	printf("TIM3 %d TIM4 %d \n", TIM3->CNT, TIM4->CNT);
	printf("left %f, right %f\n", uni.left_encoder_velocity, uni.right_encoder_velocity);
	TIM3->CNT = 0;
	TIM4->CNT = 0;
	return uni;
}



