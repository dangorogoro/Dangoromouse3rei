/*
 * encoder.cpp
 *
 *  Created on: Oct 13, 2019
 *      Author: dango
 */
#include "encoder.h"
EncoderUnit Encoder::update(){
	EncoderUnit uni;
	uni.left_encoder_velocity = (int16_t)TIM3->CNT / ENCODER_RESOLUTION * CIRCUMFERENCE;
	uni.right_encoder_velocity = (int16_t)TIM4->CNT / ENCODER_RESOLUTION * CIRCUMFERENCE;
	TIM3->CNT = 0;
	TIM4->CNT = 0;
	return uni;
}



