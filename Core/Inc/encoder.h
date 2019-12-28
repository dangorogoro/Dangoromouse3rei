/*
 * encoder.h
 *
 *  Created on: Oct 13, 2019
 *      Author: dango
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "config.h"
constexpr float CIRCUMFERENCE = 13.0 * PI;
constexpr float ENCODER_RESOLUTION = 1024.0;
constexpr float ROBOT_WIDTH = 32.0f;
struct EncoderUnit{
	float left_encoder_velocity;
	float right_encoder_velocity;
	EncoderUnit(float _left = 0.0, float _right = 0.0) :
			left_encoder_velocity(_left), right_encoder_velocity(_right) {}
	inline void setValue(const float& _left, const float& _right){
		left_encoder_velocity = _left;
		right_encoder_velocity = _right;
	}
};
class Encoder{
private:
public:
	Encoder(){}
	EncoderUnit update();
};



#endif /* INC_ENCODER_H_ */
