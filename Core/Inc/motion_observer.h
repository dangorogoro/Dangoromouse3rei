/*
 * motion_observer.h
 *
 *  Created on: Oct 13, 2019
 *      Author: dango
 */

#ifndef INC_MOTION_OBSERVER_H_
#define INC_MOTION_OBSERVER_H_
#include "spi.h"
#include "main.h"
#include "encoder.h"
struct MotionInput{
	float v,w;
	MotionInput(float _v = 0, float _w = 0) : v(_v), w(_w){}
};

struct RobotState{
	float x, y, theta;
	RobotState(float _x = 0.0f, float _y = 0.0f, float _theta = PI / 2.0) : x(_x), y(_y), theta(_theta){}
  inline void operator+=(const RobotState &obj) { x+=obj.x; y+=obj.y; theta+=obj.theta;}
  inline void operator-=(const RobotState &obj) { x-=obj.x; y-=obj.y; theta-=obj.theta;}
  void update(const MotionInput& state_input, float time_step = 0.001f){
		float sin_theta, cos_theta;
		float sin_delta_theta, cos_delta_theta;
		arm_sin_cos_f32(theta / PI * 180.0, &sin_theta, &cos_theta);
		arm_sin_cos_f32((theta + state_input.w * time_step) / PI * 180.0, &sin_delta_theta, &cos_delta_theta);
  	if(abs(state_input.w) > 0.001){
  		x += -state_input.v / state_input.w * sin_theta + state_input.v / state_input.w * sin_delta_theta;
  		y +=  state_input.v / state_input.w * cos_theta - state_input.v / state_input.w * cos_delta_theta;
  	}
  	else{
  		x += state_input.v * cos_theta;
  		y += state_input.v * sin_theta;
  	}
  	theta += state_input.w * time_step;
  }

};
class MotionObserver{
private:
	Encoder encoder;
	ICM20602 imu;
	TaskHandle_t handle;
	EncoderUnit present_velocity;
	GyroData present_gyro;
	MotorControl motor_control;
public:

	//MotionObserver() :TaskBase((const char*)"Montion", 3, 1024){}
	MotionObserver() : handle(NULL) {}
	void init();
	void createTask(const char* name, const uint16_t& stack_size, const UBaseType_t& task_priority);
	void task();

	void deleteTask(){
		vTaskDelete(handle);
	}

};


#endif /* INC_MOTION_OBSERVER_H_ */
