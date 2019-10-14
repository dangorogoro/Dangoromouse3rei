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
#include "TaskBase.h"
class MotionObserver{
private:
	Encoder encoder;
	ICM20602 imu;
	TaskHandle_t handle;
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
