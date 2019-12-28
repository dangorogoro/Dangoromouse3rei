/*
 * my_vl6180x.h
 *
 *  Created on: May 19, 2019
 *      Author: dango
 */

#ifndef INC_MY_VL6180X_H_
#define INC_MY_VL6180X_H_
#include "config.h"
#include "vl6180x_api.h"
#include "my_vl6180x_config.h"
#define SET_VL6180X_LEFT_GPIO0(status)   HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, status);
#define SET_VL6180X_FRONT_GPIO0(status)   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, status);
#define SET_VL6180X_RIGHT_GPIO0(status)   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, status);
bool robot_start(uint8_t& button_select);
void Tof_continuous_sampling();
void ToF_sampling_test();
void Sample_SimpleRanging();
void Sample_SimpleAls();
void Sample_FreeRunningRanging(void);
constexpr uint8_t LEFT_VL6180X_ADDRESS = 0x54;
constexpr uint8_t FRONT_VL6180X_ADDRESS = 0x56;
constexpr uint8_t RIGHT_VL6180X_ADDRESS = 0x58;
constexpr uint16_t DEVICE_RENAME_ADDRESS = 0x212;
struct VL6180XData{
	VL6180x_RangeData_t left_range;
	VL6180x_RangeData_t front_range;
	VL6180x_RangeData_t right_range;
};
class VL6180XController{
private:
	TaskHandle_t handle;
public:
	VL6180XController() : handle(NULL){init();}
	void init();
	void createTask(const char* name, const uint16_t& stack_size, const UBaseType_t& task_priority);
	void task();
	void deleteTask(){
		vTaskDelete(handle);
	}
};

#endif /* INC_MY_VL6180X_H_ */
