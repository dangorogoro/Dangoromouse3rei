/*
 * buzzer.h
 *
 *  Created on: 2020/05/25
 *      Author: dango
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include "config.h"

class Buzzer{
private:
  TaskHandle_t handle;
public:
  Buzzer() : handle(NULL) {}
  void init();
  void createTask(const char* name, const uint16_t& stack_size, const UBaseType_t& task_priority);
  void task();
  void deleteTask(){
    vTaskDelete(handle);
  }
};


#endif /* INC_BUZZER_H_ */
