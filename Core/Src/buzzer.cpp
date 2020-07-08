/*
 * buzzer.cpp
 *
 *  Created on: 2020/05/25
 *      Author: dango
 */


#include "buzzer.h"

void Buzzer::createTask(const char*name, const uint16_t& stack_size,
    const UBaseType_t& task_priority){
  xTaskCreate([](void* obj){
    static_cast<Buzzer*>(obj)->task();},
    name, stack_size, NULL, task_priority, NULL);
}


void Buzzer::task(){
  portTickType xLastWakeTime;
  uint32_t calledFrequency = 1000;
  portTickType xFrequency = configTICK_RATE_HZ / calledFrequency;
  xLastWakeTime = xTaskGetTickCount();
  while(1){
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
