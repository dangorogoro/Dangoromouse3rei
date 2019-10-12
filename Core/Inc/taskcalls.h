/*
 * taskcalls.h
 *
 *  Created on: Oct 3, 2019
 *      Author: dango
 */

#ifndef INC_TASKCALLS_H_
#define INC_TASKCALLS_H_

#include "main.h"
void vApplicationMallocFailedHook();
/* スタックがオーバーフローしたときのHook */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
/* systic毎にコールバックされる関数 (default = 1000Hz)*/
void vApplicationTickHook();
/* 動作すべきタスクがないときにコールされる関数 */
void vApplicationIdleHook();
void vApplicationIdleHook();
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize );
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize );

#endif /* INC_TASKCALLS_H_ */
