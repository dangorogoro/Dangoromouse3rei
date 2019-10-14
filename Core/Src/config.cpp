/*
 * config.c
 *
 *  Created on: May 25, 2019
 *      Author: dango
 */

#include "config.h"
#include "spi.h"
Flag robotFlag = {false};
/*
left_encoder = TIM3->CNT;
right_encoder = TIM4->CNT;
TIM3->CNT = 0;
TIM4->CNT = 0;
*/

float32_t po;
uint8_t mode_select(){
  int32_t reference_accel = readXAccel();
  uint8_t mode;
  int16_t left_encoder, right_encoder;
  bool flag = false;
  //while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == false){
  while(flag == false){
    left_encoder = TIM4->CNT;
    right_encoder = TIM3->CNT;
    mode = right_encoder >> 7;
    int16_t X_accel = readXAccel();
    reference_accel = reference_accel * 2 / 3 + X_accel / 3;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13| GPIO_PIN_14| GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_RESET);
    if(mode & 0x01)	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    if(mode & 0x02)	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
    if(mode & 0x04)	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
    if(mode & 0x08)	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_RESET);
    SEGGER_RTT_printf(0, "Z Gyro %d X Accel %d\n",readZGYRO(), readXAccel());
    SEGGER_RTT_printf(0, "Encoder->%d   %d\n",left_encoder, right_encoder);
    if(reference_accel - X_accel > 2000){
    	HAL_TIM_Base_Start_IT(&htim6); // buzzer
    	HAL_Delay(250);
    	HAL_TIM_Base_Stop_IT(&htim6); // buzzer
    	HAL_Delay(250);
    	HAL_TIM_Base_Start_IT(&htim6); // buzzer
    	HAL_Delay(250);
    	HAL_TIM_Base_Stop_IT(&htim6); // buzzer
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13| GPIO_PIN_14| GPIO_PIN_15, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_SET);
    	reference_accel = readXAccel();
    	flag = true;
    }
    HAL_Delay(100);
  }
  return mode % 16;
}
void debug_task(){
	if(robotFlag.debug == true){
		SEGGER_RTT_printf(0,"left %d, right %d\n", TIM3->CNT, TIM4->CNT);
		robotFlag.debug = false;
	}
}
