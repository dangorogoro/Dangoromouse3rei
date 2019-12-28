/*
 * my_vl6180x.c
 *
 *  Created on: May 19, 2019
 *      Author: dango
 */

#include "my_vl6180x_config.h"
#include "my_vl6180x.h"
#if VL6180x_SINGLE_DEVICE_DRIVER
/*
int VL6180x_I2CWrite(VL6180xDev_t addr, uint8_t *buff, uint8_t len) {
    int status;
    status = HAL_I2C_Master_Transmit(i2c_bus, addr, buff, len, def_i2c_time_out);
    if (status) {
    		printf("write error\n");
    }
    return status;
}

int VL6180x_I2CRead(VL6180xDev_t addr, uint8_t *buff, uint8_t len) {
    int status;
    status = HAL_I2C_Master_Receive(i2c_bus, addr, buff, len, def_i2c_time_out);
    if (status) {
  		printf("read error\n");
    }

    return status;
}*/
#else
int VL6180x_I2CWrite(VL6180xDev_t dev, uint8_t *buff, uint8_t len) {
    int status;
    status = HAL_I2C_Master_Transmit(i2c_bus, dev->I2cAddr, buff, len, def_i2c_time_out);
    if (status) {
        XNUCLEO6180XA1_I2C1_Init(&hi2c1);
    }
    return status;
}

int VL6180x_I2CRead(VL6180xDev_t dev, uint8_t *buff, uint8_t len) {
    int status;
    status = HAL_I2C_Master_Receive(i2c_bus, dev->I2cAddr, buff, len, def_i2c_time_out);
    if (status) {
        XNUCLEO6180XA1_I2C1_Init(&hi2c1);
    }

    return status;
}

#endif


bool robot_start(uint8_t &button_counter){
  VL6180xDev_t myDev = 0x52;
  VL6180x_RangeData_t Range;
  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)){
  	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15));
  	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
  	HAL_Delay(100);
  	button_counter++;
  }

  bool flag = false;
    SET_VL6180X_LEFT_GPIO0(GPIO_PIN_SET);
    HAL_Delay(1);
    VL6180x_InitData(myDev);
    VL6180x_Prepare(myDev);
  	VL6180x_RangePollMeasurement(myDev, &Range);
    SET_VL6180X_LEFT_GPIO0(GPIO_PIN_RESET);
    HAL_Delay(1);
    if (Range.errorStatus == 0 && Range.range_mm >= 25 && Range.range_mm <= 35)
    	flag = true;
    else return 0;

    SET_VL6180X_RIGHT_GPIO0(GPIO_PIN_SET);
    HAL_Delay(1);
    VL6180x_InitData(myDev);
    VL6180x_Prepare(myDev);
  	VL6180x_RangePollMeasurement(myDev, &Range);
    SET_VL6180X_RIGHT_GPIO0(GPIO_PIN_RESET);
    HAL_Delay(1);
    if (Range.errorStatus == 0 && Range.range_mm >= 25 && Range.range_mm <= 35)
    	flag = true;
    else	return 0;
    return flag;
}
int VL6180x_I2CWrite(VL6180xDev_t addr, uint8_t *buff, uint8_t len) {
  int status;
  status = HAL_I2C_Master_Transmit(i2c_bus, addr, buff, len, def_i2c_time_out);
  if (status) {
  		printf("write error\n");
  }
  return status;
}

int VL6180x_I2CRead(VL6180xDev_t addr, uint8_t *buff, uint8_t len) {
  int status;
  status = HAL_I2C_Master_Receive(i2c_bus, addr, buff, len, def_i2c_time_out);
  if (status) {
		printf("read error\n");
  }

  return status;
}

void Tof_continuous_sampling(){
  VL6180xDev_t theVL6180xDev = 0x52;
  VL6180x_RangeData_t Range;

  SET_VL6180X_FRONT_GPIO0(GPIO_PIN_SET);
  HAL_Delay(1);
  uint8_t newDev = 0x56 >> 1;

  VL6180x_WrByte(theVL6180xDev, 0x212, newDev);

  theVL6180xDev = newDev << 1;
  VL6180x_InitData(theVL6180xDev);
  VL6180x_Prepare(theVL6180xDev);

  VL6180x_FilterSetState(theVL6180xDev, 0); //disable filering as not effective in continuous mode

  VL6180x_Prepare(theVL6180xDev);     // default vl6180x init

  //VL6180x_UpscaleSetScaling(theVL6180xDev, 1); // set scaling  by 2  to get ranging in range 0 to 400mm

  // if slow reaction is enough then set a high time like 100 ms (up to 2550 msec)
  // if fastest reaction is required then set 0  that will set minimal possible
  VL6180x_RangeSetInterMeasPeriod(theVL6180xDev, 255);
  // set vl6180x gpio1 pin to range interrupt output with high polarity (rising edge)
  //VL6180x_SetupGPIO1(theVL6180xDev, GPIOx_SELECT_GPIO_INTERRUPT_OUTPUT, INTR_POL_HIGH);
  // set range interrupt reporting to low threshold

  VL6180x_RangeConfigInterrupt(theVL6180xDev, CONFIG_GPIO_INTERRUPT_LEVEL_LOW);
  // we don't care of high threshold as we don't use it , group hold is managed externaly
  VL6180x_RangeSetThresholds(theVL6180xDev, 0, 100, 0);
  VL6180x_ClearAllInterrupt(theVL6180xDev);
  VL6180x_RangeStartContinuousMode(theVL6180xDev);
  while(1){
  	HAL_Delay(10);
    VL6180x_RangeGetMeasurement(theVL6180xDev, &Range);
    if( Range.errorStatus == 0){
       printf("length %d\n", Range.range_mm);
    }
    else{
    	printf("error was occured\n");
    }
  	VL6180x_ClearAllInterrupt(theVL6180xDev);
      //VL6180x_RangeClearInterrupt(theVL6180xDev); // clear flags
  }

}
void ToF_sampling_test(){
  VL6180xDev_t myDev = 0x52;
  VL6180x_RangeData_t Range;
  HAL_Delay(100);
  /*
  SET_VL6180X_FRONT_GPIO0(GPIO_PIN_SET);
  HAL_Delay(100);
  VL6180x_InitData(myDev);
  VL6180x_Prepare(myDev);
  SET_VL6180X_FRONT_GPIO0(RESET);

  SET_VL6180X_LEFT_GPIO0(GPIO_PIN_SET);
  HAL_Delay(100);
  VL6180x_InitData(myDev);
  VL6180x_Prepare(myDev);
  SET_VL6180X_LEFT_GPIO0(RESET);

  SET_VL6180X_RIGHT_GPIO0(GPIO_PIN_SET);
  HAL_Delay(100);
  VL6180x_InitData(myDev);
  VL6180x_Prepare(myDev);
  SET_VL6180X_RIGHT_GPIO0(RESET);
	*/
  while(1){
    SET_VL6180X_FRONT_GPIO0(GPIO_PIN_SET);
    HAL_Delay(1);
    VL6180x_InitData(myDev);
    VL6180x_Prepare(myDev);
  	VL6180x_RangePollMeasurement(myDev, &Range);
    SET_VL6180X_FRONT_GPIO0(GPIO_PIN_RESET);
    HAL_Delay(10);
    if (Range.errorStatus == 0 && Range.range_mm >= 70 && Range.range_mm <= 120){
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_RESET);
    	printf("FRONT Vaule %d mm\n", (int)Range.range_mm);
    }
    else	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_SET);

    SET_VL6180X_LEFT_GPIO0(GPIO_PIN_SET);
    HAL_Delay(1);
    VL6180x_InitData(myDev);
    VL6180x_Prepare(myDev);
  	VL6180x_RangePollMeasurement(myDev, &Range);
    SET_VL6180X_LEFT_GPIO0(GPIO_PIN_RESET);
    HAL_Delay(10);
    if (Range.errorStatus == 0 && Range.range_mm >= 25 && Range.range_mm <= 60){
    	printf("LEFT Vaule %d mm\n", (int)Range.range_mm);
    	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_RESET);
    }
    else	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_SET);

    SET_VL6180X_RIGHT_GPIO0(GPIO_PIN_SET);
    HAL_Delay(1);
    VL6180x_InitData(myDev);
    VL6180x_Prepare(myDev);
  	VL6180x_RangePollMeasurement(myDev, &Range);
    SET_VL6180X_RIGHT_GPIO0(GPIO_PIN_RESET);
    HAL_Delay(10);
    if (Range.errorStatus == 0 && Range.range_mm >= 25 && Range.range_mm <= 60){
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
    	printf("RIGHT Vaule %d mm\n", (int)Range.range_mm);
    }
    else	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
  }
}

void Sample_SimpleRanging(void) {
  VL6180xDev_t myDev = 0x52;
  VL6180x_RangeData_t Range;
  //SET_VL6180X_FRONT_GPIO0(RESET);
  HAL_Delay(100);
  SET_VL6180X_FRONT_GPIO0(GPIO_PIN_SET);
  HAL_Delay(100);
  VL6180x_InitData(myDev);
  VL6180x_Prepare(myDev);
  while(1){
    VL6180x_RangePollMeasurement(myDev, &Range);
    HAL_Delay(100);

    if (Range.errorStatus == 0){
    	printf("%dmm -> ", (int)(Range.range_mm));
    	for(int i=0;i <= Range.range_mm; i+=5)	printf("**");
    	printf("\n");
    }
      //SEGGER_RTT_printf(0, "Range %d mm %d\n", Range.range_mm, myDev);
      //MyDev_ShowRange(myDev, Range.range_mm,0); // your code display range in mm
    //else
    //  SEGGER_RTT_printf(0, "Error status %d\n", Range.errorStatus);
      //MyDev_ShowErr(myDev, Range.errorStatus); // your code display error code
    HAL_Delay(100);
  }
}
void Sample_FreeRunningRanging(void){
    VL6180xDev_t myDev = 0x52;
    VL6180x_RangeData_t Range;
    SET_VL6180X_RIGHT_GPIO0(GPIO_PIN_SET);
    HAL_Delay(100);
    VL6180x_InitData(myDev);
    VL6180x_Prepare(myDev);

    VL6180x_RangeClearInterrupt(myDev); // make sure no interrupt is pending
    int status;
    /* kick off the first measurement */
    VL6180x_RangeStartSingleShot(myDev);
    while(1){

        // TODO add your code anything in a loop way
        VL6180x_PollDelay(dev); // simply  run default API poll delay that handle display in demo
        // check for range measure availability
        status= VL6180x_RangeGetMeasurementIfReady(myDev, &Range);
        if( status == 0 ){
            // Application must check Range.errorStatus before accessing the other data
            //    If Range.errorStatus is DataNotReady, application knows that it has to wait a bit before getting a new data
            //    If Range.errorStatus is 0, application knows it is a valid distance
            //    If Range.errorStatus is not 0, application knows that reported distance is invalid so may take some decisions depending on the errorStatus
            if (Range.errorStatus == DataNotReady)
                continue;

            if (Range.errorStatus == 0)
              SEGGER_RTT_printf(0, "Range %d mm %d\n", Range.range_mm, myDev);
              //MyDev_ShowRange(myDev, Range.range_mm,0); // your code display range in mm
            else
              SEGGER_RTT_printf(0, "Error status %d\n", Range.errorStatus);
            /* re-arm next measurement */
            VL6180x_RangeStartSingleShot(myDev);
        }
        else{
            // it is an critical error
            printf("critical error on VL6180x_RangeCheckAndGetMeasurement");
        }
    }
}

void Sample_SimpleAls(void) {
#if VL6180x_ALS_SUPPORT
    VL6180xDev_t myDev = 0x52;
    VL6180x_AlsData_t Als;

    SET_VL6180X_FRONT_GPIO0(GPIO_PIN_SET);
    HAL_Delay(100);
    VL6180x_InitData(myDev);
    VL6180x_Prepare(myDev);
    while(1){
        VL6180x_AlsPollMeasurement(myDev, &Als);
        if (Als.errorStatus == 0 )
            SEGGER_RTT_printf(0, "Range %d mm\n", Als.lux);
            //MyDev_ShowLux(myDev, Als.lux); // your code display range in mm
        else
            SEGGER_RTT_printf(0, "Error %d mm\n", Als.errorStatus);
            //MyDev_ShowErr(myDev, Als.errorStatus); // your code display error code
    }
#endif
}
void VL6180XController::createTask(const char*name, const uint16_t& stack_size,
		const UBaseType_t& task_priority){
	xTaskCreate([](void* obj){
		static_cast<VL6180XController*>(obj)->task();},
		name, stack_size, NULL, task_priority, NULL);
}
void set_continuous_mode(uint8_t targetDev){
	VL6180xDev_t theVL6180xDev = 0x52;
  VL6180x_InitData(theVL6180xDev);
  VL6180x_Prepare(theVL6180xDev);
  VL6180x_WrByte(theVL6180xDev, DEVICE_RENAME_ADDRESS, targetDev >> 1);

  theVL6180xDev = targetDev;
  VL6180x_InitData(theVL6180xDev);
  VL6180x_FilterSetState(theVL6180xDev, 0); //disable filering as not effective in continuous mode

  VL6180x_Prepare(theVL6180xDev);     // default vl6180x init

  //VL6180x_UpscaleSetScaling(theVL6180xDev, 1); // set scaling  by 2  to get ranging in range 0 to 400mm

  // if slow reaction is enough then set a high time like 100 ms (up to 2550 msec)
  // if fastest reaction is required then set 0  that will set minimal possible
  VL6180x_RangeSetInterMeasPeriod(theVL6180xDev, 100);
  // set vl6180x gpio1 pin to range interrupt output with high polarity (rising edge)
  //VL6180x_SetupGPIO1(theVL6180xDev, GPIOx_SELECT_GPIO_INTERRUPT_OUTPUT, INTR_POL_HIGH);
  // set range interrupt reporting to low threshold

  VL6180x_RangeConfigInterrupt(theVL6180xDev, CONFIG_GPIO_INTERRUPT_LEVEL_LOW);
  // we don't care of high threshold as we don't use it , group hold is managed externaly
  VL6180x_RangeSetThresholds(theVL6180xDev, 20, 100, 0);
  VL6180x_ClearAllInterrupt(theVL6180xDev);
  VL6180x_RangeStartContinuousMode(theVL6180xDev);
}
void VL6180XController::init(){
  SET_VL6180X_LEFT_GPIO0(GPIO_PIN_RESET);
  SET_VL6180X_FRONT_GPIO0(GPIO_PIN_RESET);
  SET_VL6180X_RIGHT_GPIO0(GPIO_PIN_RESET);

  //SET_VL6180X_FRONT_GPIO0(GPIO_PIN_SET);
  //set_continuous_mode(FRONT_VL6180X_ADDRESS);
  SET_VL6180X_FRONT_GPIO0(GPIO_PIN_SET);
  set_continuous_mode(FRONT_VL6180X_ADDRESS);
  SET_VL6180X_RIGHT_GPIO0(GPIO_PIN_SET);
  set_continuous_mode(RIGHT_VL6180X_ADDRESS);
  SET_VL6180X_LEFT_GPIO0(GPIO_PIN_SET);
  set_continuous_mode(LEFT_VL6180X_ADDRESS);
  /*
  set_continuous_mode(FRONT_VL6180X_ADDRESS);
  SET_VL6180X_FRONT_GPIO0(GPIO_PIN_RESET);
  SET_VL6180X_RIGHT_GPIO0(GPIO_PIN_RESET);

  SET_VL6180X_FRONT_GPIO0(GPIO_PIN_SET);
  SET_VL6180X_RIGHT_GPIO0(GPIO_PIN_SET);
  */
  //SET_VL6180X_LEFT_GPIO0(GPIO_PIN_SET);
}
void VL6180XController::task(){
	portTickType xLastWakeTime;
	const uint32_t calledFrequency = 100;
	const portTickType xFrequency = configTICK_RATE_HZ / calledFrequency;
	xLastWakeTime = xTaskGetTickCount();
	while(1){
		VL6180XData vl6180x_manage;

    VL6180x_RangeGetMeasurement(FRONT_VL6180X_ADDRESS, &vl6180x_manage.front_range);
    /*
    if (Range.errorStatus == 0 && Range.range_mm >= 20 && Range.range_mm <= 90){
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_RESET);
    	printf("FRONT Vaule %d mm\n", (int)Range.range_mm);
    }
    else	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_SET);
    */
  	VL6180x_ClearAllInterrupt(FRONT_VL6180X_ADDRESS);

    VL6180x_RangeGetMeasurement(LEFT_VL6180X_ADDRESS, &vl6180x_manage.left_range);
    /*
    if (Range.errorStatus == 0 && Range.range_mm >= 20 && Range.range_mm <= 70){
    	printf("LEFT Vaule %d mm\n", (int)Range.range_mm);
    	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_RESET);
    }
    else	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_SET);
    */
  	VL6180x_ClearAllInterrupt(LEFT_VL6180X_ADDRESS);

    VL6180x_RangeGetMeasurement(RIGHT_VL6180X_ADDRESS, &vl6180x_manage.right_range);
    /*
    if (Range.errorStatus == 0 && Range.range_mm >= 20 && Range.range_mm <= 70){
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
    	printf("RIGHT Vaule %d mm\n", (int)Range.range_mm);
    }
    else	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    */
  	VL6180x_ClearAllInterrupt(RIGHT_VL6180X_ADDRESS);

  	xQueueSendToBack(VL6180XDataQueue, &vl6180x_manage, (TickType_t) 0);
  	xQueueReset(JudgeWallQueue);
  	xQueueSendToBack(JudgeWallQueue, &vl6180x_manage, 0);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
#if 0
void VL6180XController::task(){
	portTickType xLastWakeTime;
	const uint32_t calledFrequency = 100;
	const portTickType xFrequency = configTICK_RATE_HZ / calledFrequency;
	const uint32_t delayFrequency = 1000;
	xLastWakeTime = xTaskGetTickCount();
	VL6180XData vl6180x_manage;
	VL6180x_RangeData_t Range;
	VL6180xDev_t myDev = 0x52;
	while(1){
    SET_VL6180X_FRONT_GPIO0(GPIO_PIN_SET);
    vTaskDelay(configTICK_RATE_HZ / delayFrequency);
    VL6180x_InitData(myDev);
    VL6180x_Prepare(myDev);
  	VL6180x_RangePollMeasurement(myDev, &Range);
    SET_VL6180X_FRONT_GPIO0(GPIO_PIN_RESET);
    vTaskDelay(configTICK_RATE_HZ / delayFrequency);
    if (Range.errorStatus == 0 && Range.range_mm >= 20 && Range.range_mm <= 120){
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_RESET);
    	printf("FRONT Vaule %d mm\n", (int)Range.range_mm);
    }
    else	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_SET);
    vl6180x_manage.front_range = Range;

    SET_VL6180X_LEFT_GPIO0(GPIO_PIN_SET);
    vTaskDelay(configTICK_RATE_HZ / delayFrequency);
    VL6180x_InitData(myDev);
    VL6180x_Prepare(myDev);
  	VL6180x_RangePollMeasurement(myDev, &Range);
    SET_VL6180X_LEFT_GPIO0(GPIO_PIN_RESET);
    vTaskDelay(configTICK_RATE_HZ / delayFrequency);
    if (Range.errorStatus == 0 && Range.range_mm >= 20 && Range.range_mm <= 70){
    	printf("LEFT Vaule %d mm\n", (int)Range.range_mm);
    	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_RESET);
    }
    else	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_SET);
    vl6180x_manage.left_range = Range;

    SET_VL6180X_RIGHT_GPIO0(GPIO_PIN_SET);
    vTaskDelay(configTICK_RATE_HZ / delayFrequency);
    VL6180x_InitData(myDev);
    VL6180x_Prepare(myDev);
  	VL6180x_RangePollMeasurement(myDev, &Range);
    SET_VL6180X_RIGHT_GPIO0(GPIO_PIN_RESET);
    vTaskDelay(configTICK_RATE_HZ / delayFrequency);
    if (Range.errorStatus == 0 && Range.range_mm >= 20 && Range.range_mm <= 70){
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
    	printf("RIGHT Vaule %d mm\n", (int)Range.range_mm);
    }
    else	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    vl6180x_manage.right_range = Range;

		xQueueSendToBack(VL6180XDataQueue, &vl6180x_manage, (TickType_t) 0);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
void VL6180XController::task(){
	portTickType xLastWakeTime;
	const uint32_t calledFrequency = 100;
	const portTickType xFrequency = configTICK_RATE_HZ / calledFrequency;
	xLastWakeTime = xTaskGetTickCount();
	VL6180XData vl6180x_manage;
	VL6180x_RangeData_t Range;
	while(1){
  	VL6180x_RangePollMeasurement(FRONT_VL6180X_ADDRESS, &Range);
    if (Range.errorStatus == 0 && Range.range_mm >= 20 && Range.range_mm <= 120){
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_RESET);
    	printf("FRONT Vaule %d mm\n", (int)Range.range_mm);
    }
    else	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_SET);
    vl6180x_manage.front_range = Range;

  	VL6180x_RangePollMeasurement(LEFT_VL6180X_ADDRESS, &Range);
    if (Range.errorStatus == 0 && Range.range_mm >= 20 && Range.range_mm <= 70){
    	printf("LEFT Vaule %d mm\n", (int)Range.range_mm);
    	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_RESET);
    }
    else	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_SET);
    vl6180x_manage.left_range = Range;

  	VL6180x_RangePollMeasurement(RIGHT_VL6180X_ADDRESS, &Range);
    if (Range.errorStatus == 0 && Range.range_mm >= 20 && Range.range_mm <= 70){
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
    	printf("RIGHT Vaule %d mm\n", (int)Range.range_mm);
    }
    else	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    vl6180x_manage.right_range = Range;

		xQueueSendToBack(VL6180XDataQueue, &vl6180x_manage, (TickType_t) 0);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
#endif
