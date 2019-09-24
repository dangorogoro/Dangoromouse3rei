/*
 * my_vl6180x.c
 *
 *  Created on: May 19, 2019
 *      Author: dango
 */

#include "my_vl6180x.h"
#if VL6180x_SINGLE_DEVICE_DRIVER
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

void ToF_sampling_test(){
  VL6180xDev_t myDev = 0x52;
  VL6180x_RangeData_t Range;
  HAL_Delay(100);
  /*
  SET_VL6180X_FRONT_GPIO0(SET);
  HAL_Delay(100);
  VL6180x_InitData(myDev);
  VL6180x_Prepare(myDev);
  SET_VL6180X_FRONT_GPIO0(RESET);

  SET_VL6180X_LEFT_GPIO0(SET);
  HAL_Delay(100);
  VL6180x_InitData(myDev);
  VL6180x_Prepare(myDev);
  SET_VL6180X_LEFT_GPIO0(RESET);

  SET_VL6180X_RIGHT_GPIO0(SET);
  HAL_Delay(100);
  VL6180x_InitData(myDev);
  VL6180x_Prepare(myDev);
  SET_VL6180X_RIGHT_GPIO0(RESET);
	*/
  while(1){
    SET_VL6180X_FRONT_GPIO0(SET);
    HAL_Delay(1);
    VL6180x_InitData(myDev);
    VL6180x_Prepare(myDev);
  	VL6180x_RangePollMeasurement(myDev, &Range);
    SET_VL6180X_FRONT_GPIO0(RESET);
    HAL_Delay(10);
    if (Range.errorStatus == 0 && Range.range_mm >= 70 && Range.range_mm <= 120){
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_15, RESET);
    	printf("FRONT Vaule %d mm\n", (int)Range.range_mm);
    }
    else	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_15, SET);

    SET_VL6180X_LEFT_GPIO0(SET);
    HAL_Delay(1);
    VL6180x_InitData(myDev);
    VL6180x_Prepare(myDev);
  	VL6180x_RangePollMeasurement(myDev, &Range);
    SET_VL6180X_LEFT_GPIO0(RESET);
    HAL_Delay(10);
    if (Range.errorStatus == 0 && Range.range_mm >= 25 && Range.range_mm <= 60){
    	printf("LEFT Vaule %d mm\n", (int)Range.range_mm);
    	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, RESET);
    }
    else	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, SET);

    SET_VL6180X_RIGHT_GPIO0(SET);
    HAL_Delay(1);
    VL6180x_InitData(myDev);
    VL6180x_Prepare(myDev);
  	VL6180x_RangePollMeasurement(myDev, &Range);
    SET_VL6180X_RIGHT_GPIO0(RESET);
    HAL_Delay(10);
    if (Range.errorStatus == 0 && Range.range_mm >= 25 && Range.range_mm <= 60){
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, RESET);
    	printf("RIGHT Vaule %d mm\n", (int)Range.range_mm);
    }
    else	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, SET);
  }

}

void Sample_SimpleRanging(void) {
  VL6180xDev_t myDev = 0x52;
  VL6180x_RangeData_t Range;
  //SET_VL6180X_FRONT_GPIO0(RESET);
  HAL_Delay(100);
  SET_VL6180X_FRONT_GPIO0(SET);
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
void Sample_FreeRunningRanging(void) {
    VL6180xDev_t myDev = 0x52;
    VL6180x_RangeData_t Range;
    SET_VL6180X_RIGHT_GPIO0(SET);
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

    SET_VL6180X_FRONT_GPIO0(SET);
    HAL_Delay(100);
    VL6180x_InitData(myDev);
    VL6180x_Prepare(myDev);
    while(1){
        VL6180x_AlsPollMeasurement(myDev, &Als);
        if (Als.errorStatus == 0 )
            SEGGER_RTT_printf(0, "Range %d mm %d\n", Als.lux);
            //MyDev_ShowLux(myDev, Als.lux); // your code display range in mm
        else
            SEGGER_RTT_printf(0, "Error %d mm\n", Als.errorStatus);
            //MyDev_ShowErr(myDev, Als.errorStatus); // your code display error code
    }
#endif
}
