/*
 * tof.c
 *
 *  Created on: 23 Apr 2020
 *      Author: loikvuilleumier
 */

#include "ch.h"
#include <sensors/VL53L0X/VL53L0X.h>
#include "i2c_bus.h"
#include "tof.h"

static uint16_t dist_mm = 0;

static THD_WORKING_AREA(waTOF_Thd, 512);
static THD_FUNCTION(TOF_Thd, arg) {

	chRegSetThreadName(__FUNCTION__);
	VL53L0X_Error status = VL53L0X_ERROR_NONE;

	(void)arg;
	static VL53L0X_Dev_t device;

	device.I2cDevAddr = VL53L0X_ADDR;

	status = VL53L0X_init(&device);

	if(status == VL53L0X_ERROR_NONE){
		VL53L0X_configAccuracy(&device, VL53L0X_LONG_RANGE);
	}
	if(status == VL53L0X_ERROR_NONE){
		VL53L0X_startMeasure(&device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	}

    //while (chThdShouldTerminateX() == false) {
	while(1)
	{
    		VL53L0X_getLastMeasure(&device);
   		if(device.Data.LastRangeMeasure.RangeMilliMeter)
   			dist_mm = device.Data.LastRangeMeasure.RangeMilliMeter;
		chThdSleepMilliseconds(50);
    }
}



void TOF_start(void) {
	i2c_start();

	chThdCreateStatic(waTOF_Thd, sizeof(waTOF_Thd), NORMALPRIO + 10, TOF_Thd, NULL);
}



uint16_t TOF_get_dist_mm(void) {
	return dist_mm;
}
