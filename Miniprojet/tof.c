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

#define CALIB_TOLERANCE 5

static BSEMAPHORE_DECL(measure_sem, TRUE);

static uint16_t dist_mm = 0;
static uint16_t calibration_value = 0;

//We need a distance measurement more often than every 100ms
//Therefore we more or less copied the function from the e-puck2_main-processor and made some minor changes

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
   		chBSemSignal(&measure_sem);
		chThdSleepMilliseconds(MEASURE_PERIOD);
    }
}



void TOF_start(void) {
	i2c_start();

	chThdCreateStatic(waTOF_Thd, sizeof(waTOF_Thd), NORMALPRIO + 10, TOF_Thd, NULL);
}

uint16_t TOF_get_dist_mm(void) {
	return dist_mm - calibration_value;
}

//Since we had rather big differences of the measured values when restarting the robot we implemented a basic calibration
//At the beginning of the program we face the wall which is at a distance of 80mm from the robot
void TOF_calibrate(void){
	uint16_t measure1 = TOF_get_dist_mm();
	chBSemWait(&measure_sem);
	uint16_t measure2 = TOF_get_dist_mm();
	while(measure1 - measure2 > CALIB_TOLERANCE || measure2 - measure1 > CALIB_TOLERANCE)
	{
		measure1 = measure2;
		chBSemWait(&measure_sem);
		measure2 = TOF_get_dist_mm();
	}
	calibration_value = (measure1+measure2)/2 - HOME_DIST;
}

void TOF_wait_measure(void){
	//To deactivate Semaphore (we only rarely need multiple measurements at a time therefore it will probably still be activated)
	chBSemWait(&measure_sem);
	//Wait for new measurement
	chBSemWait(&measure_sem);
}
