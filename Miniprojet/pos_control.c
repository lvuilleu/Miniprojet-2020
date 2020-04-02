/*
 * pos_control.c
 *
 *  Created on: 2 Apr 2020
 *      Author: loikvuilleumier
 */

#include "ch.h"
#include "hal.h"

#include <pos_control.h>
#include <motors.h>
#include <sensors/VL53L0X/VL53L0X.h>

typedef enum {
	SCAN,
	APPR_BARREL,
	DETECT_COLOR,
	SIDESTEP,
	POSITIONING,
	TOUCH,
	PUSH,
	HOME
} states;

#define PI                  3.1415926536f // or M_PI from math.h
#define WHEEL_DISTANCE      5.45f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     13 // [cm]
#define PERIOD				10 // 100 Hz
#define SCAN_DIST			5000 // mm
#define TOUCH_DIST			30 // mm


//evtl. move these into the thread
static float robot_angle = 0.; // in radiants
static uint8_t state = SCAN;
static int32_t appr_steps = 0;



void save_appr_steps(void){
	appr_steps = (left_motor_get_pos() + right_motor_get_pos()) / 2; // int/int!
	left_motor_set_pos(appr_steps);
	right_motor_set_pos(appr_steps);
}

void get_angle(void){
	robot_angle = (right_motor_get_pos() - appr_steps)/PERIMETER_EPUCK*NSTEP_ONE_TURN*2*PI;
}


static THD_WORKING_AREA(waPosControl, 256);
static THD_FUNCTION(PosControl, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1)
    {
    		time = chVTGetSystemTime();
    		switch(state) {
    			case SCAN:
    				if(VL53L0X_get_dist_mm() < SCAN_DIST)
    				{
    					state = APPR_BARREL;
    					save_appr_steps();
    				}
    				else
    				{
    					// scanning movement and save angle
    				}
				break;
			case APPR_BARREL:
				if(VL53L0X_get_dist_mm() < TOUCH_DIST)
				{
					state = DETECT_COLOR;
					save_appr_steps();
				}
				else if(VL53L0X_get_dist_mm() > SCAN_DIST)
				{
					state = SCAN;
					save_appr_steps();
				}
				else
				{
					// move forward
				}
				break;
			case DETECT_COLOR:

				break;
			case SIDESTEP:

				break;
			case POSITIONING:

				break;
			case TOUCH:

				break;
			case PUSH:

				break;
			case HOME:

				break;
			default:

    		}

		chThdSleepUntilWindowed(time, time + MS2ST(PERIOD));
    }

}

void pos_control_start(void){
	chThdCreateStatic(waPosControl, sizeof(waPosControl), NORMALPRIO + 1, PosControl, NULL);

	//motors_init(); schon im main
}

