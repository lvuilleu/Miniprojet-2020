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
#include <detect_color.h>
#include <selector.h>

#include <chprintf.h> //DEBUG ONLY

typedef enum {
	WAIT,
	SCAN,
	APPROACH,
	DETECT_COLOR,
	SIDESTEP,
	POSITIONING,
	TOUCH,
	PUSH,
	HOME,
} states;

typedef enum { // evtl. CLKW and ACLKW rotation as states => scan_speed zu scan_direction
	STRAIGHT,
	ROTATION,
	STOP,
} motors_states;

#define PI                  3.1415926536f // or M_PI from math.h
#define WHEEL_DISTANCE      5.45f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     13 // [cm]
#define PERIOD				50 // 20 Hz
#define SCAN_DIST			1000 // mm
#define TOUCH_DIST			150 // mm
#define ROTATION_SPEED		500 // steps/s
#define STRAIGHT_SPEED		500 // steps/s
#define SCAN_SPEED			100 //steps/s
#define ANGLE_TOLERANCE		0.01 // -> 0.57Â°
#define SIDESTEPS			1000


int32_t save_appr_steps(void){
	int32_t appr_steps = (left_motor_get_pos() + right_motor_get_pos()) / 2; // int/int!
	left_motor_set_pos(appr_steps);
	right_motor_set_pos(appr_steps);
	return appr_steps;
}

float get_angle(int32_t appr_steps){
	float r_angle = (right_motor_get_pos() - appr_steps)*2*WHEEL_PERIMETER/(WHEEL_DISTANCE*NSTEP_ONE_TURN); //TYPECASTING sust hemmr wohrschinli rundigsfehler
	return r_angle;
}

void set_motors(uint8_t motors_state, int speed){
	switch(motors_state){
		case STRAIGHT:
			right_motor_set_speed(speed);
			left_motor_set_speed(speed);
			break;
		case ROTATION:
			right_motor_set_speed(speed);
			left_motor_set_speed(-speed);
			break;
		case STOP: // isch chli schöner als default findi, odr hetts do e überlegig dehinter gha?
			right_motor_set_speed(0);
			left_motor_set_speed(0);

	}
}


static THD_WORKING_AREA(waPosControl, 256);
static THD_FUNCTION(PosControl, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    static float robot_angle = 0.; // in radiants
    static uint8_t state = WAIT;
    static int32_t appr_steps = 0;
    static int scan_speed = ROTATION_SPEED;
    static uint8_t old_state = 10; // FOR DEBUG ONLY

    while(1)
    {
    		time = chVTGetSystemTime();

    		if(get_selector() > 8 && state == WAIT)
    			state = SCAN;
    		if(get_selector() < 8)
    			state = WAIT;

    		//DEBUG ONLY
    		if(state != old_state)
    		{
    			chprintf((BaseSequentialStream *)&SD3, "state = %d\n", state);
    			old_state = state;
    		}
    		//Until HERE

    		switch(state) {
				case WAIT:
					set_motors(STOP, 0);
					robot_angle = 0;
					appr_steps = 0;
					break;

    			case SCAN:
    				if(VL53L0X_get_dist_mm() < SCAN_DIST)
    				{
    					robot_angle += get_angle(appr_steps);
    					appr_steps = save_appr_steps();
    					state = APPROACH;
    				}
    				else
    				{
    					float angle = get_angle(appr_steps) + robot_angle;
    					if(angle > PI/2.)
    					{
    						scan_speed = -SCAN_SPEED;
    					}
    					if(angle <= 0)
    					{
    						scan_speed = SCAN_SPEED;
    					}
    					set_motors(ROTATION, scan_speed);
    				}
				break;


			case APPROACH:
				if(VL53L0X_get_dist_mm() < TOUCH_DIST)
				{
					set_motors(STOP, 0);
					state = DETECT_COLOR;
					appr_steps = save_appr_steps();
					left_motor_set_pos(0);  //
					right_motor_set_pos(0); // evtl. reset function
				}
				else if(VL53L0X_get_dist_mm() > SCAN_DIST)
				{
					set_motors(STOP, 0);
					state = SCAN;
					appr_steps = save_appr_steps();
				}
				else
				{
					set_motors(STRAIGHT, STRAIGHT_SPEED);
				}
				break;
			case DETECT_COLOR:
				// teste wo dr zylinder im vrgliich zur kamera isch, wohrschinli jo eher links denn chönne mr niid die zentrale pixel uslese sondern bruuche en offset
				take_image();
				//Jetzt gohts en moment bis s bild fertig isch, entweder warte odr scho losfahre abr denn hemmr s risiko dass s bild verwacklet/ am falsche ort misst
				state = SIDESTEP;
				break;
			case SIDESTEP:; //unschÃ¶n aber sÃ¼sch geis ni, evtl angle am afang vom thread definiere
				float angle = get_angle(appr_steps) + robot_angle;
				if(angle > ANGLE_TOLERANCE)
					set_motors(ROTATION, -ROTATION_SPEED);
				else if(angle < -ANGLE_TOLERANCE)
					set_motors(ROTATION, ROTATION_SPEED);
				else if(left_motor_get_pos() < SIDESTEPS)
					set_motors(STRAIGHT, STRAIGHT_SPEED);
				else
				{
					set_motors(STOP, 0);
					left_motor_set_pos(0); //
					right_motor_set_pos(0);//
					state = POSITIONING;
				}

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
				;
    		}

		chThdSleepUntilWindowed(time, time + MS2ST(PERIOD));
    }

}

void pos_control_start(void){
	chThdCreateStatic(waPosControl, sizeof(waPosControl), NORMALPRIO + 1, PosControl, NULL);

	//motors_init();
}

