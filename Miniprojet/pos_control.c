/*
 * pos_control.c
 *
 *  Created on: 2 Apr 2020
 *      Author: loikvuilleumier
 */

#include "ch.h"
#include "hal.h"

#include <math.h>


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

#define DEBUG				1
#define PI					3.1415926536f // or M_PI from math.h
#define WHEEL_DISTANCE		5.45f    //cm
#define PERIMETER_EPUCK		(PI * WHEEL_DISTANCE)
#define NSTEP_ONE_TURN		1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER		13 // [cm]
#define PERIOD				50 // 20 Hz
#define SCAN_DIST			1000 // mm
#define TOUCH_DIST			150 // mm
#define ROTATION_SPEED		500 // steps/s
#define STRAIGHT_SPEED		500 // steps/s
#define SCAN_SPEED			100 //steps/s
#define ANGLE_TOLERANCE		0.01 // -> 0.57°
#define SIDESTEP_DIST		100 // mm

struct robot_t{
	float pos_x; //Ich ha chli angst vor relativ grosse rundigsfehler wenn mir die position all 50ms update ch�nnt sich das evlt recht uffeaddiere...
	float pos_y; //Drumm float
	float angle;
	uint8_t motor_state;
};

static struct robot_t robot;

/*int32_t save_appr_steps(void){
	int32_t appr_steps = (left_motor_get_pos() + right_motor_get_pos()) / 2; // int/int!
	left_motor_set_pos(appr_steps);
	right_motor_set_pos(appr_steps);
	return appr_steps;
}*/
//Comment

float get_angle(void){
	return ((float)(right_motor_get_pos()*2*WHEEL_PERIMETER)/(float)(WHEEL_DISTANCE*NSTEP_ONE_TURN));
}

void set_motors(uint8_t motors_state, int speed){
	switch(motors_state){
		case STRAIGHT:
			right_motor_set_speed(speed);
			left_motor_set_speed(speed);
			robot.motor_state = STRAIGHT;
			break;
		case ROTATION:
			right_motor_set_speed(speed);
			left_motor_set_speed(-speed);
			robot.motor_state = ROTATION;
			break;
		case STOP: // isch chli sch�ner als default findi, odr hetts do e �berlegig dehinter gha?
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			robot.motor_state = STOP;
			break;
	}
}

void new_coord_and_angle(void){
	switch(robot.motor_state){
	case ROTATION:
		robot.angle += get_angle();
		break;
	case STRAIGHT: ;
		uint8_t distance= right_motor_get_pos()/NSTEP_ONE_TURN*WHEEL_PERIMETER;
		robot.pos_y += cos(robot.angle)*distance;
		robot.pos_x += sin(robot.angle)*distance;
		break;
	case STOP:
		break;
	right_motor_set_pos(0);
	left_motor_set_pos(0);
	}
}

void calculate_target_pos(void) {
	// berechnet d position vom zylinder wemer gnue nach si
	// wird grad vorem DETECT_COLOR ufgrüefe
	// söt ähnlech zu new_coord_and_angle() si
}

static THD_WORKING_AREA(waPosControl, 256);
static THD_FUNCTION(PosControl, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    static uint8_t state = WAIT;
    static int scan_speed = ROTATION_SPEED;
    static uint8_t old_state = 10;

    static uint16_t y_target = 0;
    static uint16_t x_target = 0;

    robot.pos_x = 0;
    robot.pos_y = 0;
    robot.angle = 0;
    robot.motor_state = STOP;

    while(1)
    {
    		time = chVTGetSystemTime();

    		if(get_selector() > 8 && state == WAIT)
    			state = SCAN;
    		if(get_selector() < 8)
    			state = WAIT;

    		//Calculate new position
			new_coord_and_angle();

    		if(DEBUG)
    		{
				if(state != old_state)
				{
					chprintf((BaseSequentialStream *)&SD3, "state = %d\n", state);
					old_state = state;
				}
    		}

    		switch(state) {
			case WAIT:
				set_motors(STOP, 0);
				break;

    			case SCAN:
    				if(VL53L0X_get_dist_mm() < SCAN_DIST)
    				{
    					state = APPROACH;
    				}
    				else
    				{
    					if(robot.angle > PI/2.)
    						scan_speed = -SCAN_SPEED;
    					if(robot.angle <= 0)
    						scan_speed = SCAN_SPEED;
    					set_motors(ROTATION, scan_speed);
    				}
				break;

			case APPROACH:
				if(VL53L0X_get_dist_mm() < TOUCH_DIST)
				{
					set_motors(STOP, 0);
					calculate_target_pos(); // muess no implementiert wärde
					state = DETECT_COLOR;
				}
				else if(VL53L0X_get_dist_mm() > SCAN_DIST)
				{
					set_motors(STOP, 0);
					state = SCAN;
				}
				else
					set_motors(STRAIGHT, STRAIGHT_SPEED);

				break;
			case DETECT_COLOR:
				// teste wo dr zylinder im vrgliich zur kamera isch, wohrschinli jo eher links denn ch�nne mr niid die zentrale pixel uslese sondern bruuche en offset
				take_image();
				state = SIDESTEP;
				y_target = robot.pos_y + SIDESTEP_DIST;
				break;
			case SIDESTEP:
				// evtl. ds ganze ine funktion wird denn aber gloubs kompliziert
				if(robot.angle > ANGLE_TOLERANCE)
					set_motors(ROTATION, -ROTATION_SPEED);
				else if(robot.angle < -ANGLE_TOLERANCE)
					set_motors(ROTATION, ROTATION_SPEED);
				else if(robot.pos_y >= y_target)
				{
					set_motors(STOP, 0);
					y_target = 0;		// nid unbedingt nötig
					x_target = robot.pos_x + SIDESTEP_DIST;
					state = POSITIONING;
				}
				else
					set_motors(STRAIGHT, STRAIGHT_SPEED);
				break;

			case POSITIONING:
				if(robot.angle > PI/2. + ANGLE_TOLERANCE)
					set_motors(ROTATION, -ROTATION_SPEED);
				else if(robot.angle < PI/2. - ANGLE_TOLERANCE)
					set_motors(ROTATION, ROTATION_SPEED);
				else if(robot.pos_x >= x_target)
				{
					set_motors(STOP, 0);
					x_target = 0;		// nid unbedingt nötig
					state = TOUCH;
				}
				else
					set_motors(STRAIGHT, STRAIGHT_SPEED);
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
	chThdCreateStatic(waPosControl, sizeof(waPosControl), NORMALPRIO + 1, PosControl, NULL); // priorität apasse

	//motors_init();
}

