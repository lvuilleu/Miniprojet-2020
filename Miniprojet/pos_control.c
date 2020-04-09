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
#include <main.h>

#include <chprintf.h> //DEBUG ONLY

typedef enum {
	WAIT,
	SCAN,
	APPROACH,
	FINESCANRIGHT,
	FINESCANLEFT,
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
#define WHEEL_DISTANCE		54.5f    //mm
#define PERIMETER_EPUCK		(PI * WHEEL_DISTANCE)
#define NSTEP_ONE_TURN		1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER		(PI*42.5) // [mm]
#define PERIOD				10 // 100 Hz
#define SCAN_DIST			1000 // mm
#define TOUCH_DIST			150 // mm
#define FINE_DIST 			200 //mm
#define ROTATION_SPEED		500 // steps/s
#define STRAIGHT_SPEED		500 // steps/s
#define SCAN_SPEED			200 //steps/s
#define ANGLE_TOLERANCE		0.01 // -> 0.57°
#define DIST_TOLERANCE		5 // mm
#define RED_AREA				100 // mm
#define GREEN_AREA			2*RED_AREA
#define BLUE_AREA			3*RED_AREA
#define AREA_Y				0 // mm
#define SIDESTEP_DIST		250 // mm
#define CYLINDER_RADIUS 	30 //mm
#define MINFINEANGLE		0.7 //rad

struct robot_t{
	float pos_x;
	float pos_y;
	float angle;
	uint8_t motor_state;
};

static struct robot_t robot;

struct cylinder_t{
	uint16_t pos_x;
	uint16_t pos_y;
	uint8_t color;
};

static struct cylinder_t cylinder;


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
		case STOP:
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
		//Huge errors if we use int due to rounding errors
		float distance= ((float)right_motor_get_pos())/((float)NSTEP_ONE_TURN)*WHEEL_PERIMETER;
		robot.pos_y += cos(robot.angle)*distance;
		robot.pos_x += sin(robot.angle)*distance;
		break;
	case STOP:
		break;
	}
	right_motor_set_pos(0);
	left_motor_set_pos(0);
}

uint16_t calculate_positioning(void) {
	uint16_t area_x = 0;
		switch(cylinder.color) {
			case RED:
				area_x = RED_AREA;
				break;
			case GREEN:
				area_x = GREEN_AREA;
				break;
			case BLUE:
				area_x = BLUE_AREA;
				break;
		}

	return (cylinder.pos_x + (float)((robot.pos_y - cylinder.pos_y)*(cylinder.pos_x - area_x))/(float)(cylinder.pos_y - AREA_Y));
}

float calculate_target_angle(void) {
	uint16_t area_x = 0;
	switch(cylinder.color) {
		case RED:
			area_x = RED_AREA;
			break;
		case GREEN:
			area_x = GREEN_AREA;
			break;
		case BLUE:
			area_x = BLUE_AREA;
			break;
	}

	return atanf((float)(cylinder.pos_x - area_x)/(float)(cylinder.pos_y - AREA_Y));
}

static THD_WORKING_AREA(waPosControl, 256);
static THD_FUNCTION(PosControl, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    static uint8_t state = WAIT;
    static int scan_speed = SCAN_SPEED;

    static uint16_t y_target = 0;
    static uint16_t x_target = 0;
    float target_angle = 0.;

    static float fineangle = 0;

    //Robot init
    robot.pos_x = 0;
    robot.pos_y = 0;
    robot.angle = 0;
    robot.motor_state = STOP;

    //Cylinder init
    cylinder.pos_x = 0;
    cylinder.pos_y = 0;
    cylinder.color = NO_COLOR;

    while(1)
    {
		time = chVTGetSystemTime();

		if(get_selector() > 8 && state == WAIT)
			state = SCAN;
		if(get_selector() < 8)
		{
			state = WAIT;
			robot.pos_x = 0;
			robot.pos_y = 0;
			robot.angle = 0;
			robot.motor_state = STOP;
			scan_speed = SCAN_SPEED;
			reset_color();
			fineangle = 0;
		}


		//Calculate new position
		new_coord_and_angle();

		if(DEBUG)
		{
			static systime_t oldtime= 0;
			if((robot.angle || robot.pos_x || robot.pos_y) && time > oldtime+1e3)
			{
				//chprintf((BaseSequentialStream *)&SD3, "state = %d, angle = %f, pos_x = %f, pos_y = %f\n", state, robot.angle, robot.pos_x, robot.pos_y);
				oldtime = time;
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
					if(robot.angle < 0)
						scan_speed = SCAN_SPEED;
					set_motors(ROTATION, scan_speed);
				}
			break;

		case APPROACH:
			if(VL53L0X_get_dist_mm() < TOUCH_DIST)
			{
				set_motors(STOP, 0);
				calculate_target_pos(); // muess no implementiert wärde
				state = FINESCANRIGHT;
			}
			else if(VL53L0X_get_dist_mm() > SCAN_DIST)
			{
				set_motors(STOP, 0);
				state = SCAN;
			}
			else
				set_motors(STRAIGHT, STRAIGHT_SPEED);

			break;

		case FINESCANRIGHT:
			if(VL53L0X_get_dist_mm() < FINE_DIST)
			{
				set_motors(ROTATION, -SCAN_SPEED);
			}
			else
			{
				fineangle = robot.angle;
				set_motors(STOP, 0);
				state = FINESCANLEFT;
			}
			break;

		case FINESCANLEFT:
			;
			static uint16_t mindist = TOUCH_DIST;
			if((VL53L0X_get_dist_mm() < FINE_DIST) || (robot.angle - fineangle < MINFINEANGLE))
			{
				set_motors(ROTATION, SCAN_SPEED);
				if(VL53L0X_get_dist_mm() < mindist)
					mindist = VL53L0X_get_dist_mm();
			}
			else
			{
				fineangle += robot.angle;
				set_motors(STOP, 0);
				fineangle /= 2.;
				chprintf((BaseSequentialStream *)&SD3, "fineangle = %f, mindist = %d\n", fineangle, mindist);
				cylinder.pos_x = robot.pos_x + sin(fineangle)*(float)(mindist+CYLINDER_RADIUS);
				cylinder.pos_y = robot.pos_y + cos(fineangle)*(float)(mindist+CYLINDER_RADIUS);
				chprintf((BaseSequentialStream *)&SD3, "angle = %f, pos_x = %f, pos_y = %f cyl_x = %d, cyl_y = %d\n", robot.angle, robot.pos_x, robot.pos_y, cylinder.pos_x, cylinder.pos_y);
				state = DETECT_COLOR;
			}
			break;

		case DETECT_COLOR:
			if(robot.angle > fineangle)
				set_motors(ROTATION, -SCAN_SPEED);
			else
			{
				take_image();
				state = SIDESTEP;
				y_target = robot.pos_y + SIDESTEP_DIST;
			}
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
				cylinder.color = get_color();
				x_target = calculate_positioning();
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
			else if(robot.pos_x < x_target + DIST_TOLERANCE && robot.pos_x > x_target - DIST_TOLERANCE)
			{
				set_motors(STOP, 0);
				x_target = 0;		// nid unbedingt nötig
				target_angle = PI + calculate_target_angle(); // positive winkel
				state = TOUCH;
			}
			else if (x_target < robot.pos_x)
				set_motors(STRAIGHT, -STRAIGHT_SPEED);
			else
				set_motors(STRAIGHT, STRAIGHT_SPEED);
			break;

		case TOUCH:
			if(robot.angle > target_angle + ANGLE_TOLERANCE)
				set_motors(ROTATION, -ROTATION_SPEED);
			else if(robot.angle < target_angle - ANGLE_TOLERANCE)
				set_motors(ROTATION, ROTATION_SPEED);
			else if(VL53L0X_get_dist_mm() < TOUCH_DIST || robot.pos_y < 0)
			{
				set_motors(STOP, 0);
				state = PUSH;
			}
			else
				set_motors(STRAIGHT, STRAIGHT_SPEED);

			break;
		case PUSH:
			if(robot.angle > target_angle + ANGLE_TOLERANCE)
				set_motors(ROTATION, -ROTATION_SPEED);
			else if(robot.angle < target_angle - ANGLE_TOLERANCE)
				set_motors(ROTATION, ROTATION_SPEED);
			else if(robot.pos_y <= AREA_Y)
			{
				set_motors(STOP, 0);
				state = HOME;
			}
			else
				set_motors(STRAIGHT, STRAIGHT_SPEED);

			break;
		case HOME:
			if(robot.pos_x > AREA_Y + TOUCH_DIST)
				set_motors(STOP, 0);
			else
				set_motors(STRAIGHT, -STRAIGHT_SPEED);
			/*else if(robot.angle > target_angle + ANGLE_TOLERANCE)
				set_motors(ROTATION, -ROTATION_SPEED);
			else if(robot.angle < target_angle - ANGLE_TOLERANCE)
				set_motors(ROTATION, ROTATION_SPEED);*/

			break;
		default:
			;
		}

		chThdSleepUntilWindowed(time, time + MS2ST(PERIOD));
    }

}

void pos_control_start(void){
	chThdCreateStatic(waPosControl, sizeof(waPosControl), NORMALPRIO + 1, PosControl, NULL); // priorität apasse
}

