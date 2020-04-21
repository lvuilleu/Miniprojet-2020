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
	DODGE,
	SIDESTEP,
	POSITIONING,
	TOUCH,
	PUSH,
	BACK_UP,
	HOME,
	DONE
} state_t;

typedef enum { // evtl. CLKW and ACLKW rotation as states => scan_speed zu scan_direction
	STRAIGHT,
	ROTATION,
	STOP,
} motors_state_t;

#define DEBUG				1
#define PI					3.1415926536f // or M_PI from math.h
#define WHEEL_DISTANCE		55.5f    //mm
#define PERIMETER_EPUCK		(PI * WHEEL_DISTANCE)
#define NSTEP_ONE_TURN		1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER		(PI*42.1) // [mm]
#define PERIOD				10 // 100 Hz
#define SCAN_DIST			1000 // mm
#define TOUCH_DIST			170 // mm
#define FINE_DIST 			200 //mm
#define ROTATION_SPEED		200 // steps/s
#define STRAIGHT_SPEED		500 // steps/s
#define SCAN_SPEED			200 //steps/s
#define ANGLE_TOLERANCE		0.02 // -> 0.57°
#define DIST_TOLERANCE		1 // mm
#define MEASURE_TOLERANCE	10 //mm
#define RED_AREA			200 // mm
#define GREEN_AREA			300 //mm
#define BLUE_AREA			400 //mm
#define AREA_Y				-50 // mm
#define SIDESTEP_DIST		100 // mm
#define CYLINDER_RADIUS 	30 //mm
#define ROBOT_R				37 // mm
#define MINFINEANGLE		0.7 //rad
#define PHOTO_DIST			200 //mm
#define MIN_DIST			100 //mm
#define CALIBRATION			-50 //mm

typedef struct{
	float pos_x;
	float pos_y;
	float angle;
	motors_state_t motor_state;
	uint8_t progress;
} robot_t;

static robot_t robot;

typedef struct {
	uint16_t pos_x;
	uint16_t pos_y;
	colors_detected_t color;
} cylinder_t;

static cylinder_t cylinder;


float get_angle(void){
	return ((float)(right_motor_get_pos()*2*WHEEL_PERIMETER)/(float)(WHEEL_DISTANCE*NSTEP_ONE_TURN));
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

void set_motors(motors_state_t motors_state, int speed){
	//new_coord_and_angle();
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
		default:
			;
	}

	return (cylinder.pos_x + (float)((SIDESTEP_DIST)*(cylinder.pos_x - area_x))/(float)(cylinder.pos_y - AREA_Y));
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
		default:
			;
	}

	return atanf((float)(cylinder.pos_x - area_x)/(float)(cylinder.pos_y - AREA_Y));
}

static THD_WORKING_AREA(waPosControl, 350);
static THD_FUNCTION(PosControl, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    static state_t state = WAIT;
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
    robot.progress = 0;

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
			reset_color();
			set_motors(STOP,0);
			scan_speed = SCAN_SPEED;
			fineangle = 0;
			robot.progress = 0;
		}


		//Calculate new position
		new_coord_and_angle();

		if(DEBUG)
		{
			static systime_t oldtime= 0;
			if((robot.angle || robot.pos_x || robot.pos_y) && time > oldtime+1e3)
			{
				chprintf((BaseSequentialStream *)&SD3, "state = %d, angle = %f, pos_x = %f, pos_y = %f\n", state, robot.angle, robot.pos_x, robot.pos_y);
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
			//static uint16_t mindist = TOUCH_DIST;
			if((VL53L0X_get_dist_mm() < FINE_DIST) || (robot.angle - fineangle < MINFINEANGLE))
			{
				set_motors(ROTATION, SCAN_SPEED);
				//if(VL53L0X_get_dist_mm() < mindist)
					//mindist = VL53L0X_get_dist_mm();
			}
			else
			{
				set_motors(STOP, 0);
				fineangle += robot.angle;
				fineangle /= 2.; //Add the 2 angles and div by 2 to get middle angle
				chprintf((BaseSequentialStream *)&SD3, "Fineangle = %f\n", fineangle);
				//chprintf((BaseSequentialStream *)&SD3, "fineangle = %f, mindist = %d\n", fineangle, mindist);
				//cylinder.pos_x = robot.pos_x + sin(fineangle)*(float)(mindist+CYLINDER_RADIUS+ROBOT_R);
				//cylinder.pos_y = robot.pos_y + cos(fineangle)*(float)(mindist+CYLINDER_RADIUS+ROBOT_R);
				//chprintf((BaseSequentialStream *)&SD3, "angle = %f, pos_x = %f, pos_y = %f cyl_x = %d, cyl_y = %d\n", robot.angle, robot.pos_x, robot.pos_y, cylinder.pos_x, cylinder.pos_y);
				state = DETECT_COLOR;
			}
			break;

		case DETECT_COLOR:
			if(robot.angle > fineangle + ANGLE_TOLERANCE)
				set_motors(ROTATION, -SCAN_SPEED);
			else if(VL53L0X_get_dist_mm() < PHOTO_DIST)
			{
				if(cylinder.pos_y == 0 && cylinder.pos_x == 0) //Calculate position of cylinder
				{
					chprintf((BaseSequentialStream *)&SD3, "Hello\n");
					uint16_t dist = VL53L0X_get_dist_mm();
					chThdSleepMilliseconds(100); //Wait for new measurement
					uint16_t dist2 = VL53L0X_get_dist_mm();
					while(dist2 > dist + MEASURE_TOLERANCE || dist2 < dist-MEASURE_TOLERANCE) //get 2 similar measurements
					{
						chThdSleepMilliseconds(100);
						dist = dist2;
						dist2 = VL53L0X_get_dist_mm();
					}
					dist = (dist + dist2)/2;
					chprintf((BaseSequentialStream *)&SD3, "fineangle = %f, mindist = %d\n", fineangle, dist);
					cylinder.pos_x = robot.pos_x + sin(robot.angle)*(float)(dist+CYLINDER_RADIUS+ROBOT_R+CALIBRATION);
					cylinder.pos_y = robot.pos_y + cos(robot.angle)*(float)(dist+CYLINDER_RADIUS+ROBOT_R+CALIBRATION);
				}

				set_motors(STRAIGHT, -STRAIGHT_SPEED);
			}
			else
			{
				take_image();
				if(cylinder.pos_x - robot.pos_x > MIN_DIST)
					state = SIDESTEP;
				else
					state = DODGE;
				y_target = cylinder.pos_y + SIDESTEP_DIST;

			}
			break;

		case DODGE:
			if(robot.angle > PI/2+ANGLE_TOLERANCE)
				set_motors(ROTATION, -ROTATION_SPEED);
			else if(robot.angle < PI/2 -ANGLE_TOLERANCE)
				set_motors(ROTATION, ROTATION_SPEED);
			else if(cylinder.pos_x - robot.pos_x > MIN_DIST)
			{
				state = SIDESTEP;
				set_motors(STOP, 0);
			}
			else
				set_motors(STRAIGHT, -STRAIGHT_SPEED);
			break;

		case SIDESTEP:
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
				state = PUSH; ////////ADAPT
			}
			else if (x_target < robot.pos_x)
				set_motors(STRAIGHT, -STRAIGHT_SPEED);
			else
				set_motors(STRAIGHT, STRAIGHT_SPEED);
			break;

		/*case TOUCH:
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
			break;*/

		case PUSH:
			if(robot.angle > target_angle + ANGLE_TOLERANCE)
				set_motors(ROTATION, -ROTATION_SPEED);
			else if(robot.angle < target_angle - ANGLE_TOLERANCE)
				set_motors(ROTATION, ROTATION_SPEED);
			else if(robot.pos_y + (CYLINDER_RADIUS+ROBOT_R)*cos(robot.angle) <= AREA_Y)
			{
				set_motors(STOP, 0);
				state = BACK_UP;
			}
			else
				set_motors(STRAIGHT, STRAIGHT_SPEED);
			break;

		case BACK_UP:
			if(robot.pos_y > AREA_Y + TOUCH_DIST)
			{
				set_motors(STOP, 0);
				state = HOME;
			}
			else
				set_motors(STRAIGHT, -STRAIGHT_SPEED);
			break;

		case HOME:
			if(robot.pos_x > 0 && robot.angle < 3.*PI/2. - ANGLE_TOLERANCE)
				set_motors(ROTATION, ROTATION_SPEED);
			else if(robot.pos_x > 0 && robot.angle > 3.*PI/2. + ANGLE_TOLERANCE)
				set_motors(ROTATION, -ROTATION_SPEED);
			else if(robot.pos_x > 0)
				set_motors(STRAIGHT, STRAIGHT_SPEED);

			else if(robot.pos_y > 0 && robot.angle < PI - ANGLE_TOLERANCE)
				set_motors(ROTATION, ROTATION_SPEED);
			else if(robot.pos_y > 0 && robot.angle > PI + ANGLE_TOLERANCE)
				set_motors(ROTATION, -ROTATION_SPEED);
			else if(robot.pos_y > 0)
				set_motors(STRAIGHT, STRAIGHT_SPEED);

			else if(robot.angle < - ANGLE_TOLERANCE)
				set_motors(ROTATION, ROTATION_SPEED);
			else if(robot.angle > ANGLE_TOLERANCE)
				set_motors(ROTATION, -ROTATION_SPEED);
			else
			{
				reset_color();
				set_motors(STOP,0);
				scan_speed = SCAN_SPEED;
				fineangle = 0;
				cylinder.pos_x = 0;
				cylinder.pos_y = 0;

				robot.progress++;
				if(robot.progress >= 3)
					state = DONE;
				else
					state = SCAN;

			}
			break;

		case DONE:
		default:
			;
			break;
		}

		chThdSleepUntilWindowed(time, time + MS2ST(PERIOD));
    }

}

static THD_WORKING_AREA(watest, 350);
static THD_FUNCTION(test, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    float angle= 0;

    while(angle < 10. * PI){
    	right_motor_set_pos(0);
    	left_motor_set_pos(0);

    	chThdSleepMilliseconds(10);

    	if(get_selector() > 8)
    	{
    		chThdSleepMilliseconds(10);
    		chprintf((BaseSequentialStream *)&SD3, "Pause \n");
    		continue;
    	}
    	set_motors(ROTATION, ROTATION_SPEED);
    	angle += get_angle();
    	chprintf((BaseSequentialStream *)&SD3, "angle = %f\n", angle);
    	if(angle > 10* PI)
    	{
    		set_motors(STOP, 0);
    	}


    }
}


void pos_control_start(void){
	chThdCreateStatic(waPosControl, sizeof(waPosControl), NORMALPRIO + 1, PosControl, NULL); // priorität apasse
}

void testing_start(void){
	chThdCreateStatic(watest, sizeof(watest), NORMALPRIO + 1, test, NULL);
}

