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
#include <tof.h>
#include <detect_color.h>
#include <selector.h>
#include <main.h>

#include <chprintf.h> //DEBUG ONLY

typedef enum {
	WAIT,
	CALIBRATION,
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
	HOME_X,
	HOME_Y,
	DONE
} state_t;

typedef enum { // evtl. CLKW and ACLKW rotation as states => scan_speed zu scan_direction
	STRAIGHT,
	ROTATION,
	STOP,
} motors_state_t;

#define DEBUG				0
#define PI					3.1415926536f // or M_PI from math.h

#define WHEEL_DISTANCE		55.5f    //mm
#define PERIMETER_EPUCK		(PI * WHEEL_DISTANCE)
#define WHEEL_PERIMETER		(PI*42.1) // [mm]
#define NSTEP_ONE_TURN		1000 // number of step for 1 turn of the motor
#define CYLINDER_RADIUS 	30 //mm
#define ROBOT_R				37 // mm

#define PERIOD				10 // 100 Hz

#define SCAN_DIST			1000 // mm
#define TOUCH_DIST			170 // mm
#define FINE_DIST 			200 //mm
#define SIDESTEP_DIST		100 // mm
#define PHOTO_DIST			200 //mm
#define MIN_DIST			100 //mm

#define HOME_OFFSET			50 //mm
#define AREA_Y				-100 // mm

#define ROTATION_SPEED		200 // steps/s
#define STRAIGHT_SPEED		500 // steps/s
#define SCAN_SPEED			200 //steps/s

#define ANGLE_TOLERANCE				0.015 // -> 1.14°
#define DIST_TOLERANCE				1 // mm
#define SCAN_MEASURE_TOLERANCE		100 //mm
#define COLOR_MEASURE_TOLERANCE		10 //mm
#define MINFINEANGLE				0.7 //rad

#define RED_AREA			200 // mm
#define GREEN_AREA			300 //mm
#define BLUE_AREA			400 //mm
#define NO_COLOR_AREA		500 //mm

//Static global robot structure to save most important values used in most functions
typedef struct{
	float pos_x;
	float pos_y;
	float angle;
	motors_state_t motor_state;
	uint8_t progress;
	bool calibrated;
} robot_t;

static robot_t robot;

void robot_init(void){
	robot.pos_x = 0;
	robot.pos_y = 0;
	robot.angle = 0;
	robot.motor_state = STOP;
	robot.progress = 0;
	robot.calibrated = FALSE;
}

//Static global cylinder structure to save most important values used in most functions
typedef struct {
	uint16_t pos_x;
	uint16_t pos_y;
} cylinder_t;

static cylinder_t cylinder;

void cylinder_init(void){
	cylinder.pos_x = 0;
	cylinder.pos_y = 0;
}

//Calculate the rotation done by the robot based on the motor steps counted
float get_angle(void){
	return ((float)(right_motor_get_pos()*2*WHEEL_PERIMETER)/(float)(WHEEL_DISTANCE*NSTEP_ONE_TURN));
}


//Update x/y position and angle of the robot based on the steps done by the motors
void new_coord_and_angle(void){
	switch(robot.motor_state){
	case ROTATION:
		robot.angle += get_angle();
		break;
	case STRAIGHT: ;
		//Big errors if we use int due to rounding errors
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

//Set the motors to a certain speed, either straight movement or rotation on spot (or Stop).
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

//Calculate the x position the robot needs after Positioning
uint16_t calculate_positioning(void) {
	uint16_t area_x = 0;
	switch(get_color()) {
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
			area_x = NO_COLOR_AREA;
	}

	return (cylinder.pos_x + (float)((SIDESTEP_DIST)*(cylinder.pos_x - area_x))/(float)(cylinder.pos_y - AREA_Y));
}

//Calculate the angle the robot needs for pushing the cylinder
float calculate_target_angle(void) {
	uint16_t area_x = 0;
	switch(get_color()) {
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

//Orient the robot to a certain angle, returns true if the orientation is reached
bool orientation(float target_angle){
	if(robot.angle > target_angle+ANGLE_TOLERANCE)
	{
		set_motors(ROTATION, -ROTATION_SPEED);
		return FALSE;
	}
	else if(robot.angle < target_angle-ANGLE_TOLERANCE)
	{
		set_motors(ROTATION, ROTATION_SPEED);
		return FALSE;
	}
	return TRUE;
}

static THD_WORKING_AREA(waPosControl, 1024);
static THD_FUNCTION(PosControl, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    static systime_t time;

    //Some variables needed only in this function
    static state_t state = WAIT;

    static int scan_speed = SCAN_SPEED;
    static uint16_t y_target = 0;
    static uint16_t x_target = 0;
    static float target_angle = 0.;
    static float fineangle = 0;
    static bool acorrected = FALSE;

    robot_init();

    cylinder_init();

    while(1)
    {
		time = chVTGetSystemTime();

		//Start and Stop state machine
		if(get_selector() > 8 && state == WAIT)
			state = CALIBRATION;

		if(get_selector() < 8)
		{
			state = WAIT;
			robot_init();
			cylinder_init();
			reset_color();
			set_motors(STOP,0);
			scan_speed = SCAN_SPEED;
			fineangle = 0;
			acorrected = FALSE;
		}

		//Calculate new position
		new_coord_and_angle();

		//Debug notifications
		if(DEBUG)
		{
			static systime_t oldtime= 0;
			if((robot.angle || robot.pos_x || robot.pos_y) && time > oldtime+1e3)
			{
				chprintf((BaseSequentialStream *)&SD3, "state = %d, angle = %f, pos_x = %f, pos_y = %f\n", state, robot.angle, robot.pos_x, robot.pos_y);
				oldtime = time;
			}
		}

		//State machine containing all the movements the robot has to complete
		switch(state) {
		case WAIT:
			set_motors(STOP, 0);
			break;

		case CALIBRATION:
			if(orientation(-PI/2.) && !robot.calibrated)
			{
				TOF_calibrate();
				robot.calibrated = TRUE;
			}
			if(robot.calibrated && orientation(0.))
			{
				state = SCAN;
				set_motors(STOP, 0);
			}
			break;

		case SCAN:
			if(TOF_get_verified_measure(SCAN_MEASURE_TOLERANCE) < SCAN_DIST && robot.angle < PI/2.)
			{
				state = APPROACH;
			}
			else
			{
				if(robot.angle > PI/2. - ANGLE_TOLERANCE)
					scan_speed = -SCAN_SPEED;
				if(robot.angle < 0 + ANGLE_TOLERANCE)
					scan_speed = SCAN_SPEED;
				set_motors(ROTATION, scan_speed);
			}
		break;

		case APPROACH:
			if(TOF_get_dist_mm() < TOUCH_DIST)
			{
				set_motors(STOP, 0);
				state = FINESCANRIGHT;
			}
			else if(TOF_get_dist_mm() > SCAN_DIST)
			{
				set_motors(STOP, 0);
				state = SCAN;
			}
			else
				set_motors(STRAIGHT, STRAIGHT_SPEED);

			break;

		case FINESCANRIGHT:
			if(TOF_get_dist_mm() < FINE_DIST)
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
			if((TOF_get_dist_mm() < FINE_DIST) || (robot.angle - fineangle < MINFINEANGLE))
			{
				set_motors(ROTATION, SCAN_SPEED);
			}
			else
			{
				set_motors(STOP, 0);
				//Add the 2 angles and divide by 2 to get center angle
				fineangle += robot.angle;
				fineangle /= 2.;
				state = DETECT_COLOR;
			}
			break;

		case DETECT_COLOR:
			if(orientation(fineangle))
			{
				//Calculate position of cylinder
				if(cylinder.pos_y == 0 && cylinder.pos_x == 0)
				{
					uint16_t dist = TOF_get_verified_measure(COLOR_MEASURE_TOLERANCE);

					cylinder.pos_x = robot.pos_x + sin(robot.angle)*(float)(dist+CYLINDER_RADIUS);
					cylinder.pos_y = robot.pos_y + cos(robot.angle)*(float)(dist+CYLINDER_RADIUS);
				}
				//Move to optimum distance for taking the image
				if(TOF_get_dist_mm() < PHOTO_DIST)
				{
 					set_motors(STRAIGHT, -STRAIGHT_SPEED);
				}
				else
				{
					take_image(COLOR_DET);
					if(cylinder.pos_x - robot.pos_x > MIN_DIST)
						state = SIDESTEP;
					else
						state = DODGE;
					y_target = cylinder.pos_y + SIDESTEP_DIST;

				}
			}
			break;

		case DODGE:
			if(orientation(PI/2.))
			{
				if(cylinder.pos_x - robot.pos_x > MIN_DIST)
				{
					state = SIDESTEP;
					set_motors(STOP, 0);
				}
				else
					set_motors(STRAIGHT, -STRAIGHT_SPEED);
			}
			break;

		case SIDESTEP:
			if(orientation(0.))
			{
				if(robot.pos_y >= y_target)
				{
					set_motors(STOP, 0);
					y_target = 0;
					x_target = calculate_positioning();
					state = POSITIONING;
				}
				else
					set_motors(STRAIGHT, STRAIGHT_SPEED);
			}
			break;

		case POSITIONING:
			if(orientation(PI/2.))
			{
				if(robot.pos_x < x_target + DIST_TOLERANCE && robot.pos_x > x_target - DIST_TOLERANCE)
				{
					set_motors(STOP, 0);
					x_target = 0;
					target_angle = PI + calculate_target_angle();
					state = PUSH;
				}
				else if (x_target < robot.pos_x)
					set_motors(STRAIGHT, -STRAIGHT_SPEED);
				else
					set_motors(STRAIGHT, STRAIGHT_SPEED);
			}
			break;

		case PUSH:
			if(orientation(target_angle))
			{
				if(robot.pos_y + (CYLINDER_RADIUS+ROBOT_R)*cos(robot.angle) <= AREA_Y)
				{
					set_motors(STOP, 0);
					state = BACK_UP;
				}
				else
					set_motors(STRAIGHT, STRAIGHT_SPEED);
			}
			break;

		case BACK_UP:
			if(robot.pos_y > HOME_OFFSET)
			{
				set_motors(STOP, 0);
				state = HOME_X;
			}
			else
				set_motors(STRAIGHT, -STRAIGHT_SPEED);
			break;

		case HOME_X:
			if(orientation(3*PI/2.)){
					if(TOF_get_dist_mm() > HOME_DIST)
						set_motors(STRAIGHT, STRAIGHT_SPEED);

					else
					{
						set_motors(STOP, 0);
						robot.pos_x = 0;
						state = HOME_Y;
					}
				}
			break;

		case HOME_Y:
			if(orientation(PI))
			{
				set_motors(STOP,0);

				if(!acorrected)
				{
					chThdSleepMilliseconds(500);
					float anglecorr = angle_correction();
					chprintf((BaseSequentialStream *)&SD3, "anglecorr=%f\n", anglecorr);
					robot.angle = anglecorr;
					acorrected = TRUE;
					break;
				}
				if(TOF_get_dist_mm() > (HOME_DIST + DIST_TOLERANCE))
				set_motors(STRAIGHT, STRAIGHT_SPEED);

				else
				{
					set_motors(STOP,0);
					robot.pos_y = 0;

					reset_color();
					scan_speed = SCAN_SPEED;
					fineangle = 0;
					acorrected = FALSE;

					cylinder_init();

					robot.progress++;
					if(robot.progress >= 3)
						state = DONE;
					else
						state = SCAN;
				}
			}
			break;

		case DONE:
		default:
			;
		}

		chThdSleepUntilWindowed(time, time + MS2ST(PERIOD));
    }

}

void pos_control_start(void){
	chThdCreateStatic(waPosControl, sizeof(waPosControl), NORMALPRIO + 1, PosControl, NULL);
}
