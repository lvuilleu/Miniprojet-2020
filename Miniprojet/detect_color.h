/*
 * detect_color.h
 *
 *  Created on: 2 Apr 2020
 *      Author: Loik Vuilleumier & Tim Buergel
 */

#ifndef DETECT_COLOR_H
#define DETECT_COLOR_H

typedef enum{
	NO_COLOR,
	RED,
	GREEN,
	BLUE,
	NB_COLOR
} colors_detected_t;

typedef enum{
	NO_TASK,
	COLOR_DET,
	CENTER_DET
} task_t;

//Starts the Capture- and Process Image threads
void process_image_start(void);

//Orders the camera thread to take an image
void take_image(task_t newtask);

//Reads the detected color
colors_detected_t get_color(void);

//Resets the detected color to NO_COLOR
void reset_color(void);

//Performs an angular correction of the robot orientation during the homing process
float angle_correction(void);

#endif
