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

void process_image_start(void);

void take_image(task_t newtask);
colors_detected_t get_color(void);
void reset_color(void);

float angle_correction(void);

#endif
