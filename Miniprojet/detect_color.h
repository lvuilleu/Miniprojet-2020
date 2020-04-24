#ifndef DETECT_COLOR_H
#define DETECT_COLOR_H

typedef enum{
	NO_COLOR,
	RED,
	GREEN,
	BLUE,
	NB_COLOR
} colors_detected_t;

void process_image_start(void);

void take_image(void);
colors_detected_t get_color(void);
void reset_color(void);

float angle_calibration(void);

#endif
