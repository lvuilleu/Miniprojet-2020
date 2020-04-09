#ifndef DETECT_COLOR_H
#define DETECT_COLOR_H

typedef enum{
	NO_COLOR,
	RED,
	GREEN,
	BLUE,
	NB_COLOR
} Colors_detected;

void process_image_start(void);
void take_image(void);
uint8_t get_color(void);
void reset_color(void);

void process_led_start(void);

#endif
