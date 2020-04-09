#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <leds.h>

#include <detect_color.h>

#define AVG_AREA 10

//RGB-Codes for LEDs
#define SET_RED 100,0,0
#define SET_GREEN 0,100,0
#define SET_BLUE 0,0,100

#define TOGGLE 2
#define ON 1
#define OFF 0

//Blue seems to have generally rather low output values on the camera,
//therefore we slightly increase the value to improve color recognition
#define BLUE_CORRECTION_VALUE 1.5

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);
static BSEMAPHORE_DECL(take_img_sem, TRUE);

static uint8_t detected_color = NO_COLOR;

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 240, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
    	chBSemWait(&take_img_sem);
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}

static THD_WORKING_AREA(waProcessImage, 512);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t red_image[AVG_AREA] = {0};
	uint8_t green_image[AVG_AREA] = {0};
	uint8_t blue_image[AVG_AREA] = {0};

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Read Image, we only treat the 10 centered values
		//Red values
		for(int i = 0; i < AVG_AREA; i++)
		{
			uint8_t temp = 0;
			temp = *(img_buff_ptr+2*i+IMAGE_BUFFER_SIZE-AVG_AREA);
			//temp = *(img_buff_ptr+2*i+IMAGE_BUFFER_SIZE/2);
			temp &= 0b11111000;
			temp = temp>>3;
			red_image[i] = temp;
		}
		//Green values
		for(int i = 0; i < AVG_AREA; i++)
		{
			uint8_t temp = 0;
			temp = *(img_buff_ptr+2*i+IMAGE_BUFFER_SIZE-AVG_AREA);
			//temp = *(img_buff_ptr+2*i+IMAGE_BUFFER_SIZE/2);
			temp &= 0b00000111;
			temp = temp<<3;

			temp |= ((*(img_buff_ptr+2*i+1+IMAGE_BUFFER_SIZE-AVG_AREA)>>5) & 0b00000111);
			//temp |= ((*(img_buff_ptr+2*i+1+IMAGE_BUFFER_SIZE/2)>>5) & 0b00000111);
			green_image[i] = temp;
		}
		//Blue Values
		for(int i = 0; i < AVG_AREA; i++)
		{
			uint8_t temp = 0;
			temp = *(img_buff_ptr+2*i+1+IMAGE_BUFFER_SIZE-AVG_AREA);
			//temp = *(img_buff_ptr+2*i+1+IMAGE_BUFFER_SIZE/2);
			temp &= 0b00011111;
			blue_image[i] = temp;
		}

		//Mean values
		uint8_t red_mean = 0;
		uint8_t green_mean = 0;
		uint8_t blue_mean = 0;
		for(int i = 0; i < AVG_AREA; i++)
		{
			red_mean += red_image[i];
			green_mean += green_image[i];
			blue_mean += blue_image[i];
		}
		red_mean /= AVG_AREA;
		green_mean /= AVG_AREA;
		blue_mean /= AVG_AREA;
		blue_mean *= BLUE_CORRECTION_VALUE;

		chprintf((BaseSequentialStream *)&SD3, "RGB %d %d %d\n", red_mean, green_mean, blue_mean);

		if(red_mean > green_mean && red_mean > blue_mean)
		{
			detected_color = RED;
			continue;
		}
		if(green_mean > red_mean && green_mean > blue_mean)
		{
			detected_color = GREEN;
			continue;
		}
		if(blue_mean > red_mean && blue_mean > green_mean)
		{
			detected_color = BLUE;
			continue;
		}
		detected_color = NO_COLOR;
    }
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

void take_image(void){
	chBSemSignal(&take_img_sem);
	return;
}

uint8_t get_color(void){
	return detected_color;
}

void reset_color(void){
	detected_color = NO_COLOR;
	return;
}

//LED control functions

void set_all_LED(uint8_t red, uint8_t green, uint8_t blue){
	set_rgb_led(LED2, red,green,blue);
	set_rgb_led(LED4, red,green,blue);
	set_rgb_led(LED6, red,green,blue);
	set_rgb_led(LED8, red,green,blue);
}


static THD_WORKING_AREA(waLEDControl, 256);
static THD_FUNCTION(LEDControl, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    static uint8_t old_color = NB_COLOR;

    while(1){
    	if(detected_color != old_color)
    	{
    		old_color = detected_color;
    		switch(detected_color){
    		case RED:
    			clear_leds();
    			set_all_LED(SET_RED);
    			break;
    		case GREEN:
    			clear_leds();
    			set_all_LED(SET_GREEN);
    			break;
    		case BLUE:
    			clear_leds();
    			set_all_LED(SET_BLUE);
    			break;
    		case NO_COLOR:
    			clear_leds();
    			set_led(LED1, ON);
    			set_led(LED5, ON);
    			break;
    		}
    	}
    	if(detected_color == NO_COLOR)
    	{
    		set_led(LED1, TOGGLE);
    		set_led(LED3, TOGGLE);
    		set_led(LED5, TOGGLE);
    		set_led(LED7, TOGGLE);
    	}

		chThdSleepMilliseconds(500);
    }

}

void process_led_start(void){
	chThdCreateStatic(waLEDControl, sizeof(waLEDControl), NORMALPRIO-5, LEDControl, NULL);
}
