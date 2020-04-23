//LED control functions

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <leds.h>

#include <detect_color.h>

//RGB-Codes for LEDs
#define SET_RED 100,0,0
#define SET_GREEN 0,100,0
#define SET_BLUE 0,0,100

#define TOGGLE 2
#define ON 1
#define OFF 0


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

    static colors_detected_t old_color = NB_COLOR;

    while(1){
    	if(get_color() != old_color)
    	{
    		old_color = get_color();
    		switch(get_color()){
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
    		default:
    			;
    		}
    	}
    	if(get_color() == NO_COLOR)
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
