//chprintf((BaseSequentialStream *)&SDU1, "dist=%d mm\n", dist);


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <leds.h>
#include <spi_comm.h>

#include <detect_color.h>

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	process_image_start();
	//inits the motors
	motors_init();
	//Inits the TOF sensor
	VL53L0X_start();
	//Start SPI comm for RGB control
	spi_comm_start();

	clear_leds();
	set_rgb_led(LED2, 0,100,100);
	set_rgb_led(LED4, 100,100,0);
	set_rgb_led(LED6, 100,0,0);
	set_rgb_led(LED8, 100,0,100);

    /* Infinite loop. */
    while (1) {
    	//take_image();
    	uint16_t dist = VL53L0X_get_dist_mm();
    	//chprintf((BaseSequentialStream *)&SD3, "dist=%d mm\n", dist);
        chThdSleepMilliseconds(1000);
        //chprintf((BaseSequentialStream *)&SD3, "Colour=%d\n", get_color());
        //chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
