//chprintf((BaseSequentialStream *)&SD3, "dist=%d mm\n", dist);

/* ToDO
 * Printfcommands useneh & library
 */

/*
 * main.c
 *
 *  Created on: 31.03.2020
 *      Author: Loik Vuilleumier & Tim Buergel
 */

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
#include <tof.h>
#include <leds.h>
#include <spi_comm.h>

#include <detect_color.h>
#include <ledControl.h>
#include <pos_control.h>

/*
static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}*/

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    //serial_start();

    //start the USB communication
    usb_start();

    //starts the camera
    dcmi_start();
	po8030_start();
	process_image_start();

	//Inits the TOF sensor
	TOF_start();

	//init pos_control
	motors_init();
	pos_control_start();

	//Start SPI comm for RGB control
	spi_comm_start();
	process_led_start();

	clear_leds();

    /* Infinite loop. */
    while (1) {
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
