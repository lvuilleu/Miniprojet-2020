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
#include <pos_control.h>


void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

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

	//Inits the TOF sensor
	VL53L0X_start();

	//init pos_control
	motors_init();
	pos_control_start();

	//Start SPI comm for RGB control
	spi_comm_start();
	process_led_start();

	clear_leds();

    /* Infinite loop. */
    while (1) {
    	//take_image();
    	uint16_t dist = VL53L0X_get_dist_mm();
    	chprintf((BaseSequentialStream *)&SD3, "dist=%d mm\n", dist);
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
