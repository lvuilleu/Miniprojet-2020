#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <math.h>

#include <main.h>
#include <camera/po8030.h>

#include <detect_color.h>

#define AVG_AREA 10

#define AVG_DIST				10

#define LINE_WIDTH			5.  // mm
#define CALIBRATION_DIST		30. // mm
#define IMAGE_WIDTH			50. //mm  at 30mm distance to target  -> muess me nachemässe (mitem TP4) oder haut LINE_WIDTH

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);
static BSEMAPHORE_DECL(take_img_sem, TRUE);

//Global variable to save detected color
static colors_detected_t detected_color = NO_COLOR;

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

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

static THD_WORKING_AREA(waProcessImage, 256);
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
		uint16_t red_mean = 0;
		uint16_t green_mean = 0;
		uint16_t blue_mean = 0;
		for(int i = 0; i < AVG_AREA; i++)
		{
			red_mean += red_image[i];
			green_mean += green_image[i];
			blue_mean += blue_image[i];
		}
		red_mean /= AVG_AREA;
		green_mean /= AVG_AREA;
		blue_mean /= AVG_AREA;

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

colors_detected_t get_color(void){
	return detected_color;
}

void reset_color(void){
	detected_color = NO_COLOR;
	return;
}

// isch nid unbedingt nötig, chöi mer o la si
// wenn denn strich male u defines bestimme (evtl. mit TP4)
float angle_calibration(void) {
	uint8_t *img_buff_ptr;
	chBSemWait(&image_ready_sem);

	img_buff_ptr = dcmi_get_last_image_ptr();

	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint8_t filtered_image[IMAGE_BUFFER_SIZE] = {0};
	uint32_t average = 0;

	for(uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++)
	{
		// red is used
		image[i] = (*(img_buff_ptr + 2*i) & 0b11111000) >> 3;
		average += image[i];
	}
	average /= IMAGE_BUFFER_SIZE;

	// Average filtering
	// evtl. luege ob es nötig isch
	for(int16_t i = 0; i < IMAGE_BUFFER_SIZE; i++)
	{
		uint16_t temp = 0;
		uint16_t values = 0;
		for(int16_t j = i - AVG_DIST/2; j <= i + AVG_DIST/2; j++)
		{
			if(j >= 0 && j < IMAGE_BUFFER_SIZE)
			{
				temp += image[j];
				values++;
			}
		}
		filtered_image[i] = temp/values; // conversion from 16 to 8bits, hoffentlech funktionierts
	}

	uint16_t count = 0;
	uint16_t max_count = 0;
	uint16_t middle = 0;
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++)
	{
		if(filtered_image[i] < average)
			count++;
		else
		{
			if(count > max_count)
			{
				max_count = count;
				middle = i - count/2;
				count = 0;
			}
		}
	}

	// conversion from pixels to mm
	float offset = (float)(middle - IMAGE_BUFFER_SIZE)/(float)(IMAGE_BUFFER_SIZE)*IMAGE_WIDTH; // siehe defines
	//float offset = (float)(middle - IMAGE_BUFFER_SIZE)/(float)(max_count)*LINE_WIDTH; //angeri variante, max_count
																			// isch aber evtl. unzueverlässig

	float correction = atanf(offset/CALIBRATION_DIST);

	return M_PI + correction;
}
