/*
 * main.h
 *
 *  Created on: 31.03.2020
 *      Author: Tim Bürgel
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//Size of camera array
#define IMAGE_BUFFER_SIZE		640

typedef enum{
	NO_COLOR,
	RED,
	GREEN,
	BLUE
} Colors_detected;

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);


#endif /* MAIN_H_ */
