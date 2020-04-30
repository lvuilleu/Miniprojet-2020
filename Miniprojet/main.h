/*
 * main.h
 *
 *  Created on: 31.03.2020
 *      Author: Loik Vuilleumier & Tim Buergel
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//Size of camera array
#define IMAGE_BUFFER_SIZE		640

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#endif /* MAIN_H_ */
