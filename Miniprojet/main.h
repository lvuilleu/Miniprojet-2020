/*
 * main.h
 *
 *  Created on: 31.03.2020
 *      Author: Tim B�rgel
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;



#endif /* MAIN_H_ */
