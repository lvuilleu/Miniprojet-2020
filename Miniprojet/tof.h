/*
 * tof.h
 *
 *  Created on: 23 Apr 2020
 *      Author: Loik Vuilleumier & Tim Buergel
 */

#ifndef TOF_H_
#define TOF_H_

#define HOME_DIST 80

//Starts the thread of the TOF sensor
void TOF_start(void);

//Gets a single measurement of the TOF sensor
uint16_t TOF_get_dist_mm(void);

//Gets a verified measurement of the distance [mm] by comparing 2 sequential measurements
uint16_t TOF_get_verified_measure(uint16_t tolerance);

//Basic calibration of the TOF distance sensor
void TOF_calibrate(void);

#endif /* TOF_H_ */
