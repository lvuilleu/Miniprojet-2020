/*
 * tof.h
 *
 *  Created on: 23 Apr 2020
 *      Author: loikvuilleumier
 */

#ifndef TOF_H_
#define TOF_H_

#define HOME_DIST 80

void TOF_start(void);

uint16_t TOF_get_dist_mm(void);
uint16_t TOF_get_verified_measure(uint16_t tolerance);
void TOF_calibrate(void);

#endif /* TOF_H_ */
