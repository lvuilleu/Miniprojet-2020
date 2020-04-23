/*
 * tof.h
 *
 *  Created on: 23 Apr 2020
 *      Author: loikvuilleumier
 */

#ifndef TOF_H_
#define TOF_H_

#define HOME_DIST 80
#define MEASURE_PERIOD 50

static BSEMAPHORE_DECL(measure_sem, TRUE);

void TOF_start(void);

uint16_t TOF_get_dist_mm(void);
void TOF_calibrate(void);
void TOF_wait_measure(void);


#endif /* TOF_H_ */
