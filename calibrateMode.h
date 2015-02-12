/*
 * calibrateMode.h
 *
 *  Created on: Feb 14, 2013
 *      Author: bierma2
 */

#ifndef CALIBRATEMODE_H_
#define CALIBRATEMODE_H_

void calFlash(unsigned int irrad, unsigned int duration);
void checkLEDCurrent(void);
unsigned int checkBattery(void);
void writeCurrentLogSeparator(void);

extern unsigned int calLeft, calRight; 	// Irradiance calibration milliWatts/m^2 per DAC count


#endif /* CALIBRATEMODE_H_ */
