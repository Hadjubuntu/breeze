/*
 * Sensor_Altimeter.h
 *
 *  Created on: 17 oct. 2014
 *      Author: hadjmoody
 */

#ifndef SENSOR_ALTIMETER_H_
#define SENSOR_ALTIMETER_H_

#include "arch/AVR/MCU/MCU.h"
#include "peripherals/altimeter/Sensor_AltimeterBMP085.h"


// Gain to be multiplied with the ground altitude offset
// in order to force to have 0 as altitude on the ground, due to ground variations
#define K_ALTITUDE_OFFSET 1.0

// Altimeter
BMP085 dps = BMP085();
FilterAverage *altitudeBarometer;
long Temperature = 0, Pressure = 0, Altitude = 0, AltitudeOffset = 0;


void setupAltimeter() {
	//----------------------------------------------------------
	// Altitude with Barometer
	dps.init();
	long AltitudeOffsetData = 0;
	double offset = 0.0;
	int nbMeasureOffset =  100;

	for (int i = 0; i < nbMeasureOffset; i ++) {
		dps.getAltitude(&AltitudeOffsetData);
                if (AltitudeOffsetData > 0) {
		  offset = (0.8f*offset) + (0.2f*AltitudeOffsetData);
                }
		delay(10);
	}
	AltitudeOffset = (long)(K_ALTITUDE_OFFSET * offset);

	altitudeBarometer = new FilterAverage(30, 0, 20000, true);
	//dps.dumpCalData();
	delay(500);
}

void updateAltimeter() {
	dps.getTemperature(&Temperature);
	dps.getAltitude(&Altitude);
	altitudeBarometer->addValue(Altitude-AltitudeOffset, timeUs());
}

#endif /* SENSOR_ALTIMETER_H_ */
