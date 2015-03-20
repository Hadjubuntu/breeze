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
#include "math/Kalman.h"


// Gain to be multiplied with the ground altitude offset
// in order to force to have 0 as altitude on the ground, due to ground variations
#define K_ALTITUDE_OFFSET 1.0

// Altimeter
BMP085 dps = BMP085();
FilterAverage *altitudeBarometer;
long Temperature = 0, Pressure = 0, Altitude = 0, AltitudeOffset = 0;

void updateBaroTemperatureLowFreq() {
	dps.calcTrueTemperature();
}


void ultraUpdateAlt() {
	dps.calcTruePressure();
}

void setupAltimeter() {
	//----------------------------------------------------------
	// Altitude with Barometer
	dps.init();
	long AltitudeOffsetData = 0;
	double offset = 0.0;
	int nbMeasureOffset =  100;

	updateBaroTemperatureLowFreq();

	for (int i = 0; i < nbMeasureOffset; i ++) {

		updateBaroTemperatureLowFreq();
		ultraUpdateAlt();

		AltitudeOffsetData = dps.getAltitude();
		if (AltitudeOffsetData > 0) {
			offset = (0.8f*offset) + (0.2f*AltitudeOffsetData);
		}
		delay(10);
	}

	AltitudeOffset = (long)(K_ALTITUDE_OFFSET * offset);

	altitudeBarometer = new FilterAverage(2, 0, 20000, true);

	delay(500);

}

double altDt = 0.1;
long altPrevious = 0;

void updateAltimeter() {
	if (altPrevious == 0) {
		altDt = 0.1;
	}
	else {
		altDt = (micros()-altPrevious) / S_TO_US;
	}
	ultraUpdateAlt();

	Altitude = dps.getAltitude();
	altitudeBarometer->addValue(Altitude-AltitudeOffset, timeUs());

	altPrevious = micros();
}

#endif /* SENSOR_ALTIMETER_H_ */
