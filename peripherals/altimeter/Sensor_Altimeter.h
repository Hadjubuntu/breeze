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

	altitudeBarometer = new FilterAverage(30, 0, 20000, true);


	delay(500);

}


void updateAltimeter() {
	ultraUpdateAlt();

	Altitude = dps.getAltitude();
	altitudeBarometer->addValue(Altitude-AltitudeOffset, timeUs());
	Serial.print("alt = ");
	Serial.println(Altitude-AltitudeOffset);
}

#endif /* SENSOR_ALTIMETER_H_ */
