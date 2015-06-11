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

// Altimeter (Differential pressure sensor DPS)
BMP085 dps = BMP085();
long Temperature = 0, Pressure = 0, Altitude = 0, AltitudeOffset = 0;


void callUpdateAlt() {
	dps.calcTruePressure();
}

void highFreqCheckUpdateAlt() {
	dps.calTruPressureState1();
}


void setupAltimeter() {
	//----------------------------------------------------------
	// Altitude with Barometer
	dps.init();
	long AltitudeOffsetData = 0;
	double offset = 0.0;
	int nbMeasureOffset =  100;

	// Force temp update
	dps.calcTrueTemperature();

	int i = 0;
	while (i < nbMeasureOffset) {

		callUpdateAlt();
		highFreqCheckUpdateAlt();

		if (dps._pressure_updated) {
			AltitudeOffsetData = dps.getAltitude();
			if (AltitudeOffsetData > 0 && AltitudeOffsetData < 1000) {
				offset = (0.7f*offset) + (0.3f*AltitudeOffsetData);
			}

			i++;
			dps._pressure_updated = false;
		}
	}

	AltitudeOffset = (long)(K_ALTITUDE_OFFSET * offset);

	Logger.print("Altimeter offset = ");
	Logger.println(AltitudeOffset);

	delay(200);
}

float altCF = 0;
float altAvg = 0;
long lastUpdateAlt = 0;
long prevAltitude = 0;
float dtUpdateAlt = 1.0;


void updateAltimeter(bool sonarHealthy, float sonarAltCm, float climb_rate, float acc_z_bf) {

	dtUpdateAlt = (timeUs()-lastUpdateAlt) / S_TO_US;

	callUpdateAlt();
	Altitude = dps.getAltitude();

	if (sonarHealthy) {
		altCF = sonarAltCm;
	}
	else {

		// Complementary filter
		if (abs(Altitude) < 2000)
		{
			float altDiff = Altitude - prevAltitude;

			altAvg = 0.7 * Altitude + 0.3 * altAvg;

			// Converts climb rate to cm/s to cm using dt
			altCF = 0.9 * altAvg + 0.1 * (altCF + climb_rate * 100 * dtUpdateAlt);

			lastUpdateAlt = timeUs();
			prevAltitude = Altitude;
		}
	}

	//	Bound(altCF, 0, 10000);
}

#endif /* SENSOR_ALTIMETER_H_ */
