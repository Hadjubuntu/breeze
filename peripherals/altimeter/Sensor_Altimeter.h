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
FilterAverage *altitudeBarometer;
long Temperature = 0, Pressure = 0, Altitude = 0, AltitudeOffset = 0;

void updateBaroTemperatureLowFreq() {
	dps.calcTrueTemperature();
}


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

	updateBaroTemperatureLowFreq();

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

	altitudeBarometer = new FilterAverage(4, 0, 20000, true);

	Logger.print("Altimeter offset = ");
	Logger.println(AltitudeOffset);

	delay(200);
}

float previousAlt = 0, altCF = 0;

void updateAltimeter(float rel_acc_z) {

	callUpdateAlt();
	Altitude = dps.getAltitude();

//	if (abs(Altitude - previousAlt) < 500) {
//		altitudeBarometer->addValue(Altitude, timeUs());
//		previousAlt = Altitude;
//	}
	// Complementary filter
	if (abs(Altitude) < 2000) {
		rel_acc_z = ((int)(rel_acc_z*10))/10.0;
		float alpha = abs(1.0-rel_acc_z);

		altCF = altCF*(0.9-alpha) + (0.1+alpha)*Altitude;

		previousAlt = Altitude;
	}
	Bound(altCF, 0, 10000);


}

#endif /* SENSOR_ALTIMETER_H_ */
