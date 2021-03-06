/*
 * Sensor_Airspeed.h
 *
 *  Created on: 7 oct. 2014
 *      Author: hadjmoody
 */

#ifndef SENSOR_AIRSPEED_H_
#define SENSOR_AIRSPEED_H_

#include "Arduino.h"
#include "arch/AVR/MCU/MCU.h"

#define ANALOG_PIN_AIRSPEED 6
double K_airspeed_factor = 3.91; // 5.0/1023*(2/2.5)*1000.0 analog output to volt to Pascal
double _airspeed_ratio = 1.6327; // airspeed_ratio = 1.5191; Which is 2/Rho(air) altitude 0
double _vOffset = 2.5;
double airspeed_ms = 0.0;

float ref_pressure, air_pressure, pressure_diff;

void setupAirspeed() {
	long ref_pressure_sum = 0;
	int nb_samples = 300;

	Logger.println("Airspeed calibration...");

	for (int i=1;i<=nb_samples;i++) {
		ref_pressure_sum += K_airspeed_factor*(analogRead(ANALOG_PIN_AIRSPEED));
		delay(20);
	}

	ref_pressure = ref_pressure_sum / nb_samples;
	air_pressure = ref_pressure; // At start, initialize with ref pressure
}

// Function called at 20Hz
// Analogread function takes 0.1 ms (100 us) to be executed
// Further read : en.wikipedia.org/wiki/Pitot_tube
// TODO improve EAS to TAS with altitude and pressure for further flight plan with high alt
double updateAirspeed() {

	air_pressure = K_airspeed_factor*(analogRead(ANALOG_PIN_AIRSPEED))*0.4 + air_pressure*0.6;

	if (air_pressure >= ref_pressure) {
		pressure_diff = air_pressure - ref_pressure;
	}
	else {
		pressure_diff = 0.0;
	}

	airspeed_ms = sqrt(pressure_diff*_airspeed_ratio);
	return airspeed_ms;
}


#endif
