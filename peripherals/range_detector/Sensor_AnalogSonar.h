/*
 * Sensor_AnalogSonar.h
 *
 *  Created on: Jun 11, 2015
 *      Author: adrien
 */

#ifndef PERIPHERALS_RANGE_DETECTOR_SENSOR_ANALOGSONAR_H_
#define PERIPHERALS_RANGE_DETECTOR_SENSOR_ANALOGSONAR_H_

#include "Common.h"

#define ANALOG_PIN_SONAR 0
#define SONAR_MAX_ALT_CM 600.0f

bool sonarHealthy = false;
float sonarDerivativeCms = 0.0;
float sonarAltCm = 0.0;
long lastSonarUpdate = 0;

void setupSonar() {

}

/**
 * Update sonar data
 * @param cTime current time
 */
void updateSonar(long cTime) {
	float dt = 0.01;
	if (lastSonarUpdate > 0) {
		dt = (cTime - lastSonarUpdate) / S_TO_US;
	}

	// Used to read in the analog voltage output that is being sent by the MaxSonar device.
	// Scale factor is (Vcc/512) per inch. A 5V supply yields ~9.8mV/in
	// Arduino analog pin goes from 0 to 1024, so the value has to be divided by 2 to get the actual inches
	float lastSonarReadCm = ((float) analogRead(ANALOG_PIN_SONAR)) / 2.0 * 2.54;

	// Update derivative sonar in cm/s
	float newDerivative = (lastSonarReadCm - sonarAltCm) / dt;
	sonarDerivativeCms = 0.5 * sonarDerivativeCms + 0.5 * newDerivative;

	// Average altitude sonar
	sonarAltCm = 0.5 * sonarAltCm + 0.5 * lastSonarReadCm;

	// Update healthy flag
	if (sonarAltCm >= 0.0 && sonarAltCm < SONAR_MAX_ALT_CM) {
		sonarHealthy = true;
	}
	else {
		sonarHealthy = false;
	}

	lastSonarUpdate = cTime;
}


#endif /* PERIPHERALS_RANGE_DETECTOR_SENSOR_ANALOGSONAR_H_ */
