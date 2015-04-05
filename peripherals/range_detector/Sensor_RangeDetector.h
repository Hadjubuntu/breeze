/*
 * Sensor_RangeDetector2.h
 *
 * TODO To Check if cli sei block also timers for ESC

 Interrupts:

Board	int.0	int.1	int.2	int.3	int.4	int.5
Uno, Ethernet	2	3	 	 	 	 
Mega2560	2	3	21	20	19	18
 *
 *  Created on: 24 august 2014
 *      Author: Adrien HADJ-SALAH
 */

#ifndef SENSOR_RANGEDETECTOR2_H_
#define SENSOR_RANGEDETECTOR2_H_

#define USE_SONAR 0
#define PIN_SONAR 44
#define DELAY_BETWEEN_PING_US 500000

int sonar_pulseState = 0;
int sonar_receivingState = 0;
long sonar_pulseTimeUs = 0;
double sonar_distance_cm = -1.0;

long tStopTrigger = 0, tStartTrigger = 0;

/**
 * Initialize sonar range detector
 */
void setupSonar() {
}

/**
 * Make pulse, sonar emits ultrasounds
 */
void sonar_makePulse() {
	switch (sonar_pulseState) {
	case 0:
		pinMode(PIN_SONAR, OUTPUT);
		digitalWrite(PIN_SONAR, LOW);

		sonar_pulseState = 1;
		break;
	case 1:
		digitalWrite(PIN_SONAR, HIGH);

		sonar_pulseState = 2;
		break;
	case 2:
		digitalWrite(PIN_SONAR,LOW);
		pinMode(PIN_SONAR,INPUT);

		sonar_pulseTimeUs = timeUs();
		sonar_pulseState = 3;
		break;
	case 3:
		// Wait for next pulse in DELAY_BETWEEN_PING_US second
		// During this time sonar_update car do its job
		if (timeUs()-sonar_pulseTimeUs > DELAY_BETWEEN_PING_US) {
			sonar_pulseState = 0;
		}
		break;
	}

}

/**
 * update distance value
 * Must be runned at high speed
 */
void sonar_update() {
	if (sonar_pulseState == 3) {
		switch(digitalRead(PIN_SONAR)) {
		case HIGH:
			tStartTrigger = timeUs();
			sonar_receivingState = 1;
			break;
		case LOW:
			if (sonar_receivingState == 1) {
				tStopTrigger = timeUs();
				sonar_receivingState = 2;
			}
			break;
		}

		if (sonar_receivingState == 2) {
			volatile long duration = tStopTrigger - tStartTrigger;

			if (duration > 0) {
				sonar_distance_cm = (double)duration / 58.0;
			}

			sonar_receivingState = 0;
		}
	}
}

#endif /* SENSOR_RANGEDETECTOR2_H_ */
