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


// Pin 2 for allowing ISR
#define PIN_RANGEFINDER 2
#define TIMEOUT_ECHO_US 25000 // 25ms max timeout


int pulseId = 0;
long lastPulseUs = 0;

volatile int triggerId = 0;
volatile long tStartTrigger = 0;
volatile long tStopTrigger = 0;
volatile double distCm = 0.0 ;
volatile long rangeTimeMeasureUs = 0;


void ultrasonicResetIds() {
	pulseId = 0;
	triggerId = 0;
}


void ultrasonicStateHigh() {

	switch(digitalRead(PIN_RANGEFINDER)) {
	case HIGH:
		tStartTrigger = micros();
		triggerId ++;
		break;
	case LOW:
		tStopTrigger = micros();
		break;
	}

	volatile long duration = tStopTrigger - tStartTrigger;

	if (duration > 0) {
		// If we echoed the right pulse, then update the data
		if (pulseId == triggerId) { //&& duration < TIMEOUT_ECHO_US
			distCm = (double)duration / 58.0;
            rangeTimeMeasureUs = lastPulseUs;
		}


		// To prevent from too high INT
		if (pulseId > 10000) {
			ultrasonicResetIds();
		}
	}
}


void ultrasonicMakePulse() {
	cli();

	// Don't pulse if nothing echoes back except if there was no pulse since one second
	// to prevent from not pulsing anymore
	if (pulseId > triggerId && lastPulseUs > 0) {
		if ((millis()-(lastPulseUs/1000.0)) > 1000) {
			ultrasonicResetIds();
		}
		else {
			return;
		}
	}

	pinMode(PIN_RANGEFINDER, OUTPUT);
	digitalWrite(PIN_RANGEFINDER, LOW);
	delayMicroseconds(2);
	digitalWrite(PIN_RANGEFINDER, HIGH);
	delayMicroseconds(5);
	digitalWrite(PIN_RANGEFINDER,LOW);
	pinMode(PIN_RANGEFINDER,INPUT);

	pulseId ++ ;
	lastPulseUs = micros();
	sei();
}


void setupRangeDetector() {
	// Attach range finder to an interrupt ISR
	attachInterrupt(0, ultrasonicStateHigh, CHANGE );
}



//-------------------------------
// CODE EXAMPLE
//void loop() {
//  ultrasonicMakePulse();
//  delay(50);
//  Serial.print("Current measure in cm = ");
//  Serial.println(distCm);
//  Serial.print("Pulse Id = ");
//  Serial.println(pulseId);
//  Serial.print("Trigger Id = ");
//  Serial.println(triggerId);
// }

#endif /* SENSOR_RANGEDETECTOR2_H_ */
