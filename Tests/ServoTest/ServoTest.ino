#include "Arduino.h"
#include <Servo.h>

int frameCounter = 0 ;
long dt = 0 ;
long previousTime = 0, time = 0 ;
long timeStart = 0 ;

Servo servoAileronLeft, servoAileronRight, servoRudder, servoGouvern ;

void setup() {
	servoAileronLeft.attach(3) ;
	servoAileronRight.attach(4) ;
	servoRudder.attach(5) ;
	servoGouvern.attach(6) ;

	timeStart = micros() ;
}

void process100HzTask() {

}

void process1HzTask() {
	long durationSinceStartInSeconds = (micros()-timeStart) / (10^6) ;
	if (durationSinceStartInSeconds < 5) {
		servoAileronLeft.write(90-30) ;
		servoAileronLeft.write(90+30) ;
		servoRudder.write(5) ;
		servoGouvern.write(10) ;
	}
	else if (durationSinceStartInSeconds < 10) {
		servoAileronLeft.write(90) ;
		servoAileronLeft.write(90) ;
		servoRudder.write(0) ;
		servoGouvern.write(40) ;
	}
	else {
		servoGouvern.write(-15) ;
	}
}

void loop() {
	time = micros() ;
	dt = time - previousTime ;

	if (dt > 10000) {
		process100HzTask() ;

		// 50 Hz
		if (frameCounter % 2) {

		}

		// 10 Hz
		if (frameCounter % 10) {

		}

		// 1 Hz each second
		if (frameCounter % 100) {

		}
	}

	previousTime = micros() ;
	if (frameCounter >= 100) {
		frameCounter = 0;
	}
}
