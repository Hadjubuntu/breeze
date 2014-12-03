/*
 * Sensor_Servo.h
 *
 * Previous Sensor Servo code using Servo.h standard library from Arduino
 *
 *  Created on: 5 sept. 2014
 *      Author: hadjmoody
 */


#ifndef SENSOR_SERVO_H_
#define SENSOR_SERVO_H_

#include "Arduino.h"
#include "Math.h"
#include <Servo.h>

//--------------------------------------
// Configuration (Same order as Pilatus pin number)
#define PIN_SERVO_AILERON_LEFT 22 // Pilatus 1
#define PIN_SERVO_GOUVERN 23 // Pilatus 2
#define PIN_SERVO_RUBBER 23 // Pilatus 3 ..
// Pin 9 Motor
#define PIN_SERVO_AILERON_RIGHT 25 // Pilatus 5
#define PIN_SERVO_FLAPS_LEFT 26 // Pilatus 6
#define PIN_SERVO_FLAPS_RIGHT 27 // Pilatus 7


// Default ROLL position in microseconds
#define DEFAULT_ROLL_LEFT_POS 1530
#define DEFAULT_ROLL_LEFT_MIN 1170
#define DEFAULT_ROLL_LEFT_MAX 1970
#define DEFAULT_ROLL_RIGHT_POS 1410
#define DEFAULT_ROLL_RIGHT_MIN 1050
#define DEFAULT_ROLL_RIGHT_MAX 1850

// Default GOUVERN position in microseconds
#define DEFAULT_GOUVERN_POS 1480
#define DEFAULT_GOUVERN_MIN 1010
// Because we need 12mm pitch down and 28mm pitch up, we choose to have (1/3)(min - diff(pos, min))
#define DEFAULT_GOUVERN_MAX 1625


// Default Rubber position in microseconds
#define DEFAULT_RUBBER_POS 1480
#define DEFAULT_RUBBER_MIN 1140
#define DEFAULT_RUBBER_MAX 1820

#define FLAPS_LEFT_DOWN 1850
#define FLAPS_LEFT_FULL 1230

#define FLAPS_RIGHT_DOWN 1130
#define FLAPS_RIGHT_FULL 1750


int aileronCmd = 0 ;
int gouvernCmd = 0 ;
int rubberCmd = 0 ;
int flapsCmd = 0;

Servo servoAileronLeft, servoAileronRight, servoGouvern, servoRubber, servoFlapsLeft, servoFlapsRight ;

double factorCmd2Us(int pos, int vmin, int vmax) {
        return max(vmax-pos, pos-vmin) / 4500.0;
}

void setupServos() {
	servoAileronLeft.attach(PIN_SERVO_AILERON_LEFT) ;
	servoAileronLeft.writeMicroseconds(DEFAULT_ROLL_LEFT_POS) ;

	servoAileronRight.attach(PIN_SERVO_AILERON_RIGHT) ;
	servoAileronRight.write(DEFAULT_ROLL_RIGHT_POS) ;


	servoFlapsLeft.attach(PIN_SERVO_FLAPS_LEFT) ;
	servoFlapsLeft.write(FLAPS_LEFT_DOWN) ;
	servoFlapsRight.attach(PIN_SERVO_FLAPS_RIGHT) ;
	servoFlapsRight.write(FLAPS_RIGHT_DOWN) ;

	//
	servoGouvern.attach(PIN_SERVO_GOUVERN) ;
	servoGouvern.write(DEFAULT_GOUVERN_POS) ;
	//
	servoRubber.attach(PIN_SERVO_RUBBER) ;
	servoRubber.write(DEFAULT_RUBBER_POS) ;

	delay(100);
}


void writeRoll() {           
	int dCmdLeftUs = (int) (factorCmd2Us(DEFAULT_ROLL_LEFT_POS, DEFAULT_ROLL_LEFT_MIN, DEFAULT_ROLL_LEFT_MAX) * aileronCmd);
	int dCmdRightUs = (int) (factorCmd2Us(DEFAULT_ROLL_RIGHT_POS, DEFAULT_ROLL_RIGHT_MIN, DEFAULT_ROLL_RIGHT_MAX) * aileronCmd);
	int leftCmdUs, rightCmdUs;

	leftCmdUs = constrain(DEFAULT_ROLL_LEFT_POS - dCmdLeftUs, DEFAULT_ROLL_LEFT_MIN, DEFAULT_ROLL_LEFT_MAX);
	rightCmdUs = constrain(DEFAULT_ROLL_RIGHT_POS - dCmdRightUs, DEFAULT_ROLL_RIGHT_MIN, DEFAULT_ROLL_RIGHT_MAX);

	servoAileronLeft.writeMicroseconds(leftCmdUs);
	servoAileronRight.writeMicroseconds(rightCmdUs);
}

void writeGouvern() {
    int sign = -1;
      
	int dCmdUs = (int) (factorCmd2Us(DEFAULT_GOUVERN_POS, DEFAULT_GOUVERN_MIN, DEFAULT_GOUVERN_MAX) * gouvernCmd);
	int cmdUs = constrain(DEFAULT_GOUVERN_POS + sign * dCmdUs, DEFAULT_GOUVERN_MIN, DEFAULT_GOUVERN_MAX);

	servoGouvern.writeMicroseconds(cmdUs);
}

void writeRubber() {
	int dCmdUs = (int) (factorCmd2Us(DEFAULT_RUBBER_POS, DEFAULT_RUBBER_MIN, DEFAULT_RUBBER_MAX) * rubberCmd);
	int cmdUs = constrain(DEFAULT_RUBBER_POS + dCmdUs, DEFAULT_RUBBER_MIN, DEFAULT_RUBBER_MAX);

	servoRubber.writeMicroseconds(cmdUs);
}

void writeFlaps() {
	// 0 means no flaps
	// 90 means full flaps

	if (flapsCmd == 0) {
		servoFlapsRight.writeMicroseconds(FLAPS_RIGHT_DOWN);
		servoFlapsLeft.writeMicroseconds(FLAPS_LEFT_DOWN);
	}
	else if (flapsCmd > 0 && flapsCmd < 90) {

		double flapsPercent = (flapsCmd/100.0) ;
		int flapsRightCmd = FLAPS_RIGHT_DOWN + (int)(flapsPercent * (FLAPS_RIGHT_FULL - FLAPS_RIGHT_DOWN)) ;
		int flapsLeftCmd = FLAPS_LEFT_DOWN + (int)(flapsPercent* (FLAPS_LEFT_FULL - FLAPS_LEFT_DOWN));

		servoFlapsRight.writeMicroseconds(flapsRightCmd);
		servoFlapsLeft.writeMicroseconds(flapsLeftCmd);
	}
	else {
		servoFlapsRight.writeMicroseconds(FLAPS_RIGHT_FULL);
		servoFlapsLeft.writeMicroseconds(FLAPS_LEFT_FULL);
	}
}

#endif /* SENSOR_SERVO_H_ */
