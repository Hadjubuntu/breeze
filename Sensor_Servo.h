/*
 * Sensor_Servo.h
 *
 *  Created on: 5 sept. 2014
 *      Author: hadjmoody
 */


#ifndef SENSOR_SERVO_H_
#define SENSOR_SERVO_H_

#include "ServoAPM.h"
#include "Math.h"

#define USE_FLAPS 0

//--------------------------------------
// Configuration (Same order as Pilatus pin number)
#define PIN_SERVO_AILERON_LEFT 11 // Pilatus 1
#define PIN_SERVO_GOUVERN 12 // Pilatus 2
#define PIN_SERVO_RUBBER 2 // Pilatus 3 ..

#define PIN_SERVO_AILERON_RIGHT 5 // Pilatus 5

#if (USE_FLAPS == 1)
#define PIN_SERVO_FLAPS_LEFT 13 // Pilatus 6
#define PIN_SERVO_FLAPS_RIGHT 3 // Pilatus 7
#endif

/** New Breeze Model
 *
#define PIN_SERVO_AILERON_LEFT 11
#define PIN_SERVO_FLAPS_LEFT 12

#define PIN_SERVO_STEERING 13

#define PIN_SERVO_AILERON_RIGHT 5
#define PIN_SERVO_FLAPS_RIGHT 2

#define PIN_SERVO_ATAIL_LEFT 46
#define PIN_SERVO_ATAIL_RIGHT 45
 */


// Default ROLL position in microseconds
#define DEFAULT_ROLL_LEFT_POS 1650
#define DEFAULT_ROLL_LEFT_MIN 1400
#define DEFAULT_ROLL_LEFT_MAX 1900

#define DEFAULT_ROLL_RIGHT_POS 1410
#define DEFAULT_ROLL_RIGHT_MIN 1160
#define DEFAULT_ROLL_RIGHT_MAX 1660

// Default GOUVERN position in microseconds
#define DEFAULT_GOUVERN_POS 1650
#define DEFAULT_GOUVERN_MIN 1330
#define DEFAULT_GOUVERN_MAX 1970


// Default Rubber position in microseconds
#define DEFAULT_RUBBER_POS 1510
#define DEFAULT_RUBBER_MIN 950
#define DEFAULT_RUBBER_MAX 1930

#define FLAPS_LEFT_DOWN 1850
#define FLAPS_LEFT_FULL 1230

#define FLAPS_RIGHT_DOWN 1130
#define FLAPS_RIGHT_FULL 1750

/**
 * PILATUS CONFIGURATION
 * // Default ROLL position in microseconds
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
 */


int aileronCmd = 0 ;
int gouvernCmd = 0 ;
int rubberCmd = 0 ;

#define FLAPS_SLEW_RATE 1 // Percent of change every 10ms (100 Hz), 1 means one second to go from no flaps to full flaps
int flapsCurrentCmd = 0;
int flapsCmd = 0;


double factorCmd2Us(int pos, int vmin, int vmax) {
	return max(vmax-pos, pos-vmin) / 4500.0;
}

void setupServos() {
	setupServoAPM();
	delay(100);

	servoAPM_write(PIN_SERVO_AILERON_LEFT, DEFAULT_ROLL_LEFT_POS);
	servoAPM_write(PIN_SERVO_AILERON_RIGHT, DEFAULT_ROLL_RIGHT_POS);

#if (USE_FLAPS == 1)
	servoAPM_write(PIN_SERVO_FLAPS_LEFT, FLAPS_LEFT_DOWN);
	servoAPM_write(PIN_SERVO_FLAPS_RIGHT, FLAPS_RIGHT_DOWN);
#endif

	servoAPM_write(PIN_SERVO_RUBBER, DEFAULT_RUBBER_POS);
	servoAPM_write(PIN_SERVO_GOUVERN, DEFAULT_GOUVERN_POS);

	delay(100);
}


void writeRoll() {           
	int dCmdLeftUs = (int) (factorCmd2Us(DEFAULT_ROLL_LEFT_POS, DEFAULT_ROLL_LEFT_MIN, DEFAULT_ROLL_LEFT_MAX) * aileronCmd);
	int dCmdRightUs = (int) (factorCmd2Us(DEFAULT_ROLL_RIGHT_POS, DEFAULT_ROLL_RIGHT_MIN, DEFAULT_ROLL_RIGHT_MAX) * aileronCmd);
	int leftCmdUs, rightCmdUs;

	leftCmdUs = constrain(DEFAULT_ROLL_LEFT_POS + dCmdLeftUs, DEFAULT_ROLL_LEFT_MIN, DEFAULT_ROLL_LEFT_MAX);
	rightCmdUs = constrain(DEFAULT_ROLL_RIGHT_POS + dCmdRightUs, DEFAULT_ROLL_RIGHT_MIN, DEFAULT_ROLL_RIGHT_MAX);


	servoAPM_write(PIN_SERVO_AILERON_LEFT, leftCmdUs);
	servoAPM_write(PIN_SERVO_AILERON_RIGHT, rightCmdUs);
}

void writeGouvern() {
	int sign = 1;

	int dCmdUs = (int) (factorCmd2Us(DEFAULT_GOUVERN_POS, DEFAULT_GOUVERN_MIN, DEFAULT_GOUVERN_MAX) * gouvernCmd);
	int cmdUs = constrain(DEFAULT_GOUVERN_POS + sign * dCmdUs, DEFAULT_GOUVERN_MIN, DEFAULT_GOUVERN_MAX);

	servoAPM_write(PIN_SERVO_GOUVERN, cmdUs);
}

void writeRubber() {
        int sign = -1;
	int dCmdUs = (int) (factorCmd2Us(DEFAULT_RUBBER_POS, DEFAULT_RUBBER_MIN, DEFAULT_RUBBER_MAX) * rubberCmd);
	int cmdUs = constrain(DEFAULT_RUBBER_POS + sign*dCmdUs, DEFAULT_RUBBER_MIN, DEFAULT_RUBBER_MAX);

	servoAPM_write(PIN_SERVO_RUBBER, cmdUs);
}

void writeFlaps() {
#if (USE_FLAPS == 1)
	// 0 means no flaps
	// 90 means full flaps

	// Eval new flaps command depending on the current flaps using a flaps slew rate
	int dflaps = flapsCurrentCmd - flapsCmd;

	if (abs(dflaps) < FLAPS_SLEW_RATE) {
		flapsCurrentCmd = flapsCmd;
	}
	else {
		int sign = isign(dflaps);
		flapsCurrentCmd += sign * FLAPS_SLEW_RATE;
	}

	if (flapsCurrentCmd == 0) {
		servoAPM_write(PIN_SERVO_FLAPS_LEFT, FLAPS_LEFT_DOWN);
		servoAPM_write(PIN_SERVO_FLAPS_RIGHT, FLAPS_RIGHT_DOWN);

	}
	else if (flapsCurrentCmd > 0 && flapsCurrentCmd < 90) {

		double flapsPercent = (flapsCurrentCmd/100.0) ;
		int flapsRightCmd = FLAPS_RIGHT_DOWN + (int)(flapsPercent * (FLAPS_RIGHT_FULL - FLAPS_RIGHT_DOWN)) ;
		int flapsLeftCmd = FLAPS_LEFT_DOWN + (int)(flapsPercent* (FLAPS_LEFT_FULL - FLAPS_LEFT_DOWN));

		servoAPM_write(PIN_SERVO_FLAPS_LEFT, flapsLeftCmd);
		servoAPM_write(PIN_SERVO_FLAPS_RIGHT, flapsRightCmd);
	}
	else {
		servoAPM_write(PIN_SERVO_FLAPS_LEFT, FLAPS_LEFT_FULL);
		servoAPM_write(PIN_SERVO_FLAPS_RIGHT, FLAPS_RIGHT_FULL);
	}
#endif
}

#endif /* SENSOR_SERVO_H_ */
