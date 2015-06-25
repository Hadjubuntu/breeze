/*
 * Sensor_Servo.h
 *
 *  Created on: 5 sept. 2014
 *      Author: hadjmoody
 */


#ifndef SENSOR_SERVO_H_
#define SENSOR_SERVO_H_

#include "peripherals/servo/ServoAPM.h"
#include "math/Math.h"
#include "Common.h"

enum SurfaceControlMode {
	SC_THREE_AXIS_NORMAL, // Aileron, elevator, rubber
	SC_ELEVON, // Aileron and elevator
};

enum SurfaceControlMode SC_MODE = SC_THREE_AXIS_NORMAL;

#define USE_FLAPS 1
#define USE_RUBBER 1

//--------------------------------------
// Configuration (Same order as Pilatus pin number)
#define PIN_SERVO_AILERON_LEFT 5 // Pilatus 1
#define PIN_SERVO_GOUVERN 2 // Pilatus 2

#if (USE_RUBBER == 1)
#define PIN_SERVO_RUBBER 3 // Pilatus 3 ..
#endif

#define PIN_SERVO_AILERON_RIGHT 6 // Pilatus 5

#if (USE_FLAPS == 1)
#define PIN_SERVO_FLAPS_LEFT 7 // Pilatus 6
#define PIN_SERVO_FLAPS_RIGHT 8 // Pilatus 7
#endif


// Default ROLL position in microseconds
#define DEFAULT_ROLL_LEFT_POS 1560 // 1530
#define DEFAULT_ROLL_LEFT_MIN 1170
#define DEFAULT_ROLL_LEFT_MAX 1970

#define DEFAULT_ROLL_RIGHT_POS 1410
#define DEFAULT_ROLL_RIGHT_MIN 1050
#define DEFAULT_ROLL_RIGHT_MAX 1850


// Default GOUVERN position in microseconds
#define DEFAULT_GOUVERN_POS 1520 // 1480
#define DEFAULT_GOUVERN_MIN 1140
#define DEFAULT_GOUVERN_MAX 1820


// Default Rubber position in microseconds
#define DEFAULT_RUBBER_POS 1480
#define DEFAULT_RUBBER_MIN 1140
#define DEFAULT_RUBBER_MAX 1820

#define FLAPS_LEFT_DOWN 1850
#define FLAPS_LEFT_FULL 1230

#define FLAPS_RIGHT_DOWN 1130
#define FLAPS_RIGHT_FULL 1750


// Private variables
//----------------------------------------------------
int aileronCmd = 0 ;
int gouvernCmd = 0 ;
int rubberCmd = 0 ;

#define FLAPS_SLEW_RATE 1 // Percent of change every 10ms (100 Hz), 1 means one second to go from no flaps to full flaps
int flapsCurrentCmd = 0;
int flapsCmd = 0;

// Transform a position from +/- 45 degrees
// into a delay in microseconds
//----------------------------------------------------
double factorCmd2Us(int pos, int vmin, int vmax) {
	return max(vmax-pos, pos-vmin) / 90.0;
}

// Initialize servo at startup position
//----------------------------------------------------
void setupServos() {


	if (Firmware == FIXED_WING) {
		setupServoAPM();
		delay(100);

		servoAPM_write(PIN_SERVO_AILERON_LEFT, DEFAULT_ROLL_LEFT_POS);
		servoAPM_write(PIN_SERVO_AILERON_RIGHT, DEFAULT_ROLL_RIGHT_POS);

#if (USE_FLAPS == 1)
		servoAPM_write(PIN_SERVO_FLAPS_LEFT, FLAPS_LEFT_DOWN);
		servoAPM_write(PIN_SERVO_FLAPS_RIGHT, FLAPS_RIGHT_DOWN);
#endif

#if (USE_RUBBER == 1)
		servoAPM_write(PIN_SERVO_RUBBER, DEFAULT_RUBBER_POS);
		servoAPM_write(PIN_SERVO_GOUVERN, DEFAULT_GOUVERN_POS);
#endif

	}

	delay(100);
}


// Update elevon position
//---------------------------------------------------
double elevon_aileron = 0.4;
double elevon_elevator = 0.7;
void writeElevon() {
	double elevonCmd = elevon_aileron * aileronCmd + elevon_elevator * gouvernCmd;
	int dCmdLeftUs = (int) (factorCmd2Us(DEFAULT_ROLL_LEFT_POS, DEFAULT_ROLL_LEFT_MIN, DEFAULT_ROLL_LEFT_MAX) * elevonCmd);
	int dCmdRightUs = (int) (factorCmd2Us(DEFAULT_ROLL_RIGHT_POS, DEFAULT_ROLL_RIGHT_MIN, DEFAULT_ROLL_RIGHT_MAX) * elevonCmd);
	int leftCmdUs, rightCmdUs;

	leftCmdUs = constrain(DEFAULT_ROLL_LEFT_POS - dCmdLeftUs, DEFAULT_ROLL_LEFT_MIN, DEFAULT_ROLL_LEFT_MAX);
	rightCmdUs = constrain(DEFAULT_ROLL_RIGHT_POS - dCmdRightUs, DEFAULT_ROLL_RIGHT_MIN, DEFAULT_ROLL_RIGHT_MAX);


	servoAPM_write(PIN_SERVO_AILERON_LEFT, leftCmdUs);
	servoAPM_write(PIN_SERVO_AILERON_RIGHT, rightCmdUs);
}

// Update aileron position
//----------------------------------------------------
void writeRoll() {           
	int dCmdLeftUs = (int) (factorCmd2Us(DEFAULT_ROLL_LEFT_POS, DEFAULT_ROLL_LEFT_MIN, DEFAULT_ROLL_LEFT_MAX) * aileronCmd);
	int dCmdRightUs = (int) (factorCmd2Us(DEFAULT_ROLL_RIGHT_POS, DEFAULT_ROLL_RIGHT_MIN, DEFAULT_ROLL_RIGHT_MAX) * aileronCmd);
	int leftCmdUs, rightCmdUs;

	leftCmdUs = constrain(DEFAULT_ROLL_LEFT_POS - dCmdLeftUs, DEFAULT_ROLL_LEFT_MIN, DEFAULT_ROLL_LEFT_MAX);
	rightCmdUs = constrain(DEFAULT_ROLL_RIGHT_POS - dCmdRightUs, DEFAULT_ROLL_RIGHT_MIN, DEFAULT_ROLL_RIGHT_MAX);


	servoAPM_write(PIN_SERVO_AILERON_LEFT, leftCmdUs);
	servoAPM_write(PIN_SERVO_AILERON_RIGHT, rightCmdUs);
}


// Update elevator position
//----------------------------------------------------
void writeGouvern() {
	int sign = -1;

	int dCmdUs = (int) (factorCmd2Us(DEFAULT_GOUVERN_POS, DEFAULT_GOUVERN_MIN, DEFAULT_GOUVERN_MAX) * gouvernCmd);
	int cmdUs = constrain(DEFAULT_GOUVERN_POS + sign * dCmdUs, DEFAULT_GOUVERN_MIN, DEFAULT_GOUVERN_MAX);

	servoAPM_write(PIN_SERVO_GOUVERN, cmdUs);
}

// Update rubber position
//----------------------------------------------------
void writeRubber() {
#if (USE_RUBBER == 1)
	int sign = 1;

	int dCmdUs = (int) (factorCmd2Us(DEFAULT_RUBBER_POS, DEFAULT_RUBBER_MIN, DEFAULT_RUBBER_MAX) * rubberCmd);
	int cmdUs = constrain(DEFAULT_RUBBER_POS + sign*dCmdUs, DEFAULT_RUBBER_MIN, DEFAULT_RUBBER_MAX);

	servoAPM_write(PIN_SERVO_RUBBER, cmdUs);
#endif
}


// Update flaps position
//----------------------------------------------------
void writeFlaps() {
#if (USE_FLAPS == 1)
	// 0 means no flaps
	// 90 means full flaps

	// Eval new flaps command depending on the current flaps using a flaps slew rate
	int dflaps = flapsCmd - flapsCurrentCmd;

	if (abs(dflaps) <= FLAPS_SLEW_RATE) {
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




// Manage functions to be called in order to control
// surfaces
//----------------------------------------------------
void updateSurfaceControls() {
	switch (SC_MODE) {
	case SC_THREE_AXIS_NORMAL:
		writeRoll() ;
		writeGouvern();
		writeFlaps();
		writeRubber();
		break;
	case SC_ELEVON:
		writeElevon();
		break;
	}
}

#endif /* SENSOR_SERVO_H_ */
