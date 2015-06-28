/*
 * Motor.h
 *
 *  Created on: 1 august 2014
 *      Author: hadjmoody
 */

#ifndef ACTUATOR_MOTOR_H_
#define ACTUATOR_MOTOR_H_

#include "math/Math.h"
#include "peripherals/servo/Sensor_Servo.h"
#include "Common.h"


#define THRUST_SLEW_RATE_ACTIVATED 0
#define DECITHRUST_SLEW_RATE 60 // Max of decithrust difference each 50ms
int currentDeciThrustPercent;

/**
 * Quadcopter configuration
 * X1       X2
 *   .     .
 *     .  .
 *      Q
 *     .  .
 *   .     .
 * X3       X4
 *
 */
int thrustX1 = 0, thrustX2 = 0, thrustX3 = 0, thrustX4 = 0;
float quadY_yaw_us = 1250;
float quadY_optservo_us = 1900;
double motorMatrix[4][3];
float boost_motors = 1.0;

// In test mode : Limit motor power to x% (Real flight full thrust)
#define ESC_MAX_PROTECTION 2000


#define ESC_MIN 900
// For XC5020/14 #define ESC_MIN 900 + 150 = 1050 (engine starts at 17%)
#define ESC_MAX 2000

#define PWM_FREQUENCY 490 // in Hz
#define PWM_PRESCALER 8
#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)

// Core functions
void motorUpdateCommandDeciPercent(float, int);
void initMotor();
void motorUpdateCommandQuad();
void motorUpdateCommandFixedWing();

uint16_t _timer_period(uint16_t speed_hz) {
	return  2000000UL / speed_hz; // F_CPU/PWM_PRESCALER /speed_hz;
}

// Internal function setting freq on ICR1
void set_freq_ICR1(uint16_t icr) {
	ICR1 = icr;
}

// Internal function setting freq on ICR5
void set_freq_ICR5(uint16_t icr) {
	ICR5 = icr;
}


/* Output freq (1/period) control */
void set_freq(uint16_t freq_hz) {
	uint16_t icr = _timer_period(freq_hz);
	set_freq_ICR1(icr);

	if (Firmware == QUADCOPTER) {
		set_freq_ICR5(icr);
	}
}

void setupServoYaw() {
	// Pins on timer 3
	pinMode(6, OUTPUT); // OC4A
	pinMode(7, OUTPUT); // OC4B
	pinMode(8, OUTPUT); // OC4C

	// Timer 3
	TCCR4A =((1<<WGM41));
	TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
	OCR4A = 0xFFFF; // Init OCR registers to nil output signal
	OCR4B = 0xFFFF;
	OCR4C = 0xFFFF;

	ICR4 = 40000; // 0.5us tick => 50hz freq

	TCCR4A |= (1<<COM4A1);
	TCCR4A |= (1<<COM4B1);
	TCCR4A |= (1<<COM4C1);
}


void setupMotors() {
	// Initialize thrust percent (deci percent) to zero
	currentDeciThrustPercent = 0;

	PORTE |= _BV(0);
	PORTD |= _BV(2);
	PORTH |= _BV(0);

	// Pins on timer 5
	pinMode(46, OUTPUT); // OC5A
	pinMode(45, OUTPUT); // OC5B
	pinMode(44, OUTPUT); // OC5C

	// Timer 5
	TCCR5A =((1<<WGM51));
	TCCR5B = (1<<WGM53)|(1<<WGM52)|(1<<CS51); // |(1<<ICES5);
	OCR5A = 0xFFFF; // Init OCR registers to nil output signal
	OCR5B = 0xFFFF;
	OCR5C = 0xFFFF;
	ICR5 = 40000; // 0.5us tick => 50hz freq


	TCCR5A |= (1<<COM5A1);
	TCCR5A |= (1<<COM5B1);
	TCCR5A |= (1<<COM5C1);

	// In QUADCOPTER firmware mode
	// we need another timer for the fourth motor
	if (Firmware == QUADCOPTER) {

		// Pins on timer 1
		pinMode(11, OUTPUT); // OC1A
		pinMode(12, OUTPUT); // OC1B
		pinMode(13, OUTPUT); // OC1C

		// Timer 1
		TCCR1A =((1<<WGM11));
		TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
		ICR1 = 40000; // 0.5us tick => 50hz freq
		OCR1A = 0xFFFF; // Init OCR registers to nil output signal
		OCR1B = 0xFFFF;
		OCR1C = 0xFFFF;

		TCCR1A |= (1<<COM1A1);
		TCCR1A |= (1<<COM1B1);
		TCCR1A |= (1<<COM1C1);

		if (QuadType == PLUS) {
			motorMatrix[0][0] = 0; // roll
			motorMatrix[0][1] = 1; // pitch
			motorMatrix[0][2] = 1; // yaw

			motorMatrix[1][0] = -1;
			motorMatrix[1][1] = 0;
			motorMatrix[1][2] = -1; // yaw

			motorMatrix[2][0] = 1;
			motorMatrix[2][1] = 0;
			motorMatrix[2][2] = -1; // yaw

			motorMatrix[3][0] = 0;
			motorMatrix[3][1] = -1;
			motorMatrix[3][2] = 1; // yaw
		}
		else if (QuadType == X) {
			motorMatrix[0][0] = 1; // roll
			motorMatrix[0][1] = 1; // pitch
			motorMatrix[0][2] = -1; // yaw

			motorMatrix[1][0] = -1;
			motorMatrix[1][1] = 1;
			motorMatrix[1][2] = 1; // yaw

			motorMatrix[2][0] = 1;
			motorMatrix[2][1] = -1;
			motorMatrix[2][2] = 1; // yaw

			motorMatrix[3][0] = -1;
			motorMatrix[3][1] = -1;
			motorMatrix[3][2] = -1; // yaw
		}
		else if (QuadType == Y) {

			setupServoYaw();

			motorMatrix[0][0] = 1; // roll
			motorMatrix[0][1] = 2/3; // pitch
			motorMatrix[0][2] = 0; // yaw

			motorMatrix[1][0] = -1;
			motorMatrix[1][1] = 2/3;
			motorMatrix[1][2] = 0; // yaw

			motorMatrix[2][0] = 0;
			motorMatrix[2][1] = -4/3;
			motorMatrix[2][2] = 0; // yaw

			motorMatrix[3][0] = 0;
			motorMatrix[3][1] = 0;
			motorMatrix[3][2] = 0; // yaw
		}
	}


	// Set frequency
	set_freq(PWM_FREQUENCY);

	// Init motor to zero thrust value at start
	initMotor();
}

void initMotor() {
	motorUpdateCommandDeciPercent(1.0, 0);

	switch (Firmware) {
	case FIXED_WING:
		motorUpdateCommandFixedWing();
		break;
	case QUADCOPTER:
		thrustX1 = ESC_MIN;
		thrustX2 = ESC_MIN;
		thrustX3 = ESC_MIN;
		thrustX4 = ESC_MIN;

		motorUpdateCommandQuad();
		break;
	}
	delay(2000);
}



void motorUpdateCommandQuad()
{
	Bound(thrustX1, ESC_MIN, ESC_MAX);
	Bound(thrustX2, ESC_MIN, ESC_MAX);
	Bound(thrustX3, ESC_MIN, ESC_MAX);
	Bound(thrustX4, ESC_MIN, ESC_MAX);

	OCR5C = thrustX1  << 1 ; // pin 44
	OCR5B = thrustX2  << 1 ; // pin 45
	OCR5A = thrustX3  << 1 ; // pin 46


	if (QuadType == Y) {
		servoAPM_write(6, quadY_yaw_us);
		servoAPM_write(7, quadY_optservo_us);
	}
	// For quadType = X or +
	else {
		OCR1C = thrustX4  << 1 ; // pin 13
	}

}

void motorUpdateCommandFixedWing() {
	int pw = ESC_MIN + currentDeciThrustPercent;

	if (pw > ESC_MAX_PROTECTION) {
		pw = ESC_MAX_PROTECTION;
	}

	OCR5C = pw << 1; // Pin 44

	if (FixedWingType == DOUBLE_MOTOR) {
		OCR5B = pw << 1; // Pin 45
	}
}

// Update motor thrust using deci percent (deci for precision)
void motorUpdateCommandDeciPercent(float pBoost_motors, int deciThrustPercentNewCmd) {

	// Set input
	boost_motors = pBoost_motors;

	// Diff between thrust demanded and current thrust
	int dDeciThrust = deciThrustPercentNewCmd - currentDeciThrustPercent;

	// Adapt thrust using slew rate
	if (THRUST_SLEW_RATE_ACTIVATED == 0 || abs(dDeciThrust) < DECITHRUST_SLEW_RATE) {
		currentDeciThrustPercent = deciThrustPercentNewCmd ;
	}
	else {
		int sign = isign(dDeciThrust);
		currentDeciThrustPercent += sign * DECITHRUST_SLEW_RATE;
	}
}


double aileronOut, gouvernOut, yawOut;

void updateMotorRepartition() {
	// Minimum for the quad to hover
	int min_hover_decithrust = 50;

	// Protection to shutdown all motors
	if (currentDeciThrustPercent < 30) {
		thrustX1 = ESC_MIN;
		thrustX2 = ESC_MIN;
		thrustX3 = ESC_MIN;
		thrustX4 = ESC_MIN;
	}
	else {
		aileronOut = aileronCmd;
		gouvernOut = gouvernCmd;
		yawOut = rubberCmd;

		// We are using a coef boost on the third motor to compensate lift on rear of the tricopter
		// Previous coeff value * 1.175
		int deciThrustBoosted = (int)(boost_motors * (min_hover_decithrust + currentDeciThrustPercent));

		thrustX1 = (int) (ESC_MIN + deciThrustBoosted + (motorMatrix[0][0]*aileronOut + motorMatrix[0][1]*gouvernOut + motorMatrix[0][2]*yawOut)) ;
		thrustX2 = (int) (ESC_MIN + deciThrustBoosted + (motorMatrix[1][0]*aileronOut + motorMatrix[1][1]*gouvernOut + motorMatrix[1][2]*yawOut)) ;
		thrustX3 = (int) (ESC_MIN + deciThrustBoosted * 1.07 + (motorMatrix[2][0]*aileronOut + motorMatrix[2][1]*gouvernOut + motorMatrix[2][2]*yawOut));

		if (QuadType == Y) {
			quadY_yaw_us = 1250 + yawOut * 3.0;
			Bound(quadY_yaw_us, 1000, 1450);
		}
		else {
			thrustX4 = (int) (ESC_MIN + deciThrustBoosted + (motorMatrix[3][0]*aileronOut + motorMatrix[3][1]*gouvernOut + motorMatrix[3][2]*yawOut)) ;
		}
	}
}

void testMotor() {
	int iter = 0;
	while (iter < 5) {
		motorUpdateCommandDeciPercent(1.0, 100);

		delay(1000);
		iter ++;
	}
}


#endif /* MOTOR_H_ */
