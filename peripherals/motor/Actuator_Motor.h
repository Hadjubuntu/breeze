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


#define THRUST_SLEW_RATE_ACTIVATED 1
#define DECITHRUST_SLEW_RATE 40 // Max of decithrust difference each 50ms
int currentDeciThrustPercent;

/**
 * Quadcopter configuration
 * X1       X2			pin6     pin7
 *   .     .
 *     .  .
 *      Q
 *     .  .
 *   .     .
 * X3       X4         pin8     pin5
 *
 */
int thrustX1 = 0, thrustX2 = 0, thrustX3 = 0, thrustX4 = 0;
int motorMatrix[4][3];


// In test mode : Limit motor power to x% (Real flight full thrust)
#define ESC_MAX_PROTECTION 2000


#define ESC_MIN 900
// For XC5020/14 #define ESC_MIN 900 + 150 = 1050 (engine starts at 17%)
#define ESC_MAX 2000

#define PWM_FREQUENCY 490 // in Hz
#define PWM_PRESCALER 8
#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)

// Core functions
void motorUpdateCommandDeciPercent(int);
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


void setupMotors() {
	// Initialize thrust percent (deci percent) to zero
	currentDeciThrustPercent = 0;

	PORTE |= _BV(0);
	PORTD |= _BV(2);
	PORTH |= _BV(0);

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

	// In QUADCOPTER firmware mode
	// we need another timer for the fourth motor
	if (Firmware == QUADCOPTER) {

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
	}


	// Set frequency
	set_freq(PWM_FREQUENCY);

	// Init motor to zero thrust value at start
	initMotor();
}

void initMotor() {
	motorUpdateCommandDeciPercent(0);

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

	OCR5C = ESC_MIN  << 1 ; // pin 44
	OCR5B = ESC_MIN  << 1 ; // pin 45
	OCR5A = ESC_MIN  << 1 ; // pin 46
	OCR1C = ESC_MIN  << 1 ; // pin 13

}

void motorUpdateCommandFixedWing() {
	int pw = ESC_MIN + currentDeciThrustPercent;

	if (pw > ESC_MAX_PROTECTION) {
		pw = ESC_MAX_PROTECTION;
	}

	OCR1C = pw  << 1 ;
}

// Update motor thrust using deci percent (deci for precision)
void motorUpdateCommandDeciPercent(int deciThrustPercentNewCmd) {

	int dDeciThrust = deciThrustPercentNewCmd - currentDeciThrustPercent;

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

	// Protection to shutdown all motors
	if (currentDeciThrustPercent < 10) {
		thrustX1 = ESC_MIN;
		thrustX2 = ESC_MIN;
		thrustX3 = ESC_MIN;
		thrustX4 = ESC_MIN;
	}
	else {
		aileronOut = aileronCmd / 100.0;
		gouvernOut = gouvernCmd / 100.0;
		yawOut = rubberCmd / 100.0;

		thrustX1 = ESC_MIN + 360 + currentDeciThrustPercent + (motorMatrix[0][0]*aileronOut + motorMatrix[0][1]*gouvernOut + motorMatrix[0][2]*yawOut) ;
		thrustX2 = ESC_MIN + 360 + currentDeciThrustPercent + (motorMatrix[1][0]*aileronOut + motorMatrix[1][1]*gouvernOut + motorMatrix[1][2]*yawOut) ;
		thrustX3 = ESC_MIN + 360 + currentDeciThrustPercent + (motorMatrix[2][0]*aileronOut + motorMatrix[2][1]*gouvernOut + motorMatrix[2][2]*yawOut);
		thrustX4 = ESC_MIN + 360 + currentDeciThrustPercent + (motorMatrix[3][0]*aileronOut + motorMatrix[3][1]*gouvernOut + motorMatrix[3][2]*yawOut) ;
	}
}

void testMotor() {
	int iter = 0;
	while (iter < 5) {
		motorUpdateCommandDeciPercent(100);

		delay(1000);
		iter ++;
	}
}


#endif /* MOTOR_H_ */
