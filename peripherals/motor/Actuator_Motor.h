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
int motorMatrix[4][2];


// In test mode : Limit motor power to x% (Real flight full thrust)
#define ESC_MAX_PROTECTION 2000


#define ESC_MIN 900
// For XC5020/14 #define ESC_MIN 900 + 150 = 1050 (engine starts at 17%)
#define ESC_MAX 2000

#define PWM_FREQUENCY 490 // in Hz
#define PWM_PRESCALER 8
#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)

// Core functions
void motorUpdateCommand(int);
void initMotor();

uint16_t _timer_period(uint16_t speed_hz) {
	return  2000000UL / speed_hz; // F_CPU/PWM_PRESCALER /speed_hz;
}

// Internal function setting freq on ICR4
void set_freq_ICR4(uint16_t icr) {
	ICR4 = icr;
}

// Internal function setting freq on ICR5
void set_freq_ICR5(uint16_t icr) {
	ICR5 = icr;
}


/* Output freq (1/period) control */
void set_freq(uint16_t freq_hz) {
	uint16_t icr = _timer_period(freq_hz);
	set_freq_ICR4(icr);

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

	pinMode(8,OUTPUT); // CH_3 (PH5/OC4C)
	pinMode(7,OUTPUT); // CH_4 (PH4/OC4B)
	pinMode(6,OUTPUT); // CH_5 (PH3/OC4A)

	// Create timer 4

	// WGM: 1 1 1 0. Clear Timer on Compare, TOP is ICR4.
	// CS41: prescale by 8 => 0.5us tick
	TCCR4A =((1<<WGM41));
	TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
	OCR4A = 0xFFFF; // Init OCR registers to nil output signal
	OCR4B = 0xFFFF;
	OCR4C = 0xFFFF;
	ICR4 = 40000; // 0.5us tick => 50hz freq


	TCCR4A |= (1<<COM4C1); // CH_3 : OC4C
	TCCR4A |= (1<<COM4B1); // CH_4 : OC4B
	TCCR4A |= (1<<COM4A1); // CH_5 : OC4A

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
			motorMatrix[0][0] = 0;
			motorMatrix[0][1] = 1;
			motorMatrix[1][0] = -1;
			motorMatrix[1][1] = 0;
			motorMatrix[2][0] = 1;
			motorMatrix[2][1] = 0;
			motorMatrix[3][0] = 0;
			motorMatrix[3][1] = -1;
		}
		else if (QuadType == X) {
			motorMatrix[0][0] = 1;
			motorMatrix[0][1] = 1;
			motorMatrix[1][0] = -1;
			motorMatrix[1][1] = 1;
			motorMatrix[2][0] = 1;
			motorMatrix[2][1] = -1;
			motorMatrix[3][0] = -1;
			motorMatrix[3][1] = -1;
		}
	}


	// Set frequency
	set_freq(PWM_FREQUENCY);

	// Init motor to zero thrust value at start
	initMotor();
}

void initMotor() {
	motorUpdateCommand(ESC_MIN);
	delay(2000);
}



void motorUpdateCommand(int pThrust)
{
	if (pThrust > ESC_MAX_PROTECTION) {
		pThrust = ESC_MAX_PROTECTION;
	}

	switch (Firmware) {
	case FIXED_WING:
		OCR4A = pThrust  << 1 ;
		break;

	case QUADCOPTER:

		Bound(thrustX1, ESC_MIN, ESC_MAX);
		Bound(thrustX2, ESC_MIN, ESC_MAX);
		Bound(thrustX3, ESC_MIN, ESC_MAX);
		Bound(thrustX4, ESC_MIN, ESC_MAX);

		OCR5C = thrustX1  << 1 ; // pin 44
		OCR5B = thrustX2  << 1 ; // pin 45
		OCR5A = thrustX3  << 1 ; // pin 46
		OCR4C = thrustX4  << 1 ; // pin 8
		break;
	case ROCKET:
		break;
	}


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

	int pw = ESC_MIN + currentDeciThrustPercent;
	motorUpdateCommand(pw);
}

void updateMotorRepartition() {
	int factor = 60;

	// Protection to shutdown all motors
	if (currentDeciThrustPercent < 10) {
		thrustX1 = ESC_MIN;
		thrustX2 = ESC_MIN;
		thrustX3 = ESC_MIN;
		thrustX4 = ESC_MIN;
	}
	else {
		thrustX1 = ESC_MIN + currentDeciThrustPercent + motorMatrix[0][0]*(aileronCmd/4500.0)*factor + motorMatrix[0][1]*(gouvernCmd/4500.0)*factor ;
		thrustX2 = ESC_MIN + currentDeciThrustPercent + motorMatrix[1][0]*(aileronCmd/4500.0)*factor + motorMatrix[1][1]*(gouvernCmd/4500.0)*factor ;
		thrustX3 = ESC_MIN + currentDeciThrustPercent + motorMatrix[2][0]*(aileronCmd/4500.0)*factor + motorMatrix[2][1]*(gouvernCmd/4500.0)*factor;
		thrustX4 = ESC_MIN + currentDeciThrustPercent + motorMatrix[3][0]*(aileronCmd/4500.0)*factor + motorMatrix[3][1]*(gouvernCmd/4500.0)*factor ;
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
