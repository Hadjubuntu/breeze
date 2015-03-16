/*
 * Motor.h
 *
 *  Created on: 1 august 2014
 *      Author: hadjmoody
 */

#ifndef ACTUATOR_MOTOR_H_
#define ACTUATOR_MOTOR_H_

#include "math/Math.h"
#include "Breeze.h"
#include "Common.h"


#define THRUST_SLEW_RATE_ACTIVATED 1
#define DECITHRUST_SLEW_RATE 20 // Max of decithrust difference each 50ms
int currentDeciThrustPercent;

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

// Internal function setting freq on ICR3
void set_freq_ICR3(uint16_t icr) {
	ICR4 = icr;
}


/* Output freq (1/period) control */
void set_freq(uint16_t freq_hz) {
	uint16_t icr = _timer_period(freq_hz);
	set_freq_ICR4(icr);

	if (Firmware == QUADCOPTER) {
		set_freq_ICR3(icr);
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
		// Pins on timer 3
		pinMode(5, OUTPUT); // OC3A
		pinMode(2, OUTPUT); // OC3B
		pinMode(3, OUTPUT); // OC3C

		// Timer3
		TCCR3A =((1<<WGM31));
		TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
		OCR3A = 0xFFFF; // Init OCR registers to nil output signal
		OCR3B = 0xFFFF;
		OCR3C = 0xFFFF;
		ICR3 = 40000; // 0.5us tick => 50hz freq

		TCCR3A |= (1<<COM3A1);
		TCCR3A |= (1<<COM3B1);
		TCCR3A |= (1<<COM3C1);
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

	OCR4A = pThrust  << 1 ;
	OCR4B = pThrust  << 1 ;
	OCR4C = pThrust  << 1 ;
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
