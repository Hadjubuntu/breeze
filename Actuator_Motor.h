/*
 * Motor.h
 *
 *  Created on: 1 august 2014
 *      Author: hadjmoody
 */

#ifndef ACTUATOR_MOTOR_H_
#define ACTUATOR_MOTOR_H_

#include "Math.h"


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
/* Output freq (1/period) control */
void set_freq(uint16_t freq_hz) {
	uint16_t icr = _timer_period(freq_hz);
	ICR4 = icr;
}


void setupMotors()
{
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


	// Set frequence
	set_freq(PWM_FREQUENCY);

	TCCR4A |= (1<<COM4C1); // CH_3 : OC4C
	TCCR4A |= (1<<COM4B1); // CH_4 : OC4B
	TCCR4A |= (1<<COM4A1); // CH_5 : OC4A

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

	int pw = 1000 + currentDeciThrustPercent;
	motorUpdateCommand(pw);
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
