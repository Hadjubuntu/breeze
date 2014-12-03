/*
 * ServoAPM.h
 *
 *  Created on: 29 oct. 2014
 *      Author: hadjmoody
 */

#ifndef SERVOAPM_H_
#define SERVOAPM_H_

#define NB_SERVO_MAX 9
#define MIN_SERVO_US 544
#define MAX_SERVO_US 2100


#include "Arduino.h"
#include <avr/interrupt.h>


// Servo on timer1, timer3 and timer5
void setupServoAPM() {
	// Pins on timer 1
	pinMode(11, OUTPUT); // OC1A
	pinMode(12, OUTPUT); // OC1B
	pinMode(13, OUTPUT); // OC1C

	// Pins on timer 3
	pinMode(5, OUTPUT); // OC3A
	pinMode(2, OUTPUT); // OC3B
	pinMode(3, OUTPUT); // OC3C

	// Pins on timer 5
	pinMode(46, OUTPUT); // OC5A
	pinMode(45, OUTPUT); // OC5B
	pinMode(44, OUTPUT); // OC5C

	// Timer 1
	TCCR1A =((1<<WGM11));
	TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
	ICR1 = 40000; // 0.5us tick => 50hz freq
	OCR1A = 0xFFFF; // Init OCR registers to nil output signal
	OCR1B = 0xFFFF;
	OCR1C = 0xFFFF;

	// Timer 3
	TCCR3A =((1<<WGM31));
	TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
	OCR3A = 0xFFFF; // Init OCR registers to nil output signal
	OCR3B = 0xFFFF;
	OCR3C = 0xFFFF;
	ICR3 = 40000; // 0.5us tick => 50hz freq

	// Timer 5
	TCCR5A =((1<<WGM51));
	TCCR5B = (1<<WGM53)|(1<<WGM52)|(1<<CS51);
	OCR5A = 0xFFFF; // Init OCR registers to nil output signal
	OCR5B = 0xFFFF;
	OCR5C = 0xFFFF;
	ICR5 = 40000; // 0.5us tick => 50hz freq

	// Enable channel
	TCCR1A |= (1<<COM1A1);
	TCCR1A |= (1<<COM1B1);
	TCCR1A |= (1<<COM1C1);
	TCCR3A |= (1<<COM3A1);
	TCCR3A |= (1<<COM3B1);
	TCCR3A |= (1<<COM3C1);
	TCCR5A |= (1<<COM5A1);
	TCCR5A |= (1<<COM5B1);
	TCCR5A |= (1<<COM5C1);
}

void servoAPM_write(int pin, int period_us) {
	if (period_us < MIN_SERVO_US) {
		period_us = MIN_SERVO_US;
	}
	else if (period_us > MAX_SERVO_US) {
		period_us = MAX_SERVO_US;
	}

	uint16_t pwm = period_us << 1;
	switch (pin) {
	// Timer 1
	case 11:
		OCR1A = pwm;
		break;
	case 12:
		OCR1B = pwm;
		break;
	case 13:
		OCR1C = pwm;
		break;

		// Timer 3
	case 5:
		OCR3A = pwm;
		break;
	case 2:
		OCR3B = pwm;
		break;
	case 3:
		OCR3C = pwm;
		break;


		// Timer 5
	case 46:
		OCR5A = pwm;
		break;
	case 45:
		OCR5B = pwm;
		break;
	case 44:
		OCR5C = pwm;
		break;
	}
}

#endif /* SERVOAPM_H_ */
