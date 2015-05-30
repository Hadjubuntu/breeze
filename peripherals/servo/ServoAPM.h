/*
 * ServoAPM.h
 * Pin mapping :
 *
Arduino Pin	 Register
2	OCR3B
3	OCR3C
4	OCR4C
5	OCR3A
6	OCR4A
7	OCR4B
8	OCR4C
9	OCR2B
10	OCR2A
11	OCR1A
12	OCR1B
13	OCR0A
44	OCR5C
45	OCR5B
46	OCR5A
 *  Created on: 29 oct. 2014
 *      Author: Adrien Hadj-Salah
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
//	// Pins on timer 1
	pinMode(11, OUTPUT); // OC1A
	pinMode(12, OUTPUT); // OC1B
	pinMode(13, OUTPUT); // OC1C

	// Pins on timer 3
	pinMode(5, OUTPUT); // OC3A
	pinMode(2, OUTPUT); // OC3B
	pinMode(3, OUTPUT); // OC3C


	// Pins on timer 4
	pinMode(6, OUTPUT); // OC4A
	pinMode(7, OUTPUT); // OC4B
	pinMode(8, OUTPUT); // OC4C

	// Pins on timer 5
//	pinMode(46, OUTPUT); // OC5A
//	pinMode(45, OUTPUT); // OC5B
//	pinMode(44, OUTPUT); // OC5C

	// PPM on timer 5 reader (ICP5 PL1 pin 48)
	// pinMode(48, INPUT);

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

	// Timer 4
	TCCR4A =((1<<WGM41));
	TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
	OCR4A = 0xFFFF; // Init OCR registers to nil output signal
	OCR4B = 0xFFFF;
	OCR4C = 0xFFFF;
	ICR4 = 40000; // 0.5us tick => 50hz freq

	// Timer 5
//	TCCR5A =((1<<WGM51));
//	TCCR5B = (1<<WGM53)|(1<<WGM52)|(1<<CS51); // |(1<<ICES5);
//	OCR5A = 0xFFFF; // Init OCR registers to nil output signal
//	OCR5B = 0xFFFF;
//	OCR5C = 0xFFFF;
//	ICR5 = 40000; // 0.5us tick => 50hz freq

	// Enable channel
	TCCR1A |= (1<<COM1A1);
	TCCR1A |= (1<<COM1B1);
	TCCR1A |= (1<<COM1C1);
	TCCR3A |= (1<<COM3A1);
	TCCR3A |= (1<<COM3B1);
	TCCR3A |= (1<<COM3C1);
	TCCR4A |= (1<<COM4A1);
	TCCR4A |= (1<<COM4B1);
	TCCR4A |= (1<<COM4C1);
//	TCCR5A |= (1<<COM5A1);
//	TCCR5A |= (1<<COM5B1);
//	TCCR5A |= (1<<COM5C1);

	// Capture mask
	//TIMSK5 |= (1<<ICIE5);
}


// Variable definition for Input Capture interrupt
/*int NUM_CHANNELS = 8;
volatile unsigned int ICR5_old;
volatile unsigned char PPM_Counter=0;
volatile uint16_t PWM_RAW[8] = {2400,2400,2400,2400,2400,2400,2400,2400};
volatile unsigned char radio_status=0;*/

/****************************************************
   Input Capture Interrupt ICP5 => PPM signal read
 ***************************************************
ISR(TIMER5_CAPT_vect)
{
  unsigned int Pulse;
  unsigned int Pulse_Width;

  Pulse=ICR5;
  if (Pulse<ICR5_old)     // Take care of the overflow of Timer5 (TOP=40000)
    Pulse_Width=(Pulse + 40000)-ICR5_old;  //Calculating pulse
  else
    Pulse_Width=Pulse-ICR5_old;            //Calculating pulse
  if (Pulse_Width>8000)   // SYNC pulse?
    PPM_Counter=0;
  else
    {
    PPM_Counter &= 0x07;  // For safety only (limit PPM_Counter to 7)
    PWM_RAW[PPM_Counter++]=Pulse_Width;  //Saving pulse.
    Serial.print("pulse width  ");
    Serial.print(PPM_Counter);
    Serial.print(" => ");
    Serial.println(Pulse_Width);
    if (PPM_Counter >= NUM_CHANNELS)
      radio_status = 1;
    }
  ICR5_old = Pulse;
}*/


void servoAPM_write(int pin, int period_us) {
	Bound(period_us, MIN_SERVO_US, MAX_SERVO_US);

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

		// Timer 4
	case 6:
		OCR4A = pwm;
		break;
	case 7:
		OCR4B = pwm;
		break;
	case 8:
		OCR4C = pwm;
		break;


		// Timer 5
//	case 46:
//		OCR5A = pwm;
//		break;
//	case 45:
//		OCR5B = pwm;
//		break;
//	case 44:
//		OCR5C = pwm;
//		break;
	}
}

#endif /* SERVOAPM_H_ */
