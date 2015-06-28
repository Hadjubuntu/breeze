/*
 * MCU.h
 *
 *  Created on: Jan 22, 2015
 *      Author: adrien
 */

#ifndef MCU_H_
#define MCU_H_

#include "arch/AVR/UART/UARTDriver.h"

AVRUARTDriverISRs(0);
AVRUARTDriverInstance(uartA, 0);

//#define SerialLogger Serial

long timeUs() {
	return micros();
}


long timeMs() {
	return millis();
}

class LoggerFacade {
public:

	void begin(unsigned long baudRate) {
	//	SerialLogger.begin(baudRate);
		uartA.begin(baudRate);
	}

	void print(const char *s) {
	//	SerialLogger.print(s);
		uartA.print(s);
	}
	void print(int a) {
//		SerialLogger.print(a);
		uartA.print(a);
	}
	void print(float a) {
//		SerialLogger.print(a);

		uartA.print(a);
	}
	void print(double a) {
//		SerialLogger.print(a);

		uartA.print(a);
	}
	void print(long a) {
//		SerialLogger.print(a);

		uartA.print(a);
	}

	void print(uint8_t a) {
		uartA.print(a);
	}

	void println(const char *s) {
//		print(s);
//		print("\n");

		uartA.println(s);
	}
	void println(int a) {
		print(a);
		print("\n");
	}
	void println(byte a) {
		print(a);
		print("\n");
	}
	void println(float a) {
		print(a);
		print("\n");
	}
	void println(long a) {
		print(a);
		print("\n");
	}
	void println(double a) {
		print(a);
		print("\n");
	}
};


LoggerFacade Logger;

#endif /* MCU_H_ */
