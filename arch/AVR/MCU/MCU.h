/*
 * MCU.h
 *
 *  Created on: Jan 22, 2015
 *      Author: adrien
 */

#ifndef MCU_H_
#define MCU_H_

#define SerialLogger Serial

long timeUs() {
	return micros();
}

class LoggerFacade {
public:

	void begin(unsigned long baudRate) {
		SerialLogger.begin(baudRate);
	}

	void print(const char *s) {
		SerialLogger.print(s);
	}
	void print(int a) {
		SerialLogger.print(a);
	}
	void print(float a) {
		SerialLogger.print(a);
	}
	void print(double a) {
		SerialLogger.print(a);
	}
	void print(long a) {
		SerialLogger.print(a);
	}

	void println(const char *s) {
		print(s);
		print("\n");
	}
	void println(int a) {
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
