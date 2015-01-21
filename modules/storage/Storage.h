/*
 * Storage.h
 *
 * Used to store data as a black box
 *
 * Created on: 17 sept. 2014
 *      Author: Adrien HADJ-SALAH
 */

#ifndef STORAGE_H_
#define STORAGE_H_

// #include <EEPROM.h>

/**
 *
 * NEVER USE EEPROM :
 *
 * An EEPROM write takes 3.3 ms to complete.
 *  The EEPROM memory has a specified life of 100,000 write/erase cycles,
 *  so you may need to be careful about how often you write to it.
 */

#if defined(__AVR_ATmega2560__)
#define STORAGE_SIZE 4096
#else
#define STORAGE_SIZE 512
#endif

int currentAddr = 0;

void writeData(int data) {
	data = constrain(data, 0, 255);
	// eeprom.write
}


#endif /* STORAGE_H_ */
