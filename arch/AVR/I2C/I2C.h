/*
 * I2C.h
 *
 *  Created on: 14 août 2014
 *      Author: hadjmoody
 */

#include "arch/AVR/wire/Wire.h"
#include "Arduino.h"

#ifndef I2C_H_
#define I2C_H_


//fonction ecriture I2C
//---------------------
void writeTo(byte device, byte toAddress, byte val)
{
	Wire.beginTransmission(device);
	Wire.write(toAddress);
	Wire.write(val);
	Wire.endTransmission();
}

uint8_t i2cWriteArray(byte device, uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
	Wire.beginTransmission(device);
	Wire.write(registerAddress);
	Wire.write(data, length);
	uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
	if (rcode) {
		//Logger.print(F("i2cWrite failed: "));
		//Logger.println(rcode);
	}
	return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}


uint8_t i2cWrite(byte device, uint8_t registerAddress, uint8_t data, bool sendStop) {
	return i2cWriteArray(device, registerAddress, &data, 1, sendStop); // Returns 0 on success
}


//fonction lecture I2C
//--------------------
void readFrom(byte device, byte fromAddress, int num, byte result[])
{
	Wire.beginTransmission(device);
	Wire.write(fromAddress);
	Wire.endTransmission();
	Wire.requestFrom((int)device, num);

	int i = 0;

	while(Wire.available())
	{
		result[i] = Wire.read();
		i++;
	}
}



#endif /* I2C_H_ */
