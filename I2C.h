/*
 * I2C.h
 *
 *  Created on: 14 août 2014
 *      Author: hadjmoody
 */

#include <Wire.h>
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
