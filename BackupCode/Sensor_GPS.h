/*
 * Sensor_GPS.h
 *
 * This code works for the EM406-A Sirf III GPS
 *
 * We won't use software serial anymore, because its interfering with the servo library with using
 * cli() function which stop interruption. It may also interfer with ESC PWM code.
 *
 * Therefore we use only TinyGPS and the serial2 on arduino mega port : 17 (rx)
 * The Arduino Mega has three additional serial ports: Serial1 on pins 19 (RX) and 18 (TX), Serial2 on pins 17 (RX) and 16 (TX), Serial3 on pins 15 (RX) and 14 (TX).
 *
 *
 *  Created on: 8 sept. 2014
 *      Author: Adrien Hadj-Salah
 */

#ifndef SENSOR_GPS_H_
#define SENSOR_GPS_H_
/**
#include <TinyGPS.h>
#include "Navig.h"



// Maximum error of dilution for horizontal GPS location
#define HDOP_MAX_TOLERANCE 3.0
#define NB_DATA_TO_THROWOUT 5

// Last celerity in m/s
double lastVms = 0.0;
bool isGPSArmed = false ;
double GPSAltitudeAtStart = -1.0;

// Define which pins you will use on the Arduino to communicate with your
// GPS. Because we uses hardware-serial com, we used Serial 2 <=> Rx port 17 on arduino mega
#define RXPIN 17

//Set this value equal to the baud rate of your GPS
#define GPSBAUD 4800

// Create an instance of the TinyGPS object
TinyGPS gps;

// This is where you declare prototypes for the functions that will be
// using the TinyGPS library.
void updateSensorGPS(TinyGPS &gps);

// In the setup function, we connect GPS to the Serial2 of Arduino's Mega
// Then we throw out first data to prevent from first glitches
void setupGPS() {

  // Before starting throwing first data, wait for the GPS to have a fix using cold start duration
  // from GPS Sirf III features
  delay(GPS_COLD_START_DURATION_S * 1000);

  // Then start throwing out first data
  int throwOutNumber = 0;
  
  //Sets baud rate of your GPS
  Serial2.begin(GPSBAUD);
  
  while (throwOutNumber < NB_DATA_TO_THROWOUT)     // While there is data on the RX pin...
  {
    if (Serial2.available()) {
      int c = Serial2.read();    // load the data into a variable...
      if(gps.encode(c))      // if there is a new valid sentence...
      {
        float latitude, longitude;
        gps.f_get_position(&latitude, &longitude);
        
        throwOutNumber ++ ;
      }
    }

  }
  
  isGPSArmed = true;
}

// This must be runned as fast as possible (more than 200 Hz)
// All it does is check for data on
// the RX pin of the arduino serial choosen (Serial2 on Arduino Mega),
// makes sure the data is valid NMEA sentences,
// then jumps to the getgps() function.
void updateGPS()
{
  if (Serial2.available())     // While there is data on the RX pin...
  {
      int c = Serial2.read();    // load the data into a variable...
      if(gps.encode(c))      // if there is a new valid sentence...
      {
    	  updateSensorGPS(gps);         // then grab the data.
      }
  }
}

// The getgps function will get and print the values we want.
void updateSensorGPS(TinyGPS &gps) {
	if (GPSAltitudeAtStart == -1.0) {
		GPSAltitudeAtStart = gps.f_altitude();
	}

  // Define the variables that will be used
  float latitude, longitude;
  // Then call this function
  gps.f_get_position(&latitude, &longitude);


  // Update data in naviguation
 // if (gps.hdop() < HDOP_MAX_TOLERANCE) {
	  updateGPSData(latitude, longitude, gps.f_altitude()-GPSAltitudeAtStart);
	  lastVms = gps.f_speed_mps();
 // }
}


*/

#endif /* SENSOR_GPS_H_ */
