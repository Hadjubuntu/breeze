/*
 * Sensor_GPS.h
 *
 * We won't use software serial anymore, because its interfering with the servo library with using
 * cli() function which stop interruption. It may also interfer with ESC PWM code.
 *
 * Therefore we use only TinyGPS and the serial2 on arduino mega port : 17 (rx)
 * The Arduino Mega has three additional serial ports: Serial1 on pins 19 (RX) and 18 (TX), Serial2 on pins 17 (RX) and 16 (TX), Serial3 on pins 15 (RX) and 14 (TX).
 *
 *  Created on: 8 sept. 2014
 *      Author: hadjmoody
 */
#include "ublox.h"
#include "Navig.h"

#ifndef SENSOR_GPS_H_
#define SENSOR_GPS_H_

// Maximum error of dilution for horizontal GPS location
#define HDOP_MAX_TOLERANCE 3.0
#define NB_DATA_TO_THROWOUT 5
#define MIN_SATELLITE_IN_VIEW 5

// Last celerity in m/s
double lastVms = 0.0;
bool isGPSArmed = false ;
double GPSAltitudeAtStart = -1.0;

// Define the variables that will be used
int32_t latitude, longitude, gpsAltMeters;
int gpsCourseDegrees;
int nbSatsInView;

// Define which pins you will use on the Arduino to communicate with your
// GPS. Because we uses hardware-serial com, we used Serial 2 <=> Rx port 17 on arduino mega
#define RXPIN 17

//Set this value equal to the baud rate of your GPS
#define GPSBAUD 38400

// In the setup function, we connect GPS to the Serial2 of Arduino's Mega
// Then we throw out first data to prevent from first glitches
void setupGPS() {
	latitude = 0;
	longitude = 0;
	gpsAltMeters = 0;
	nbSatsInView = 0;
	gpsCourseDegrees = 0;

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
			char charac = (char)(Serial2.read());    // load the data into a variable...
			int parsedRes = ubloxProcessData(charac);

			if (parsedRes) {
				if (gpsData.sats >= MIN_SATELLITE_IN_VIEW) {
					throwOutNumber ++ ;
				}
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
		char charac = (char)(Serial2.read());    // load the data into a variable...
		int parsedRes = ubloxProcessData(charac);
		// Upon a successfully parsed sentence, zero the idlecounter and update position data
		if (parsedRes) {
			if (gpsData.state == GPS_DETECTING) {
				gpsData.state = GPS_NOFIX; // make sure to lose detecting state (state may not have been updated by parser)
			}

			gpsData.idlecount = 0;
			latitude = gpsData.lat; // 10^7 deg
			longitude = gpsData.lon ; // 10^7 deg
			gpsAltMeters = gpsData.height / 1000.0; // mm to meters
			nbSatsInView = gpsData.sats;
			lastVms = gpsData.speed / 100.0; // cm/s to m/s
			gpsCourseDegrees = (int)(gpsData.course / (1.0e3));

			if (GPSAltitudeAtStart == -1.0) {
				GPSAltitudeAtStart = gpsAltMeters;
			}

			updateGPSData(latitude, longitude, gpsAltMeters-GPSAltitudeAtStart, lastVms, gpsCourseDegrees, gpsData.fixtime);
		}
	}
}




#endif /* SENSOR_GPS_H_ */
