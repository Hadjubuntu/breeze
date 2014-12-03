/*
 * Sensor_RangeDetector.h
 *
 *	4meters at 350m/s means 4/350 seconds also means approx 11 ms
 
 *      With Seedstudio ultrasonic range finder
 *      we can't use Interrupt instead of pulseIn which is a blocking process
 *
 *  Created on: 14 août 2014
 *      Author: hadjmoody
 */

#ifndef SENSOR_RANGEDETECTOR_H_
#define SENSOR_RANGEDETECTOR_H_

// Pin 2 for allowing ISR
#define PIN_RANGEFINDER 2
#define MAX_DURATION 20000

// If we want to use roll and pitch as factor to determine the z of the UAV body
// NOT USED WITH BEAM DETECTOR AS ULTRASONIC
// #define Z_PITCHROLL_FACTOR 0


class Ultrasonic {
public:
	Ultrasonic(int pin);
	void DistanceMeasure(void);
	long microsecondsToCentimeters(void);
	long microsecondsToInches(void);

	// Variables public
        volatile long duration;// the Pulse time received
private:
	int _pin;//pin number of Arduino that is connected with SIG pin of Ultrasonic Ranger.
	
};

Ultrasonic::Ultrasonic(int pin) {
	_pin = pin;
}


/*Begin the detection and get the pulse back signal*/
void Ultrasonic::DistanceMeasure(void) {
	pinMode(_pin, OUTPUT);
	digitalWrite(_pin, LOW);
	delayMicroseconds(2);
	digitalWrite(_pin, HIGH);
	delayMicroseconds(5);
	digitalWrite(_pin,LOW);
	pinMode(_pin,INPUT);

	long tmpDuration = pulseIn(_pin,HIGH); // 4 meters <=> max 23ms
        if (tmpDuration < MAX_DURATION) {
          duration = tmpDuration;
        }
}


Ultrasonic ultrasonic(PIN_RANGEFINDER);



void setupRangeDetector() {
	// Test with ISR
	//attachInterrupt(0, ultrasonicStateHigh, CHANGE );
}
//
//void ultrasonicStateHigh() {
//  if (digitalRead(PIN_RANGEFINDER) == HIGH) {
//	ultrasonic.tLastReceivingUs = micros() ;
//	ultrasonic.frameReceivingCounter ++ ;
//
//	if (ultrasonic.tLastReceivingUs > ultrasonic.tLastSendingUs) {
//		// Revelant
//		long dt = ultrasonic.tLastReceivingUs - ultrasonic.tLastSendingUs ;
//		lastBrutDistance = dt / 5800.0 ;                
//	}
//	else {
//		Serial.println("Not revelant data from range detector") ;
//	}
//  }
//}


/**
 * Returns the distance in meters
 */
double getDistance(double roll, double pitch) {
		/**

NOT USED because its a beam detection (and aperture angle of vision)

		 * 1) Determine angle between sonar vector direction and (-ez)
		 * 2) Find real distance to ground
		 x
		double angle = 180 - pitch - roll ;
                double Kangle = 1.0 ;
                if (Z_PITCHROLL_FACTOR) {
                  Kangle = cos(angle);
                }*/
 		return ultrasonic.duration / 58 ;
}


#endif /* SENSOR_RANGEDETECTOR_H_ */
