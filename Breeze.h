/*
 * Breeze.h
 *
 *  Created on: 1 august 2014
 *      Author: hadjmoody
 */

#ifndef BREEZE_H_
#define BREEZE_H_


#include <stdlib.h>

#include "Common.h"
#include "peripherals/servo/Sensor_Servo.h"

#if GYRO_TYPE == T_GYRO_MPU6050
#include "peripherals/IMU/Sensor_Gyro.h"
#elif GYRO_TYPE == T_GYRO_ITG3200
#include "peripherals/IMU/Sensor_Gyro_10DOF.h"
#endif

#include "peripherals/range_detector/Sensor_RangeDetector2.h"
#include "peripherals/motor/Actuator_Motor.h"
#include "peripherals/airspeed/Sensor_Airspeed.h"
#include "peripherals/GPS/Sensor_GPS.h"
#include "modules/nav/Navig.h"
#include "modules/flight_control/FlightControl.h"
#include "modules/rflink/RFLink.h"
#include "peripherals/altimeter/Sensor_Altimeter.h"

#include "peripherals/radio_sbus/RadioSBUSFutaba.h"

long dt100HzSum = 0;
long iter100Hz = 0;

// main loop time variable
unsigned long frameCounter = 0; // main loop executive frame counter

// Memorable times
unsigned long time_TakeOffStart = 0;
unsigned long time_TakeOffDone = 0;

unsigned long time_ClimbStart = 0;
unsigned long time_ClimbDone = 0;

unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;

#define TASK_100HZ 1
#define TASK_50HZ 2
#define TASK_20HZ 5
#define TASK_10HZ 10
#define TASK_5HZ 20
#define TASK_2HZ 50
#define TASK_1HZ 100

// sub loop time variable
unsigned long oneHZpreviousTime = 0;
unsigned long tenHZpreviousTime = 0;
unsigned long fiftyHZpreviousTime = 0;
unsigned long hundredHZpreviousTime = 0;
double G_Dt = 0.0; // Dt in task




// Initialize attitude structure
//----------------------------------------------------
void initializeAttitude(Attitude *att) {
	att->roll = 0.0 ;
	att->pitch = 0.0 ;
	att->yaw = 0.0 ;
}


// UAV structure
//----------------------------------------------------
typedef struct T_UAV {
	bool autopilot;
	FlightState flightState ;

	// Thrust in deci-% (from 0 to 1000)
	int deciThrustPercent ;

	// Altitude sonar (filtered)
	// Not used anymore because of lake of precision
	// FilterAverage *altitudeSonar;

	// Celerity in m/s
	double v_ms ;
	double v_ms_goal;

	// Attitude from gyro
	Attitude *currentAttitude ;

	// Gyro rate in deg/s
	double gyroXrate ;
	double gyroYrate ;

	// Roll, pitch, yaw defined by the PID to hold naviguation
	Attitude *attitudeCommanded ;

	// Heading cap using gyro (or magnometer)
	double headingCap ;

} Uav;

Uav *UAVCore = new Uav() ;

void initializeUAVStructure(Uav *uav) {

	// UAVCore->altitudeSonar = new  FilterAverage(15, 0, 300, true);
	UAVCore->autopilot = START_IN_AUTOPILOT ? true : false;
	UAVCore->deciThrustPercent = 0 ;

	UAVCore->currentAttitude = new Attitude() ;
	initializeAttitude(UAVCore->currentAttitude) ;


	UAVCore->attitudeCommanded = new Attitude() ;
	UAVCore->attitudeCommanded->roll = 0.0 ;
	UAVCore->attitudeCommanded->pitch = 0.0 ;
	UAVCore->attitudeCommanded->yaw = 0 ;


	// Ready to take-off
	UAVCore->flightState = SETUP ;

	// Set speed to zero
	UAVCore->v_ms_goal = 0.0;

	// Heading cap at start
	UAVCore->headingCap = 0.0 ;

}




// Delete all pointers
//----------------------------------------------------
void shutdown() {
	free(UAVCore) ;
}



#endif /* BREEZE_H_ */
