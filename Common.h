/*
 * Common.h
 *
 *  Created on: 10 sept. 2014
 *      Author: hadjmoody
 */

#ifndef COMMON_H_
#define COMMON_H_

enum FlightMode {
	FULL_MANUAL,
	FULL_FLIGHT_GPS,
	TAKE_OFF_AND_LAND,
	STABILIZE_ONLY,
	GROUND_NAV,
};

// Flight mode at start
FlightMode flightMode = FULL_MANUAL;
// Flight mode in autopilot mode
FlightMode flightModeAutopilot = GROUND_NAV;

enum FlightState {
	SETUP,
	TAKEOFF,
	CLIMB,
	CRUISE,
	LANDING,
};


//---------------------------------------
// Configuration
#define START_IN_AUTOPILOT 0
#define V_MIN_TAKEOFF_MS 5 // Speed m/s through GPS or Pitot sensor
#define USE_GPS_NAVIGUATION 1
#define GPS_COLD_START_DURATION_S 20 // Actually its 42s but we'll wait for the first data to throw..
// @deprecated #define USE_RANGE_FINDER 0
#define USE_AIRSPEED_SENSOR 1
#define MAX_PITCH_UP 40
#define MAX_PITCH_DOWN -30
#define MAX_ROLL 50
#define MAX_DELAY_TAKEOFF_US 30000000L // Timeout of 30 seconds max for takeoff, afterwise go in cruise mode
#define CLIMB_DURATION_SECONDS 3

int AUTOSPEED_CONTROLLER = 0;  // If used, then define a speed vms as a goal, thrust in controlled by a PD controller
							   // If not, then you must define the thrust during cruise phase
							   // Could be change in flight

#define S_TO_US 1000000.0f // Converts second to us
#define MS_TO_US 1000.0f // Converts ms to us

// To protect GPS glitches
#define GPS_GLITCH_PROTECTION 0
#define NB_GPS_GLITCHES_MAX 5
#define MAX_DISTANCE_BETWEEN_ESTIM_AND_POS 30 // 25 meters acceptance between estimation and real position

// Define min max authorized pitch command while flying with a gps
#define FLIGHT_BY_GPS_MIN_PITCH -30
#define FLIGHT_BY_GPS_MAX_PITCH 35
#define FLIGHT_BY_GPS_MAX_ROLL 40


// For a smooth flight use droll max and dpitch max
// To blank slowly decrease those values
// If value set to 0, then the option is desactived
#define DROLL_MAX 80 // Max roll speed (80°/s means 40° in half second)
#define DPITCH_MAX 80 // Max pitch speed

#define YAW_ROLL_COMPENSATION 1
#define YAW_MAX_COMPENSATION 15
#define YAW2SRV_ROLL 0.1

#define PITCH_ROLL_COMPENSATION 1
#define PITCH_MAX_COMPENSATION 10 // Max Compensation at 45° bank angle
#define PITCH2SRV_ROLL 0.8

// To move aileron depending on the speed, the more speed the less control surface we need
#define SCALING_SPEED   14.0f
#define THROTTLE_CRUISE 50
#define MAX_V_MS_PLANE 30

#define T_GYRO_MPU6050 1
#define T_GYRO_ITG3200 2
#define GYRO_TYPE T_GYRO_ITG3200


// Common structures
typedef struct T_ATTITUDE {
	double roll, pitch, yaw ;
	long time ;
} Attitude ;



int sign(double v) {
	if (v < 0.0) {
		return -1;
	}
	else {
		return 1;
	}
}


#define NB_PARAMETERS 7

#define ID_KP_GROUNDNAV 0
#define ID_G_TAU 1
#define ID_G_P_ROLL 2
#define ID_G_D_ROLL 3
#define ID_K_THRUST 4
#define ID_G_P_THRUST 5
#define ID_G_D_THRUST 6

double param[NB_PARAMETERS] ;

void initializeUAVConfiguration() {
	// Ground navigation parameters
	param[ID_KP_GROUNDNAV] = 0.15f;

	// Flight control stabilization parameters
	param[ID_G_TAU] = 0.5f;
	param[ID_G_P_ROLL] = 1.2f;
	param[ID_G_D_ROLL] = 0.05f;

	// Thrust autospeed parameters
	param[ID_K_THRUST] = 3;
	param[ID_G_P_THRUST] = 0.5f;
	param[ID_G_D_THRUST] = 0.15f;
}


#endif /* COMMON_H_ */
