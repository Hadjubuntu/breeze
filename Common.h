/*
 * Common.h
 *
 *  Created on: 10 sept. 2014
 *      Author: hadjmoody
 */

#ifndef COMMON_H_
#define COMMON_H_

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define Bound(_x, _min, _max) { if (_x > (_max)) _x = (_max); else if (_x < (_min)) _x = (_min); }
#define BoundAbs(_x, _max) Bound(_x, -(_max), (_max))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define NormRadAngle(x) { \
    while (x > M_PI) x -= 2 * M_PI; \
    while (x < -M_PI) x += 2 * M_PI; \
  }

enum FIRMWARE {
	FIXED_WING,
	QUADCOPTER,
	ROCKET
};
FIRMWARE Firmware = QUADCOPTER;

#if Firmware == QUADCOPTER
#include "firmware/quadcopter/AltitudeController.h"

enum QUAD_TYPE {
	PLUS,
	X
};
QUAD_TYPE QuadType = X;
#endif

enum FlightMode {
	FULL_MANUAL,
	FULL_FLIGHT_GPS,
	TAKE_OFF_AND_LAND,
	STABILIZE_ONLY,
	GROUND_NAV,
	RTL
};

// Flight mode at start
FlightMode flightMode = FULL_MANUAL;
// Flight mode in autopilot mode
FlightMode flightModeAutopilot = RTL;

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
#define USE_GPS_NAVIGUATION 0
#define USE_RADIO_FUTABA 1
#define USE_AIRSPEED_SENSOR 0
#define V_MIN_TAKEOFF_MS 5 // Speed m/s through GPS or Pitot sensor
#define GPS_COLD_START_DURATION_S 8 // Actually its 42s but we'll wait for the first data to throw..
// @deprecated #define USE_RANGE_FINDER 0
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
#define FLIGHT_BY_GPS_MAX_PITCH 40
#define FLIGHT_BY_GPS_MAX_ROLL 50


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
#define T_GYRO_MPU9150 3
#define GYRO_TYPE T_GYRO_MPU9150


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


#define NB_PARAMETERS 13

#define ID_KP_GROUNDNAV 0
// Flight control PID global
#define ID_G_TAU 1
// roll
#define ID_G_P_ROLL 2
#define ID_G_D_ROLL 3
#define ID_G_I_ROLL 11
// pitch
#define ID_G_P_PITCH 4
#define ID_G_D_PITCH 5
#define ID_G_I_PITCH 12
// thrust
#define ID_K_THRUST 6
#define ID_G_P_THRUST 7
#define ID_G_D_THRUST 8
#define ID_G_I_THRUST 9
#define ID_AUTOAIRSPEED_MAX_SUM_ERR 10

double param[NB_PARAMETERS] ;

void initializeUAVConfiguration() {
	// Ground navigation parameters
	param[ID_KP_GROUNDNAV] = 0.5f;

	// Flight control stabilization parameters
	param[ID_G_TAU] = 0.5f;

	param[ID_G_P_ROLL] = 1.0f;
	param[ID_G_D_ROLL] = 0.05f;
	param[ID_G_I_ROLL] = 0.05f;

	param[ID_G_P_PITCH] = 1.0f;
	param[ID_G_D_PITCH] = 0.05f;
	param[ID_G_I_PITCH] = 0.05f;

	// Thrust autospeed parameters
	param[ID_K_THRUST] = 1;
	param[ID_G_P_THRUST] = 0.8f;
	param[ID_G_D_THRUST] = 0.1f;
	param[ID_G_I_THRUST] = 0.45f;
	param[ID_AUTOAIRSPEED_MAX_SUM_ERR] = 200;
}





#endif /* COMMON_H_ */
