/*
 * Navig2.h
 *
 *  Created on: Jan 4, 2015
 *      Author: adrien
 */

// Inspired by paparazzi uav => fixedwing > nav.c

#include "arch/AVR/MCU/MCU.h"
#include "modules/mission/MissionCommon.h"
#include "modules/nav/NavigCommon.h"
#include "modules/flight_control/FlightControl.h"
#include "peripherals/altimeter/Sensor_Altimeter.h"

#ifndef NAVIG2_H_
#define NAVIG2_H_


// Input
//------------------------------------------

// Waypoints,
// Current step in flight plan
// navMode (nav, return home)

// Output
//------------------------------------------
float _roll_nav = 0.0;
float _pitch_nav = 0.0;


/**
 * Tells whether the UAV is near a waypoint
 */
bool navCloseEnoughToWP(MissionWP el) {
	if (geoDistance(el.wp, currentPosition) < DIST_TO_WAYPOINT_FOR_VALIDATION) {
		return true;
	}
	else {
		return false;
	}
}

/**
 * Naviguate to waypoint
 */
void navToWP(MissionWP el)
{
	double dist2WPMeters = 0.0;

	dist2WPMeters = geoDistance(currentPosition, el.wp);

	Vector2 groundspeedVector = getGroundSpeedVect2();
	double horizontalSpeedDir = atan2f(groundspeedVector.x, groundspeedVector.y);

	// Update flight plan step and nav mode if needed
	double h_ctl_course_pgain = 1.2;
	double carrot = 1.0;

	// Update roll navigation
	Vector2 vectWP = geoPositionToVector2(el.wp);
	Vector2 currentPos = geoPositionToVector2(currentPosition);

	double desired_x = vectWP.x;
	double desired_y = vectWP.y;

	float diff = atan2f(desired_x - currentPos.x, desired_y - currentPos.y) - horizontalSpeedDir;
	NormRadAngle(diff);
	BoundAbs(diff,M_PI/2.);
	float s = sinf(diff);
	float speed = airspeed_ms_mean->getAverage();
	double h_ctl_roll_setpoint = toDeg(atanf(2 * speed * speed * s * h_ctl_course_pgain / (carrot * SCALING_SPEED * 9.81)));
	BoundAbs(h_ctl_roll_setpoint, FLIGHT_BY_GPS_MAX_ROLL);
	gpsRollDesired = h_ctl_roll_setpoint;

	// Update pitch navigation
	double currentAltitude = currentPosition.alt;
		currentAltitude = (int) altCF;
	double dzMeters = (el.wp.alt - currentAltitude);
	double tmpPitchDesired = toDeg(atan(dzMeters / dist2WPMeters));
	gpsPitchDesired = constrain(tmpPitchDesired, FLIGHT_BY_GPS_MIN_PITCH, FLIGHT_BY_GPS_MAX_PITCH);
}





#endif /* NAVIG2_H_ */
