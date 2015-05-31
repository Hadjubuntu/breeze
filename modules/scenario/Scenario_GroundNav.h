/*
 * Scenario_GroundNav.h
 *
 *  Created on: Dec 19, 2014
 *      Author: adrien
 */

#ifndef SCENARIO_GROUNDNAV_H_
#define SCENARIO_GROUNDNAV_H_

#include "Common.h"
#include "modules/nav/Navig.h"

/**
 * Ground Nav aims to test the GPS naviguation by steering the plane on the ground
 */
void groundNavDemo() {
	if (time_TakeOffStart == 0) {
		time_TakeOffStart = currentTime;
	}

	//AUTOSPEED_CONTROLLER = 0;
	// Configuration autospeed controller
	// with 1.2 m/s and almost 50% thrust max to achieve that goal
	AUTOSPEED_CONTROLLER = 1;
	UAVCore->v_ms_goal = 3.0; // m/s

	// Stop motor if wrong value of pitch (on the nose..)
	if (UAVCore->currentAttitude->pitch < - 30) {
		UAVCore->deciThrustPercent = 0;
	}

	if (GPSState == GPS_STATE_LOST) {
		UAVCore->deciThrustPercent = 0;
	}
	// If the mission is done, stop motor and do flaps full
	else if (MissionDone) {
		UAVCore->deciThrustPercent = 0;
		flapsCmd = 90;
	}
	else {
		double yawDesired = 0.0;
		updateHeading();


		// After 2 seconds, start guidance, before just maintain steering
		if (currentTime - time_TakeOffStart > S_TO_US * 2) {
			yawDesired = gpsRollDesired;
		}

		if (AUTOSPEED_CONTROLLER == 0) {
			// Just enough thrust to move the plane (15% is enough)
			UAVCore->deciThrustPercent = 150;
		}

		// Simple move the rubber depending on gps roll demand
		UAVCore->attitudeCommanded->roll = 0;
		UAVCore->attitudeCommanded->pitch = 10;
		UAVCore->attitudeCommanded->yaw = param[ID_KP_GROUNDNAV] * yawDesired;
	}
}


#endif /* SCENARIO_GROUNDNAV_H_ */
