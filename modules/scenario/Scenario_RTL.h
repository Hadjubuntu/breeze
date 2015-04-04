/*
 * Scenario_RTL.h
 *
 *  Created on: Dec 19, 2014
 *      Author: adrien
 */



#ifndef SCENARIO_RTL_H_
#define SCENARIO_RTL_H_

#include "Common.h"
#include "modules/nav/Navig.h"


bool rtlFromSky = false;
void rtlNav() {
	if (time_TakeOffStart == 0) {
		time_TakeOffStart = currentTime;
		if (altitudeBarometer->getAverage() > 400) {
			rtlFromSky = true;
		}
	}


	AUTOSPEED_CONTROLLER = 1;

	if (GPSState == GPS_STATE_LOST) {
		UAVCore->deciThrustPercent = 0;
	}
	// If the mission is done, stop motor and do flaps full
	else if (MissionDone) {
		UAVCore->deciThrustPercent = 0;
		flapsCmd = 90;
	}
	else {
		updateHeading(altitudeBarometer);

		if (rtlFromSky == true) {
			if (altitudeBarometer->getAverage() > 180) {
				UAVCore->v_ms_goal = 12; // m/s

				UAVCore->attitudeCommanded->roll = gpsRollDesired;
				UAVCore->attitudeCommanded->pitch = gpsPitchDesired;
				UAVCore->attitudeCommanded->yaw = 0;
				flapsCmd = 0;
			}
			else {
				UAVCore->v_ms_goal = 9; // m/s
				UAVCore->attitudeCommanded->roll = 0;
				UAVCore->attitudeCommanded->pitch = 6;
				UAVCore->attitudeCommanded->yaw = 0;
				flapsCmd = 60;
			}
		}
		else {
			UAVCore->attitudeCommanded->roll = 0;
			UAVCore->attitudeCommanded->pitch = 0;
			UAVCore->attitudeCommanded->yaw = param[ID_KP_GROUNDNAV] * angleDiff;

			UAVCore->v_ms_goal = 3; // m/s
			flapsCmd = 90;
		}
	}

}

#endif
