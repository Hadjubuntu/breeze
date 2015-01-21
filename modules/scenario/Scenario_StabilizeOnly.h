/*
 * Scenario_StabilizeOnly.h
 *
 *  Created on: Dec 19, 2014
 *      Author: adrien
 */

#ifndef SCENARIO_STABILIZEONLY_H_
#define SCENARIO_STABILIZEONLY_H_

#include "Common.h"
/**
 * The plane must try to keep the roll and pitch to 0
 */
void stabilizeOnly() {
	if (time_TakeOffStart == 0) {
		time_TakeOffStart = currentTime;
	}

	if (currentTime-time_TakeOffStart <= 3*S_TO_US) {
		UAVCore->deciThrustPercent = 0; // 15 is enough for the plane to ride !
	}
	else {
		UAVCore->deciThrustPercent = 0;
	}
	flapsCmd = 90;
	controlAttitudeCommand(UAVCore->currentAttitude, UAVCore->attitudeCommanded, G_Dt, currentTime,  0, 0, 0);
}

#endif /* SCENARIO_STABILIZEONLY_H_ */
