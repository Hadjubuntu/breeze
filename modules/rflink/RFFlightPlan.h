/*
 * RFFlightPlan.h
 *
 *  Created on: Mar 1, 2015
 *      Author: Adrien Hadj Salah
 */

#ifndef RFFLIGHTPLAN_H_
#define RFFLIGHTPLAN_H_

#include "modules/mission/Mission.h"
#include "arch/common/StrUtils.h"
#include "arch/common/StrStack.h"



// The GCS send a flag containing new_fp and the size of the new flight plan
// The UAV doesn't add any data if this state is not engaged.
//
// When UAV has been initialized its new state, the GCS send flight plan
// element by element such as ([0] => waypoint, [1] => circle, ..) with a delay between packets.
//
// The UAV saves the unreceived ID and ask the GCS to resend the missed datas
// At the end, the UAV must send a ack to tell the GCS that everything has been uploaded

bool fpIncoming = false;
int newFpSize = 0;

void initFlightPlanIncoming(int pSize) {
	fpIncoming = true;
	newFpSize = pSize;
}

void endFlightPlanIncoming() {
	fpIncoming = false;
	newFpSize = 0;
}

void updateFlightPlan(char *rfcmd, EStringArray tokens) {

	// TBC

}


#endif /* RFFLIGHTPLAN_H_ */
