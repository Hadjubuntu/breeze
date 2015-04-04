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
int newFpLength = 0;
int newFpIdx = 0;
int missingFpIdxList[MISSION_ELEMENT_NB];
int missingFpIdx = 0;

void reinitMissingFpIdx() {
	for (int i = 0; i < MISSION_ELEMENT_NB; i ++) {
		missingFpIdxList[i] = -1;
	}
}

void initFlightPlanIncoming(int pSize) {

	// Init variables
	fpIncoming = true;
	newFpLength = pSize;

	// Delete all mission elements from the previous mission
	missionErase();

	// Set missing idx to null
	reinitMissingFpIdx();
}

void endFlightPlanIncoming() {
	fpIncoming = false;
	newFpLength = 0;
	newFpIdx = 0;
	missingFpIdx = 0;

	// Send missing elements
	// TODO keep a cpt to define a maximum of retries

	// Re init missing elements
	reinitMissingFpIdx();
}


// Check current flightplan incoming step
// Parse payload and add element to mission list
// tokens
// 0 : flightplan
// 1 : idx
// 2 : type of mission element
// 3-.. : payload
void updateFlightPlan(char *rfcmd, EStringArray tokens) {

	// Check idx, the GCS must send flight plan is the same exact order
	// from element 0 to the element sizeFlightPlan-1
	int cIdx = atoi(tokens.array[1]);
	if (cIdx != newFpIdx) {
		missingFpIdxList[missingFpIdx] = newFpIdx;
	}
	// Otherwise parse payload and add mission element to the list
	else {
		MissionElement el = parseRFMissionInput(rfcmd);
		missionAdd(&el); // TODO what if parse failed

		newFpIdx ++;
	}
}


#endif /* RFFLIGHTPLAN_H_ */
