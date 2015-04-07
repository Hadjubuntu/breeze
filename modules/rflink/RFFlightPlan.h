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

// Variables
//------------------------------------------------
bool fpIncoming = false;
int newFpLength = 0;
int newFpIdx = 0;
int missingFpIdxList[MISSION_ELEMENT_NB];
int missingFpIdx = 0;

// Retry send config variables
//------------------------------------------------
bool retryMode = false;
int retryIdx = -1;

// Pointer to retry mission uploading function in RFLink2.h
void (*ptnRFRetryFunc)(int) ;

// Initialize RF FlightPlan
//------------------------------------------------
void setupRFFlightPlan(void (*pPtnRetryFunc)(int)) {
	ptnRFRetryFunc = pPtnRetryFunc;
}

void reinitMissingFpIdx() {
	for (int i = 0; i < MISSION_ELEMENT_NB; i ++) {
		missingFpIdxList[i] = -1;
	}
}

void reinitRetryMode() {
	retryIdx = -1;
	retryMode = false;
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

	// Re init missing elements
	reinitMissingFpIdx();
}

// Tell GCS we didn't received data from a certain index
// GCS must stop sending data and resend data from this starting
// point
void callForRetryFromIdx(int idxRetryStartingPoint) {
	// Call RF
	ptnRFRetryFunc(idxRetryStartingPoint);
	retryIdx = idxRetryStartingPoint;
}


// Check current flightplan incoming step
// Parse payload and add element to mission list
// tokens
// 0 : flightplan
// 1 : idx
// 2 : type of mission element
// 3-.. : payload
void updateFlightPlan(char *rfcmd, EStringArray tokens) {

	// Check FP incoming state has been sent
	if (fpIncoming) {
		// Get current flightplan element index
		int cIdx = atoi(tokens.array[1]);

		// If retry succeeded, quit retry mode
		if (retryMode
				&& cIdx == retryIdx
				&& cIdx == newFpIdx) {
			reinitRetryMode();
		}

		// Check idx, the GCS must send flight plan is the same exact order
		// from element 0 to the element sizeFlightPlan-1
		if (cIdx != newFpIdx) {
			missingFpIdxList[missingFpIdx] = newFpIdx;
			callForRetryFromIdx(newFpIdx);
		}
		// Otherwise parse payload and add mission element to the list
		else {
			MissionElement el = parseRFMissionInput(rfcmd);
			missionAdd(&el); // TODO Manage the case when parsing failed

			// Increment idx to be received
			newFpIdx ++;

			// If we received all the flightplan elements,
			// then stops incoming state
			if (newFpIdx >= newFpLength) {
				endFlightPlanIncoming();
			}
		}
	}
	else {
		// TODO otherwise tells GCS error :
		// incoming FP data without receiving new flight plan message
	}
}


#endif /* RFFLIGHTPLAN_H_ */
