/*
 * Mission.cpp
 *
 *  Created on: Jan 21, 2015
 *      Author: adrien
 */
#include "Mission.h"

void missionInit() {
	_mission.currentIdx = 0;
	_mission.insertIdx = 0;
	_mission.elementTime = 0.0;
}



void missionAdd(MissionElement *element) {
	uint8_t index = _mission.insertIdx;

	if (index < MISSION_ELEMENT_NB) {
		_mission.elements[index] = (*element);
		_mission.insertIdx ++;
	}
}

void missionErase() {
	missionInit();
	for (int i = 0; i < MISSION_ELEMENT_NB; i ++) {
		_mission.elements[i] = 0;
	}
}

bool missionIsDone() {
	return (_mission.currentIdx >= _mission.insertIdx) ;
}

bool missionWPNav(MissionElement *el) {
	bool elRunning = true;

	if (navCloseEnoughToWP(el->missionWP)) {
		elRunning = false;
	}
	else {
		navToWP(el->missionWP);
	}


	return elRunning;
}

bool missionCircleNav(MissionElement *el) {
	bool elRunning = true;
	double distToCenter = geoDistance(el->missionCircle.center, currentPosition);

	if (distToCenter >= el->missionCircle.radiusMeters * 2) {
		MissionWP wpCenter;
		wpCenter.wp = el->missionCircle.center;

		navToWP(wpCenter);
	}
	else {
		if (distToCenter <= el->missionCircle.radiusMeters) {

		}
	}

	return elRunning;
}

void missionRun() {
	long time = timeUs();

	if (missionIsDone()) {

	}
	else {
		bool el_running = false;
		MissionElement *el = &(_mission.elements[_mission.currentIdx]);

		switch (el->type) {
		case _missionWP:
			el_running = missionWPNav(el);
			break;
		case _missionCircle:
			el_running = missionCircleNav(el);
			break;
		default:
			// Unknow mission type, therefore, step to next one
			el_running = true;
			break;
		}

		if (lastMissionRunExecUs > 0) {
			_mission.elementTime += (time - lastMissionRunExecUs) ;
		}

		// test if element is finished or element time is elapsed
		if (el_running == false || (el->durationSeconds > 0 && _mission.elementTime >= el->durationSeconds * S_TO_US)) {
			// reset timer
			_mission.elementTime = 0.;
			// go to next element
			_mission.currentIdx = (_mission.currentIdx + 1) % MISSION_ELEMENT_NB;
		}
	}

	lastMissionRunExecUs = time;
}


/**********************************************************************
 * Parsing function
 **********************************************************************/

MissionElement parseRFMissionInput(char *rf_str) {
	// Parse data
	//-------------------------------------------------------

	EStringArray rfPayload = _embedded_str_explode(rf_str, '|');
	int typeAction = atoi(rfPayload.array[1]);


	// Make action
	//-------------------------------------------------------
	if (typeAction > 0) {
		bool missionPrepared = true;
		MissionElement missionEl1 ;

		switch (typeAction) {
		case 1:
			// Add a new WP
			//-----------------------------------------------

			// Create the element
			missionEl1.durationSeconds = atoi(rfPayload.array[2]);
			missionEl1.type = _missionWP;

			MissionWP missionElementWP;
			missionElementWP.wp.lat = atof(rfPayload.array[3]);
			missionElementWP.wp.lon = atof(rfPayload.array[4]);
			missionElementWP.wp.alt = atof(rfPayload.array[5]);

			missionEl1.missionWP = missionElementWP;

			break;

		case 2:
			// Add a new circular mission element
			//------------------------------------------------
			missionEl1.durationSeconds = atoi(rfPayload.array[2]);
			missionEl1.type = _missionCircle;

			MissionCircle missionElementCircle;
			missionElementCircle.center.lat = atof(rfPayload.array[3]);
			missionElementCircle.center.lon = atof(rfPayload.array[4]);
			missionElementCircle.center.alt = atof(rfPayload.array[5]);
			missionElementCircle.radiusMeters = atof(rfPayload.array[6]);

			missionEl1.missionWP = missionElementCircle;
			break;
		default:
			missionPrepared = false;
			break;
		}


		if (missionPrepared) {
			// Add it to the list
			return missionEl1;
		}
	}

	return 0;
}

void printMissionFlightPlan() {
	Logger.println("Mission flight plan printing");
	Logger.print("Current : ");
	Logger.print(_mission.currentIdx);
	Logger.print(" | over : ");
	Logger.print(_mission.insertIdx);
	Logger.println("----------------------------");

	for (int i = 0; i < _mission.insertIdx; i ++) {
		MissionElement cElement = _mission.elements[i];

		Logger.print("#");
		Logger.print(i);

		switch (cElement.type) {
		case _missionWP:
			Logger.print("WP");
			break;
		case _missionCircle:
			Logger.print("Circle");
			break;
		default:
			Logger.print("Unknow mission type");
			break;
		}

		Logger.print("\n");
	}
}
