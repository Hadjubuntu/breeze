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
	_mission.elements[index] = (*element);

	_mission.insertIdx ++;
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

void parseRFMissionInput(char *rf_str) {
	// Parse data
	//-------------------------------------------------------
	int typeAction = -1;
	long lat, lon;
	int altitudeMeters, durationSeconds;

	int iPos = 0;
	char * pch;
	pch = strtok (rf_str,"|");
	while (pch != NULL)	{
		switch (iPos) {
		case 0:
			// Do nothing with the header
			break;
		case 1:
			typeAction = atoi(pch);
			break;
		case 2:
			lat = atol(pch);
			break;
		case 3:
			lon = atol(pch);
			break;
		case 4:
			altitudeMeters = atoi(pch);
			break;
		case 4:
			durationSeconds = atoi(pch);
			break;
		default:
			// Do nothing
			break;
		}

		pch = strtok (NULL, "|");
		iPos ++;
	}

	// Make action
	//-------------------------------------------------------
	if (typeAction > 0) {
		MissionElement *missionEl1 = new MissionElement();

		switch (typeAction) {
		case 1:
			// Add a new WP
			//-----------------------------------------------

			// Create the element
			missionEl1->durationSeconds = durationSeconds;
			missionEl1->type = _missionWP;

			MissionWP missionElementWP;
			missionElementWP.wp.alt = altitudeMeters;
			missionElementWP.wp.lat = lat;
			missionElementWP.wp.lon = lon;

			missionEl1->missionWP = missionElementWP;

			// Add it to the list
			missionAdd(missionEl1);
			break;

		case 2:
			// Add a new circular mission element
			//------------------------------------------------
			missionEl1->durationSeconds = durationSeconds;
			missionEl1->type = _missionCircle;

			MissionCircle missionElementCircle;
			missionElementCircle.center.alt = altitudeMeters;
			missionElementCircle.center.lat = lat;
			missionElementCircle.center.lon = lon;

			missionElementCircle.radiusMeters = 10; // TODO to be defined by RF parameter

			break;
		}
	}
}
