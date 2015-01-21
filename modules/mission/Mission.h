/*
 * Mission.h

 *
 *  Created on: Jan 5, 2015
 *      Author: Adrien Hadj-Salah
 */
// #include <stdint.h>
#include "math/Math.h"
#include "modules/mission/MissionCommon.h"
#include "modules/nav/Navig2.h"

#ifndef MISSION_H_
#define MISSION_H_




// Initialization of the mission structure
//--------------------------------------------------
void missionInit();

// Insert a new mission element to the list
//--------------------------------------------------
void missionAdd(MissionElementType t, MissionElement *e);

// Run the mission to determine which attitude must be commanded
//--------------------------------------------------
void missionRun();


//--------------------------------------------------
//--------------------------------------------------

void missionInit() {
	_mission.currentIdx = 0;
	_mission.insertIdx = 0;
	_mission.elementTime = 0.0;
}


void missionAdd(MissionElementType type, MissionElement *element) {
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
	long time = micros();

	if (missionIsDone()) {

	}
	else {
		bool el_running = false;
		MissionElement *el = _mission[_mission.currentIdx] ;

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

#endif /* MISSION_H_ */
