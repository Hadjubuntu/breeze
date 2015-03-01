/*
 * MissionCommon.h
 *
 *  Created on: Jan 9, 2015
 *      Author: adrien
 */

#ifndef MISSIONCOMMON_H_
#define MISSIONCOMMON_H_

// Variables
//--------------------------------------------------
#define MISSION_ELEMENT_NB 20
long lastMissionRunExecUs = 0;

enum MissionElementType {
	_missionWP = 1,
	_missionCircle = 2,
	// _missionHeadingHold = 3,
};

// Structures
//--------------------------------------------------
typedef struct T_MISSION_WP {
	GeoPosition wp;
} MissionWP ;

typedef struct T_MISSION_CIRCLE {
	GeoPosition center;
	double radiusMeters;
} MissionCircle ;

// Generic mission element
typedef struct T_MISSION_ELEMENT {

	long durationSeconds; // = 0, disabled
	enum MissionElementType type;

	union {
		MissionWP missionWP;
		MissionCircle missionCircle;
	};
} MissionElement;

typedef struct T_MISSION {
	MissionElement elements[MISSION_ELEMENT_NB];
	uint8_t currentIdx; // Current index on the mission
	uint8_t insertIdx; // Insertion index
	float elementTime; // Time spend on the current element
} Mission;

Mission _mission;


#endif /* MISSIONCOMMON_H_ */
