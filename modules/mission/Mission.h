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
#include "arch/AVR/MCU/MCU.h"

#ifndef MISSION_H_
#define MISSION_H_




// Initialization of the mission structure
//--------------------------------------------------
void missionInit();

// Insert a new mission element to the list
//--------------------------------------------------
void missionAdd(MissionElement *e);


// Erase the mission'elements list
//--------------------------------------------------
void missionErase();

// Run the mission to determine which attitude must be commanded
//--------------------------------------------------
void missionRun();

// Parse rf input into mission action
//--------------------------------------------------
MissionElement parseRFMissionInput(char *);

// Print mission flight plan on Serial
//--------------------------------------------------
void printMissionFlightPlan();


//--------------------------------------------------
//--------------------------------------------------


#endif /* MISSION_H_ */
