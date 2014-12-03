/*
 * RFLink.h
 *
 *  Created on: 11 oct. 2014
 *      Author: hadjmoody

 Serial1 mega : RX Pin 19 (green), Tx Pin 18 (black). Note that Rx RF link on Tx Arduino Pin
 TODO to improve this part of the code, String is not good for memory and speed
 therefore uses only char var[size], and strok and strcmp.
 */

#include "Common.h"

#ifndef RFLINK_H_
#define RFLINK_H_

#define BPS_RF_LINK 57600 // default: 9600
#define RF_STACK_SIZE 20


bool *rf_uavAutomode;
int *rf_deciThrustPercent;
int *rf_aileronCmd, *rf_gouvernCmd, *rf_rubberCmd, *rf_flapsCmd;
Attitude *rf_attitudeCommanded;

// In manual flight mode, we can flight with defined roll pitch yaw or servo command
// Therefore the stabilize function is called or not
int rf_manual_StabilizedFlight = 0;

long lastTimeLinkWithGS = 0;
String inputString = "";

String RFStack[RF_STACK_SIZE];
int stackComputedPosition = 0;
int stackReceivedPosition = 0;
int stackLoopDiff = 0;

void incrStackReceivedPosition() {
	if (stackReceivedPosition+1 >= RF_STACK_SIZE) {
		stackReceivedPosition = 1;
		stackLoopDiff ++;
	}
	else {
		stackReceivedPosition ++;
	}
}

void incrStackComputedPosition() {
	if (stackComputedPosition+1 >= RF_STACK_SIZE) {
		stackComputedPosition = 0;
	}
	else {
		stackComputedPosition ++;
	}
}


void setupRFLink(bool *pUavAutomode, int *pDeciThrustPercent,
		int *pAileronCmd, int *pGouvernCmd, int *pRubberCmd, int *pFlapsCmd,
		Attitude *pAttitudeCommanded) {

	// Start serial communication @BAUD_RATE per second with ground station
	Serial1.begin(BPS_RF_LINK);

	// Set pointer to variables
	rf_uavAutomode = pUavAutomode;
	rf_deciThrustPercent = pDeciThrustPercent;
	rf_aileronCmd = pAileronCmd;
	rf_gouvernCmd = pGouvernCmd;
	rf_rubberCmd = pRubberCmd;
	rf_flapsCmd = pFlapsCmd;
	rf_attitudeCommanded = pAttitudeCommanded;

	rf_manual_StabilizedFlight = 0;

	Serial.println("Starting com RF");
	for (int i = 0; i < RF_STACK_SIZE; i ++) {
		RFStack[i].reserve(24); // 24*50 = 1200 = 9600/8 bps to Byte/s
	}
}



void makeActionInStack(int i) {

	if (RFStack[i].startsWith("auto")) {
		String mode = RFStack[i].substring(5, 7);
		if (mode.equals("on")) {
			(*rf_uavAutomode) = true;
		}
		else {
			(*rf_uavAutomode) = false;
		}
	}
	else if (RFStack[i].startsWith("thro")) {
		(*rf_deciThrustPercent) = RFStack[i].substring(5).toInt();
	}
	else if (RFStack[i].startsWith("flaps")) {
		(*rf_flapsCmd) = RFStack[i].substring(6).toInt();
	}
	else if (RFStack[i].startsWith("att_cmd")) {
		// Force UAV to stabilize to the desired attitude
		rf_manual_StabilizedFlight = 1;

		// Split string and fill aileron, gouvern, rubber, flaps cmd with new values
		// Converts arduino String to char[]
		char rf_str[50];
		RFStack[i].toCharArray(rf_str, 50);

		// Then split attitude command 3-axis into parts to have values
		int iPos = 0;
		char * pch;
		pch = strtok (rf_str,"|");
		while (pch != NULL)	{
			switch (iPos) {
			case 0:
				// Do nothing with the header
				break;
			case 1:
				rf_attitudeCommanded->roll = atoi(pch)/100.0;
				break;
			case 2:
				rf_attitudeCommanded->pitch = atoi(pch)/100.0;
				break;
			case 3:
				rf_attitudeCommanded->yaw = atoi(pch)/100.0;
				break;
			default:
				printf("Attitude command with too many parameters.");
				break;
			}

			pch = strtok (NULL, "|");
			iPos ++;
		}
	}
	else if (RFStack[i].startsWith("srv_cmd")) {

		// Force UAV to stabilize to the desired attitude
		rf_manual_StabilizedFlight = 0;

		// Split string and fill aileron, gouvern, rubber, flaps cmd with new values
		// Converts arduino String to char[]
		char rf_str[50];
		RFStack[i].toCharArray(rf_str, 50);

		// Then split attitude command 3-axis into parts to have values
		int iPos = 0;
		char * pch;
		pch = strtok (rf_str,"|");
		while (pch != NULL)	{
			switch (iPos) {
			case 0:
				// Do nothing with the header
				break;
			case 1:
				(*rf_aileronCmd) = atoi(pch);
				break;
			case 2:
				(*rf_gouvernCmd) = atoi(pch);
				break;
			case 3:
				(*rf_rubberCmd) = atoi(pch);
				break;
			default:
				printf("Attitude command with too many parameters.");
				break;
			}

			pch = strtok (NULL, "|");
			iPos ++;
		}
	}
	// Stop the airplane, no thrust and no command on servo.
	else if (RFStack[i].startsWith("shutdown")) {
		(*rf_aileronCmd) = 0;
		(*rf_gouvernCmd) = 0;
		(*rf_rubberCmd) = 0;
		(*rf_deciThrustPercent) = 0;
		(*rf_flapsCmd) = 0;

	}
	// Process others commands ..
}

// Update RF link each 20 ms (50 Hz)
void updateRFLink50Hz() {
	if (stackComputedPosition < stackReceivedPosition) {
		for (int i = stackComputedPosition; i < stackReceivedPosition; i ++) {
			makeActionInStack(i);
		}

		stackComputedPosition = stackReceivedPosition;
	}
	// Loop in stack
	else if (stackReceivedPosition < stackComputedPosition && stackLoopDiff > 0) {
		for (int i = stackComputedPosition; i < RF_STACK_SIZE; i ++) {
			makeActionInStack(i);
		}

		for (int i = 0; i < stackReceivedPosition; i ++) {
			makeActionInStack(i);
		}

		stackComputedPosition = stackReceivedPosition;

		stackLoopDiff --;
	}
}

/**
 * Send to ground station, informations about UAV
 * such as attitude, speed and location
 * Takes around 1 ms
 */
void updateRFLink1hz(int rollCenti, int pitchCenti, int cap, int altCm, int airspeedCentiMs, long latPow6, long lonPow6, int angleDiffToTarget, bool autopilot, int currentWP) {
	char buf[250];
	int autopilotInt = 0;
	if (autopilot == true) {
		autopilotInt = 1;
	}

	sprintf(buf, "att|%d|%d|%d|%d|%d|%lu|%lu|%d|%d|%d|\n",
			rollCenti, pitchCenti, cap,
			altCm, airspeedCentiMs,
			latPow6, lonPow6,
			angleDiffToTarget, autopilotInt, currentWP);

	Serial1.write(buf);
}

/**
 * Send message to the RF ground-station
 */
void sendRFMessage(char *msg) {
	char buf[250];
	sprintf(buf, "msg|%s|\n", msg);
	Serial1.write(buf);
}


void updateCriticalRFLink() {

	if (Serial1.available()) {
		// get the new byte:
		char inChar = (char)Serial1.read();
		// add it to the inputString:
		inputString += inChar;
		// if the incoming character is a newline, set a flag
		// so the main loop can do something about it:
		if (inChar == '\n') {
			RFStack[stackReceivedPosition] = inputString;
			inputString = "";

			lastTimeLinkWithGS = micros();

			incrStackReceivedPosition();
		}
	}
}



#endif /* RFLINK_H_ */
