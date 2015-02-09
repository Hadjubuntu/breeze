/*
 * RFLink2.h
 *
 *  Created on: Jan 28, 2015
 *      Author: Adrien Hadj-Salah
 */

#ifndef RFLINK2_H_
#define RFLINK2_H_


#include <stdio.h>
#include "arch/AVR/MCU/MCU.h"
#include "Common.h"
#include "arch/common/StrUtils.h"
#include "arch/common/StrStack.h"


// Configuration RF
//------------------------------------------------------------
#define RFSerial Serial1

#define BPS_RF_LINK 57600 // default: 9600
#define RF_STACK_SIZE 20
#define MAX_PACKET_SIZE 125



// RF variables
//------------------------------------------------------------
long lastTimeLinkWithGS = 0;

// Packet data
//------------------------------------------------------------
bool packetValid = true;
int inputStringIdx = 0;
char inputString[MAX_PACKET_SIZE];
StrStack rfstack;


// Config pointers
//------------------------------------------------------------
bool *rf_uavAutomode;
bool rf_automodeSwitchToken; // True means automode switched
int *rf_deciThrustPercent;
int *rf_aileronCmd, *rf_gouvernCmd, *rf_rubberCmd, *rf_flapsCmd;
Attitude *rf_attitudeCommanded;
int *rf_navigationMethodAngleDiff;
double *rf_parameters;
int *rf_autospeed_controller;
double *rf_v_ms_goal;

// In manual flight mode, we can flight with defined roll pitch yaw or servo command
// Therefore the stabilize function is called or not
int rf_manual_StabilizedFlight = 0;



// RF Configuration
//------------------------------------------------------------
int currentParameterConfiguration = 0;
bool sendingConfiguration = false;


/****************************************************************************
 * Send configuration to the ground station
 ****************************************************************************/

void sendConfiguration() {
	sendingConfiguration = true;
}

// Function call at 10 Hz
void updateLowPriorityRFLink() {
	// We send the configuration each parameter is send each time the function is called
	if (sendingConfiguration) {
		char buf[60];

		sprintf(buf, "config|%d|%lu|\n", currentParameterConfiguration, (long)(param[currentParameterConfiguration]*10000));
		RFSerial.write(buf);

		currentParameterConfiguration ++;

		if (currentParameterConfiguration >= NB_PARAMETERS) {
			currentParameterConfiguration = 0;
			sendingConfiguration = false;
		}
	}
}


/****************************************************************************
 * Initialize RF link
 ****************************************************************************/

void setupRFLink(bool *pUavAutomode, int *pDeciThrustPercent,
		int *pAileronCmd, int *pGouvernCmd, int *pRubberCmd, int *pFlapsCmd,
		Attitude *pAttitudeCommanded,
		int *pNavigMethod,
		double *pArrayParameters, int *pAutospeedController, double *pV_ms_goal) {

	// Start serial communication @BAUD_RATE per second with ground station
	RFSerial.begin(BPS_RF_LINK);

	// Set pointer to variables
	rf_uavAutomode = pUavAutomode;
	rf_automodeSwitchToken = false;
	rf_deciThrustPercent = pDeciThrustPercent;
	rf_aileronCmd = pAileronCmd;
	rf_gouvernCmd = pGouvernCmd;
	rf_rubberCmd = pRubberCmd;
	rf_flapsCmd = pFlapsCmd;
	rf_attitudeCommanded = pAttitudeCommanded;

	rf_manual_StabilizedFlight = 0;

	rf_navigationMethodAngleDiff = pNavigMethod;

	rf_parameters = pArrayParameters;

	rf_autospeed_controller = pAutospeedController;
	rf_v_ms_goal = pV_ms_goal;


	Serial.println("Starting com RF");
}



void makeActionInStack(int i) {
	char *rfcmd = rfstack.get(i);
	if (rfcmd == NULL) {
		return ;
	}

	EStringArray rfcmd_tokens = _embedded_str_explode(rfcmd, '|');


	if (str_startsWith(rfcmd, "auto")) {
		if (str_equals(rfcmd_tokens.array[1], "on")) {
			(*rf_uavAutomode) = true;
		}
		else {
			(*rf_uavAutomode) = false;
		}
	}
	else if (str_startsWith(rfcmd, "speedauto")) {
		(*rf_autospeed_controller) = atoi(rfcmd_tokens.array[1]);
	}
	else if (str_startsWith(rfcmd, "thro")) {
		(*rf_deciThrustPercent) = atoi(rfcmd_tokens.array[1]);
	}
	else if (str_startsWith(rfcmd, "flaps")) {
		(*rf_deciThrustPercent) = atoi(rfcmd_tokens.array[1]);
	}

	if (USE_RADIO_FUTABA == 0) {
		if (str_startsWith(rfcmd, "ac")) {
			// Force UAV to stabilize to the desired attitude
			rf_manual_StabilizedFlight = 1;


			rf_attitudeCommanded->roll = atoi(rfcmd_tokens.array[1])/100.0;
			rf_attitudeCommanded->pitch = atoi(rfcmd_tokens.array[2])/100.0;
			rf_attitudeCommanded->yaw = atoi(rfcmd_tokens.array[3])/100.0;

			// Using autospeed controller, we control speed
			if (*rf_autospeed_controller == 1) {
				(*rf_v_ms_goal) = 1.7f*SCALING_SPEED* atoi(rfcmd_tokens.array[4])/1000.0;
			}
			// Without autospeed, user controls throttle
			else {
				(*rf_deciThrustPercent) =  atoi(rfcmd_tokens.array[4]);
			}

		}
		else if (str_startsWith(rfcmd, "sc")) {

			// Force UAV to stabilize to the desired attitude
			rf_manual_StabilizedFlight = 0;
			// Force UAV to not use autospeed controller
			rf_autospeed_controller = 0;

			rf_attitudeCommanded->roll = atoi(rfcmd_tokens.array[1])/100.0;
			rf_attitudeCommanded->pitch = atoi(rfcmd_tokens.array[2])/100.0;
			rf_attitudeCommanded->yaw = atoi(rfcmd_tokens.array[3])/100.0;
			(*rf_deciThrustPercent) =  atoi(rfcmd_tokens.array[4]);
		}
	}

	// Stop the airplane, no thrust and no command on servo.
	if (str_startsWith(rfcmd, "shutdown")) {
		(*rf_aileronCmd) = 0;
		(*rf_gouvernCmd) = 0;
		(*rf_rubberCmd) = 0;
		(*rf_deciThrustPercent) = 0;
		(*rf_flapsCmd) = 0;
	}
	else if (str_startsWith(rfcmd, "nav_method")) {
		(*rf_navigationMethodAngleDiff) =  atoi(rfcmd_tokens.array[1]);
	}
	else if (str_startsWith(rfcmd, "conf")) {
		// Define parameter's id and value
		int ID_parameter = -1;
		double value_parameter = 0.0;
		bool hasValue = false;


		ID_parameter = atoi(rfcmd_tokens.array[1]);

		if (rfcmd_tokens.sizeArray > 1) {
			value_parameter = atof(rfcmd_tokens.array[2]);
			hasValue = true;
		}

		if (hasValue && ID_parameter >= 0) {
			rf_parameters[ID_parameter] = value_parameter;
		}
	}
	else if (str_startsWith(rfcmd, "request_conf")) {
		sendConfiguration();
	}
	// Process others commands ..

}

// Update RF link each 20 ms (50 Hz)
// Could be improve : throw old attitude command and make action with the last one
void updateRFLink50Hz() {
	rfstack.make(&makeActionInStack) ;
}


/****************************************************************************
 * Send to ground station, informations about UAV
 * such as attitude, speed and location
 * Takes around 1 ms
 *****************************************************************************/
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

	RFSerial.write(buf);
}



/****************************************************************************
 * Serial event manager
 ****************************************************************************/

void updateCriticalRFLink() {

	if (RFSerial.available()) {

		// Retrieve new byte
		//----------------------------------------------------------
		char inChar = (char)RFSerial.read();

		// Concatenate new char received into current input string
		//----------------------------------------------------------
		char inCharToStr[1];
		inCharToStr[0] = inChar;

		if (inputStringIdx < MAX_PACKET_SIZE) {
			strncat(inputString, inCharToStr, 1) ;
		}
		else {
			packetValid = false;
			str_resetCharArray(inputString);
		}


		inputStringIdx ++;


		// if the incoming character is a newline, set a flag
		// so the main loop can do something about it:
		//----------------------------------------------------------
		if (inChar == '\n') {
			if (packetValid) {
				rfstack.insert(inputString);
			}

			str_resetCharArray(inputString);
			inputStringIdx = 0;
			packetValid = true;

			lastTimeLinkWithGS = timeUs();
		}

	}
}






#endif /* RFLINK2_H_ */
