/*
 * Scenario_FlyByGPS.h
 *
 *  Created on: Dec 19, 2014
 *      Author: adrien
 */

#ifndef SCENARIO_FLYBYGPS_H_
#define SCENARIO_FLYBYGPS_H_


#include "Common.h"
#include "modules/nav/Navig.h"
#include "modules/flight_control/FlightControl.h"

void flightByGPS() {
	// If UAV high than 5 meters, then go to flying mode directly
	if (altitudeBarometer->getAverage() > 500) {
		UAVCore->flightState = CRUISE;
	}

	if (UAVCore->flightState == TAKEOFF) {
		if (time_TakeOffStart == 0) {
			time_TakeOffStart = currentTime;
		}

		// Full thrust
		UAVCore->deciThrustPercent = 700 ;

		// Give 30% thrust the 2 first seconds, then full thrust
		if ((currentTime-time_TakeOffStart) < 2 * S_TO_US) {
			UAVCore->deciThrustPercent = 300;
		}

		double pitchDesired = 30;

		// Don't try to takeoff before v_min or timeout of 7 seconds
		// But give a pitch offset, to prevent to put the nose down
		if ((USE_AIRSPEED_SENSOR && airspeed_ms_mean->getAverage() < V_MIN_TAKEOFF_MS)
				|| (currentTime-time_TakeOffStart) < 5 * S_TO_US) {
			pitchDesired = 5 ;
		}

		controlAttitudeCommand(UAVCore->currentAttitude, UAVCore->attitudeCommanded,
				G_Dt, currentTime,
				0, pitchDesired, 0);

		// Flaps down
		flapsCmd = 60;


		// After z higher than 3 meters, takeoff complete
		if (((currentTime-time_TakeOffStart) > MAX_DELAY_TAKEOFF_US) || (altitudeBarometer->areDataRelevant(currentTime)
				&& altitudeBarometer->getAverage() > 300.0)) {

			time_TakeOffDone = currentTime ;
			UAVCore->flightState = CLIMB ;
		}
	}
	else if (UAVCore->flightState == CLIMB) {
		if (time_ClimbStart == 0) {
			time_ClimbStart = currentTime;
		}

		flapsCmd = 30;
		UAVCore->deciThrustPercent = 700;
		controlAttitudeCommand(UAVCore->currentAttitude, UAVCore->attitudeCommanded,
				G_Dt, currentTime,
				0, 30, 0);

		if ((currentTime-time_ClimbStart) > CLIMB_DURATION_SECONDS * S_TO_US) {
			time_ClimbDone = currentTime;
			UAVCore->flightState = CRUISE ;
		}
	}
	else if (UAVCore->flightState == CRUISE) {

		if (AUTOSPEED_CONTROLLER == 0) {
			// cruise speed depends on current pitch (default thrust 50%)
			UAVCore->deciThrustPercent = 700 ;

			if (UAVCore->currentAttitude->pitch < 0) {
				UAVCore->deciThrustPercent = 500 ;
			}
			else if (UAVCore->currentAttitude->pitch < -10) {
				UAVCore->deciThrustPercent = 400 ;
			}
			else if (UAVCore->currentAttitude->pitch > 15) {
				UAVCore->deciThrustPercent = 900;
			}
		}
		else {
			// In auto speed control, we just define a speed to achieve, thrust is controlled by a PD controller
			// We choose 38 km/h to have enough lift to compensate the weight without trying to pitch-up
			UAVCore->v_ms_goal = 13; // ~40 km/h
		}

		// Update GPS heading
		updateHeading(altitudeBarometer);

		// Create related attitude command
		// If no signal GPS, then fly in circle low thrust, flaps 30% down
		if (GPSState == GPS_STATE_LOST) {
			// Desactivate autospeed controller and set low thrust
			AUTOSPEED_CONTROLLER = 0;
			UAVCore->deciThrustPercent = 400 ;

			// Circle zero pitch and flaps down
			controlAttitudeCommand(UAVCore->currentAttitude, UAVCore->attitudeCommanded, G_Dt, currentTime,  0, 0, 20);
			flapsCmd = 30;
		}
		else {
			controlAttitudeCommand(UAVCore->currentAttitude, UAVCore->attitudeCommanded, G_Dt, currentTime,  gpsRollDesired, gpsPitchDesired, 0);
		}


		// Retrieve flaps
		flapsCmd = 0;

		// If mission done or in go home mode and distance acceptable to land or after 6 minutes, start landing
		// Change state and stop autospeed controller
		if (MissionDone
				|| (modeGoHome && altitudeBarometer->areDataRelevant(currentTime) && altitudeBarometer->getAverage() < 250.0)
				|| (currentTime - time_ClimbDone) > 6*60*S_TO_US) {

			UAVCore->flightState = LANDING ;
		}
	}
	else if (UAVCore->flightState == LANDING) {

		double yawDesired = 0.0;

		if (HOLD_HEADING_IN_TAKEOFF_AND_LANDING) {
			yawDesired = yawToHoldHeading(currentHeading, landingHoldHeading);
		}

		// If airplane 2m50 over the ground, descent 12 degrees down
		if (altitudeBarometer->getAverage() > 250) {
			if (AUTOSPEED_CONTROLLER == 0) {
				UAVCore->deciThrustPercent = 400 ;
			}
			else {
				UAVCore->v_ms_goal = 10;
			}

			flapsCmd = 50;
			controlAttitudeCommand(UAVCore->currentAttitude, UAVCore->attitudeCommanded, G_Dt, currentTime,  0, -12, yawDesired);
		}
		else {
			flapsCmd = 70;
			// Descent from (2m50 to 50 centimeters)
			if (altitudeBarometer->getAverage() > 50) {

				if (AUTOSPEED_CONTROLLER == 0) {
					UAVCore->deciThrustPercent = 350 ;
				}
				else {
					UAVCore->v_ms_goal = 8;
				}

				controlAttitudeCommand(UAVCore->currentAttitude, UAVCore->attitudeCommanded, G_Dt, currentTime,  0, 0, yawDesired);
			}
			// Arrondi
			else {
				// Flaps for flare
				AUTOSPEED_CONTROLLER = 0;
				flapsCmd = 90;
				UAVCore->deciThrustPercent = 0 ;
				controlAttitudeCommand(UAVCore->currentAttitude, UAVCore->attitudeCommanded, G_Dt, currentTime,  0, 5, 0);
			}
		}
	}

}


#endif /* SCENARIO_FLYBYGPS_H_ */
