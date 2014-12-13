/*
 * FlightControl.h
 *
 *  Created on: 10 sept. 2014
 *      Author: hadjmoody
 */

#ifndef FLIGHTCONTROL_H_
#define FLIGHTCONTROL_H_

#include "Common.h"
//#include "PID.h"

// Method PID parameters (stabilize2 function)
#define G_P_PITCH 1.2f // Coeff prop
#define G_D_PITCH 0.04f // Coeff derivative

#define MAX_DURATION_BURST_S 3
#define DELAY_BETWEEN_BURST_S 6
#define K_THRUST   3 // Converts error of airpseed to thrust percent (10Hz)
#define G_P_THRUST 0.5f // Gain prop for thrust
#define G_D_THRUST 0.15f // High gain derivative to have dampening effect
long thrustBurstTimeStartUs = 0;
long thrustBurstTimeEndUs = 0;
bool thrustBurstMode = false;

#define G_P_YAW_HOLDING 0.5f

// Max thrust change in percent each 100ms
#define AUTOTHROTTLE_THRUST_SLEW_RATE 8

FilterAverage *v_ms_mean, *airspeed_ms_mean;
double previousVmsError = 0.0;


//-------------------------------------------------------
// Initialize flight controller
void initFlightControl() {
	if (USE_GPS_NAVIGUATION) {
		v_ms_mean = new  FilterAverage(4, 0, MAX_V_MS_PLANE, true);
	}
	if (USE_AIRSPEED_SENSOR) {
		airspeed_ms_mean = new FilterAverage(4, 0, MAX_V_MS_PLANE, true);
	}
}


//-------------------------------------------------------
// Evaluate yaw to hold a heading cap (takeoff, landing)
double yawToHoldHeading(double currentHeadingDeg, double headingCapDeg) {
	double yaw = 0.0;

	// If positive error, means turns left, then, deflection rubber left (negative)
	yaw = - G_P_YAW_HOLDING * (headingCapDeg - currentHeadingDeg);
	yaw = constrain(yaw, -20.0, 20.0);

	return yaw;
}

//-------------------------------------------------------
// Adapt deflection depending on plane's speed
float getSpeedScaler(int deciThrustPercent)
{
	float aspeed, speed_scaler;
	if (USE_AIRSPEED_SENSOR) {
		double v_ms = airspeed_ms_mean->getAverage();

		if (v_ms > 0) {
			speed_scaler = SCALING_SPEED / v_ms;
		} else {
			speed_scaler = 2.0;
		}
		speed_scaler = constrain(speed_scaler, 0.5, 2.0);
	}
	else {
		if (deciThrustPercent > 0) {
			speed_scaler = 0.5f + ((float)THROTTLE_CRUISE / (deciThrustPercent/10.0) / 2.0f);                 // First order taylor expansion of square root
		}else{
			speed_scaler = 1.67f;
		}
		// This case is constrained tighter as we don't have real speed info
		speed_scaler = constrain(speed_scaler, 0.6, 1.67);
	}

	return speed_scaler;
}


//-------------------------------------------------------
// Stabilize airplane with PID controller
void stabilize2(double errorRoll, double errorPitch, double yawDesired,
		int *aileronCmd, int *gouvernCmd, int *rubberCmd,
		double gyroXrate, double gyroYrate, int deciThrustPercent) {


	double v_ms ;
	if (USE_AIRSPEED_SENSOR) {
		v_ms = airspeed_ms_mean->getAverage();
	}
	else {
		v_ms = SCALING_SPEED * (deciThrustPercent / 1000); // 1000 : 100 percent and deci 10
	}
	if (v_ms < 0.5) {
		v_ms = 0.5;
	}


	// Get the scaler to minimize surface command in high speed ..
	double scaler = getSpeedScaler(deciThrustPercent);

	double desiredRollRate = errorRoll / param[ID_G_TAU];
	double desiredPitchRate = errorPitch / param[ID_G_TAU];

	if (desiredRollRate > DROLL_MAX) {
		desiredRollRate = DROLL_MAX;
	}
	else if (desiredRollRate < -DROLL_MAX) {
		desiredRollRate = -DROLL_MAX;
	}
	if (desiredPitchRate > DPITCH_MAX) {
		desiredPitchRate = DPITCH_MAX;
	}
	else if (desiredPitchRate < -DPITCH_MAX) {
		desiredPitchRate = -DPITCH_MAX;
	}

	double rateRollError = (desiredRollRate - gyroXrate) * scaler;
	double ratePitchError = (desiredPitchRate - gyroYrate) * scaler;

	double kp_ff_roll = max(param[ID_G_P_ROLL] * param[ID_G_TAU]  - param[ID_G_D_ROLL], 0) / v_ms;
	double kp_ff_pitch = max(G_P_PITCH * param[ID_G_TAU]  - G_D_PITCH, 0) / v_ms;
	if (kp_ff_roll < 0) {
		kp_ff_roll = 0.0;
	}
	if (kp_ff_pitch < 0) {
		kp_ff_pitch = 0.0;
	}

	double outputRollCmd = ((rateRollError * param[ID_G_D_ROLL]) + (desiredRollRate * kp_ff_roll)) * scaler;
	double outputPitchCmd = ((ratePitchError * G_D_PITCH) + (desiredPitchRate * kp_ff_pitch)) * scaler;

	double yawCmd = yawDesired;

	// Update surfaces command
	(*aileronCmd) = (int) constrain(outputRollCmd * 100.0, -9000, 9000);
	(*gouvernCmd) = (int) constrain(outputPitchCmd * 100.0, -9000, 9000);
	(*rubberCmd) = (int) constrain(yawCmd * 100.0, -9000, 9000);
}


//---------------------------------------------------------------------------------
//---------------------------------------------------------------------------------
/**
 * Adapt attitude commanded depending on the current attitude :
 * pitch more if roll, yaw compensation in roll mvt
 */
void controlAttitudeCommand(Attitude *currentAttitude, Attitude *previousCmd,
		double dt, long ctime, double desiredRoll, double desiredPitch, double desiredYaw)
{
	int sign = 1;

	// If error with dt, then set roll pitch and yaw as desired
	if (dt <= 0) {
		previousCmd->roll = desiredRoll;
		previousCmd->pitch = desiredPitch;
		previousCmd->yaw = 0;
	}
	else {

		// Evaluate pitch offset to compensate the roll action
		// Compensate only if desired roll superior to a minimum (5.0 degrees)
		double pitch_offset = 0.0;
		if (PITCH_ROLL_COMPENSATION == 1 && fabs(desiredRoll) > 5.0) {
			double bankAngle = fabs(currentAttitude->roll);
			bankAngle = constrain(bankAngle, 0, 90);

			// Compensate only with acceptable roll angle from 0 to 70 deg
			if (bankAngle < 70) {
				pitch_offset = PITCH2SRV_ROLL * bankAngle * (PITCH_MAX_COMPENSATION / 45.0);
				pitch_offset = constrain(pitch_offset, 0, PITCH_MAX_COMPENSATION);
			}
		}


		previousCmd->roll = desiredRoll;
		previousCmd->pitch = desiredPitch + pitch_offset;
	}

	// To compensate roll, turn rubber a bit
	double yaw_offset = 0.0;
	if (YAW_ROLL_COMPENSATION == 1) {
		yaw_offset = - YAW2SRV_ROLL * currentAttitude->roll * (YAW_MAX_COMPENSATION / 45.0) ;
		yaw_offset = constrain(yaw_offset, -YAW_MAX_COMPENSATION, YAW_MAX_COMPENSATION);
	}

	previousCmd->yaw = desiredYaw + yaw_offset;
	previousCmd->time = ctime / MS_TO_US;
}


//---------------------------------------------------------------------------------
//-------------------------------------------------------
// Calls this function at 10Hz (100 ms)
int controlSpeedWithThrust(long cTimeUs, int currentDeciThrust, double v_ms_goal, Attitude *currentAttitude) {
	// Thrust in percent to adopt to control speed
	int deciThrustOutput = currentDeciThrust;
	int deciThrustMin = 0;
	int deciThrustMax = 900;
	int deciThrustBurst = 1000;

	// Map v_ms_goal with thrust max
	if (v_ms_goal < 2) {
		deciThrustMax = 40;
		deciThrustBurst = 45;
	}
	else if (v_ms_goal < 5) {
		deciThrustMax = 500;
		deciThrustBurst = 550;
	}
	else if (v_ms_goal < 10) {
		deciThrustMax = 700;
		deciThrustBurst = 800;
	}

	// Only adapt thrust when in cruise flight-state
	int tolerancePercent = 0.02; // Accept v_ms with +/- 2% tolerance

	double airspeed_ms_average = airspeed_ms_mean->getAverage() ;
	double error = v_ms_goal - airspeed_ms_average;

	if (airspeed_ms_average >= v_ms_goal - tolerancePercent*v_ms_goal
			&& airspeed_ms_average <= v_ms_goal + tolerancePercent*v_ms_goal) {

		// Don't change the thurst
		deciThrustOutput = currentDeciThrust;
		previousVmsError = error;
	}
	else {

		double deltaThrust = param[ID_K_THRUST] * (param[ID_G_P_THRUST] * error + param[ID_G_D_THRUST]*(error-previousVmsError));
		deltaThrust = constrain(deltaThrust, -AUTOTHROTTLE_THRUST_SLEW_RATE, AUTOTHROTTLE_THRUST_SLEW_RATE);
		int deltaDeciThrustInteger = (int) (deltaThrust*10);

		// Force to change of thrust if needed
		if (deltaDeciThrustInteger == 0) {
			if (deltaThrust > 0.1) {
				deltaDeciThrustInteger = 1;
			}
			else if (deltaThrust < -0.1) {
				deltaDeciThrustInteger = -1;
			}
		}

		deciThrustOutput = deciThrustOutput + deltaDeciThrustInteger;

		// Protection for two low thrust and too high
		// using a max thrust value and a burst value acceptable during x seconds


		if (thrustBurstMode == false) {
			if ((cTimeUs - thrustBurstTimeEndUs) > (DELAY_BETWEEN_BURST_S * S_TO_US)) {
				deciThrustMax = deciThrustBurst;
			}

			if (deciThrustOutput > deciThrustMax) {
				thrustBurstMode = true;
				thrustBurstTimeStartUs = cTimeUs;
			}
		}

		// If burst timeout, then return to deciThrustMax
		if (thrustBurstMode == true) {
			if ((cTimeUs - thrustBurstTimeStartUs) > (MAX_DURATION_BURST_S * S_TO_US)) {
				thrustBurstTimeEndUs = cTimeUs;
				thrustBurstMode = false;
			}
			else {
				deciThrustMax = deciThrustBurst;
			}
		}

		deciThrustOutput = constrain(deciThrustOutput, deciThrustMin, deciThrustMax);

		// Store previous error
		previousVmsError = error;
	}


	return deciThrustOutput;
}


#endif /* FLIGHTCONTROL_H_ */
