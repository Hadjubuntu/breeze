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


#define MAX_DURATION_BURST_S 3
#define DELAY_BETWEEN_BURST_S 6
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
	float speed_scaler;
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

double sumErrorRoll = 0.0, sumErrorPitch = 0.0;
double MAX_I = 10;

#define ATTITUDE_CONTROL_DEG 57.29578f
//-------------------------------------------------------
// Stabilize airplane with PID controller
// Takes around 1 ms on Atmega2560
void stabilize2(double G_Dt, double errorRoll, double errorPitch, double yawDesired,
		int *aileronCmd, int *gouvernCmd, int *rubberCmd,
		double gyroXrate, double gyroYrate, int deciThrustPercent) {


#if Firmware == QUADCOPTER

	double desiredRollRate = errorRoll * 4.5;
	double desiredPitchRate = errorPitch * 4.5;

	// TODO reactivate gyro rate with good values of rate
	double rateRollError = (desiredRollRate - gyroXrate * ATTITUDE_CONTROL_DEG);
	double ratePitchError = (desiredPitchRate + gyroYrate * ATTITUDE_CONTROL_DEG);

	if (deciThrustPercent > 300) {
		sumErrorRoll += errorRoll*0.1;
		sumErrorPitch += errorPitch*0.1;

		BoundAbs(sumErrorRoll, MAX_I);
		BoundAbs(sumErrorPitch, MAX_I);
	}
	else if (deciThrustPercent < 150) {
		sumErrorRoll = 0.0;
		sumErrorPitch = 0.0;
	}

	// TO BE CONTINUED for quadcopter firmware
	// TODO ArduCopter used : rollrate with P coeff and derivative of of roll rate error with D coeff
//	double outputRollCmd = errorRoll * param[ID_G_P_ROLL] + rateRollError  * param[ID_G_D_ROLL];
//	double outputPitchCmd = errorPitch * param[ID_G_P_PITCH] + ratePitchError * param[ID_G_D_PITCH];
	double outputRollCmd = rateRollError  * param[ID_G_P_ROLL] + sumErrorRoll * param[ID_G_I_ROLL];
	double outputPitchCmd = ratePitchError * param[ID_G_P_PITCH] + sumErrorPitch * param[ID_G_I_PITCH];

	double yawCmd = yawDesired;

	(*aileronCmd) = (int) constrain(outputRollCmd * 100.0, -9000, 9000);
	(*gouvernCmd) = (int) constrain(outputPitchCmd * 100.0, -9000, 9000);
	(*rubberCmd) = (int) constrain(yawCmd * 100.0, -9000, 9000);

#elif Firmware == FIXED_WING

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
	double kp_ff_pitch = max(param[ID_G_P_PITCH] * param[ID_G_TAU]  - param[ID_G_D_PITCH], 0) / v_ms;
	if (kp_ff_roll < 0) {
		kp_ff_roll = 0.0;
	}
	if (kp_ff_pitch < 0) {
		kp_ff_pitch = 0.0;
	}

	double outputRollCmd = ((rateRollError * param[ID_G_D_ROLL]) + (desiredRollRate * kp_ff_roll)) * scaler;
	double outputPitchCmd = ((ratePitchError * param[ID_G_D_PITCH]) + (desiredPitchRate * kp_ff_pitch)) * scaler;

	double yawCmd = yawDesired;


	// Update surfaces command
	(*aileronCmd) = (int) constrain(outputRollCmd * 100.0, -9000, 9000);
	(*gouvernCmd) = (int) constrain(outputPitchCmd * 100.0, -9000, 9000);
	(*rubberCmd) = (int) constrain(yawCmd * 100.0, -9000, 9000);
#endif
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
			Bound(bankAngle, 0, 90);

			// Compensate only with acceptable roll angle from 0 to 70 deg
			if (bankAngle < 70) {
				pitch_offset = PITCH2SRV_ROLL * bankAngle * (PITCH_MAX_COMPENSATION / 45.0);
				Bound(pitch_offset, 0, PITCH_MAX_COMPENSATION);
			}
		}


		previousCmd->roll = desiredRoll;
		previousCmd->pitch = desiredPitch + pitch_offset;
	}

	// To compensate roll, turn rubber a bit
	double yaw_offset = 0.0;
	if (YAW_ROLL_COMPENSATION == 1) {
		yaw_offset = - YAW2SRV_ROLL * currentAttitude->roll * (YAW_MAX_COMPENSATION / 45.0) ;
		Bound(yaw_offset, -YAW_MAX_COMPENSATION, YAW_MAX_COMPENSATION);
	}

	previousCmd->yaw = desiredYaw + yaw_offset;
	previousCmd->time = ctime / MS_TO_US;
}


//---------------------------------------------------------------------------------
//---------------------------------------------------------------------------------
// Auto speed using airspeed and PID on throttle
// Function called at 20 Hz (each 50 ms)

double v_ctl_auto_airspeed_sum_err = 0.0;
double previous_err_airspeed = 0.0;

int controlSpeedWithThrustV2(long cTimeUs, int currentDeciThrust, double v_ms_goal, Attitude *currentAttitude) {
	// Variables
	int deciThrustOutput = 0;
	float thrustOutput = 0;

	// PID error
	float err_airspeed = (v_ms_goal - airspeed_ms_mean->getAverage());
	v_ctl_auto_airspeed_sum_err += err_airspeed;
	BoundAbs(v_ctl_auto_airspeed_sum_err, param[ID_AUTOAIRSPEED_MAX_SUM_ERR]);

	// Output
	thrustOutput = param[ID_G_P_THRUST] * err_airspeed
			+ param[ID_G_I_THRUST] * v_ctl_auto_airspeed_sum_err
			+ param[ID_G_D_THRUST] * (err_airspeed - previous_err_airspeed);


	// Constrain output
	deciThrustOutput = (int)(10*thrustOutput);
	Bound(deciThrustOutput, 0, 1000);

	// Store previous variables
	previous_err_airspeed = err_airspeed;

	return deciThrustOutput;
}

//---------------------------------------------------------------------------------
//---------------------------------------------------------------------------------
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
		Bound(deltaThrust, -AUTOTHROTTLE_THRUST_SLEW_RATE, AUTOTHROTTLE_THRUST_SLEW_RATE);
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

		Bound(deciThrustOutput, deciThrustMin, deciThrustMax);

		// Store previous error
		previousVmsError = error;
	}


	return deciThrustOutput;
}


#endif /* FLIGHTCONTROL_H_ */
