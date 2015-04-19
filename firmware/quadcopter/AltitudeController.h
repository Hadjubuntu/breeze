#ifndef ALTITUDE_CONTROLLER_H_
#define ALTITUDE_CONTROLLER_H_

#include "arch/AVR/MCU/MCU.h"

float sumErrorAlt = 0.0;
float altSetPointCm = 25.0; // TO BE -1 and defined at start
float output_alt_controller = 0.0;
long altc_time = 0;

/**
 * Altitude Hold Controller aims to keep UAV to level
 */
void altitudeHoldController(float climb_rate_ms, int currentAltCm, int deciThrustCmd) {

	// PARAMETERS : int altSetPointCm, int currentAltCm, int *ptnDeciThrustPercent
	// Update altsetpoint depending on the decithrust cmd
	// Increase/Decrease at 20Hz of 1cm means 20cm/s of change
	// TODO
//	if (deciThrustCmd > 200) {
//		altSetPointCm = altSetPointCm + 0.5;
//	}
//	else if (deciThrustCmd < 150 && altSetPointCm > 0) {
//		altSetPointCm = altSetPointCm - 0.5;
//	}
//	Bound(altSetPointCm, 0, 3000);

	// Input
	double Ki = 0.02;
	int I_max = 40;

	// Truncate precision on error
	float climb_rate_desired = (altSetPointCm - currentAltCm)*0.05; // Into cm (0.01), then Kp=5
	BoundAbs(climb_rate_desired, 1.5); // Bound to +/- 2m/s

	float errorClimbRateCm = climb_rate_desired - climb_rate_ms;

	sumErrorAlt = sumErrorAlt + errorClimbRateCm;
	Bound(sumErrorAlt, -I_max, I_max);

	output_alt_controller =  output_alt_controller + 10.0*(Ki * sumErrorAlt);
	Bound(output_alt_controller, 0, 330);

}

/**
 * Memo saved previous version
 * // Input
	double Ki = 0.02;
	int I_max = 40;

	// Truncate precision on error
	float errorCm = altSetPointCm - currentAltCm;
	sumErrorAlt = sumErrorAlt + errorCm;
	Bound(sumErrorAlt, -I_max, I_max);

	output_alt_controller =  output_alt_controller + 10.0*(Ki * sumErrorAlt);
	Bound(output_alt_controller, 0, 280);
 */

#endif
