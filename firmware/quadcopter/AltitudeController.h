#ifndef ALTITUDE_CONTROLLER_H_
#define ALTITUDE_CONTROLLER_H_

long sumErrorAlt = 0;
int altSetPointCm = 120; // TO BE -1 and defined at start

/**
 * Altitude Hold Controller aims to keep UAV to level
 */
int altitudeHoldController(int currentAltCm, int deciThrustCmd, int deciThrustPercent) {

	// Init altitude setpoint
	if (altSetPointCm == -1) {
		altSetPointCm = currentAltCm;
	}

	// PARAMETERS : int altSetPointCm, int currentAltCm, int *ptnDeciThrustPercent
	// Update altsetpoint depending on the decithrust cmd
	// Increase/Decrease at 20Hz of 1cm means 20cm/s of change
	// TODO
//	if (deciThrustCmd > 550) {
//		altSetPointCm ++;
//	}
//	else if (deciThrustCmd < 450 && altSetPointCm > 0) {
//		altSetPointCm --;
//	}

	// Input
	double Kp = 0.1;
	double Ki = 0.05;
	int I_max = 20;

	// Truncate precision on error
	int errorCm = altSetPointCm - currentAltCm;
	sumErrorAlt = sumErrorAlt + errorCm;
	Bound(sumErrorAlt, -I_max, I_max);

	float output = deciThrustPercent + 10.0*(Kp * errorCm + Ki * sumErrorAlt);
	Bound(output, 0, 1000);

	return (int) output;
}

#endif
