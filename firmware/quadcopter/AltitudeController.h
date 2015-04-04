#ifndef ALTITUDE_CONTROLLER_H_
#define ALTITUDE_CONTROLLER_H_

long sumErrorAlt = 0;
int altSetPointCm = -1;

/**
 * Altitude Hold Controller aims to keep UAV to level
 */
void altitudeHoldController(int currentAltCm, int deciThrustCmd, int *ptnDeciThrustPercent) {

	// Init altitude setpoint
	if (altSetPointCm == -1) {
		altSetPointCm = currentAltCm;
	}

	// PARAMETERS : int altSetPointCm, int currentAltCm, int *ptnDeciThrustPercent
	// Update altsetpoint depending on the decithrust cmd
	// Increase/Decrease at 20Hz of 1cm means 20cm/s of change
	if (deciThrustCmd > 550) {
		altSetPointCm ++;
	}
	else if (deciThrustCmd < 450 && altSetPointCm > 0) {
		altSetPointCm --;
	}

	// Input
	double Kp = 1.0;
	double Ki = 0.05;
	int I_max = 100;

	// Truncate precision on error
	int errorCm = 10 * (int)((altSetPointCm - currentAltCm)/10.0);
	sumErrorAlt = sumErrorAlt + errorCm;
	Bound(sumErrorAlt, -I_max, I_max);

	(*ptnDeciThrustPercent) = (*ptnDeciThrustPercent) + Kp * errorCm + Ki * sumErrorAlt;
}

#endif
