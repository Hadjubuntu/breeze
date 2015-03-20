#ifndef ALTITUDE_CONTROLLER_H_
#define ALTITUDE_CONTROLLER_H_

/**
 *
 */
long sumError = 0;
void altitudeHoldController() { // PARAMETERS : int altSetPointCm, int currentAltCm, int *ptnDeciThrustPercent
	// Input
	double Kp = 1.0;
	double Ki = 0.1;
	int I_max = 200;

//	int errorCm = altSetPointCm - currentAltCm;
//	sumError = sumError + errorCm;

	// Input altitude setPoint
	// Get estimated altitude
	// Error altitude
	// Update PID
}

#endif
