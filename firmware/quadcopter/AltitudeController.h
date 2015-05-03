#ifndef ALTITUDE_CONTROLLER_H_
#define ALTITUDE_CONTROLLER_H_

#include "arch/AVR/MCU/MCU.h"

int i = 0;
float altSetPointCm = 25.0; // TO BE -1 and defined at start
float output_alt_controller = 0.0;

// 1 m/s means 50 deci throttle
int climbrateToDeciThrottle(float climbrate_ms) {
	return climbrate_ms*50;
}

/**
 * Altitude Hold Controller aims to keep UAV to level
 */
void altitudeHoldController(float climb_rate_ms, int currentAltCm, int deciThrustCmd) {

	// Output
	int _throttle_hover = 250;
	int p = 0;
	int d = 0;

	// Input
	float K_climbrate = 1.0;
	float Ki = 0.01;
	int I_max = 80;


	float error_meters = ((int)((altSetPointCm - currentAltCm)/10.0))*10;
	float climb_rate_desired_ms = error_meters * K_climbrate;
	BoundAbs(climb_rate_desired_ms, 2.0);

	int errorThrottle = climbrateToDeciThrottle(climb_rate_desired_ms - climb_rate_ms);

	p = errorThrottle;
	i = i + Ki * errorThrottle;
	BoundAbs(i, I_max);

	output_alt_controller = _throttle_hover + p + i + d;
	Bound(output_alt_controller, 0, 370);

}


#endif
