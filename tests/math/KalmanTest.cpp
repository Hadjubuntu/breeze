/*
 * KalmanTest.cpp
 *
 *  Created on: Apr 30, 2015
 *      Author: adrien
 */




#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "../../math/Kalman.h"

int main() {

	// Create kalman object
	Kalman kal;

	printf("Kalman test\n");

	// Set to 0 degrees
	kal.setOutput(0.0);

	double dt = 0.01;
	double finalDeg = 30.0;
	double rate = 0.5; // deg / s
	int n = finalDeg / (rate * dt);

	int i = 0;
	double cAngle = 0.0;

	while (i < n)
	{
		float error = 5*((float)(rand()) / RAND_MAX - 0.5) ;

		cAngle += rate * dt + error *dt ; // plus error bias
		kal.update(cAngle, rate, dt);

		i ++;
	}

	printf("Kalman output = %f | angle meas = %f\n", kal.getOutput(), cAngle);
	printf("Kalman bias = %f\n\n", kal.getBias());

	return 0;
}
