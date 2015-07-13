/*
 * AHRS_Kalman.h
 *
 * Attitude and heading reference system
 *
 *  Created on: May 1, 2015
 *      Author: adrien
 */

#ifndef MODULES_AHRS_AHRS_KALMAN_H_
#define MODULES_AHRS_AHRS_KALMAN_H_

#include "AHRS.h"
#include "Common.h"
#include "math/Kalman.h"
#include "math/Math.h"


// Kalman filters for roll and pitch axis
//-------------------------------------------
Kalman kalX, kalY;
double roll_cpfilter = 0.0;
double pitch_cpfilter = 0.0;


void initAHRS(float initRoll, float initPitch) {
	kalX.setOutput(initRoll);
	kalY.setOutput(initPitch);
}


/**
 * Update system prediction and state
 */
void updateAHRS(float pAccRoll, float pGyroXrateRad, float pAccPitch, float pGyroYrateRad, float dt)
{
	// Update kalman prediction and state
	//-----------------------------------------------
	kalX.update(pAccRoll, pGyroXrateRad * RAD2DEG, dt);
	kalY.update(pAccPitch, pGyroYrateRad * RAD2DEG, dt);

	// Complementary filter
	roll_cpfilter = roll_cpfilter + toDeg(pGyroXrateRad) * dt;
	roll_cpfilter = 0.9 * roll_cpfilter +  pAccRoll * 0.1;

	pitch_cpfilter = pitch_cpfilter + toDeg(pGyroYrateRad) * dt;
	pitch_cpfilter = 0.9 * pitch_cpfilter +  pAccPitch * 0.1;
}


#endif /* MODULES_AHRS_AHRS_KALMAN_H_ */
