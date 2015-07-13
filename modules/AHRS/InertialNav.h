/*
 * InertialNav.h
 *
 *  Created on: Jun 3, 2015
 *      Author: adrien
 */

#ifndef MODULES_AHRS_INERTIALNAV_H_
#define MODULES_AHRS_INERTIALNAV_H_

#include "math/Math.h"

class InertialNav
{
private:
	IntegralSmooth climbRateSmooth;
	IntegralSmooth velXsmooth;
	IntegralSmooth velYsmooth;
	Vector3f delta_pos;
	Vector3f vect_acc_ef_approx;
	Vector3f vect_acc_ef;
	long lastUpdate;
	bool sonarHealthy;
	float sonarClimbRateMs;
	float sonarAltCmTrack[50];
	bool climbRateHealthyData;
	int sonarAltIdx;

public:
	InertialNav() {
		climbRateSmooth.init(0.98, 50); // Smooth climb rate integral with alpha = 0.98 and update Hz = 50
		velXsmooth.init(0.98, 50);
		velYsmooth.init(0.98, 50);
		vect_acc_ef_approx.x = 0.0;
		vect_acc_ef_approx.y = 0.0;
		vect_acc_ef_approx.z = 0.0;
		delta_pos.x = 0.0;
		delta_pos.y = 0.0;
		delta_pos.z = 0.0;
		lastUpdate = 0;
		sonarClimbRateMs = 0.0;
		sonarAltIdx = 0;
		climbRateHealthyData = false;
		sonarHealthy = false;
	}

	/**
	 * Update inertial navigation using a 50Hz loop
	 * @param ctime current time in microseconds
	 * @param pSonarHealthy tells whether if the data from sonar are good
	 * @param sonarAltCm Altitude measured from sonar in centimeters
	 */
	void update50Hz(long ctime, bool pSonarHealthy, float sonarAltCm)
	{
		float dt = 0.0001;

		if (lastUpdate > 0) {
			dt = (ctime - lastUpdate) / S_TO_US;
		}

		// Rotate body-frame acceleration to acceleration in earth frame
		vect_acc_ef = rot_bf_ef(accelFiltered, UAVCore->currentAttitude);

		// Approximate acceleration earth-frame vector
		vect_acc_ef_approx.x = approx(vect_acc_ef.x);
		vect_acc_ef_approx.y = approx(vect_acc_ef.y);
		vect_acc_ef_approx.z = approx(vect_acc_ef.z);

		sonarHealthy = pSonarHealthy;

		if (sonarHealthy) {
			/**
			 * To measure climb rate, the data from sonar are measure around 1 second before and at the instant t
			 * as soon as
			 */
			sonarAltCmTrack[sonarAltIdx] = sonarAltCm;

			if (climbRateHealthyData)
			{
				int indexPast1Second = (sonarAltIdx + 1) % 50;
				sonarClimbRateMs = (sonarAltCmTrack[sonarAltIdx] - sonarAltCmTrack[indexPast1Second]) / 100.0;
			}

			sonarAltIdx ++;

			if (sonarAltIdx >= 50)
			{
				// Reset index
				sonarAltIdx = 0;
				climbRateHealthyData = true;
			}

		}
		else {

			// Smooth integral on acceleration to get lean velocity on x, y and z-axis
			// z-axis velocity defines the climb rate
			climbRateSmooth.update(G_MASS * (vect_acc_ef_approx.z - 1.0), dt);
		}

		// Inertial nav doesn't work well due to accelerometer imprecision
		velXsmooth.update(G_MASS * vect_acc_ef_approx.x, dt);
		velYsmooth.update(G_MASS * vect_acc_ef_approx.y, dt);

		float V_MAX = 15.0;
		delta_pos.x = 0.99 * delta_pos.x + velXsmooth.getOutput() * dt + (-UAVCore->currentAttitude->pitch/90.0)*V_MAX*dt;
		delta_pos.y = 0.99 * delta_pos.y + velYsmooth.getOutput() * dt + (UAVCore->currentAttitude->roll/90.0)*V_MAX*dt;

		lastUpdate = ctime;
	}


	float getClimbRateMs()
	{
		if (sonarHealthy) {
			return sonarClimbRateMs;
		}
		else {
			return climbRateSmooth.getOutput();
		}
	}

	float getVelX()
	{
		return velXsmooth.getOutput();
	}

	float getDeltaPosX()
	{
		return delta_pos.x;
	}

	float getDeltaPosY()
	{
		return delta_pos.y;
	}

	float getVelY()
	{
		return velYsmooth.getOutput();
	}


	float getAccX_ef()
	{
		return vect_acc_ef_approx.x;
	}

	float getAccY_ef()
	{
		return vect_acc_ef_approx.y;
	}

	float getAccZ_ef()
	{
		return vect_acc_ef_approx.z;
	}
};

// Inertial naviguation
InertialNav insNav;


#endif /* MODULES_AHRS_INERTIALNAV_H_ */
