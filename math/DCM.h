/*
 * DCM.h
 *
 *  Created on: May 26, 2015
 *      Author: adrien
 */

#ifndef MATH_DCM_H_
#define MATH_DCM_H_

#include "Matrix.h"


// the limit (in degrees/second) beyond which we stop integrating
// omega_I. At larger spin rates the DCM PI controller can get 'dizzy'
// which results in false gyro drift. See
// http://gentlenav.googlecode.com/files/fastRotations.pdf
#define SPIN_RATE_LIMIT 20


class DCM {
protected:
	// primary representation of attitude of board used for all inertial calculations
	Matrix3f _dcm_matrix;

	// primary representation of attitude of flight vehicle body
	Matrix3f _body_dcm_matrix;



public:
	DCM()
{

}


	/**
	 * Update DCM matrix
	 *
	 * @param delta_t represents dt between two IMU readings
	 */
	void update(Vector3<float> gyro, float delta_t)
	{
		// if the update call took more than 0.2 seconds then discard it,
		// otherwise we may move too far. This happens when arming motors
		// in ArduCopter
		if (delta_t > 0.2f) {
			return;
		}

		// Integrate the DCM matrix using gyro inputs
		matrix_update(gyro, delta_t);

		// Normalize the DCM matrix
		normalize();

		// Perform drift correction
		drift_correction(delta_t);

		// paranoid check for bad values in the DCM matrix
		check_matrix();

		// Calculate pitch, roll, yaw for stabilization and navigation
		euler_angles();

		// update trig values including _cos_roll, cos_pitch
		update_trig();
	}

	void drift_correction(float deltat)
	{

	}



	void matrix_update(Vector3<float> gyro, float _G_Dt)
	{

	}

	void normalize(void)
	{

	}


	// renormalise one vector component of the DCM matrix
	// this will return false if renormalization fails
	bool renorm(Vect3f const &a, Vect3f &result)
	{

		return true;
	}

	void reset(bool recover_eulers)
	{

	}

	void check_matrix(void)
	{

	}



	// calculate the euler angles and DCM matrix which will be used for high level
	// navigation control. Apply trim such that a positive trim value results in a
	// positive vehicle rotation about that axis (ie a negative offset)
	void euler_angles(void)
	{

	}



};


#endif /* MATH_DCM_H_ */
