/*
 * AHRS_DCM.h
 *
 *  Created on: Jul 12, 2015
 *      Author: adrien
 */

#ifndef MODULES_AHRS_AHRS_DCM_H_
#define MODULES_AHRS_AHRS_DCM_H_


#include "AHRS.h"
#include "math/Matrix3f.h"
#include "InertialNav.h"


class AHRS_DCM {
private:
	long _last_startup_ms;

	// primary representation of attitude of board used for all inertial calculations
	Matrix3f _dcm_matrix;

	// primary representation of attitude of flight vehicle body
	Matrix3f _body_dcm_matrix;

	Vector3f _omega_P;                          // accel Omega proportional correction
	Vector3f _omega_yaw_P;                      // proportional yaw correction
	Vector3f _omega_I;                          // Omega Integrator correction
	Vector3f _omega_I_sum;
	float _omega_I_sum_time;
	Vector3f _omega;                            // Corrected Gyro_Vector data

	// state of accel drift correction
	Vector3f _ra_sum[1];
	Vector3f _last_velocity;
	float _ra_deltat;
	uint32_t _ra_sum_start;

	// state to support status reporting
	float _renorm_val_sum;
	uint16_t _renorm_val_count;
	float _error_rp;
	float _error_yaw;

	long _last_failure_ms;

	float _P_gain(float);
	float _yaw_gain();

	float _kp_yaw;
	float _ki_yaw;
	float _kp;
	float _ki;
public:
	void resetGyroDrift();
	void resetAttitude(const float &roll, const float &pitch, const float &yaw);
	void update();
	void matrixUpdate(float);
	void reset(bool);
	void resetAttitude();
	void checkMatrix();
	bool renorm(Vector3f const &a, Vector3f &result);
	void normalize();
	void driftCorrectionYaw();
	void raDelayed();
	void driftCorrection(float);
	void eulerAngles();
};

void
AHRS_DCM::resetGyroDrift(void)
{
	_omega_I = vect3fInstance(0.0, 0.0, 0.0);
	_omega_I_sum = vect3fInstance(0.0, 0.0, 0.0);
	_omega_I_sum_time = 0;
}


// run a full DCM update round
void
AHRS_DCM::update(void)
{
	float delta_t;

	if (_last_startup_ms == 0) {
		_last_startup_ms = timeMs();
	}


	// if the update call took more than 0.2 seconds then discard it,
	// otherwise we may move too far. This happens when arming motors
	// in ArduCopter
	if (dt_IMU > 0.2f) {
		memset(&_ra_sum[0], 0, sizeof(_ra_sum));
		_ra_deltat = 0;
		return;
	}

	// Integrate the DCM matrix using gyro inputs
	matrixUpdate(dt_IMU);

	// Normalize the DCM matrix
	normalize();

	// Perform drift correction
	driftCorrection(dt_IMU);

	// paranoid check for bad values in the DCM matrix
	checkMatrix();

	// Calculate pitch, roll, yaw for stabilization and navigation
	eulerAngles();
}

// update the DCM matrix using only the gyros
void AHRS_DCM::matrixUpdate(float _G_Dt)
{
	// note that we do not include the P terms in _omega. This is
	// because the spin_rate is calculated from _omega.length(),
	// and including the P terms would give positive feedback into
	// the _P_gain() calculation, which can lead to a very large P
	// value
	_omega = vect3fInstance(0.0, 0.0, 0.0);

	// average across first two healthy gyros. This reduces noise on
	// systems with more than one gyro. We don't use the 3rd gyro
	// unless another is unhealthy as 3rd gyro on PH2 has a lot more
	// noise
	uint8_t healthy_count = 0;
	_omega = vect3fAdd(_omega, gyroFiltered);
	healthy_count++;

	_omega = vect3fAdd(_omega, _omega_I);
	Vector3f rot = vect3fAdd(vect3fAdd(_omega, _omega_P), _omega_yaw_P);
	_dcm_matrix.rotate(vect3fMultiply(rot, _G_Dt));
}


/*
 *  reset the DCM matrix and omega. Used on ground start, and on
 *  extreme errors in the matrix
 */
void AHRS_DCM::reset(bool recover_eulers, Attitude *att)
{
	// reset the integration terms
	_omega_I = vect3fInstance(0.0, 0.0, 0.0);
	_omega_P = vect3fInstance(0.0, 0.0, 0.0);
	_omega_yaw_P = vect3fInstance(0.0, 0.0, 0.0);
	_omega = vect3fInstance(0.0, 0.0, 0.0);

	// if the caller wants us to try to recover to the current
	// attitude then calculate the dcm matrix from the current
	// roll/pitch/yaw values
	_dcm_matrix.from_euler(att->roll, att->pitch, att->yaw);

	_last_startup_ms = timeMs();
}



// reset the current attitude, used by HIL
void AHRS_DCM::resetAttitude(const float &_roll, const float &_pitch, const float &_yaw)
{
	_dcm_matrix.from_euler(_roll, _pitch, _yaw);
}

/*
 *  check the DCM matrix for pathological values
 */
void AHRS_DCM::checkMatrix(void)
{
	if (_dcm_matrix.is_nan()) {
		reset(true);
		return;
	}
	// some DCM matrix values can lead to an out of range error in
	// the pitch calculation via asin().  These NaN values can
	// feed back into the rest of the DCM matrix via the
	// error_course value.
	if (!(_dcm_matrix.c.x < 1.0f &&
			_dcm_matrix.c.x > -1.0f)) {
		// We have an invalid matrix. Force a normalisation.
		normalize();

		if (_dcm_matrix.is_nan() ||
				fabsf(_dcm_matrix.c.x) > 10) {
			// normalisation didn't fix the problem! We're
			// in real trouble. All we can do is reset
			//Serial.printf("ERROR: DCM matrix error. _dcm_matrix.c.x=%f\n",
			//	   _dcm_matrix.c.x);
			reset(true);
		}
	}
}



// renormalise one vector component of the DCM matrix
// this will return false if renormalization fails
bool AHRS_DCM::renorm(Vector3f const &a, Vector3f &result)
{
	float renorm_val;

	// numerical errors will slowly build up over time in DCM,
	// causing inaccuracies. We can keep ahead of those errors
	// using the renormalization technique from the DCM IMU paper
	// (see equations 18 to 21).

	// For APM we don't bother with the taylor expansion
	// optimisation from the paper as on our 2560 CPU the cost of
	// the sqrt() is 44 microseconds, and the small time saving of
	// the taylor expansion is not worth the potential of
	// additional error buildup.

	// Note that we can get significant renormalisation values
	// when we have a larger delta_t due to a glitch eleswhere in
	// APM, such as a I2c timeout or a set of EEPROM writes. While
	// we would like to avoid these if possible, if it does happen
	// we don't want to compound the error by making DCM less
	// accurate.

	renorm_val = 1.0f / vect3fnorm(a);

	// keep the average for reporting
	_renorm_val_sum += renorm_val;
	_renorm_val_count++;

	if (!(renorm_val < 2.0f && renorm_val > 0.5f)) {
		// this is larger than it should get - log it as a warning
		if (!(renorm_val < 1.0e6f && renorm_val > 1.0e-6f)) {
			// we are getting values which are way out of
			// range, we will reset the matrix and hope we
			// can recover our attitude using drift
			// correction before we hit the ground!
			//Serial.printf("ERROR: DCM renormalisation error. renorm_val=%f\n",
			//	   renorm_val);
			return false;
		}
	}

	result = vect3fMultiply(a, renorm_val);
	return true;
}

/*************************************************
 *  Direction Cosine Matrix IMU: Theory
 *  William Premerlani and Paul Bizard
 *
 *  Numerical errors will gradually reduce the orthogonality conditions expressed by equation 5
 *  to approximations rather than identities. In effect, the axes in the two frames of reference no
 *  longer describe a rigid body. Fortunately, numerical error accumulates very slowly, so it is a
 *  simple matter to stay ahead of it.
 *  We call the process of enforcing the orthogonality conditions �renormalization�.
 */
void AHRS_DCM::normalize(void)
{
	float error;
	Vector3f t0, t1, t2;

	error = _dcm_matrix.a * _dcm_matrix.b;                                              // eq.18

	t0 = _dcm_matrix.a - (_dcm_matrix.b * (0.5f * error));              // eq.19
	t1 = _dcm_matrix.b - (_dcm_matrix.a * (0.5f * error));              // eq.19
	t2 = t0 % t1;                                                       // c= a x b // eq.20

	if (!renorm(t0, _dcm_matrix.a) ||
			!renorm(t1, _dcm_matrix.b) ||
			!renorm(t2, _dcm_matrix.c)) {
		// Our solution is blowing up and we will force back
		// to last euler angles
		_last_failure_ms = timeMs();
		reset(true);
	}
}

// the _P_gain raises the gain of the PI controller
// when we are spinning fast. See the fastRotations
// paper from Bill.
float AHRS_DCM::_P_gain(float spin_rate)
{
	if (spin_rate < toRad(50)) {
		return 1.0f;
	}
	if (spin_rate > toRad(500)) {
		return 10.0f;
	}
	return spin_rate/toRad(50);
}


// _yaw_gain reduces the gain of the PI controller applied to heading errors
// when observability from change of velocity is good (eg changing speed or turning)
// This reduces unwanted roll and pitch coupling due to compass errors for planes.
// High levels of noise on _accel_ef will cause the gain to drop and could lead to
// increased heading drift during straight and level flight, however some gain is
// always available. TODO check the necessity of adding adjustable acc threshold
// and/or filtering accelerations before getting magnitude
float AHRS_DCM::_yaw_gain()
{
	float VdotEFmag = pythagorous2(insNav.getAccX_ef(), insNav.getAccY_ef());
	if (VdotEFmag <= 4.0f) {
		return 0.2f*(4.5f - VdotEFmag);
	}
	return 0.1f;
}


// yaw drift correction using the compass or GPS
// this function prodoces the _omega_yaw_P vector, and also
// contributes to the _omega_I.z long term yaw drift estimate
void AHRS_DCM::driftCorrectionYaw()
{
	bool new_value = false;
	float yaw_error;
	float yaw_deltat;

	if (!new_value) {
		// we don't have any new yaw information
		// slowly decay _omega_yaw_P to cope with loss
		// of our yaw source
		_omega_yaw_P *= 0.97f;
		return;
	}

	// convert the error vector to body frame
	float error_z = _dcm_matrix.c.z * yaw_error;

	// the spin rate changes the P gain, and disables the
	// integration at higher rates
	float spin_rate = vect3fnorm(_omega);

	// sanity check _kp_yaw
	if (_kp_yaw < 0.1) {
		_kp_yaw = 0.1;
	}

	// update the proportional control to drag the
	// yaw back to the right value. We use a gain
	// that depends on the spin rate. See the fastRotations.pdf
	// paper from Bill Premerlani
	// We also adjust the gain depending on the rate of change of horizontal velocity which
	// is proportional to how observable the heading is from the acceerations and GPS velocity
	// The accelration derived heading will be more reliable in turns than compass or GPS

	_omega_yaw_P.z = error_z * _P_gain(spin_rate) * _kp_yaw * _yaw_gain();


	// don't update the drift term if we lost the yaw reference
	// for more than 2 seconds
	if (yaw_deltat < 2.0f && spin_rate < toRad(20)) {
		// also add to the I term
		_omega_I_sum.z += error_z * _ki_yaw * yaw_deltat;
	}

	_error_yaw = 0.8f * _error_yaw + 0.2f * fabsf(yaw_error);
}


// perform drift correction. This function aims to update _omega_P and
// _omega_I with our best estimate of the short term and long term
// gyro error. The _omega_P value is what pulls our attitude solution
// back towards the reference vector quickly. The _omega_I term is an
// attempt to learn the long term drift rate of the gyros.
//
// This drift correction implementation is based on a paper
// by Bill Premerlani from here:
//   http://gentlenav.googlecode.com/files/RollPitchDriftCompensation.pdf
void AHRS_DCM::driftCorrection(float deltat)
{
	Vector3f velocity;
	uint32_t last_correction_time;
	Vector3f _accel_ef;

	// perform yaw drift correction if we have a new yaw reference
	// vector
	driftCorrectionYaw();

	// rotate accelerometer values into the earth frame
	_accel_ef = _dcm_matrix * accelFiltered;
	// integrate the accel vector in the earth frame between GPS readings
	_ra_sum[0] += _accel_ef[0] * deltat;



	// keep a sum of the deltat values, so we know how much time
	// we have integrated over
	_ra_deltat += deltat;


	// see if this is our first time through - in which case we
	// just setup the start times and return
	if (_ra_sum_start == 0) {
		_ra_sum_start = last_correction_time;
		_last_velocity = velocity;
		return;
	}

	// equation 9: get the corrected acceleration vector in earth frame. Units
	// are m/s/s
	Vector3f GA_e;
	GA_e = Vector3f(0, 0, -1.0f);

	bool using_gps_corrections = false;
	float ra_scale = 1.0f/(_ra_deltat*9.81);


	// calculate the error term in earth frame.
	// we do this for each available accelerometer then pick the
	// accelerometer that leads to the smallest error term. This takes
	// advantage of the different sample rates on different
	// accelerometers to dramatically reduce the impact of aliasing
	// due to harmonics of vibrations that match closely the sampling
	// rate of our accelerometers. On the Pixhawk we have the LSM303D
	// running at 800Hz and the MPU6000 running at 1kHz, by combining
	// the two the effects of aliasing are greatly reduced.
	Vector3f error[1];
	float error_dirn[1];
	Vector3f GA_b[1];
	int8_t besti = -1;
	float best_error = 0;

	_ra_sum[0] *= ra_scale;

	// get the delayed ra_sum to match the GPS lag
	GA_b[0] = _ra_sum[0];

	if (vect3fnorm(GA_b[0]) == 0.0) {
		// wait for some non-zero acceleration information
		continue;
	}

	vect3fnormalize(&GA_b[0]);



	error[0] = GA_b[0] % GA_e;
	// Take dot product to catch case vectors are opposite sign and parallel
	error_dirn[0] = GA_b[0] * GA_e;
	float error_length = vect3fnorm(error[0]);

	if (besti == -1 || error_length < best_error) {
		besti = 0;
		best_error = error_length;
	}
	// Catch case where orientation is 180 degrees out
	if (error_dirn[besti] < 0.0f) {
		best_error = 1.0f;
	}


	if (besti == -1) {
		// no healthy accelerometers!
		_last_failure_ms = timeMs();
		return;
	}




	_error_rp = 0.8f * _error_rp + 0.2f * best_error;

	// base the P gain on the spin rate
	float spin_rate = vect3fnorm(_omega);

	// sanity check _kp value
	if (_kp < 0.1) {
		_kp = 0.1;
	}

	// we now want to calculate _omega_P and _omega_I. The
	// _omega_P value is what drags us quickly to the
	// accelerometer reading.
	_omega_P = error[besti] * _P_gain(spin_rate) * _kp;


	// accumulate some integrator error
	if (spin_rate < toRad(20)) {
		_omega_I_sum += error[besti] * _ki * _ra_deltat;
		_omega_I_sum_time += _ra_deltat;
	}

	if (_omega_I_sum_time >= 5) {
		// limit the rate of change of omega_I to the hardware
		// reported maximum gyro drift rate. This ensures that
		// short term errors don't cause a buildup of omega_I
		// beyond the physical limits of the device
		float _gyro_drift_limit = 1.0;
		float change_limit = _gyro_drift_limit * _omega_I_sum_time;
		_omega_I_sum.x = constrain(_omega_I_sum.x, -change_limit, change_limit);
		_omega_I_sum.y = constrain(_omega_I_sum.y, -change_limit, change_limit);
		_omega_I_sum.z = constrain(_omega_I_sum.z, -change_limit, change_limit);
		_omega_I += _omega_I_sum;
		_omega_I_sum = vect3fInstance(0.0, 0.0, 0.0);
		_omega_I_sum_time = 0;
	}

	// zero our accumulator ready for the next GPS step
	memset(&_ra_sum[0], 0, sizeof(_ra_sum));
	_ra_deltat = 0;
	_ra_sum_start = last_correction_time;

	// remember the velocity for next time
	_last_velocity = velocity;
}




#endif /* MODULES_AHRS_AHRS_DCM_H_ */
