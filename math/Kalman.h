#ifndef KALMAN_H_
#define KALMAN_H_

/**
 * Kalman filter helps the fusion of two sensors raw data.
 * It is used for the IMU in order to have nice roll and pitch values.
 * Inspired by : http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/#step1
 */
class Kalman {
public:
	/**
	 * Constructor to initialize variables
	 */
	Kalman() {

		// Qi represents the model noise
		// Q = Q_meas * dt 0.001, 0.003, 0.03 (Relative trust from accelerometer relative to the gyro)
		Q_sensor1 = 0.005; // 0.001; // 0.1; // 0.001  0.15;
		Q_sensor2 = 0.005; //0.003; // 0.15; // 0.003    0.15;

		// R represents the measurement noise (accelerometer jitter in deg)
		R_measure = 0.5; // 5.0; // 0.3    10.0; // Variance

		output = 0; // Reset the angle
		bias = 0; // Reset bias
		rate = 0;
		S = 0;
		y = 0;

		// Starts with really low covariance matrix because it increases with time until convergence
		P[0][0] = 0.001;
		P[0][1] = 0.0;
		P[1][0] = 0.0;
		P[1][1] = 0.001;
	};

	/**
	 * Update and get output
	 * ----------------------------------------------------------------	 *
	 * RawInputSensor1 must be in degrees from accelerometer
	 * RawInputSensor2 in degrees/sec from gyroscope
	 * and dt in seconds
	 */
	double update(double rawInputSensor1, double rawInputSensor2, double dt) {

		// Discrete Kalman filter time update equations - Time Update ("Predict")
		// Update xhat - Project the state ahead
		/* Step 1 */
		rate = rawInputSensor2 - bias;
		output += dt * rate;

		// Update estimation error covariance - Project the error covariance ahead
		/* Step 2 */
		P[0][0] += dt * (dt * P[1][1] - 2 * P[0][1] + Q_sensor1);
		P[0][1] -= dt * P[1][1];
		P[1][0] -= dt * P[1][1];
		P[1][1] += Q_sensor2 * dt;

		// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
		// Calculate Kalman gain - Compute the Kalman gain
		/* Step 4 */
		S = P[0][0] + R_measure;
		/* Step 5 */
		K[0] = P[0][0] / S;
		K[1] = P[1][0] / S;

		// Calculate output and bias - Update estimate with measurement zk (newAngle)
		/* Step 3 */
		y = rawInputSensor1 - output;
		/* Step 6 */
		output += K[0] * y;
		bias += K[1] * y;

		// Calculate estimation error covariance - Update the error covariance
		/* Step 7 */
		P[0][0] -= K[0] * P[0][0];
		P[0][1] -= K[0] * P[0][1];
		P[1][0] -= K[1] * P[0][0];
		P[1][1] -= K[1] * P[0][1];

		return output;
	};

	float getCov_00() {
		return P[0][0];
	}

	//----------------------------------------------------------------------
	// Getter
	//----------------------------------------------------------------------
	void setOutput(double pOutput) { output = pOutput; }; // Start with output value to calibrate
	double getRate() { return rate; }; // Return the unbiased rate

	/* These are used to tune the Kalman filter */
	void setQangle(double newQ_angle) { Q_sensor1 = newQ_angle; };
	void setQbias(double newQ_bias) { Q_sensor2 = newQ_bias; };
	void setRmeasure(double newR_measure) { R_measure = newR_measure; };

	double getQangle() { return Q_sensor1; };
	double getQbias() { return Q_sensor2; };
	double getRmeasure() { return R_measure; };

	double getOutput() { return output; }

	double getBias() { return bias; }

	double getP00() { return P[0][0]; }
	double getP11() { return P[1][1]; }

private:
	/* Kalman filter variables */
	double Q_sensor1; // Process noise variance for the sensor 1
	double Q_sensor2; // Process noise variance for the sensor 2 bias
	double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

	double output; // Output fusion data calculated by the Kalman filter - part of the 2x1 state vector
	double bias; // Sensor 1 bias calculated by the Kalman filter - part of the 2x1 state vector
	double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	double P[2][2]; // Error covariance matrix - This is a 2x2 matrix
	double K[2]; // Kalman gain - This is a 2x1 vector
	double y; // Difference between prediction and input data
	double S; // Estimate error
};





#endif /* KALMAN_H_ */
