/*
 * PID.h
 *
 * @deprecated Not used anymore (see FlightControl instead)
 *
 *  Created on: 9 august 2014
 *      Author: hadjmoody
 */

#ifndef PID_H_
#define PID_H_

// Input roll / pitch / yaw
// Output aileron / gouvern / rudder command

#define MAX_INTEGRAL 15

typedef struct T_PID {
	double integralValue ;
	double previousError ;
	double Kp, Ki, Kd ;
} PID ;

void initializePID(PID *pid, double pKp, double pKi, double pKd) {
	pid->integralValue = 0.0 ;
	pid->previousError = 0.0 ;
	pid->Kp = pKp ;
	pid->Ki = pKi ;
	pid->Kd = pKd ;
}



double getDerivative(PID *pid, double error, double previousError, double dt) {
	if (dt <= 0) {
		return 0.0;
	}
	else {
		return pid->Kd * (error - previousError ) / dt ;
	}
}

double getIntegral(PID *pid, double error, double previousError, double dt) {
	if (dt <= 0) {
		return pid->Ki * pid->integralValue ;
	}
	else {
		// Add new error diff only if integral value belongs to interval (-max, max)
		if ((pid->Ki * pid->integralValue < MAX_INTEGRAL)
				|| (pid->Ki * pid->integralValue > -MAX_INTEGRAL)) {
			pid->integralValue += (error-previousError) / dt ;
		}

		return pid->Ki * pid->integralValue;
	}
}

double getOutput(PID *pid, double e, double dt) {    
	return pid->Kp * e  + getDerivative(pid, e, pid->previousError, dt)  + getIntegral(pid, e, pid->previousError, dt);
}

// Centi-degree command
int PID2ServoAngle(double value) {
	int delta = (int) (value * 100.0f) ;
	if (delta > 9000) {
		delta = 9000 ;
	}
	else if (delta < -9000) {
		delta =  -9000;
	}

	return delta ;
}



#endif /* PID_H_ */
