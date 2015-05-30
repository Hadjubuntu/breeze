/*
 * PID.h
 *
 * Generic PID implementation simple non-linear model with a K(t) = Kp * Ke(t) depending on error
 *
 *  Created on: 9 august 2014
 *      Author: hadjmoody
 */

#ifndef PID_H_
#define PID_H_

#include "Common.h"

// TODO create Kp(t) = K0*Ke(t) with right parameters

class PIDe {
protected:
	float Kp;
	float Ki;
	float Kd;
	float maxI;
	float i;
	float d;
	float alpha_d;
	float prevError;
	float output;

	// See papers : http://downloads.deusm.com/designnews/1477-Elsevier06.pdf
	bool useEnhancePID;
	float Kboost;
	float Kboost_max;
public:
	PIDe();
	void init(float pKp, float pKi, float pKd, float pMaxI);
	void update(float error, float dtSeconds);
	float getOutput();
	void reset();
};

PIDe::PIDe()
{
	init(1.0, 0.1, 0.01, 10);
}

void PIDe::init(float pKp, float pKi, float pKd, float pMaxI) {
	Kp = pKp;
	Ki = pKi;
	Kd = pKd;
	maxI = pMaxI;
	i = 0.0;
	d = 0.0;
	alpha_d = 0.6;
	prevError = 0.0;
	output = 0.0;
	useEnhancePID = true;
	Kboost = 0.125;
	Kboost_max = 2.0;
}

void PIDe::reset()
{
	i = 0.0;
	d = 0.0;
	prevError = 0.0;
}

void PIDe::update(float e, float dtSeconds)
{
	// Evaluate differential
	float dError = 0.0;
	if (dtSeconds > 0.0) {
		dError = (e - prevError) / dtSeconds;
	}
	prevError = e;

	// Evaluate integral
	i = i + e * dtSeconds;
	BoundAbs(i, maxI);

	d = (1.0 - alpha_d) * d + alpha_d * dError;


	float Ke = 1.0;
	if (useEnhancePID)
	{
		Ke = (exp(Kboost * e) + exp(-Kboost * e)) / 2.0;
		Bound(Ke, 0.5, Kboost_max);
	}

	// Computes output
	output = Ke * (Kp * e + Ki * i + Kd * d);
}

float PIDe::getOutput()
{
	return output;
}


#endif /* PID_H_ */
